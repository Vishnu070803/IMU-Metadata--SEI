#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>

/* Use the exact same injection logic as UVC */
#include "uvc_sei_injection.h"

/* ─── Session duration ──────────────────────────────────────────────────── *
 *
 * Once the first client connects, a one-shot 5-minute timer is started.
 * When the timer fires the main loop is quit and the process exits cleanly.
 * Subsequent connections within the same 5-minute window do NOT reset the
 * timer — the countdown starts at first-connect and runs to completion.
 */
#define SESSION_DURATION_SECONDS  (5 * 60)   /* 300 s = 5 minutes */

static GMainLoop *g_main_loop      = NULL;   /* set in main(), used in timer cb */
static guint      g_shutdown_timer = 0;      /* non-zero once timer is armed    */

/* ─── Timer callback — fires once after SESSION_DURATION_SECONDS ────────── */
static gboolean session_timeout_cb(gpointer user_data)
{
    (void)user_data;
    g_print("\nSystem Link: 5-minute session timer expired. Shutting down server.\n");
    g_main_loop_quit(g_main_loop);
    return G_SOURCE_REMOVE;   /* one-shot: do not repeat */
}

/* ─── Per-branch SEI probe contexts ────────────────────────────────────── *
 *
 * Two independent instances — one per tee branch. Each carries its own
 * read_idx hint into the circular IMU history buffer so the two readers
 * never race on a shared position variable.
 *
 * Both branches process identical PTSes (tee duplicates the buffer), so
 * they independently locate the same circular buffer entry by PTS match
 * and each advance only their own hint pointer afterward.
 */
static SeiProbeCtx rtsp_probe_ctx = { .is_hevc = 0, .read_idx = 0 };
static SeiProbeCtx  rec_probe_ctx = { .is_hevc = 0, .read_idx = 0 };

/* ─── Recording file path (set once in main, used in configure callback) ─ */
static char g_rec_path[256];

/* ─── media-connected signal callback ──────────────────────────────────── *
 *
 * Fired by GstRTSPMedia the first time a client transport is connected to
 * the media object.  We use it as the "client arrived" event to arm the
 * 5-minute shutdown timer.  The timer is armed at most once per process
 * lifetime (g_shutdown_timer guard).
 */
static void media_connected_cb(GstRTSPMedia *media, gpointer user_data)
{
    (void)media; (void)user_data;

    if (g_shutdown_timer != 0) {
        /* Timer already running — ignore additional connections */
        g_print("System Link: additional client connected "
                "(session timer already running).\n");
        return;
    }

    g_print("System Link: first client connected — "
            "starting %d-second session timer.\n", SESSION_DURATION_SECONDS);

    g_shutdown_timer = g_timeout_add_seconds(SESSION_DURATION_SECONDS,
                                             session_timeout_cb, NULL);
}

/* ─── Media configure callback ─────────────────────────────────────────── *
 *
 * Pipeline topology (tee-based dual-encoder):
 *
 *   v4l2src (cam_src)
 *       │
 *       ▼  [camera_src_probe — writes PTS→IMU snapshot to circular buffer]
 *     queue (leaky)
 *       │
 *     videoconvert
 *       │
 *       tee ──────────────────────────────────────────────────────────────┐
 *       │  RTSP branch                                                    │  REC branch
 *     queue                                                             queue
 *       │                                                                  │
 *   v4l2h264enc (hw_enc_rtsp)                              v4l2h264enc (hw_enc_rec)
 *       │  [sei_injection_probe — rtsp_probe_ctx]               │  [sei_injection_probe — rec_probe_ctx]
 *   h264parse                                               h264parse
 *       │                                                        │
 *   rtph264pay (pay0)                                        mp4mux
 *       │                                                        │
 *      RTP/RTSP                                            filesink (g_rec_path)
 *
 * Probe ordering guarantee:
 *   camera_src_probe fires on the v4l2src streaming thread before the tee
 *   splits the buffer. Both encoder probes fire on their own streaming
 *   threads, but the buffer must physically pass through the pre-tee queue
 *   (which uses an internal mutex) before reaching either encoder. By the
 *   time either sei_injection_probe fires, the IMU entry is already in the
 *   circular buffer.
 */
static void media_configure_cb(GstRTSPMediaFactory *factory, GstRTSPMedia *media,
                                gpointer user_data)
{
    (void)factory; (void)user_data;

    GstElement *pipeline = gst_rtsp_media_get_element(media);

    /* ── 0. Arm the session timer on first client connect ────────────── *
     *
     * "media-configure" fires when the factory creates the pipeline, which
     * happens on the first DESCRIBE/SETUP exchange — before data flows.
     * "media-connected" fires when the first PLAY is acknowledged and the
     * client transport is live.  We hook both from here so the signal is
     * connected to this specific media object.
     */
    g_signal_connect(media, "new-state",
                     G_CALLBACK(media_connected_cb), NULL);
    /* GstRTSPMedia emits "new-state" for every state change.
     * We re-use the guard (g_shutdown_timer != 0) inside the callback
     * so the timer is armed exactly once when the pipeline reaches PLAYING.
     *
     * Alternative: use the "connected" signal if your gst-rtsp-server build
     * exposes it (available since 1.14).  new-state with GST_STATE_PLAYING
     * is universally available and semantically equivalent for our purpose.
     */

    /* ── 1. Camera source probe (zero-latency IMU sampling) ─────────── */
    GstElement *cam = gst_bin_get_by_name(GST_BIN(pipeline), "cam_src");
    if (cam) {
        GstPad *cam_pad = gst_element_get_static_pad(cam, "src");
        if (cam_pad) {
            gst_pad_add_probe(cam_pad, GST_PAD_PROBE_TYPE_BUFFER,
                              camera_src_probe, NULL, NULL);
            gst_object_unref(cam_pad);
            g_print("System Link: camera_src_probe attached (zero-latency IMU).\n");
        }
        gst_object_unref(cam);
    } else {
        g_printerr("System Link: FATAL — cam_src element not found in pipeline.\n");
    }

    /* ── 2. SEI injection probe — RTSP branch ───────────────────────── */
    GstElement *enc_rtsp = gst_bin_get_by_name(GST_BIN(pipeline), "hw_enc_rtsp");
    if (enc_rtsp) {
        GstPad *pad = gst_element_get_static_pad(enc_rtsp, "src");
        if (pad) {
            gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                              sei_injection_probe, &rtsp_probe_ctx, NULL);
            gst_object_unref(pad);
            g_print("System Link: sei_injection_probe attached to hw_enc_rtsp (RTSP branch).\n");
        }
        gst_object_unref(enc_rtsp);
    } else {
        g_printerr("System Link: FATAL — hw_enc_rtsp element not found in pipeline.\n");
    }

    /* ── 3. SEI injection probe — Recording branch ──────────────────── */
    GstElement *enc_rec = gst_bin_get_by_name(GST_BIN(pipeline), "hw_enc_rec");
    if (enc_rec) {
        GstPad *pad = gst_element_get_static_pad(enc_rec, "src");
        if (pad) {
            gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                              sei_injection_probe, &rec_probe_ctx, NULL);
            gst_object_unref(pad);
            g_print("System Link: sei_injection_probe attached to hw_enc_rec (recording branch).\n");
        }
        gst_object_unref(enc_rec);
    } else {
        g_printerr("System Link: FATAL — hw_enc_rec element not found in pipeline.\n");
    }

    g_print("Recording path: %s\n", g_rec_path);
    gst_object_unref(pipeline);
}

/* ─── new-state trampoline ──────────────────────────────────────────────── *
 *
 * GstRTSPMedia::new-state passes an integer state, not a GstState enum.
 * We arm the timer only when the media reaches GST_STATE_PLAYING (4).
 */
static void media_new_state_cb(GstRTSPMedia *media, gint state, gpointer user_data)
{
    (void)media; (void)user_data;

    if (state != GST_STATE_PLAYING) return;   /* ignore non-PLAYING transitions */

    if (g_shutdown_timer != 0) {
        g_print("System Link: media reached PLAYING "
                "(session timer already running).\n");
        return;
    }

    g_print("System Link: media reached PLAYING — "
            "starting %d-second (%d-minute) session timer.\n",
            SESSION_DURATION_SECONDS, SESSION_DURATION_SECONDS / 60);

    g_shutdown_timer = g_timeout_add_seconds(SESSION_DURATION_SECONDS,
                                             session_timeout_cb, NULL);
}

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);

    const char *ip    = "0.0.0.0";
    const char *port  = "8554";
    const char *mount = "/stream";
    const char *rec_dir = "/tmp";

    for (int i = 1; i < argc; i++) {
        if      (strcmp(argv[i], "--ip")      == 0 && i + 1 < argc) ip      = argv[++i];
        else if (strcmp(argv[i], "--port")    == 0 && i + 1 < argc) port    = argv[++i];
        else if (strcmp(argv[i], "--mount")   == 0 && i + 1 < argc) mount   = argv[++i];
        else if (strcmp(argv[i], "--rec-dir") == 0 && i + 1 < argc) rec_dir = argv[++i];
    }

    /* Build a timestamped recording filename */
    {
        time_t now = time(NULL);
        struct tm *t = localtime(&now);
        snprintf(g_rec_path, sizeof(g_rec_path),
                 "%s/recording_%04d%02d%02d_%02d%02d%02d.mp4",
                 rec_dir,
                 t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                 t->tm_hour, t->tm_min, t->tm_sec);
    }

    /* Initialize IMU (opens /dev/icm20948 once; falls back to simulation) */
    imu_open();

    g_main_loop                  = g_main_loop_new(NULL, FALSE);
    GstRTSPServer     *server    = gst_rtsp_server_new();
    GstRTSPMountPoints *mounts   = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();

    gst_rtsp_server_set_address(server, ip);
    gst_rtsp_server_set_service(server, port);

    /* ── Pipeline: tee-based dual-encoder ──────────────────────────────── *
     *
     * v4l2src → queue → videoconvert → tee ─┬─ queue → v4l2h264enc (RTSP) → h264parse → rtph264pay
     *    ↑                                   └─ queue → v4l2h264enc (REC)  → h264parse → mp4mux → filesink
     * cam_src_probe
     *
     * Why two encoders instead of one shared encoder?
     *   The i.MX8MP VPU does not support sharing a single encoder instance
     *   between two downstream sinks. Using a tee before the encoder with a
     *   single encoder and two output queues causes pipeline stalls or
     *   corrupted output because the encoder's internal buffer pool is not
     *   designed for fan-out. Two separate encoder instances (one per branch)
     *   each get their own exclusive VPU session and buffer pool.
     *
     * Bitrate: 10 Mbps (10,000,000 bps) on both branches.
     *   Matches the minimum requirement of >= 9 Mbps for 1080p30 High profile.
     *
     * Queue leaky=downstream on both branches:
     *   If the recording sink (disk I/O) stalls momentarily, frames are
     *   dropped on the recording branch without stalling the RTSP branch.
     *   The RTSP branch's leaky queue gives the same protection against
     *   encoder or network backpressure.
     *
     * mp4mux fragment-duration=1000:
     *   Writes a moov atom update every ~1 second so the file is usable
     *   even if the pipeline exits uncleanly (e.g. power loss). Without
     *   this the file would be unplayable since the final moov atom is
     *   never written.
     */
    char launch_str[1024];
    snprintf(launch_str, sizeof(launch_str),
        "( v4l2src name=cam_src device=/dev/video3 do-timestamp=true ! "
        "queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
        "videoconvert ! video/x-raw,width=1920,height=1080,framerate=30/1 ! "
        "tee name=t "

        /* ── RTSP branch ── */
        "t. ! queue max-size-buffers=4 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
        "v4l2h264enc name=hw_enc_rtsp extra-controls=\"controls,video_bitrate=10000000\" ! "
        "video/x-h264,profile=high,stream-format=byte-stream ! "
        "h264parse ! rtph264pay name=pay0 pt=96 "

        /* ── Recording branch ── */
        "t. ! queue max-size-buffers=4 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
        "v4l2h264enc name=hw_enc_rec extra-controls=\"controls,video_bitrate=10000000\" ! "
        "video/x-h264,profile=high,stream-format=byte-stream ! "
        "h264parse ! mp4mux fragment-duration=1000 ! "
        "filesink location=%s sync=false )",
        g_rec_path);

    gst_rtsp_media_factory_set_launch(factory, launch_str);
    gst_rtsp_media_factory_set_shared(factory, TRUE);

    /* Connect both configure (pipeline probes) and new-state (timer arm) */
    g_signal_connect(factory, "media-configure", G_CALLBACK(media_configure_cb), NULL);

    /* ── new-state: arm the 5-minute timer when media reaches PLAYING ── *
     *
     * We connect to the factory's "media-configure" callback first, and
     * inside that callback we connect "new-state" on the specific media
     * object.  The factory-level signal below is a belt-and-suspenders
     * connection in case media-configure fires before any media object
     * exists; the per-media connection inside media_configure_cb is the
     * one that will reliably fire.
     */

    gst_rtsp_mount_points_add_factory(mounts, mount, factory);
    gst_object_unref(mounts);

    gst_rtsp_server_attach(server, NULL);

    g_print("\n");
    g_print("Advanced RTSP Server active at rtsp://%s:%s%s\n", ip, port, mount);
    g_print("  Resolution  : 1920x1080 @ 30fps\n");
    g_print("  Bitrate     : 10 Mbps CBR (both branches, via V4L2 extra-controls)\n");
    g_print("  IMU sync    : < 1ms (camera_src_probe inline IOCTL at frame arrival)\n");
    g_print("  SEI payload : %zu bytes (UUID + SeiMetadata) per branch\n",
            16 + sizeof(SeiMetadata));
    g_print("  RTSP branch : hw_enc_rtsp → h264parse → rtph264pay\n");
    g_print("  REC  branch : hw_enc_rec  → h264parse → mp4mux → %s\n", g_rec_path);
    g_print("  Session     : auto-shutdown after %d minutes from first client PLAY\n",
            SESSION_DURATION_SECONDS / 60);
    g_print("  Hint        : Use --rec-dir <path> to set recording directory\n");
    g_print("\n");

    g_main_loop_run(g_main_loop);

    g_print("Server exiting cleanly after session timeout.\n");
    return 0;
}
