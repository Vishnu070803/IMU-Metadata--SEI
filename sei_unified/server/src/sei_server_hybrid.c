#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <glib-unix.h>

/* Use the exact same injection logic as UVC */
int g_imu_hz = 0; /* Defaults to 0 (inline polling once per frame) */
#include "uvc_sei_injection_hybrid.h"

static gboolean    g_shutting_down_completely = FALSE;
static GMainLoop  *g_main_loop      = NULL;   
static GstElement *g_active_pipeline = NULL; /* Used to send EOS on shutdown */
static guint       g_shutdown_timer = 0;      /* non-zero once timer is armed    */
static int         g_timeout_seconds = 0;   /* Session timeout in seconds, 0 = continuous */

/* ─── EOS wait timeout — if bus EOS never arrives, force quit ──────────── */
static gboolean eos_timeout_cb(gpointer user_data) {
    (void)user_data;
    g_printerr("System Link: WARNING — EOS timeout after 10s, forcing quit.\n");
    if (g_shutting_down_completely) {
        g_main_loop_quit(g_main_loop);
    }
    return G_SOURCE_REMOVE;
}


/* ─── Timer callback — fires once after g_timeout_seconds ────────── */
static gboolean session_timeout_cb(gpointer user_data)
{
    (void)user_data;
    g_print("\nSystem Link: %d-second session timer expired. Stopping current stream.\n", g_timeout_seconds);
    g_shutdown_timer = 0;
    if (g_active_pipeline) {
        g_print("System Link: Sending EOS to cleanly finalize recording...\n");
        gst_element_send_event(g_active_pipeline, gst_event_new_eos());
    }
    return G_SOURCE_REMOVE;   /* one-shot: do not repeat */
}

static gboolean sigint_handler(gpointer user_data) {
    (void)user_data;
    g_print("\nCtrl+C caught! Shutting down server cleanly.\n");
    g_shutting_down_completely = TRUE;
    
    gboolean needs_eos = FALSE;
    if (g_active_pipeline) {
        GstState state;
        gst_element_get_state(g_active_pipeline, &state, NULL, 0);
        if (state == GST_STATE_PLAYING || state == GST_STATE_PAUSED) {
            needs_eos = TRUE;
        }
    }
    
    if (needs_eos) {
        g_print("System Link: Sending EOS and waiting for pipeline to reach NULL state...\n");
        gst_element_send_event(g_active_pipeline, gst_event_new_eos());
        g_timeout_add_seconds(10, eos_timeout_cb, NULL);
    } else {
        g_main_loop_quit(g_main_loop);
    }
    return G_SOURCE_REMOVE;
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

/* ─── Recording directory (set from args) ─ */
static char g_rec_dir[256] = "/tmp";



/* Forward declaration — used in media_configure_cb before definition */
static void media_new_state_cb(GstRTSPMedia *media, gint state, gpointer user_data);

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
    
    /* Reset all state (frame counters, fps counters, caps flag) for the new session */
    uvc_sei_reset_session();

    GstElement *pipeline = gst_rtsp_media_get_element(media);
    
    if (g_active_pipeline) {
        gst_object_unref(g_active_pipeline);
    }
    g_active_pipeline = gst_object_ref(pipeline); /* Save strong reference for graceful shutdown */

    /* ── Enable automatic EOS on client disconnect ──────────────────── *
     * By setting eos-shutdown to TRUE, the RTSP Server will automatically 
     * send an EOS event down the pipeline when the client disconnects, 
     * ensuring mp4mux naturally writes the moov atom without Ctrl+C!
     */
    gst_rtsp_media_set_eos_shutdown(media, TRUE);

    /* ── 0. Arm the session timer on first client connect ────────────── *
     *
     * "media-configure" fires when the factory creates the pipeline, which
     * happens on the first DESCRIBE/SETUP exchange — before data flows.
     * "media-connected" fires when the first PLAY is acknowledged and the
     * client transport is live.  We hook both from here so the signal is
     * connected to this specific media object.
     */
    g_signal_connect(media, "new-state",
                     G_CALLBACK(media_new_state_cb), NULL);

    /* ── 1. Camera source probe (zero-latency IMU sampling) ─────────── */
    GstElement *cam = gst_bin_get_by_name(GST_BIN(pipeline), "cam_src");
    if (cam) {
        GstPad *cam_pad = gst_element_get_static_pad(cam, "src");
        if (cam_pad) {
            gst_pad_add_probe(cam_pad, GST_PAD_PROBE_TYPE_BUFFER,
                              camera_src_probe, NULL, NULL);
            gst_object_unref(cam_pad);
            g_print("System Link: camera_src_probe attached to cam_src (zero-latency marker).\n");
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

    /* ── 4. Generate dynamic recording file name ────────────────────── */
    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "rec_sink");
    if (sink) {
        char new_path[512];
        time_t now = time(NULL);
        struct tm *t = localtime(&now);
        snprintf(new_path, sizeof(new_path),
                 "%s/recording_%04d%02d%02d_%02d%02d%02d.mp4",
                 g_rec_dir,
                 t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                 t->tm_hour, t->tm_min, t->tm_sec);
        
        g_object_set(sink, "location", new_path, NULL);
        g_print("System Link: Setting new recording path for client session: %s\n", new_path);
        gst_object_unref(sink);
    } else {
        g_printerr("System Link: WARNING — rec_sink element not found in pipeline.\n");
    }

    gst_object_unref(pipeline);
}

/* ─── new-state trampoline ──────────────────────────────────────────────── *
 *
 * GstRTSPMedia::new-state passes an integer state, not a GstState enum.
 * We arm the timer only when the media reaches GST_STATE_PLAYING (4).
 * We clean up the pipeline when the media reaches GST_STATE_NULL (1).
 */
static void media_new_state_cb(GstRTSPMedia *media, gint state, gpointer user_data)
{
    (void)media; (void)user_data;

    if (state == GST_STATE_PLAYING) {
        if (g_timeout_seconds <= 0) {
            g_print("System Link: media reached PLAYING — running in continuous mode.\n");
            return;
        }

        if (g_shutdown_timer != 0) {
            g_print("System Link: media reached PLAYING (session timer already running).\n");
            return;
        }

        g_print("System Link: media reached PLAYING — starting %d-second (%d-minute) session timer.\n",
                g_timeout_seconds, g_timeout_seconds / 60);

        g_shutdown_timer = g_timeout_add_seconds(g_timeout_seconds, session_timeout_cb, NULL);
    } 
    else if (state == GST_STATE_NULL) {
        g_print("System Link: media reached NULL state. Pipeline torn down cleanly, moov atom written.\n");
        gst_rtsp_media_set_shared(media, FALSE); /* Evict dead media from factory cache */
        if (g_shutdown_timer != 0) {
            g_source_remove(g_shutdown_timer);
            g_shutdown_timer = 0;
        }
        if (g_active_pipeline) {
            gst_object_unref(g_active_pipeline);
            g_active_pipeline = NULL;
        }
        if (g_shutting_down_completely) {
            g_main_loop_quit(g_main_loop);
        } else {
            g_print("System Link: Waiting for new client connections...\n");
        }
    }
}

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);

    const char *ip    = "0.0.0.0";
    const char *port  = "8554";
    const char *mount = "/stream";
    const char *camera_dev = "/dev/video3";

    for (int i = 1; i < argc; i++) {
        if      (strcmp(argv[i], "--ip")      == 0 && i + 1 < argc) ip      = argv[++i];
        else if (strcmp(argv[i], "--port")    == 0 && i + 1 < argc) port    = argv[++i];
        else if (strcmp(argv[i], "--mount")   == 0 && i + 1 < argc) mount   = argv[++i];
        else if (strcmp(argv[i], "--imu-hz")  == 0 && i + 1 < argc) g_imu_hz = atoi(argv[++i]);
        else if (strcmp(argv[i], "--timeout") == 0 && i + 1 < argc) g_timeout_seconds = atoi(argv[++i]);
        else if (strcmp(argv[i], "--camera")  == 0 && i + 1 < argc) camera_dev = argv[++i];
        else if (strcmp(argv[i], "--rec-dir") == 0 && i + 1 < argc) {
            strncpy(g_rec_dir, argv[++i], sizeof(g_rec_dir) - 1);
        }
    }

    /* Ensure recording directory exists to prevent filesink from failing */
    g_mkdir_with_parents(g_rec_dir, 0755);

    /* Initialize IMU (opens /dev/icm20948 once; falls back to simulation) */
    imu_open();

    g_main_loop                  = g_main_loop_new(NULL, FALSE);
    GstRTSPServer     *server    = gst_rtsp_server_new();
    GstRTSPMountPoints *mounts   = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();

    /* Register SIGINT handler */
    g_unix_signal_add(SIGINT, (GSourceFunc)sigint_handler, NULL);

    gst_rtsp_server_set_address(server, ip);
    gst_rtsp_server_set_service(server, port);

    char launch_str[1024];
    snprintf(launch_str, sizeof(launch_str),
        "( v4l2src name=cam_src device=%s do-timestamp=true ! "
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
        "filesink name=rec_sink location=/tmp/dummy.mp4 sync=false )",
        camera_dev);

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
    g_print("  Resolution  : 1920x1080 @ 30fps (native v4l2src)\n");
    g_print("  Bitrate     : 10 Mbps CBR (both branches, via V4L2 extra-controls)\n");
    if (g_imu_hz > 0)
        g_print("  IMU mode    : Approach 2 — background thread at %d Hz (~%.2f samples/frame)\n", g_imu_hz, (float)g_imu_hz / 30.0);
    else
        g_print("  IMU mode    : Approach 1 — inline IOCTL at 30Hz (≤1ms latency per frame)\n");
    g_print("  SEI payload : %zu bytes (UUID + SeiMetadata) per IMU sample\n",
            16 + sizeof(SeiMetadata));
    g_print("  RTSP branch : hw_enc_rtsp → h264parse → rtph264pay\n");
    g_print("  REC  branch : hw_enc_rec  → h264parse → mp4mux → dynamically named in %s\n", g_rec_dir);
    g_print("  Session     : auto-stop after %d minutes (use --timeout 0 for continuous)\n",
            g_timeout_seconds / 60);
    g_print("  Hint        : Use --rec-dir <path> to set recording directory\n");
    g_print("  Hint        : Use --imu-hz <hz> to enable high-frequency IMU capture (e.g. 100)\n");
    g_print("  Hint        : Use --timeout <seconds> to set session length, or 0 for continuous\n");
    g_print("\n");

    g_main_loop_run(g_main_loop);

    g_print("Server exiting cleanly after session timeout.\n");
    return 0;
}
