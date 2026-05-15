/*
 * sei_client.c  — SEI Metadata Extractor with Parallel MP4 Recording + Cairo HUD
 *
 * Pipeline:
 *
 *   rtspsrc → rtph264depay → h264parse (disp_parser) [SEI probe]
 *           → video/x-h264,stream-format=byte-stream
 *           → avdec_h264 → videoconvert
 *           → cairooverlay name=hud_overlay   ← HUD rendered here (decoded pixels)
 *           → tee name=t
 *
 *       DISPLAY branch:
 *           t. → queue → videoconvert → autovideosink
 *
 *       RECORD branch (overlayed video encoded back to H.264 → MP4):
 *           t. → queue → videoconvert
 *                      → x264enc tune=zerolatency bitrate=8000
 *                      → video/x-h264,stream-format=avc,alignment=au
 *                      → qtmux fragment-duration=1000   ← fragmented MP4
 *                      → filesink location=<rec_path>
 *
 * Why tee AFTER cairooverlay?
 *   The overlay is applied once on the decoded YUV frame.  The tee then fans
 *   out the same post-overlay buffer to both the display sink and the encoder,
 *   so the recorded MP4 contains exactly the same HUD that the viewer sees on
 *   screen — no duplicate rendering, no double-decode overhead.
 *
 * Why x264enc for the record branch?
 *   On the client machine there is no VPU; x264enc is the universal software
 *   H.264 encoder available on every Linux desktop/laptop.  Swap for
 *   v4l2h264enc / nvh264enc / vaapih264enc on embedded or GPU-equipped hosts.
 *   tune=zerolatency keeps encoder latency near zero so the display branch
 *   (sharing the same tee output) is not held back by encoder buffering.
 *
 * Why qtmux with fragment-duration?
 *   Same reason as the server: fragmented MP4 is valid even after Ctrl-C
 *   because the moov atom is written incrementally, not at end-of-file.
 *
 * Recording filename:
 *   client_overlay_YYYYMMDD_HHMMSS.mp4 in the current working directory
 *   (overridable with --rec-dir <path>).
 *
 * Build:
 *   gcc sei_client.c -o sei_client \
 *       $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-video-1.0 cairo) \
 *       -pthread -lm
 */

#include <gst/gst.h>
#include <cairo/cairo.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <time.h>
#include <pthread.h>

#include "sei_metadata.h"

/* ─── Verbosity flag ────────────────────────────────────────────────────── */
static int g_verbose = 0;
static FILE *g_csv_file = NULL;

/* ─── Expected UUID (must match server injection) ───────────────────────── */
static const uint8_t EXPECTED_UUID[16] = {
    0xdc, 0x45, 0xe9, 0xbd, 0xe6, 0xd9, 0x48, 0xb7,
    0x96, 0x2c, 0xd3, 0x9c, 0x24, 0x5a, 0xc8, 0x1a
};

/* ─── Shared state: display streaming thread → video sink thread ────────── *
 *
 * The SEI probe fires on the GStreamer streaming thread.
 * The Cairo draw callback fires on the video sink thread.
 * A mutex guards the shared copy; both sides hold it only for a memcpy.
 */
typedef struct {
    pthread_mutex_t lock;
    SeiMetadata     meta;
    int             valid;
} OverlayState;

static OverlayState g_overlay = {
    .lock  = PTHREAD_MUTEX_INITIALIZER,
    .valid = 0,
};

/* ─── EPB removal ──────────────────────────────────────────────────────── */
static int remove_emulation_prevention(const uint8_t *src, int src_len, uint8_t *dst)
{
    int dst_len = 0, zero_count = 0;
    for (int i = 0; i < src_len; i++) {
        if (zero_count == 2 && src[i] == 0x03) {
            zero_count = 0;
            continue;
        }
        zero_count = (src[i] == 0x00) ? zero_count + 1 : 0;
        dst[dst_len++] = src[i];
    }
    return dst_len;
}

/* ─── SEI extraction probe (disp_parser src pad — byte-stream format) ───── */
static GstPadProbeReturn sei_extraction_probe(GstPad *pad, GstPadProbeInfo *info,
                                               gpointer user_data)
{
    (void)pad; (void)user_data;

    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buffer) return GST_PAD_PROBE_OK;

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
        return GST_PAD_PROBE_OK;

    uint8_t *data = map.data;
    gsize    size = map.size;

    for (gsize i = 0; i + 6 < size; i++) {

        int is_h264_sei = (data[i]   == 0x00 && data[i+1] == 0x00 &&
                           data[i+2] == 0x00 && data[i+3] == 0x01 &&
                           data[i+4] == 0x06 && data[i+5] == 0x05);

        int is_hevc_sei = (i + 7 < size) &&
                          (data[i]   == 0x00 && data[i+1] == 0x00 &&
                           data[i+2] == 0x00 && data[i+3] == 0x01 &&
                           data[i+4] == 0x4E && data[i+5] == 0x01 &&
                           data[i+6] == 0x05);

        if (!is_h264_sei && !is_hevc_sei) continue;

        int payload_offset = is_hevc_sei ? (int)(i + 7) : (int)(i + 6);

        /* Multi-byte RBSP payload size */
        int payload_size = 0;
        while (payload_offset < (int)size && data[payload_offset] == 0xFF) {
            payload_size += 255;
            payload_offset++;
        }
        if (payload_offset >= (int)size) break;
        payload_size += data[payload_offset++];
        if (payload_offset + payload_size > (int)size) break;

        /* Walk EBSP counting RBSP bytes to find exact EBSP span */
        int ebsp_end     = payload_offset;
        int rbsp_decoded = 0;
        int zero_count   = 0;
        while (ebsp_end < (int)size && rbsp_decoded < payload_size) {
            uint8_t byte = data[ebsp_end];
            if (zero_count == 2 && byte == 0x03) {
                ebsp_end++; zero_count = 0; continue;
            }
            zero_count = (byte == 0x00) ? zero_count + 1 : 0;
            rbsp_decoded++; ebsp_end++;
        }

        /* Remove EPB → RBSP = UUID(16) + SeiMetadata */
        uint8_t rbsp[16 + sizeof(SeiMetadata) + 32];
        int decoded_len = remove_emulation_prevention(
                              &data[payload_offset], ebsp_end - payload_offset, rbsp);

        if (decoded_len < 16 + (int)sizeof(SeiMetadata)) break;
        if (memcmp(rbsp, EXPECTED_UUID, 16) != 0)         break;

        SeiMetadata meta;
        memcpy(&meta, rbsp + 16, sizeof(SeiMetadata));
        if (meta.magic != SEI_METADATA_MAGIC) break;

        /* Write to CSV if enabled */
        if (g_csv_file) {
            fprintf(g_csv_file, "%u,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ","
                                "%d,%d,%d,"
                                "%d,%d,%d,"
                                "%d,%d,%d,"
                                "%.16s,%u,%d,%d,%.16s\n",
                    meta.frame_id, meta.frame_pts_ns, meta.imu_timestamp_ns, meta.utc_timestamp_us,
                    meta.accel_x_mg, meta.accel_y_mg, meta.accel_z_mg,
                    meta.gyro_x_mdps, meta.gyro_y_mdps, meta.gyro_z_mdps,
                    meta.mag_x_ut, meta.mag_y_ut, meta.mag_z_ut,
                    meta.device_serial, meta.environment_type, meta.latitude_e7, meta.longitude_e7, meta.session_id);
        }

        /* Push to overlay state */
        pthread_mutex_lock(&g_overlay.lock);
        g_overlay.meta  = meta;
        g_overlay.valid = 1;
        pthread_mutex_unlock(&g_overlay.lock);

        if (g_verbose) {
            g_print("\n── SEI Frame #%u ────────────────────────────\n", meta.frame_id);
            g_print("  PTS      : %lu ns\n",  (unsigned long)meta.frame_pts_ns);
            g_print("  IMU ts   : %lu ns\n",  (unsigned long)meta.imu_timestamp_ns);
            g_print("  Accel    : X:%-5d  Y:%-5d  Z:%-5d  mg\n",
                    meta.accel_x_mg, meta.accel_y_mg, meta.accel_z_mg);
            g_print("  Gyro     : X:%-5d  Y:%-5d  Z:%-5d  mdps\n",
                    meta.gyro_x_mdps, meta.gyro_y_mdps, meta.gyro_z_mdps);
            g_print("  Mag      : X:%-5d  Y:%-5d  Z:%-5d  uT\n",
                    meta.mag_x_ut, meta.mag_y_ut, meta.mag_z_ut);
            g_print("  Device   : %.16s | Session: %.16s\n",
                    meta.device_serial, meta.session_id);
            g_print("─────────────────────────────────────────────\n");
        } else {
            g_print("\rSEI #%u | A(%d,%d,%d)mg G(%d,%d,%d)mdps M(%d,%d,%d)uT     ",
                    meta.frame_id,
                    meta.accel_x_mg, meta.accel_y_mg, meta.accel_z_mg,
                    meta.gyro_x_mdps, meta.gyro_y_mdps, meta.gyro_z_mdps,
                    meta.mag_x_ut, meta.mag_y_ut, meta.mag_z_ut);
            fflush(stdout);
        }
        break; /* One SEI NAL per frame */
    }

    gst_buffer_unmap(buffer, &map);
    return GST_PAD_PROBE_OK;   /* Buffer untouched */
}

/* ─── Cairo HUD helpers ────────────────────────────────────────────────── */

static void draw_axis_bar(cairo_t *cr, double x, double y,
                           double value, double range,
                           const char *label, double bar_w)
{
    const double bar_h = 10.0;
    const double mid   = x + bar_w / 2.0;

    /* Track background */
    cairo_set_source_rgba(cr, 0.15, 0.15, 0.20, 0.70);
    cairo_rectangle(cr, x, y, bar_w, bar_h);
    cairo_fill(cr);

    /* Filled portion — clamp to [-range, range] */
    double norm  = value / range;
    if (norm >  1.0) norm =  1.0;
    if (norm < -1.0) norm = -1.0;

    double bar_x = (norm >= 0) ? mid : mid + norm * (bar_w / 2.0);
    double bar_len = fabs(norm) * (bar_w / 2.0);

    /* Color: green=positive, orange=negative */
    if (norm >= 0)
        cairo_set_source_rgba(cr, 0.20, 0.85, 0.40, 0.85);
    else
        cairo_set_source_rgba(cr, 1.00, 0.55, 0.10, 0.85);
    cairo_rectangle(cr, bar_x, y, bar_len, bar_h);
    cairo_fill(cr);

    /* Centre tick */
    cairo_set_source_rgba(cr, 0.70, 0.70, 0.80, 0.60);
    cairo_set_line_width(cr, 0.8);
    cairo_move_to(cr, mid, y);
    cairo_line_to(cr, mid, y + bar_h);
    cairo_stroke(cr);

    /* Axis label */
    cairo_set_source_rgba(cr, 0.85, 0.85, 0.95, 0.90);
    cairo_set_font_size(cr, 9.0);
    cairo_move_to(cr, x - 12, y + bar_h - 1);
    cairo_show_text(cr, label);
}

static void draw_section_header(cairo_t *cr, double x, double y,
                                  const char *title,
                                  double r, double g, double b)
{
    cairo_set_source_rgba(cr, r, g, b, 0.90);
    cairo_select_font_face(cr, "sans-serif",
                           CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 11.5);
    cairo_move_to(cr, x, y);
    cairo_show_text(cr, title);
    cairo_select_font_face(cr, "sans-serif",
                           CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
}

static void draw_kv(cairo_t *cr, double x, double y,
                     const char *key, const char *val)
{
    cairo_set_font_size(cr, 11.0);
    cairo_set_source_rgba(cr, 0.65, 0.85, 1.0, 0.85);
    cairo_move_to(cr, x, y);
    cairo_show_text(cr, key);
    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.95);
    cairo_move_to(cr, x + 90, y);
    cairo_show_text(cr, val);
}

/* ─── Cairo draw callback (video sink thread) ──────────────────────────── */
static void on_draw(GstElement *overlay, cairo_t *cr,
                    guint64 timestamp, guint64 duration,
                    gpointer user_data)
{
    (void)overlay; (void)timestamp; (void)duration; (void)user_data;

    /* Snapshot — don't hold the lock during rendering */
    pthread_mutex_lock(&g_overlay.lock);
    if (!g_overlay.valid) { pthread_mutex_unlock(&g_overlay.lock); return; }
    SeiMetadata meta = g_overlay.meta;
    pthread_mutex_unlock(&g_overlay.lock);

    const double PX = 16.0, PY = 16.0, PW = 200.0;
    const double BX = PX + 18.0, BW = PW - 20.0;
    const double LH = 17.0, RH = 16.0;
    char buf[64];

    /* Panel background */
    cairo_set_source_rgba(cr, 0.03, 0.05, 0.10, 0.72);
    cairo_rectangle(cr, PX - 8, PY - 8, PW + 16, 342.0);
    cairo_fill(cr);
    /* Accent border */
    cairo_set_source_rgba(cr, 0.30, 0.60, 1.00, 0.55);
    cairo_set_line_width(cr, 1.2);
    cairo_rectangle(cr, PX - 8, PY - 8, PW + 16, 342.0);
    cairo_stroke(cr);

    double cy = PY + 10.0;

    /* ── Header ─── */
    draw_section_header(cr, PX, cy, "● IMU OVERLAY", 0.30, 0.75, 1.00); cy += LH + 4;

    snprintf(buf, sizeof(buf), "#%u", meta.frame_id);
    draw_kv(cr, PX, cy, "Frame",    buf); cy += LH;

    time_t utc_sec = (time_t)(meta.utc_timestamp_us / 1000000ULL);
    strftime(buf, sizeof(buf), "%H:%M:%S UTC", gmtime(&utc_sec));
    draw_kv(cr, PX, cy, "Time",     buf); cy += LH;

    snprintf(buf, sizeof(buf), "%.16s", meta.device_serial);
    draw_kv(cr, PX, cy, "Device",   buf); cy += LH + 4;

    /* ── Accelerometer ─── */
    draw_section_header(cr, PX, cy, "Accelerometer (mg)", 0.25, 1.00, 0.45); cy += LH + 2;
    draw_axis_bar(cr, BX, cy, meta.accel_x_mg, 2000.0, "X", BW); cy += RH;
    draw_axis_bar(cr, BX, cy, meta.accel_y_mg, 2000.0, "Y", BW); cy += RH;
    draw_axis_bar(cr, BX, cy, meta.accel_z_mg, 2000.0, "Z", BW); cy += RH;
    snprintf(buf, sizeof(buf), "%+5d %+5d %+5d",
             meta.accel_x_mg, meta.accel_y_mg, meta.accel_z_mg);
    cairo_set_source_rgba(cr, 0.85, 1.0, 0.85, 0.90);
    cairo_set_font_size(cr, 10.5);
    cairo_move_to(cr, PX, cy); cairo_show_text(cr, buf); cy += LH + 2;

    /* ── Gyroscope ─── */
    draw_section_header(cr, PX, cy, "Gyroscope (mdps)", 1.00, 0.75, 0.15); cy += LH + 2;
    draw_axis_bar(cr, BX, cy, meta.gyro_x_mdps, 500.0, "X", BW); cy += RH;
    draw_axis_bar(cr, BX, cy, meta.gyro_y_mdps, 500.0, "Y", BW); cy += RH;
    draw_axis_bar(cr, BX, cy, meta.gyro_z_mdps, 500.0, "Z", BW); cy += RH;
    snprintf(buf, sizeof(buf), "%+5d %+5d %+5d",
             meta.gyro_x_mdps, meta.gyro_y_mdps, meta.gyro_z_mdps);
    cairo_set_source_rgba(cr, 1.0, 0.90, 0.70, 0.90);
    cairo_set_font_size(cr, 10.5);
    cairo_move_to(cr, PX, cy); cairo_show_text(cr, buf); cy += LH + 2;

    /* ── Magnetometer ─── */
    draw_section_header(cr, PX, cy, "Magnetometer (µT)", 0.85, 0.40, 1.00); cy += LH + 2;
    draw_axis_bar(cr, BX, cy, meta.mag_x_ut, 100.0, "X", BW); cy += RH;
    draw_axis_bar(cr, BX, cy, meta.mag_y_ut, 100.0, "Y", BW); cy += RH;
    draw_axis_bar(cr, BX, cy, meta.mag_z_ut, 100.0, "Z", BW); cy += RH;
    snprintf(buf, sizeof(buf), "%+5d %+5d %+5d",
             meta.mag_x_ut, meta.mag_y_ut, meta.mag_z_ut);
    cairo_set_source_rgba(cr, 0.90, 0.70, 1.0, 0.90);
    cairo_set_font_size(cr, 10.5);
    cairo_move_to(cr, PX, cy); cairo_show_text(cr, buf); cy += LH + 2;

    /* ── Session footer ─── */
    cairo_set_source_rgba(cr, 0.55, 0.75, 0.95, 0.70);
    cairo_set_font_size(cr, 9.5);
    cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    snprintf(buf, sizeof(buf), "session: %.16s", meta.session_id);
    cairo_move_to(cr, PX, cy + 2); cairo_show_text(cr, buf);
}

/* ─── main ─────────────────────────────────────────────────────────────── */
int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);

    const char *ip      = "10.0.0.1";
    const char *port    = "8554";
    const char *mount   = "/stream";
    const char *rec_dir = ".";        /* default: current working directory */
    const char *csvfile = NULL;

    for (int i = 1; i < argc; i++) {
        if      (strcmp(argv[i], "--verbose") == 0 || strcmp(argv[i], "-v") == 0) g_verbose = 1;
        else if (strcmp(argv[i], "--ip")      == 0 && i+1 < argc) ip      = argv[++i];
        else if (strcmp(argv[i], "--port")    == 0 && i+1 < argc) port    = argv[++i];
        else if (strcmp(argv[i], "--mount")   == 0 && i+1 < argc) mount   = argv[++i];
        else if (strcmp(argv[i], "--rec-dir") == 0 && i+1 < argc) rec_dir = argv[++i];
        else if (strcmp(argv[i], "--csv")     == 0 && i+1 < argc) csvfile = argv[++i];
    }

    /* ── Build timestamped recording path ─────────────────────────────── */
    char rec_path[512];
    {
        time_t now = time(NULL);
        struct tm *t = localtime(&now);
        snprintf(rec_path, sizeof(rec_path),
                 "%s/client_overlay_%04d%02d%02d_%02d%02d%02d.mp4",
                 rec_dir,
                 t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                 t->tm_hour, t->tm_min, t->tm_sec);
    }

    char rtsp_url[256];
    snprintf(rtsp_url, sizeof(rtsp_url), "rtsp://%s:%s%s", ip, port, mount);

    g_print("SEI Client — Cairo HUD Display + Overlay Recording (v%d)\n",
            SEI_METADATA_VERSION);
    g_print("  Stream    : %s\n",  rtsp_url);
    g_print("  Recording : %s\n",  rec_path);
    g_print("  Verbose   : %s\n",  g_verbose ? "yes" : "no (-v for detail)");
    if (csvfile) {
        g_print("  CSV Out   : %s\n", csvfile);
        g_csv_file = fopen(csvfile, "w");
        if (!g_csv_file) {
            g_printerr("Failed to open CSV file for writing: %s\n", csvfile);
            return -1;
        }
        fprintf(g_csv_file, "frame_id,frame_pts_ns,imu_timestamp_ns,utc_timestamp_us,"
                            "accel_x_mg,accel_y_mg,accel_z_mg,"
                            "gyro_x_mdps,gyro_y_mdps,gyro_z_mdps,"
                            "mag_x_ut,mag_y_ut,mag_z_ut,"
                            "device_serial,environment_type,latitude_e7,longitude_e7,session_id\n");
    }
    g_print("  SEI size  : UUID(16) + %zu bytes = %zu bytes RBSP\n\n",
            sizeof(SeiMetadata), 16 + sizeof(SeiMetadata));

    /*
     * ── Pipeline ──────────────────────────────────────────────────────────
     *
     *   rtspsrc → rtph264depay
     *           → h264parse (disp_parser)     ← SEI probe on SRC pad (byte-stream)
     *           → video/x-h264,byte-stream
     *           → avdec_h264 → videoconvert
     *           → cairooverlay (hud_overlay)  ← HUD drawn here (decoded RGB/YUV)
     *           → tee name=t
     *
     *   DISPLAY:  t. → queue → videoconvert → autovideosink
     *   RECORD:   t. → queue → videoconvert
     *                        → x264enc tune=zerolatency bitrate=8000
     *                        → video/x-h264,stream-format=avc,alignment=au
     *                        → qtmux fragment-duration=1000
     *                        → filesink location=<rec_path>
     *
     * Note on videoconvert before x264enc:
     *   cairooverlay outputs BGRA (or whatever autovideosink negotiated).
     *   x264enc requires I420/NV12. The videoconvert between the tee and
     *   x264enc handles the pixel format conversion transparently.
     */
    char pipeline_str[2048];
    snprintf(pipeline_str, sizeof(pipeline_str),

        /* ── Source + decode + HUD ───────────────────────────────────── */
        "rtspsrc location=%s latency=100 ! "
        "rtph264depay ! "
        "h264parse name=disp_parser ! "
        "video/x-h264,stream-format=byte-stream ! "
        "avdec_h264 ! "
        "videoconvert ! "
        "cairooverlay name=hud_overlay ! "     /* HUD applied once here */

        /* ── Fan-out to display AND recorder ─────────────────────────── */
        "tee name=t "

        /* DISPLAY branch */
        "t. ! queue max-size-buffers=4 leaky=downstream ! "
        "videoconvert ! "
        "autovideosink sync=false "

        /* RECORD branch — re-encode post-overlay frames to MP4 */
        "t. ! queue max-size-buffers=4 leaky=downstream ! "
        "videoconvert ! "                      /* BGRA → I420 for x264enc */
        "x264enc tune=zerolatency bitrate=8000 key-int-max=30 ! "
        "video/x-h264,stream-format=avc,alignment=au ! "
        "qtmux fragment-duration=1000 ! "
        "filesink location=%s sync=false",

        rtsp_url, rec_path);

    GError *error = NULL;
    GstElement *pipeline = gst_parse_launch(pipeline_str, &error);
    if (error || !pipeline) {
        g_printerr("Pipeline parse error: %s\n", error ? error->message : "(null)");
        return -1;
    }

    /* ── SEI probe: disp_parser SRC pad ──────────────────────────────── *
     *
     * The probe MUST be on the SRC pad, not the sink pad.
     * Upstream may deliver AVC format (length-prefixed NALs),
     * but our extraction code searches for Annex-B start codes
     * (00 00 00 01 06 05).  h264parse converts to byte-stream
     * (Annex-B) on its src pad, which is what the code expects.
     */
    GstElement *disp_parser = gst_bin_get_by_name(GST_BIN(pipeline), "disp_parser");
    if (disp_parser) {
        GstPad *src_pad = gst_element_get_static_pad(disp_parser, "src");
        if (src_pad) {
            gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                              sei_extraction_probe, NULL, NULL);
            gst_object_unref(src_pad);
            g_print("System Link: SEI probe → disp_parser src pad (byte-stream).\n");
        }
        gst_object_unref(disp_parser);
    } else {
        g_printerr("FATAL: disp_parser element not found in pipeline.\n");
        return -1;
    }

    /* ── Cairo draw signal ───────────────────────────────────────────── */
    GstElement *hud = gst_bin_get_by_name(GST_BIN(pipeline), "hud_overlay");
    if (hud) {
        g_signal_connect(hud, "draw", G_CALLBACK(on_draw), NULL);
        gst_object_unref(hud);
        g_print("System Link: Cairo HUD → hud_overlay draw signal.\n");
    } else {
        g_printerr("FATAL: hud_overlay not found.\n"
                   "  Install: sudo apt-get install gstreamer1.0-plugins-good\n");
        return -1;
    }

    g_print("\nPipeline running. Press Ctrl-C to stop.\n");
    g_print("Recording overlayed stream to: %s\n\n", rec_path);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
                          GST_MESSAGE_ERROR | GST_MESSAGE_EOS);
    if (msg) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError *err = NULL; gchar *dbg = NULL;
            gst_message_parse_error(msg, &err, &dbg);
            g_printerr("\nError: %s\nDebug: %s\n", err->message, dbg ? dbg : "none");
            g_error_free(err); g_free(dbg);
        } else {
            g_print("\nEnd of stream.\n");
        }
        gst_message_unref(msg);
    }

    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    if (g_csv_file) {
        fclose(g_csv_file);
        g_print("CSV saved to: %s\n", csvfile);
    }

    g_print("Extractor exiting. Recording saved to: %s\n", rec_path);
    return 0;
}
