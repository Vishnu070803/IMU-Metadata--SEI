/*
 * sei_extractor.c  — SEI Metadata MP4 Extractor
 *
 * Reads an MP4 file with embedded SEI IMU metadata and extracts it.
 * Can write the extracted metadata to a CSV file at maximum speed,
 * or optionally play back the video with the Cairo HUD overlay.
 *
 * Build:
 *   gcc sei_extractor.c -o sei_extractor \
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

/* ─── Global State ──────────────────────────────────────────────────────── */
static int g_verbose = 0;
static FILE *g_csv_file = NULL;

static const uint8_t EXPECTED_UUID[16] = {
    0xdc, 0x45, 0xe9, 0xbd, 0xe6, 0xd9, 0x48, 0xb7,
    0x96, 0x2c, 0xd3, 0x9c, 0x24, 0x5a, 0xc8, 0x1a
};

/* ─── Shared state (for Playback mode) ──────────────────────────────────── */
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

/* ─── SEI extraction probe ─────────────────────────────────────────────── */
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
    return GST_PAD_PROBE_OK;
}

/* ─── Cairo HUD helpers ────────────────────────────────────────────────── */

static void draw_axis_bar(cairo_t *cr, double x, double y,
                           double value, double range,
                           const char *label, double bar_w)
{
    const double bar_h = 10.0;
    const double mid   = x + bar_w / 2.0;

    /* Track */
    cairo_set_source_rgba(cr, 0.15, 0.15, 0.15, 0.75);
    cairo_rectangle(cr, x, y, bar_w, bar_h);
    cairo_fill(cr);

    /* Centre line */
    cairo_set_source_rgba(cr, 0.5, 0.5, 0.5, 0.8);
    cairo_rectangle(cr, mid - 0.5, y, 1.0, bar_h);
    cairo_fill(cr);

    /* Value fill */
    double clamped = value < -range ? -range : (value > range ? range : value);
    double fill_w  = fabs(clamped) / range * (bar_w / 2.0);
    if (clamped >= 0.0) {
        cairo_set_source_rgba(cr, 0.20, 0.85, 0.40, 0.90);
        cairo_rectangle(cr, mid, y + 1, fill_w, bar_h - 2);
    } else {
        cairo_set_source_rgba(cr, 1.00, 0.55, 0.10, 0.90);
        cairo_rectangle(cr, mid - fill_w, y + 1, fill_w, bar_h - 2);
    }
    cairo_fill(cr);

    /* Axis label */
    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.95);
    cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 10.0);
    cairo_move_to(cr, x - 14, y + bar_h - 1);
    cairo_show_text(cr, label);
}

static void draw_section_header(cairo_t *cr, double x, double y,
                                 const char *title,
                                 double r, double g, double b)
{
    cairo_select_font_face(cr, "sans-serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 13.0);
    cairo_set_source_rgba(cr, r, g, b, 1.0);
    cairo_move_to(cr, x, y);
    cairo_show_text(cr, title);
    cairo_set_source_rgba(cr, r, g, b, 0.4);
    cairo_rectangle(cr, x, y + 3, 140, 1.5);
    cairo_fill(cr);
}

static void draw_kv(cairo_t *cr, double x, double y,
                    const char *key, const char *val)
{
    cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
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

    const char *infile  = NULL;
    const char *csvfile = NULL;
    int play_mode = 0;

    for (int i = 1; i < argc; i++) {
        if      (strcmp(argv[i], "--verbose") == 0 || strcmp(argv[i], "-v") == 0) g_verbose = 1;
        else if (strcmp(argv[i], "--play") == 0) play_mode = 1;
        else if (strcmp(argv[i], "--in")   == 0 && i+1 < argc) infile  = argv[++i];
        else if (strcmp(argv[i], "--csv")  == 0 && i+1 < argc) csvfile = argv[++i];
    }

    if (!infile) {
        g_printerr("Usage: %s --in <video.mp4> [--csv <out.csv>] [--play] [-v]\n", argv[0]);
        return -1;
    }

    g_print("SEI MP4 Extractor (v%d)\n", SEI_METADATA_VERSION);
    g_print("  Input     : %s\n", infile);
    
    if (csvfile) {
        g_print("  CSV Out   : %s\n", csvfile);
        g_csv_file = fopen(csvfile, "w");
        if (!g_csv_file) {
            g_printerr("Failed to open CSV file for writing: %s\n", csvfile);
            return -1;
        }
        /* Write CSV Header */
        fprintf(g_csv_file, "frame_id,frame_pts_ns,imu_timestamp_ns,utc_timestamp_us,"
                            "accel_x_mg,accel_y_mg,accel_z_mg,"
                            "gyro_x_mdps,gyro_y_mdps,gyro_z_mdps,"
                            "mag_x_ut,mag_y_ut,mag_z_ut,"
                            "device_serial,environment_type,latitude_e7,longitude_e7,session_id\n");
    }

    char pipeline_str[1024];

    if (play_mode) {
        g_print("  Mode      : Playback (Cairo HUD)\n\n");
        snprintf(pipeline_str, sizeof(pipeline_str),
            "filesrc location=%s ! qtdemux ! "
            "h264parse name=disp_parser ! "
            "video/x-h264,stream-format=byte-stream ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "cairooverlay name=hud_overlay ! "
            "videoconvert ! "
            "autovideosink sync=true",
            infile);
    } else {
        g_print("  Mode      : Data Extraction (Max Speed)\n\n");
        snprintf(pipeline_str, sizeof(pipeline_str),
            "filesrc location=%s ! qtdemux ! "
            "h264parse name=disp_parser ! "
            "video/x-h264,stream-format=byte-stream ! "
            "fakesink sync=false",
            infile);
    }

    GError *error = NULL;
    GstElement *pipeline = gst_parse_launch(pipeline_str, &error);
    if (error || !pipeline) {
        g_printerr("Pipeline parse error: %s\n", error ? error->message : "(null)");
        if (g_csv_file) fclose(g_csv_file);
        return -1;
    }

    /* ── SEI probe: disp_parser SRC pad ──────────────── *
     *
     * The probe MUST be on the SRC pad, not the sink pad.
     * qtdemux outputs AVC format (4-byte length-prefixed NALs)
     * to the sink pad, but our extraction code searches for
     * Annex-B start codes (00 00 00 01 06 05).  h264parse
     * converts to byte-stream (Annex-B) on its src pad,
     * which is what the extraction code expects.
     */
    GstElement *disp_parser = gst_bin_get_by_name(GST_BIN(pipeline), "disp_parser");
    if (disp_parser) {
        GstPad *src_pad = gst_element_get_static_pad(disp_parser, "src");
        if (src_pad) {
            gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                              sei_extraction_probe, NULL, NULL);
            gst_object_unref(src_pad);
        }
        gst_object_unref(disp_parser);
    } else {
        g_printerr("FATAL: disp_parser element not found in pipeline.\n");
        if (g_csv_file) fclose(g_csv_file);
        return -1;
    }

    if (play_mode) {
        GstElement *hud = gst_bin_get_by_name(GST_BIN(pipeline), "hud_overlay");
        if (hud) {
            g_signal_connect(hud, "draw", G_CALLBACK(on_draw), NULL);
            gst_object_unref(hud);
        } else {
            g_printerr("WARNING: hud_overlay not found. Is Cairo installed?\n");
        }
    }

    g_print("Pipeline running. %s\n", play_mode ? "Press Ctrl-C to stop." : "Extracting metadata...");
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
            g_print("\nExtraction complete (End of stream).\n");
        }
        gst_message_unref(msg);
    }

    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    if (g_csv_file) {
        fclose(g_csv_file);
        g_print("Saved to %s\n", csvfile);
    }

    return 0;
}
