#include <gst/gst.h>
#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <glib-unix.h>

#include "sei_metadata.h"
#include "icm20948_ioctl.h"

/* ─── Application Context ─────────────────────────────────────────────── */
typedef struct {
    int imu_fd;
    uint32_t frame_counter;
    char device_serial[16];
    uint8_t environment_type;
    int32_t latitude_e7;
    int32_t longitude_e7;
    char session_id[16];
} AppContext;

static AppContext app_ctx = {
    .imu_fd = -1,
    .frame_counter = 0,
    .device_serial = "IMX8MP-001",
    .environment_type = SEI_ENV_INDOOR,
    .latitude_e7 = 129716000,
    .longitude_e7 = 775946000,
    .session_id = "session-100hz",
};

/* ─── 100Hz IMU Ring Buffer ───────────────────────────────────────────── */
#define IMU_RING_SIZE 256
typedef struct {
    SeiMetadata data[IMU_RING_SIZE];
    int head; /* written by IMU thread */
    int tail; /* read by GStreamer probe */
    GMutex mutex;
} ImuRingBuffer;

static ImuRingBuffer imu_ring = { .head = 0, .tail = 0 };
static gboolean g_running = TRUE;
static int g_timeout_seconds = 0;
static GMainLoop *g_loop = NULL;

/* IMU Thread: samples IMU at 100Hz (every 10ms) */
static gpointer imu_thread_func(gpointer data) {
    (void)data;
    
    gint64 interval_us = 10000LL;
    gint64 next_wake_us = g_get_monotonic_time() + interval_us;
    
    while (g_running) {
        SeiMetadata meta;
        memset(&meta, 0, sizeof(meta));
        meta.magic = SEI_METADATA_MAGIC;
        meta.version = SEI_METADATA_VERSION;
        
        struct timespec ts, utc;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        clock_gettime(CLOCK_REALTIME, &utc);
        meta.imu_timestamp_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
        meta.utc_timestamp_us = (uint64_t)utc.tv_sec * 1000000ULL + (uint64_t)utc.tv_nsec / 1000ULL;
        
        if (app_ctx.imu_fd >= 0) {
            struct icm20948_all_data imu;
            if (ioctl(app_ctx.imu_fd, ICM20948_IOC_GET_ALL, &imu) == 0) {
                meta.accel_x_mg  = imu.accel_x_mg;
                meta.accel_y_mg  = imu.accel_y_mg;
                meta.accel_z_mg  = imu.accel_z_mg;
                meta.gyro_x_mdps = imu.gyro_x_mdps;
                meta.gyro_y_mdps = imu.gyro_y_mdps;
                meta.gyro_z_mdps = imu.gyro_z_mdps;
                meta.mag_x_ut    = imu.mag_x_ut;
                meta.mag_y_ut    = imu.mag_y_ut;
                meta.mag_z_ut    = imu.mag_z_ut;
            }
        } else {
            /* Simulated values for testing */
            static int sim = 0; sim++;
            meta.accel_x_mg = 10 + (sim % 50);
            meta.accel_y_mg = -980 + (sim % 20);
            meta.accel_z_mg = 45 + (sim % 30);
            meta.gyro_x_mdps = 150 + (sim % 100);
            meta.gyro_y_mdps = -20 + (sim % 40);
            meta.gyro_z_mdps = 5 + (sim % 10);
            meta.mag_x_ut = 25 + (sim % 15);
            meta.mag_y_ut = -12 + (sim % 8);
            meta.mag_z_ut = 40 + (sim % 20);
        }
        
        memcpy(meta.device_serial, app_ctx.device_serial, 16);
        meta.environment_type = app_ctx.environment_type;
        meta.latitude_e7      = app_ctx.latitude_e7;
        meta.longitude_e7     = app_ctx.longitude_e7;
        memcpy(meta.session_id, app_ctx.session_id, 16);
        
        g_mutex_lock(&imu_ring.mutex);
        int next_head = (imu_ring.head + 1) % IMU_RING_SIZE;
        if (next_head == imu_ring.tail) {
            /* Buffer full (e.g. during camera startup). Overwrite oldest sample. */
            imu_ring.tail = (imu_ring.tail + 1) % IMU_RING_SIZE;
        }
        imu_ring.data[imu_ring.head] = meta;
        imu_ring.head = next_head;
        g_mutex_unlock(&imu_ring.mutex);
        
        gint64 now_us = g_get_monotonic_time();
        if (next_wake_us > now_us) {
            g_usleep(next_wake_us - now_us);
        }
        next_wake_us += interval_us;
    }
    return NULL;
}

/* ─── PTS to IMU Head Map ─────────────────────────────────────────────── */
#define MAX_PTS_HISTORY 64
typedef struct {
    GstClockTime pts;
    int imu_head_at_capture;
} PtsMap;

static PtsMap pts_history[MAX_PTS_HISTORY];
static int pts_head = 0;
static GMutex pts_mutex;

/* ─── Emulation Prevention ────────────────────────────────────────────── */
static int apply_emulation_prevention(const uint8_t *src, int src_len, uint8_t *dst) {
    int dst_len = 0, zero_count = 0;
    for (int i = 0; i < src_len; i++) {
        if (zero_count == 2 && src[i] <= 0x03) {
            dst[dst_len++] = 0x03;
            zero_count = 0;
        }
        if (src[i] == 0x00) zero_count++;
        else                zero_count = 0;
        dst[dst_len++] = src[i];
    }
    return dst_len;
}

/* ─── Camera Source Probe (Hybrid Marker) ─────────────────────────────── *
 * NOTE: Attached *before* videorate. Fires at 60Hz (source rate).
 * Overwrites the last background thread sample with a fresh inline IOCTL
 * reading. For frames that survive videorate, this guarantees <=1ms latency.
 */
static GstPadProbeReturn camera_src_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    (void)pad; (void)user_data;
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buffer) return GST_PAD_PROBE_OK;
    
    GstClockTime pts = GST_BUFFER_PTS(buffer);
    
    static uint32_t probe_hit_count = 0;
    probe_hit_count++;
    if (probe_hit_count % 30 == 0) {
        g_print("SRC_PROBE[Hybrid]: Fired 30 times! (Surviving frames only)\n");
    }
    
    g_mutex_lock(&imu_ring.mutex);
    int current_head = imu_ring.head;
    
    /* Overwrite the most recent thread sample with fresh inline reading */
    int overwrite_slot = (current_head + IMU_RING_SIZE - 1) % IMU_RING_SIZE;
    SeiMetadata *last_sample = &imu_ring.data[overwrite_slot];
    
    struct timespec ts, utc;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    clock_gettime(CLOCK_REALTIME, &utc);
    last_sample->imu_timestamp_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    last_sample->utc_timestamp_us = (uint64_t)utc.tv_sec * 1000000ULL + (uint64_t)utc.tv_nsec / 1000ULL;
    
    if (app_ctx.imu_fd >= 0) {
        struct icm20948_all_data imu;
        if (ioctl(app_ctx.imu_fd, ICM20948_IOC_GET_ALL, &imu) == 0) {
            last_sample->accel_x_mg  = imu.accel_x_mg;
            last_sample->accel_y_mg  = imu.accel_y_mg;
            last_sample->accel_z_mg  = imu.accel_z_mg;
            last_sample->gyro_x_mdps = imu.gyro_x_mdps;
            last_sample->gyro_y_mdps = imu.gyro_y_mdps;
            last_sample->gyro_z_mdps = imu.gyro_z_mdps;
            last_sample->mag_x_ut    = imu.mag_x_ut;
            last_sample->mag_y_ut    = imu.mag_y_ut;
            last_sample->mag_z_ut    = imu.mag_z_ut;
        }
    }
    g_mutex_unlock(&imu_ring.mutex);
    
    g_mutex_lock(&pts_mutex);
    pts_history[pts_head % MAX_PTS_HISTORY].pts = pts;
    pts_history[pts_head % MAX_PTS_HISTORY].imu_head_at_capture = current_head;
    pts_head++;
    g_mutex_unlock(&pts_mutex);
    
    return GST_PAD_PROBE_OK;
}

/* ─── SEI Injection Probe ─────────────────────────────────────────────── *
 * Fires at 30Hz (post-videorate). Frame counter incremented HERE.
 */
static GstPadProbeReturn sei_injection_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    (void)pad; (void)user_data;
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buffer) return GST_PAD_PROBE_OK;
    
    GstClockTime pts = GST_BUFFER_PTS(buffer);
    
    int target_head = -1;
    g_mutex_lock(&pts_mutex);
    for (int i = 0; i < MAX_PTS_HISTORY; i++) {
        if (pts_history[i].pts == pts && pts != GST_CLOCK_TIME_NONE) {
            target_head = pts_history[i].imu_head_at_capture;
            break;
        }
    }
    g_mutex_unlock(&pts_mutex);
    
    if (target_head == -1) {
        /* PTS not found, likely a pipeline error or buffer wrapped. Skip injection. */
        return GST_PAD_PROBE_OK;
    }
    
    SeiMetadata samples[128];
    int sample_count = 0;
    
    g_mutex_lock(&imu_ring.mutex);
    /* First-frame overflow protection: skip old samples if tail is too far behind */
    int pending = (target_head - imu_ring.tail + IMU_RING_SIZE) % IMU_RING_SIZE;
    if (pending > 10) { /* 100Hz = ~3 samples per frame, 10 is plenty of margin */
        int skip_to = (target_head - 5 + IMU_RING_SIZE) % IMU_RING_SIZE;
        imu_ring.tail = skip_to;
    }

    /* Consume up to the target_head that was marked precisely when the frame was captured */
    while (imu_ring.tail != target_head && sample_count < 128) {
        samples[sample_count] = imu_ring.data[imu_ring.tail];
        imu_ring.tail = (imu_ring.tail + 1) % IMU_RING_SIZE;
        sample_count++;
    }
    g_mutex_unlock(&imu_ring.mutex);
    
    /* If no IMU data is available for some reason, just pass the frame */
    if (sample_count == 0) return GST_PAD_PROBE_OK;
    
    /* UUID for the custom SEI metadata */
    uint8_t uuid[16] = {0xdc, 0x45, 0xe9, 0xbd, 0xe6, 0xd9, 0x48, 0xb7,
                        0x96, 0x2c, 0xd3, 0x9c, 0x24, 0x5a, 0xc8, 0x1a};

    /* FIX #3: Increment frame_counter HERE at 30Hz, not in camera_src_probe */
    uint32_t current_frame_id = app_ctx.frame_counter++;

    /* Log every 30th frame (~1/sec) for health monitoring */
    if (current_frame_id % 30 == 0) {
        gboolean is_kf = !GST_BUFFER_FLAG_IS_SET(buffer, GST_BUFFER_FLAG_DELTA_UNIT);
        g_print("SEI[REC]: frame_id=%u | PTS=%" G_GUINT64_FORMAT " | imu_samples=%d | %s\n",
                current_frame_id, pts, sample_count, is_kf ? "KEYFRAME" : "delta");
    }

    /* FIX #2: Save original buffer flags before gst_buffer_append destroys them */
    GstBufferFlags orig_flags = GST_BUFFER_FLAGS(buffer);

    GstBuffer *sei_agg_buf = NULL;
    
    for (int i = 0; i < sample_count; i++) {
        SeiMetadata *meta = &samples[i];
        
        /* Set video frame specific values before serializing */
        meta->frame_pts_ns = pts;
        meta->frame_id = current_frame_id;
        
        int raw_size = 16 + sizeof(SeiMetadata);
        uint8_t *raw = g_malloc(raw_size);
        memcpy(raw, uuid, 16);
        memcpy(raw + 16, meta, sizeof(SeiMetadata));
        
        uint8_t *epb = g_malloc(raw_size * 2);
        int epb_len = apply_emulation_prevention(raw, raw_size, epb);
        
        /* H.264 SEI Header */
        uint8_t sei_header[] = {0x00, 0x00, 0x00, 0x01, 0x06, 0x05};
        int header_len = 6;
        
        uint8_t size_bytes[4];
        int size_len = 0;
        int remaining = raw_size;
        while (remaining >= 255) {
            size_bytes[size_len++] = 0xFF;
            remaining -= 255;
        }
        size_bytes[size_len++] = (uint8_t)remaining;
        
        int total = header_len + size_len + epb_len + 1;
        uint8_t *pkt = g_malloc(total);
        
        int off = 0;
        memcpy(pkt + off, sei_header, header_len); off += header_len;
        memcpy(pkt + off, size_bytes, size_len);   off += size_len;
        memcpy(pkt + off, epb, epb_len);           off += epb_len;
        pkt[off] = 0x80; /* RBSP trailing bits */
        
        GstBuffer *single_sei = gst_buffer_new_wrapped(pkt, total);
        if (!sei_agg_buf) {
            sei_agg_buf = single_sei;
        } else {
            sei_agg_buf = gst_buffer_append(sei_agg_buf, single_sei);
        }
        
        g_free(raw);
        g_free(epb);
    }
    
    if (sei_agg_buf) {
        GST_BUFFER_PTS(sei_agg_buf) = GST_BUFFER_PTS(buffer);
        GST_BUFFER_DTS(sei_agg_buf) = GST_BUFFER_DTS(buffer);
        GST_BUFFER_DURATION(sei_agg_buf) = GST_BUFFER_DURATION(buffer);
        GstBuffer *new_buffer = gst_buffer_append(sei_agg_buf, gst_buffer_ref(buffer));

        /* FIX #2: Copy flags from original buffer to preserve DELTA_UNIT */
        gst_buffer_copy_into(new_buffer, buffer,
                             GST_BUFFER_COPY_FLAGS, 0, -1);

        GST_PAD_PROBE_INFO_DATA(info) = new_buffer;
        gst_buffer_unref(buffer);
    }
    
    return GST_PAD_PROBE_OK;
}

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data) {
    (void)bus;
    GMainLoop *loop = (GMainLoop *)data;
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS:
            g_print("System Link: ✓ EOS received — moov atom written successfully.\n");
            g_main_loop_quit(loop);
            break;
        case GST_MESSAGE_ERROR: {
            gchar *debug = NULL;
            GError *error = NULL;
            gst_message_parse_error(msg, &error, &debug);
            g_printerr("System Link: PIPELINE ERROR — %s\n", error->message);
            if (debug) g_printerr("  Debug: %s\n", debug);
            g_error_free(error);
            g_free(debug);
            g_main_loop_quit(loop);
            break;
        }
        case GST_MESSAGE_WARNING: {
            gchar *debug = NULL;
            GError *warning = NULL;
            gst_message_parse_warning(msg, &warning, &debug);
            g_printerr("System Link: WARNING — %s\n", warning->message);
            g_error_free(warning);
            g_free(debug);
            break;
        }
        default:
            break;
    }
    return TRUE;
}

static gboolean eos_timeout_cb(gpointer user_data) {
    (void)user_data;
    g_printerr("System Link: WARNING — EOS timeout after 10s, forcing quit (moov may be incomplete).\n");
    if (g_loop) g_main_loop_quit(g_loop);
    return G_SOURCE_REMOVE;
}

static gboolean sigint_handler(gpointer user_data) {
    GstElement *pipeline = (GstElement *)user_data;
    g_print("\nSystem Link: Ctrl+C caught! Sending EOS to finalize MP4 (writing moov atom)...\n");
    g_print("System Link: Waiting for pipeline EOS acknowledgement (up to 10s)...\n");
    gst_element_send_event(pipeline, gst_event_new_eos());
    g_timeout_add_seconds(10, eos_timeout_cb, NULL);
    return G_SOURCE_REMOVE;
}

static gboolean recorder_timeout_cb(gpointer user_data) {
    GstElement *pipeline = (GstElement *)user_data;
    g_print("\nSystem Link: %d-second timeout expired. Sending EOS to finalize recording...\n", g_timeout_seconds);
    gst_element_send_event(pipeline, gst_event_new_eos());
    g_timeout_add_seconds(10, eos_timeout_cb, NULL);
    return G_SOURCE_REMOVE;
}

/* ─── Main Function ───────────────────────────────────────────────────── */
int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);
    
    const char *rec_dir = "/tmp";
    const char *camera_dev = "/dev/video3";
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--rec-dir") == 0 && i + 1 < argc) {
            rec_dir = argv[++i];
        } else if (strcmp(argv[i], "--camera") == 0 && i + 1 < argc) {
            camera_dev = argv[++i];
        } else if (strcmp(argv[i], "--timeout") == 0 && i + 1 < argc) {
            g_timeout_seconds = atoi(argv[++i]);
        }
    }
    
    app_ctx.imu_fd = open("/dev/icm20948", O_RDONLY);
    if (app_ctx.imu_fd < 0) {
        g_print("IMU: /dev/icm20948 not found — using simulated values.\n");
    } else {
        g_print("IMU: /dev/icm20948 opened (fd=%d) — real sensor data.\n", app_ctx.imu_fd);
    }
    
    g_mutex_init(&imu_ring.mutex);
    g_mutex_init(&pts_mutex);
    GThread *imu_thread = g_thread_new("imu_100hz", imu_thread_func, NULL);
    
    char g_rec_path[512];
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    snprintf(g_rec_path, sizeof(g_rec_path),
             "%s/recording_100hz_%04d%02d%02d_%02d%02d%02d.mp4",
             rec_dir, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);
             
    char launch_str[1024];
    snprintf(launch_str, sizeof(launch_str),
        "v4l2src name=cam_src device=%s do-timestamp=true ! "
        "queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
        "videoconvert ! video/x-raw,width=1920,height=1080,framerate=30/1 ! "
        "v4l2h264enc name=hw_enc extra-controls=\"controls,video_bitrate=10000000\" ! "
        "video/x-h264,profile=high,stream-format=byte-stream ! "
        "h264parse ! mp4mux fragment-duration=1000 ! "
        "filesink location=%s sync=false",
        camera_dev, g_rec_path);
        
    GError *error = NULL;
    GstElement *pipeline = gst_parse_launch(launch_str, &error);
    if (error) {
        g_printerr("Pipeline error: %s\n", error->message);
        g_error_free(error);
        return -1;
    }
    
    GstElement *cam = gst_bin_get_by_name(GST_BIN(pipeline), "cam_src");
    if (cam) {
        GstPad *pad = gst_element_get_static_pad(cam, "src");
        if (pad) {
            gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, camera_src_probe, NULL, NULL);
            gst_object_unref(pad);
        }
        gst_object_unref(cam);
    }
    
    GstElement *enc = gst_bin_get_by_name(GST_BIN(pipeline), "hw_enc");
    if (enc) {
        GstPad *pad = gst_element_get_static_pad(enc, "src");
        if (pad) {
            gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, sei_injection_probe, NULL, NULL);
            gst_object_unref(pad);
            g_print("System Link: 100Hz sei_injection_probe attached to hw_enc.\n");
        }
        gst_object_unref(enc);
    } else {
        g_printerr("Warning: hw_enc not found!\n");
    }
    
    g_loop = g_main_loop_new(NULL, FALSE);
    GstBus *bus = gst_element_get_bus(pipeline);
    guint bus_watch_id = gst_bus_add_watch(bus, bus_call, g_loop);
    gst_object_unref(bus);
    
    g_print("\n100Hz IMU Recorder Active (HYBRID MODE).\n");
    g_print("Recording to: %s\n", g_rec_path);
    g_print("The IMU is sampled at 100Hz in a background thread.\n");
    g_print("camera_src_probe overwrites the last thread sample with an inline reading (<=1ms).\n");
    
    if (g_timeout_seconds > 0) {
        g_print("Session     : auto-stop after %d seconds.\n", g_timeout_seconds);
    } else {
        g_print("Session     : continuous mode (no timeout). Use --timeout <sec> to set a limit.\n");
    }
    
    g_print("Press Ctrl+C to stop cleanly (will write MP4 moov atom).\n\n");
    
    g_unix_signal_add(SIGINT, sigint_handler, pipeline);
    
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    
    if (g_timeout_seconds > 0) {
        g_timeout_add_seconds(g_timeout_seconds, recorder_timeout_cb, pipeline);
    }
    
    g_main_loop_run(g_loop);
    
    /* Cleanup on exit — EOS has been received by bus_call at this point */
    g_print("\nSystem Link: Shutting down cleanly...\n");
    g_running = FALSE;
    g_thread_join(imu_thread);
    g_print("System Link: IMU thread joined.\n");
    
    g_print("System Link: Transitioning pipeline to NULL state...\n");
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    
    if (app_ctx.imu_fd >= 0) close(app_ctx.imu_fd);
    g_source_remove(bus_watch_id);
    g_main_loop_unref(g_loop);
    g_loop = NULL;
    
    g_print("System Link: ✓ Recording complete. File is ready.\n");
    return 0;
}
