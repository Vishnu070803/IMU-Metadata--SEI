#ifndef UVC_SEI_INJECTION_H
#define UVC_SEI_INJECTION_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <gst/gst.h>

#include "sei_metadata.h"
#include "icm20948_ioctl.h"

/* ─── Configurable IMU Hz (Set from main args) ─────────────────────────── */
extern int g_imu_hz; 

/* ─── Application context (no threads, no mutex) ───────────────────────── */
typedef struct {
    int imu_fd;                    /* /dev/icm20948 fd, or -1 if unavailable */
    uint32_t frame_counter;        /* Monotonic frame ID */
    /* Pre-filled static metadata */
    char     device_serial[16];
    uint8_t  environment_type;
    int32_t  latitude_e7;
    int32_t  longitude_e7;
    char     session_id[16];
} AppContext;

static AppContext app_ctx = {
    .imu_fd = -1,
    .frame_counter = 0,
    .device_serial = "IMX8MP-001",
    .environment_type = SEI_ENV_INDOOR,
    .latitude_e7 = 129716000,       /* 12.9716° N */
    .longitude_e7 = 775946000,      /* 77.5946° E */
    .session_id = "session-0001",
};

/* ─── Circular Buffers for High-Hz IMU Support ─────────────────────────── */
#define IMU_RING_SIZE 1024
static SeiMetadata imu_ring[IMU_RING_SIZE];
static uint32_t imu_head = 0;
static GMutex imu_mutex;
static gboolean imu_thread_running = FALSE;

#define MAX_PTS_HISTORY 64
typedef struct {
    GstClockTime pts;
    uint32_t start_idx;
    uint32_t count;
    uint32_t frame_id;   /* Assigned once by camera_src, shared by both tee branches */
} PtsToImuMap;

static PtsToImuMap pts_map[MAX_PTS_HISTORY];
static uint32_t pts_head = 0;
static GMutex pts_mutex;

/* ─── Per-branch probe context ─────────────────────────────────────────── */
typedef struct {
    int is_hevc;   /* 0 = H.264 SEI (0x06), 1 = HEVC SEI (0x4E 0x01) */
    int read_idx;  /* Per-branch circular buffer search hint            */
    uint32_t last_injected_imu_head; /* Used for zero-latency marker tracking */
} SeiProbeCtx;

/* ─── Read IMU Helper ──────────────────────────────────────────────────── */
static void read_imu_data(SeiMetadata *meta)
{
    if (app_ctx.imu_fd >= 0) {
        struct icm20948_all_data imu;
        if (ioctl(app_ctx.imu_fd, ICM20948_IOC_GET_ALL, &imu) == 0) {
            meta->accel_x_mg  = imu.accel_x_mg;
            meta->accel_y_mg  = imu.accel_y_mg;
            meta->accel_z_mg  = imu.accel_z_mg;
            meta->gyro_x_mdps = imu.gyro_x_mdps;
            meta->gyro_y_mdps = imu.gyro_y_mdps;
            meta->gyro_z_mdps = imu.gyro_z_mdps;
            meta->mag_x_ut    = imu.mag_x_ut;
            meta->mag_y_ut    = imu.mag_y_ut;
            meta->mag_z_ut    = imu.mag_z_ut;
            return;
        }
    }
    /* Fallback: simulated values */
    static int sim_counter = 0;
    sim_counter++;
    meta->accel_x_mg  = 10 + (sim_counter % 50);
    meta->accel_y_mg  = -980 + (sim_counter % 20);
    meta->accel_z_mg  = 45 + (sim_counter % 30);
    meta->gyro_x_mdps = 150 + (sim_counter % 100);
    meta->gyro_y_mdps = -20 + (sim_counter % 40);
    meta->gyro_z_mdps = 5 + (sim_counter % 10);
    meta->mag_x_ut    = 25 + (sim_counter % 15);
    meta->mag_y_ut    = -12 + (sim_counter % 8);
    meta->mag_z_ut    = 40 + (sim_counter % 20);
}

/* ─── Thread for >0 Hz Polling ─────────────────────────────────────────── */
static gpointer imu_thread_func(gpointer data) {
    (void)data;
    gint64 interval_us = 1000000LL / g_imu_hz;
    gint64 next_wake_us = g_get_monotonic_time() + interval_us;

    while (imu_thread_running) {
        SeiMetadata meta;
        memset(&meta, 0, sizeof(meta));
        meta.magic = SEI_METADATA_MAGIC;
        meta.version = SEI_METADATA_VERSION;
        
        struct timespec ts, utc;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        clock_gettime(CLOCK_REALTIME, &utc);
        meta.imu_timestamp_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
        meta.utc_timestamp_us = (uint64_t)utc.tv_sec * 1000000ULL + (uint64_t)utc.tv_nsec / 1000ULL;
        
        read_imu_data(&meta);
        
        memcpy(meta.device_serial, app_ctx.device_serial, 16);
        meta.environment_type = app_ctx.environment_type;
        meta.latitude_e7      = app_ctx.latitude_e7;
        meta.longitude_e7     = app_ctx.longitude_e7;
        memcpy(meta.session_id, app_ctx.session_id, 16);
        
        g_mutex_lock(&imu_mutex);
        imu_ring[imu_head % IMU_RING_SIZE] = meta;
        imu_head++;
        g_mutex_unlock(&imu_mutex);
        
        gint64 now_us = g_get_monotonic_time();
        if (next_wake_us > now_us) {
            g_usleep(next_wake_us - now_us);
        }
        next_wake_us += interval_us;
    }
    return NULL;
}

/* ─── IMU device open ──────────────────────────────────────────────────── */
static int imu_open(void)
{
    g_mutex_init(&imu_mutex);
    g_mutex_init(&pts_mutex);
    
    app_ctx.imu_fd = open("/dev/icm20948", O_RDONLY);
    if (app_ctx.imu_fd < 0) {
        printf("IMU: /dev/icm20948 not found — using simulated values.\n");
    } else {
        printf("IMU: /dev/icm20948 opened (fd=%d) — real sensor data.\n", app_ctx.imu_fd);
    }
    
    if (g_imu_hz > 0) {
        imu_thread_running = TRUE;
        g_thread_new("imu_thread", imu_thread_func, NULL);
        printf("IMU: Started background thread polling at %d Hz.\n", g_imu_hz);
    } else {
        printf("IMU: Using inline 0-latency polling (1 sample per frame).\n");
    }
    return 0;
}

/* ─── Emulation Prevention Bytes ───────────────────────────────────────── */
static int apply_emulation_prevention(const uint8_t *src, int src_len, uint8_t *dst)
{
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

/* ─── Session State ──────────────────────────────────────────────────────── */
static GstClockTime g_last_pts = 0;
static uint32_t g_total_dropped = 0;
static uint32_t g_total_passed = 0;
static uint32_t g_passed_this_sec = 0;
static GstClockTime g_last_sec_time = 0;
static gboolean g_caps_printed = FALSE;
static __thread GstClockTime t_last_enc_pts = GST_CLOCK_TIME_NONE;

static void uvc_sei_reset_session(void) {
    app_ctx.frame_counter = 0;
    g_last_pts = 0;
    g_total_dropped = 0;
    g_total_passed = 0;
    g_passed_this_sec = 0;
    g_last_sec_time = 0;
    g_caps_printed = FALSE;
    t_last_enc_pts = GST_CLOCK_TIME_NONE;
}

/* ─── GStreamer pad probe: Camera Source ──────────────────────────────── *
 * This probe fires at the native v4l2src rate (30Hz).
 * Approach 1 (imu_hz=0): inline IOCTL, 1 sample/frame, ≤1ms latency.
 * Approach 2 (imu_hz>0): just mark the ring buffer head position.
 * frame_counter is incremented in sei_injection_probe, not here.
 */
static GstPadProbeReturn camera_src_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
    (void)pad; (void)user_data;
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buffer) return GST_PAD_PROBE_OK;

    GstClockTime pts = GST_BUFFER_PTS(buffer);

    /* --- DYNAMIC MANUAL FRAMERATE LIMITER (Max 30fps) ---
     * Because videorate is broken/unreliable with this hardware,
     * we manually enforce 30fps by dropping buffers that arrive too soon.
     * 30fps = 33.3ms gap. We drop anything that arrives in under 30ms.
     */
    if (pts != GST_CLOCK_TIME_NONE && g_last_pts != 0) {
        if (pts > g_last_pts && (pts - g_last_pts) < 30000000) {
            g_total_dropped++;
            return GST_PAD_PROBE_DROP; // Drop this frame!
        }
    }
    g_last_pts = pts;

    g_total_passed++;
    g_passed_this_sec++;

    if (g_last_sec_time == 0) {
        g_last_sec_time = pts;
    } else if (pts >= g_last_sec_time + 1000000000ULL) {
        g_print("SRC_PROBE: Passed=%u fps | Total Passed=%u | Total Dropped=%u\n", 
                g_passed_this_sec, g_total_passed, g_total_dropped);
        g_passed_this_sec = 0;
        g_last_sec_time = pts;
    }

    if (!g_caps_printed) {
        GstCaps *caps = gst_pad_get_current_caps(pad);
        if (caps) {
            gchar *caps_str = gst_caps_to_string(caps);
            g_print("\n>>> CAMERA SRC CAPS: %s <<<\n\n", caps_str);
            g_free(caps_str);
            gst_caps_unref(caps);
        }
        g_caps_printed = TRUE;
    }

    uint32_t current_frame_id = app_ctx.frame_counter++;

    if (g_imu_hz == 0) {
        /* Inline sampling exactly on frame arrival */
        SeiMetadata meta;
        memset(&meta, 0, sizeof(meta));
        meta.magic   = SEI_METADATA_MAGIC;
        meta.version = SEI_METADATA_VERSION;
        
        struct timespec ts, utc;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        clock_gettime(CLOCK_REALTIME, &utc);
        meta.imu_timestamp_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
        meta.utc_timestamp_us = (uint64_t)utc.tv_sec * 1000000ULL + (uint64_t)utc.tv_nsec / 1000ULL;
        read_imu_data(&meta);
        
        memcpy(meta.device_serial, app_ctx.device_serial, 16);
        meta.environment_type = app_ctx.environment_type;
        meta.latitude_e7      = app_ctx.latitude_e7;
        meta.longitude_e7     = app_ctx.longitude_e7;
        memcpy(meta.session_id, app_ctx.session_id, 16);

        g_mutex_lock(&imu_mutex);
        uint32_t start_idx = imu_head;
        imu_ring[imu_head % IMU_RING_SIZE] = meta;
        imu_head++;
        g_mutex_unlock(&imu_mutex);

        g_mutex_lock(&pts_mutex);
        pts_map[pts_head % MAX_PTS_HISTORY].pts = pts;
        pts_map[pts_head % MAX_PTS_HISTORY].start_idx = start_idx;
        pts_map[pts_head % MAX_PTS_HISTORY].count = 1;
        pts_map[pts_head % MAX_PTS_HISTORY].frame_id = current_frame_id;
        pts_head++;
        g_mutex_unlock(&pts_mutex);
        
    } else {
        /* High-Hz background thread is writing to imu_ring.
         * Just mark the current head exactly at frame capture (zero latency marker). */
        g_mutex_lock(&imu_mutex);
        uint32_t current_head = imu_head;
        g_mutex_unlock(&imu_mutex);
        
        g_mutex_lock(&pts_mutex);
        pts_map[pts_head % MAX_PTS_HISTORY].pts = pts;
        pts_map[pts_head % MAX_PTS_HISTORY].start_idx = current_head;
        pts_map[pts_head % MAX_PTS_HISTORY].count = 0; /* Unused by new logic */
        pts_map[pts_head % MAX_PTS_HISTORY].frame_id = current_frame_id;
        pts_head++;
        g_mutex_unlock(&pts_mutex);
    }

    return GST_PAD_PROBE_OK;
}

/* ─── GStreamer pad probe: SEI injection ──────────────────────────────── *
 * This probe fires at 30Hz (post-videorate, post-encoder). It is the
 * correct place to increment the frame counter so frame_ids are
 * sequential (0, 1, 2, 3…) without skips.
 */
static GstPadProbeReturn sei_injection_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
    (void)pad;
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buffer) return GST_PAD_PROBE_OK;

    SeiProbeCtx *ctx = (SeiProbeCtx *)user_data;
    int is_hevc = ctx->is_hevc;
    GstClockTime pts = GST_BUFFER_PTS(buffer);

    uint32_t start_idx_imu = 0;
    uint32_t count_imu = 0;
    int found = 0;

    /* FIX #4: Prevent double-counting if the hardware encoder pushes multiple NAL units 
     * (like SPS/PPS and Slice) as separate GStreamer buffers for the SAME frame PTS! 
     * We use __thread to give the RTSP encoder and REC encoder their own isolated variables. */
    if (pts == t_last_enc_pts && pts != GST_CLOCK_TIME_NONE) {
        // We already injected SEI into the first buffer of this frame. Pass it through!
        return GST_PAD_PROBE_OK;
    }
    t_last_enc_pts = pts;

    uint32_t frame_id = 0;

    g_mutex_lock(&pts_mutex);
    int start_idx = ctx->read_idx;
    for (int i = 0; i < MAX_PTS_HISTORY; i++) {
        int idx = (start_idx + i) % MAX_PTS_HISTORY;
        if (pts_map[idx].pts == pts && pts != GST_CLOCK_TIME_NONE) {
            frame_id = pts_map[idx].frame_id;
            if (g_imu_hz == 0) {
                start_idx_imu = pts_map[idx].start_idx;
                count_imu = pts_map[idx].count;
            } else {
                uint32_t target_head = pts_map[idx].start_idx;
                count_imu = target_head - ctx->last_injected_imu_head;
                
                /* Handle first frame or wrapped buffer (massive drops) */
                if (ctx->last_injected_imu_head == 0 || count_imu > IMU_RING_SIZE) {
                    if (target_head >= 64) {
                        ctx->last_injected_imu_head = target_head - 64;
                        count_imu = 64;
                    } else {
                        ctx->last_injected_imu_head = 0;
                        count_imu = target_head;
                    }
                }
                start_idx_imu = ctx->last_injected_imu_head;
                ctx->last_injected_imu_head = target_head;
            }
            found = 1;
            ctx->read_idx = (idx + 1) % MAX_PTS_HISTORY;
            break;
        }
    }
    g_mutex_unlock(&pts_mutex);

    if (!found) {
        g_printerr("SEI[%s]: WARNING — PTS lookup miss for PTS=%" G_GUINT64_FORMAT " (frame_id=%u)\n",
                    is_hevc ? "HEVC" : "H264", pts, frame_id);
        return GST_PAD_PROBE_OK;
    }
    
    if (count_imu == 0) {
        /* No IMU samples generated since last frame (e.g. if framerate > IMU hz) */
        g_print("SEI[%s]: frame_id=%u — no IMU samples available, passing frame through.\n",
                is_hevc ? "HEVC" : "H264", frame_id);
        return GST_PAD_PROBE_OK;
    }

    /* Log every 30th frame (~1 per second at 30fps) for health monitoring */
    if (frame_id % 30 == 0) {
        g_print("SEI: frame_id=%u | imu_samples=%u\n", frame_id, count_imu);
    }

    uint8_t uuid[16] = {0xdc, 0x45, 0xe9, 0xbd, 0xe6, 0xd9, 0x48, 0xb7,
                        0x96, 0x2c, 0xd3, 0x9c, 0x24, 0x5a, 0xc8, 0x1a};

    /* FIX #2: Save the original buffer flags BEFORE any gst_buffer_append.
     * gst_buffer_append creates a new buffer that does NOT inherit the
     * original flags, stripping GST_BUFFER_FLAG_DELTA_UNIT off P-frames.
     * This causes downstream parsers to treat every frame as an I-frame,
     * breaking playback timing and doubling perceived framerate.          */
    GstBufferFlags orig_flags = GST_BUFFER_FLAGS(buffer);

    GstBuffer *sei_agg_buf = NULL;
    
    for (uint32_t i = 0; i < count_imu; i++) {
        g_mutex_lock(&imu_mutex);
        SeiMetadata meta = imu_ring[(start_idx_imu + i) % IMU_RING_SIZE];
        g_mutex_unlock(&imu_mutex);
        
        meta.frame_pts_ns = pts;
        meta.frame_id = frame_id;
        
        int raw_size = 16 + sizeof(SeiMetadata);
        uint8_t *raw = g_malloc(raw_size);
        memcpy(raw, uuid, 16);
        memcpy(raw + 16, &meta, sizeof(SeiMetadata));

        uint8_t *epb = g_malloc(raw_size * 2);
        int epb_len = apply_emulation_prevention(raw, raw_size, epb);

        uint8_t sei_header[10];
        int header_len = 0;
        if (is_hevc) {
            sei_header[0] = 0; sei_header[1] = 0; sei_header[2] = 0; sei_header[3] = 1;
            sei_header[4] = 0x4E; sei_header[5] = 0x01; sei_header[6] = 0x05;
            header_len = 7;
        } else {
            sei_header[0] = 0; sei_header[1] = 0; sei_header[2] = 0; sei_header[3] = 1;
            sei_header[4] = 0x06; sei_header[5] = 0x05;
            header_len = 6;
        }

        uint8_t size_bytes[4];
        int size_len = 0;
        int remaining = raw_size;
        while (remaining >= 255) { size_bytes[size_len++] = 0xFF; remaining -= 255; }
        size_bytes[size_len++] = (uint8_t)remaining;

        int total = header_len + size_len + epb_len + 1;
        uint8_t *pkt = g_malloc(total);
        int off = 0;
        memcpy(pkt + off, sei_header, header_len); off += header_len;
        memcpy(pkt + off, size_bytes, size_len);   off += size_len;
        memcpy(pkt + off, epb, epb_len);           off += epb_len;
        pkt[off] = 0x80; 

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

        /* FIX #2: Explicitly copy the flags from the original buffer to the
         * new combined buffer. This preserves GST_BUFFER_FLAG_DELTA_UNIT on
         * P-frames so downstream h264parse/mp4mux correctly distinguish
         * keyframes from delta frames.                                      */
        gst_buffer_copy_into(new_buffer, buffer,
                             GST_BUFFER_COPY_FLAGS, 0, -1);

        GST_PAD_PROBE_INFO_DATA(info) = new_buffer;
        gst_buffer_unref(buffer);
    }

    return GST_PAD_PROBE_OK;
}

#endif /* UVC_SEI_INJECTION_H */
