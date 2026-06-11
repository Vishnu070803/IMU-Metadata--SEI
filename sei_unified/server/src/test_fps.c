#include <gst/gst.h>
#include <stdio.h>
#include <time.h>

static int frame_count = 0;

static GstPadProbeReturn frame_counter_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    (void)pad; (void)info; (void)user_data;
    frame_count++;
    return GST_PAD_PROBE_OK;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    g_print("Building isolated 30fps test pipeline...\n");

    /* We ask for exactly 300 buffers.
     * If the camera respects 30fps, this will take exactly 10 seconds.
     * If the camera forces 60fps, this will finish in 5 seconds.
     */
    const char *launch_str = 
        "v4l2src device=/dev/video3 num-buffers=300 ! "
        "video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 ! "
        "fakesink name=sink sync=true";

    GError *error = NULL;
    GstElement *pipeline = gst_parse_launch(launch_str, &error);
    if (error) {
        g_printerr("Failed to parse pipeline: %s\n", error->message);
        g_error_free(error);
        return -1;
    }

    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    GstPad *pad = gst_element_get_static_pad(sink, "sink");
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, frame_counter_probe, NULL, NULL);
    gst_object_unref(pad);
    gst_object_unref(sink);

    g_print("Starting pipeline... (Should take 10 seconds if 30fps, 5 seconds if 60fps)\n");
    time_t start_time = time(NULL);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
        GST_MESSAGE_EOS | GST_MESSAGE_ERROR);

    time_t end_time = time(NULL);
    int elapsed = end_time - start_time;

    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
        GError *err;
        gchar *debug_info;
        gst_message_parse_error(msg, &err, &debug_info);
        g_printerr("Error received from element %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
        g_printerr("Debugging information: %s\n", debug_info ? debug_info : "none");
        g_clear_error(&err);
        g_free(debug_info);
    } else {
        g_print("\nPipeline finished successfully (EOS).\n");
    }

    g_print("========================================\n");
    g_print("Total Frames Processed: %d\n", frame_count);
    g_print("Total Time Elapsed: %d seconds\n", elapsed);
    if (elapsed > 0) {
        g_print("Actual Physical Framerate: %d FPS\n", frame_count / elapsed);
    }
    g_print("========================================\n");

    gst_message_unref(msg);
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}
