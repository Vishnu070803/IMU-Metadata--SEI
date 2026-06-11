#!/bin/bash
echo "Compiling 100Hz IMU Recorder App..."

gcc ../src/sei_recorder_100hz.c -I../include -o ../build/sei_recorder_100hz \
    $(pkg-config --cflags --libs gstreamer-1.0 glib-2.0) \
    -pthread

if [ $? -eq 0 ]; then
    echo "Build successful. Starting Recorder..."
    # Optional: pass --rec-dir /mnt/sdcard or any writable path for recording output.
    # Default recording directory is /tmp.
    ../build/sei_recorder_100hz "$@"
else
    echo "Build failed. Check your GStreamer includes."
fi
