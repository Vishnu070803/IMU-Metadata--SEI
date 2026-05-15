#!/bin/bash
echo "Compiling RTSP SEI Server App..."

gcc sei_server.c -o sei_server \
    $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-rtsp-server-1.0) \
    -pthread

if [ $? -eq 0 ]; then
    echo "Build successful. Starting Server..."
    # Optional: pass --rec-dir /mnt/sdcard or any writable path for recording output.
    # Default recording directory is /tmp.
    ./sei_server "$@"
else
    echo "Build failed. Check your GStreamer includes."
fi
