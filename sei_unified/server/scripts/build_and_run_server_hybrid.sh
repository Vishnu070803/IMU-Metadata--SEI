#!/bin/bash
echo "Compiling Hybrid RTSP SEI Server App..."
gcc -o ../build/sei_server_hybrid ../src/sei_server_hybrid.c -I../include $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-rtsp-server-1.0) -Wall -Wextra -O2

if [ $? -eq 0 ]; then
    echo "Build successful. Starting Hybrid Server..."
    ../build/sei_server_hybrid "$@"
else
    echo "Build failed."
fi
