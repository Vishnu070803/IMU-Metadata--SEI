#!/bin/bash
echo "Compiling Hybrid 100Hz IMU Recorder App..."
gcc -o ../build/sei_recorder_100hz_hybrid ../src/sei_recorder_100hz_hybrid.c -I../include $(pkg-config --cflags --libs gstreamer-1.0) -Wall -Wextra -O2

if [ $? -eq 0 ]; then
    echo "Build successful. Starting Hybrid Recorder..."
    ../build/sei_recorder_100hz_hybrid "$@"
else
    echo "Build failed."
fi
