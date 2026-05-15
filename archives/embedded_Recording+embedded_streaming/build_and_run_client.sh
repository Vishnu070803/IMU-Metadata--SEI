#!/bin/bash
# build_and_run_client.sh
#
# Dependencies (Ubuntu/Debian):
#   sudo apt-get install \
#       libgstreamer1.0-dev \
#       libgstreamer-plugins-base1.0-dev \
#       libcairo2-dev \
#       gstreamer1.0-plugins-good \
#       gstreamer1.0-libav \
#       gstreamer1.0-rtsp

set -e

echo "Compiling SEI Client (Cairo HUD Display)..."

gcc sei_client.c -o sei_client \
    $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-video-1.0 cairo) \
    -pthread -lm -Wall -O2

echo "Build successful."
echo ""
echo "Usage:"
echo "  ./sei_client                        # default: rtsp://10.0.0.1:8554/stream"
echo "  ./sei_client -v                     # verbose SEI console dump"
echo "  ./sei_client --ip 192.168.1.5       # custom board IP"

echo ""
echo "Starting..."
./sei_client "$@"
