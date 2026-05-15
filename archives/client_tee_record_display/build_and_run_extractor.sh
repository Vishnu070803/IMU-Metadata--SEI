#!/bin/bash
# build_and_run_extractor.sh

set -e

echo "Compiling SEI Extractor..."

gcc sei_extractor.c -o sei_extractor \
    $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-video-1.0 cairo) \
    -pthread -lm -Wall -O2

echo "Build successful."
echo ""
echo "Usage:"
echo "  ./sei_extractor --in <file.mp4>                          # extract and print to console"
echo "  ./sei_extractor --in <file.mp4> --csv out.csv            # extract and save to CSV"
echo "  ./sei_extractor --in <file.mp4> --play                   # play video with HUD overlay"
echo "  ./sei_extractor --in <file.mp4> --csv out.csv --play     # extract, save, and play"
echo ""
