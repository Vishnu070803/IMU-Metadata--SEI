# SEI Metadata Extractor (Offline Tool)

This tool processes recorded MP4 files to extract embedded IMU metadata.

## Features
- Fast CSV extraction
- Video playback with HUD overlay
- Supports H.264 and HEVC SEI

## Installation
```bash
chmod +x install_extractor_deps.sh
./install_extractor_deps.sh
```

## How to Build & Run
```bash
chmod +x build_and_run_extractor.sh
./build_and_run_extractor.sh --in <file.mp4> [--csv out.csv] [--play]
```
