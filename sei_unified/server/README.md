# SEI Metadata Server (i.MX8MP Board)

This application runs on the embedded board. It captures video from the camera, samples IMU data from the hardware sensor, and injects it into the video stream as SEI NAL units.

## Features
- RTSP Streaming (H.264)
- Local MP4 Recording (Sync with metadata)
- Real-time IMU sampling via IOCTL
- 5-minute auto-shutdown session timer

## Installation
Run the dependency script to install GStreamer and RTSP server libraries:
```bash
chmod +x install_server_deps.sh
./install_server_deps.sh
```

## How to Build & Run
```bash
chmod +x build_and_run_server.sh
./build_and_run_server.sh [--rec-dir /path/to/save]
```

## Hardware Setup
1. Ensure the ICM20948 driver is loaded and `/dev/icm20948` is accessible.
2. If using USB CDC-NCM, run `./setup_board.sh` to initialize the IP (10.0.0.1).
