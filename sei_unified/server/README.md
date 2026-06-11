# SEI Metadata Server (i.MX8MP Board)

This application runs on the embedded board. It captures video from the camera, samples IMU data from the hardware sensor, and injects it into the video stream as SEI NAL units.

## Installation
Run the dependency script to install GStreamer and RTSP server libraries:
```bash
chmod +x scripts/install_server_deps.sh
./scripts/install_server_deps.sh
```

## Applications & Usage

Both applications support graceful shutdown. Pressing `Ctrl+C` sends an End-Of-Stream (EOS) signal to properly finalize the MP4 file (writing the `moov` atom) before exiting. Both applications enforce a strict **30 FPS** output, utilizing a zero-latency marker architecture to ensure 100% of IMU samples are retained and perfectly synchronized, even when camera frames are dropped by `videorate`.

### 1. Live RTSP Server (`sei_server.c`)
- RTSP Streaming (H.264)
- Local MP4 Recording (Fragmented MP4 for crash resilience)
- Real-time IMU sampling via IOCTL
- Configurable auto-stop session timer (supports continuous mode)
- Automatically waits for new connections without exiting

**How to Build & Run:**
```bash
chmod +x scripts/build_and_run_server.sh
./scripts/build_and_run_server.sh [--rec-dir /path/to/save] [--imu-hz <hz>] [--timeout <sec>] [--camera /dev/videoX] [--ip <bind_ip>]
```
*Note: Use `--imu-hz <hz>` (e.g., 100) to enable high-frequency IMU capture in a background thread. Defaults to 0 (inline polling once per video frame).*
*Note: The server runs in continuous mode by default (no timeout). Use `--timeout <seconds>` to set a session timer that automatically stops the stream and finalizes the recording.*

### 2. 100Hz Dedicated Recorder (`sei_recorder_100hz.c`)
- **Standalone autonomous recording:** Starts recording immediately to `.mp4` using a direct `filesink`.
- **No Client Connections:** Bypasses RTSP entirely and does not wait for or accept client connections.
- Optimizes IMU sampling at exactly 100Hz for high-speed offline data collection.

**How to Build & Run:**
```bash
chmod +x scripts/build_and_run_recorder_100hz.sh
./scripts/build_and_run_recorder_100hz.sh [--rec-dir /path/to/save] [--camera /dev/videoX]
```

## Hardware Setup
1. Ensure the ICM20948 driver is loaded and `/dev/icm20948` is accessible.
   **Important:** The IOCTL calls used to communicate with the driver are hardcoded in the server source code. You may need to change these IOCTL calls according to your specific driver implementation.
2. If using USB CDC-NCM, run `./scripts/setup_board.sh` to initialize the IP (10.0.0.1).

---

## Caveats & Known Issues
- **V4L2 Warnings:** Benign warnings like "Uncertain or not enough buffers" or "Failed to probe pixel aspect ratio" may appear during initialization. These are NXP driver quirks and do not impact encoding or SEI injection.
- **Graceful Exit Required:** You **must** exit the application via `Ctrl+C` or let the session timer expire (or client disconnect) to properly finalize the `.mp4` file. Hard kills (`kill -9`) or power failures will result in missing `moov` atoms.
- **Recording Fragmentation (No Splitmux):** The application relies on `mp4mux fragment-duration=1000` rather than `splitmuxsink`. While a new file is dynamically created for each *new* RTSP connection, a single continuous stream will write to one growing file. In the event of a power failure during a very long continuous session, you risk losing the metadata/`moov` atom for that entire long session since the file is never segmented.
- **IMU Rate vs Frame Rate:** The 100Hz recorder uses an independent thread to gather IMU data to bypass frame-rate bottlenecks and prevent dropping IMU samples when the camera stutters.
