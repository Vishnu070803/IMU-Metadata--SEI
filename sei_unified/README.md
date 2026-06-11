# Unified SEI Video Pipeline (Record, Stream & Extract)

This folder contains the consolidated source code for embedding, streaming, and extracting SEI (Supplemental Enhancement Information) IMU metadata. The project is organized into sub-folders for better maintainability.

## Directory Structure

```text
sei_unified/
├── common/             # Shared header files
│   ├── sei_metadata.h
│   ├── uvc_sei_injection.h
│   └── icm20948_ioctl.h
├── server/             # Embedded Board application
│   ├── src/            # C source files
│   ├── include/        # Header files
│   ├── scripts/        # Build and setup scripts
│   └── build/          # Compiled binaries
├── client/             # PC/Host Live Client
│   ├── sei_client.c
│   ├── build_and_run_client.sh
│   └── setup_pc.sh
├── extractor/          # Offline MP4 Extractor
│   ├── sei_extractor.c
│   └── build_and_run_extractor.sh
├── TESTING_GUIDE.md         # Comprehensive testing and usage guide
├── install_dependencies.sh  # Setup script
```

---

## Installation

Before building any application, install the required GStreamer and Cairo libraries:

```bash
./install_dependencies.sh
```

---

### 1. Common Folder (`common/`)

Contains shared logic and definitions:

- `sei_metadata.h`: Defines the exact binary layout (110 bytes) of the IMU payload.
- `uvc_sei_injection.h`: Server-side routines for sampling IMU and injecting NALs.
- `icm20948_ioctl.h`: IOCTL interface for the ICM20948 driver. **Note:** The IOCTL calls (e.g., magic numbers, commands) are hardcoded. These calls may need to be changed according to your specific driver implementation.

### 2. Server (`server/`)

**Purpose:** Runs on the embedded board (i.MX8MP). Captures video, samples real-time IMU data via IOCTL, and injects it as SEI NALs. Includes two main applications:

1. **Live RTSP Server (`sei_server.c`):** Streams video over RTSP and records locally.
2. **100Hz Dedicated Recorder (`sei_recorder_100hz.c`):** Optimizes IMU sampling at 100Hz for high-speed offline data collection without RTSP streaming overhead.

**Usage:**

*Run the standard RTSP Server:*

```bash
cd server/
./scripts/build_and_run_server.sh [--rec-dir /mnt/sdcard] [--imu-hz <hz>] [--timeout <sec>]
```

Note: Use `--imu-hz <hz>` to enable high-frequency IMU capture. Defaults to 0 (inline polling once per frame). The server runs continuously by default (`--timeout 0`). Use `--timeout <sec>` to specify an auto-stop session duration.

*Run the 100Hz Dedicated Recorder:*

```bash
cd server/
./scripts/build_and_run_recorder_100hz.sh [--rec-dir /mnt/sdcard]
```

### 3. Client (`client/`)

**Purpose:** Connects to the RTSP stream. Extracts live IMU metadata and displays a Cairo HUD overlay in real-time. Can also export data to CSV. (Note: The C client application is for display/monitoring only; it does not record video).

**Usage:**

```bash
cd client/
./build_and_run_client.sh --ip <board_ip> [--csv live_data.csv]
```

#### Command-Line Client Recording
To record the pristine stream with SEI metadata intact on the client side, use one of the following `gst-launch-1.0` commands:

**Variant A: Record AND Display Simultaneously**
```bash
gst-launch-1.0 -e rtspsrc location=rtsp://<board_ip>:8554/stream latency=1000 ! rtph264depay ! h264parse ! tee name=t t. ! queue ! avdec_h264 ! autovideosink sync=false t. ! queue ! mp4mux faststart=true ! filesink location=recorded_with_display.mp4
```

**Variant B: Record ONLY (Headless, no display)**
```bash
gst-launch-1.0 -e rtspsrc location=rtsp://<board_ip>:8554/stream latency=1000 ! rtph264depay ! h264parse ! video/x-h264,stream-format=avc,alignment=au ! mp4mux faststart=true ! filesink location=recorded_only.mp4
```

### 4. Offline Extractor (`extractor/`)

**Purpose:** Processes recorded `.mp4` files to extract metadata to CSV or play back with a HUD overlay at maximum speed.

**Usage:**

```bash
cd extractor/
./build_and_run_extractor.sh --in <video.mp4> [--csv out.csv] [--play]
```

---

### Network Setup

If using USB Ethernet (CDC-NCM):

- **Board:** Run `server/scripts/setup_board.sh` to initialize the gadget IP `10.0.0.1`.
- **PC:** Run `client/setup_pc.sh` to configure the host IP `10.0.0.2`.

---

## Known Issues, Caveats & Workarounds

1. **State Thrashing in Multi-Branch Pipelines**:
   - **Issue:** Running simultaneous RTSP streaming and MP4 recording can cause encoder state thrashing and segmentation faults.
   - **Workaround:** We have enforced `__thread` local storage for SEI injection variables to isolate thread states safely.

2. **MP4 File Corruption on Power Loss (No Splitmux)**:
   - **Issue:** Hard power loss prevents GStreamer from writing the `moov` atom, rendering standard MP4s unplayable. Furthermore, we are NOT using `splitmuxsink`, meaning long continuous sessions are written to a single growing file rather than segmented files.
   - **Workaround:** We use fragmented MP4s (`fragment-duration=1000`) for partial crash resilience. However, a power failure during a very long session still risks losing metadata for the entire session. Always shut down the server gracefully or use the `--timeout` argument to stop streams and rotate files safely.

3. **Frame Dropping & IMU Synchronization**:
   - **Issue:** If the upstream camera stutters, GStreamer's `videorate` might drop frames, leading to lost IMU data.
   - **Workaround:** The pipeline implements manual frame-dropping via a zero-latency marker probe to ensure 100% of IMU data is mapped to the closest surviving frame.

4. **i.MX8MP V4L2 Encoder Warnings**:
   - **Issue:** You may see `Uncertain or not enough buffers` or `Failed to probe pixel aspect ratio` warnings in the server logs.
   - **Workaround:** These are known non-fatal quirks of the NXP VPU driver and can be safely ignored.
