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
│   ├── sei_server.c
│   ├── build_and_run_server.sh
│   └── setup_board.sh
├── client/             # PC/Host Live Client
│   ├── sei_client.c
│   ├── build_and_run_client.sh
│   └── setup_pc.sh
└── extractor/          # Offline MP4 Extractor
    ├── sei_extractor.c
    └── build_and_run_extractor.sh
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
- `icm20948_ioctl.h`: IOCTL interface for the ICM20948 driver.

### 2. Server (`server/`)

**Purpose:** Runs on the embedded board (i.MX8MP). Captures video, samples real-time IMU data via IOCTL, injects it as SEI NALs, streams over RTSP, and records locally.

**Usage:**

```bash
cd server/
./build_and_run_server.sh [--rec-dir /mnt/sdcard]
```

### 3. Client (`client/`)

**Purpose:** Connects to the RTSP stream. Extracts live IMU metadata, displays a Cairo HUD overlay, and records the HUD-overlayed video. Can also export data to CSV.

**Usage:**

```bash
cd client/
./build_and_run_client.sh --ip <board_ip> [--csv live_data.csv]
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

- **Board:** Run `server/setup_board.sh` to initialize the gadget IP `10.0.0.1`.
- **PC:** Run `client/setup_pc.sh` to configure the host IP `10.0.0.2`.
