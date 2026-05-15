# SEI Metadata Client (PC / Host)

This application connects to an RTSP stream, extracts embedded IMU metadata, and displays it in real-time using a Cairo HUD overlay.

## Features
- Real-time SEI extraction
- Cairo HUD Visualization
- Parallel MP4 Recording (with overlay)
- CSV Export of live metadata

## Installation
```bash
chmod +x install_client_deps.sh
./install_client_deps.sh
```

## How to Build & Run
```bash
chmod +x build_and_run_client.sh
./build_and_run_client.sh --ip <board_ip> [--csv out.csv]
```

## Network Setup
If connecting via USB CDC-NCM, run `./setup_pc.sh` to initialize the host IP (10.0.0.2).
