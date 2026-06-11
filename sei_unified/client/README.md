# SEI Metadata Client (PC / Host)

This application connects to an RTSP stream, extracts embedded IMU metadata, and displays it in real-time using a Cairo HUD overlay.

## Features
- Real-time SEI extraction (supports multiple IMU samples per frame)
- Cairo HUD Visualization
- Parallel MP4 Recording (with overlay and crash resilience)
- Graceful shutdown via `Ctrl+C` for perfect MP4 `moov` atom finalization
- CSV Export of live metadata

## Installation
```bash
chmod +x install_client_deps.sh
./install_client_deps.sh
```

## How to Build & Run
```bash
chmod +x build_and_run_client.sh
./build_and_run_client.sh --ip <server_ip> [--csv out.csv]
```
*Note: The `--ip` argument specifies the IP address of the server hosting the RTSP stream. Defaults to `10.0.0.1`.*

## Network Setup
If connecting via USB CDC-NCM, run `./setup_pc.sh` to initialize the host IP (10.0.0.2).

---

## Known Issues & Workarounds
- **RTSP UDP Packet Loss:** If streaming over a congested network via UDP, dropped video packets can result in lost SEI metadata frames. **Workaround:** Force RTSP over TCP if lossless IMU metadata is strictly required on the client side.
- **Python Pad Probes:** Previously, using Python-based pad probes caused memory corruption. This client avoids those issues by strictly using C-based extraction for high stability.
