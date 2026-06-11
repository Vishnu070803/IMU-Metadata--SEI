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

---

## Caveats & Limitations

- **File Finalization:** The extractor requires valid `.mp4` files. If a file is completely corrupted (no `moov` atom and not using fragmented MP4), extraction will fail. Ensure the server gracefully shuts down the recording.
- **Playback Speed:** Playback with HUD overlay runs as fast as the host CPU can decode and render Cairo frames, which may be faster or slower than real-time depending on your system's hardware.
