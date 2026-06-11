# IMU-Avigna SEI Unified Pipeline - Release Notes

## Overview
This release finalizes the transition to a highly stable, C-based unified pipeline for embedding, streaming, and extracting IMU data via H.264/HEVC Supplemental Enhancement Information (SEI) NAL units on the i.MX8M Plus platform. 

The update focuses on resolving critical stability issues found in previous iterations (such as memory corruption from Python pad probes), ensuring frame-perfect synchronization under varying load conditions, and reorganizing the server codebase for long-term maintainability.

---

## 🚀 Key Changes & Architectural Approaches

### 1. Unified C-Based Pipeline Architecture
- **Deprecation of Python Probes:** We have completely moved away from using Python-based GStreamer pad probes due to inherent runtime memory corruption and segmentation faults. The injection (Server) and extraction (Client/Extractor) are now 100% C-based, offering maximum memory safety and performance.
- **Server Codebase Restructuring:** Reorganized the `server/` directory into a clean standard layout: `src/` for source code, `include/` for headers, `scripts/` for build scripts, and `build/` for output binaries.

### 2. Perfect Frame-to-IMU Synchronization
- **Zero-Latency Marker Probe:** Implemented a new marker-based synchronization system at the GStreamer source.
- **Manual Frame-Dropping Management:** Replaced reliance on `videorate` with manual frame-drop detection. This guarantees that 100% of the sampled IMU data is accurately retained and mapped to the closest surviving video frame, eliminating data loss during camera stutters.

### 3. Thread-Safe Multi-Branching
- **`__thread` Local Storage:** Solved severe state thrashing and segmentation faults occurring when running simultaneous RTSP streaming and MP4 local recording. SEI injection variables are now isolated per-thread, completely preventing pipeline cross-contamination.

### 4. Headless 100Hz Recorder Mode
- **Dedicated Application (`sei_recorder_100hz.c`):** Introduced a standalone, RTSP-bypassed application tailored for offline data collection. It operates an independent thread to gather IMU data at a strict 100Hz rate, overcoming standard video frame-rate bottlenecks.

### 5. Custom Kernel Module Streamlining
- **BSP Cleanup:** Removed unnecessary driver templates from the Yocto `meta-custom` layer. The repository now exclusively provides the highly-optimized `icm20948` out-of-tree kernel module.

---

## ⚠️ Known Limitations & Constraints

1. **Hardcoded IOCTL Interface:**
   - The IOCTL definitions (magic numbers, commands) used to interface with the `/dev/icm20948` driver are currently hardcoded in the server source code (`icm20948_ioctl.h`). Any future switch to a different IMU sensor or substantial driver update will require manual updates to these IOCTL calls.
   
2. **NXP V4L2 Encoder Quirks:**
   - The i.MX8MP hardware video encoder occasionally emits warnings such as *"Uncertain or not enough buffers"* or *"Failed to probe pixel aspect ratio"*. These are known non-fatal quirks of the NXP VPU driver and do not affect the integrity of the stream or SEI metadata.

3. **Network Constraints (UDP Packet Loss):**
   - When streaming RTSP over congested networks using UDP, dropped video packets will naturally result in lost SEI metadata frames at the client side.

4. **File Corruption on Hard Power Loss:**
   - If the system experiences a hard power loss (`kill -9` or physical power cut), standard MP4 file finalization fails, omitting the critical `moov` atom required for playback.

5. **Client Application Recording Bugs (Fragmented MP4 vs Faststart):**
   - The C-based client application (`sei_client.c`) currently experiences issues properly recording the dual-branch (display + record) architecture. 
   - If `fragment-duration=1000` is used, network jitter from the RTSP stream causes `rtpjitterbuffer` to output frames with out-of-order PTS timestamps. This causes the `mp4mux` fragmenter to create overlapping `moof` chunks, which severely breaks downstream extraction tools like `analyze_video.py` (`Invalid NAL unit size` errors).
   - If `faststart=true` is used, the MP4 is perfectly valid, but it is highly vulnerable to corruption. If the client window is abruptly closed without a graceful `Ctrl+C`, the `moov` atom is never written, resulting in an unplayable file.

---

## 🔧 Workarounds & Best Practices

1. **Graceful Shutdown for MP4 Integrity:**
   - Always terminate the application gracefully using `Ctrl+C` or allow the session timer to expire. This sends a proper End-Of-Stream (EOS) signal, guaranteeing the `moov` atom is written.

2. **Fragmented MP4 Recording:**
   - To mitigate data loss from unexpected crashes or power failures, the local recording pipeline now enforces fragmented MP4 recording (`fragment-duration=1000`). Even if the final `moov` atom isn't written, the fragmented chunks remain recoverable.

3. **Lossless RTSP Streaming:**
   - If perfectly intact IMU metadata is strictly required on the client side over an unstable network, force the RTSP client to connect via TCP rather than UDP.

4. **Client Recording Command-Line Alternative:**
   - Until the C-client recording bug is fully resolved, use the following `gst-launch-1.0` command to successfully record the pristine stream with SEI metadata while displaying it simultaneously:
     ```bash
     gst-launch-1.0 -e rtspsrc location=rtsp://192.168.1.125:8554/stream latency=1000 ! rtph264depay ! h264parse ! tee name=t t. ! queue ! avdec_h264 ! autovideosink sync=false t. ! queue ! mp4mux faststart=true ! filesink location=recorded.mp4
     ```
