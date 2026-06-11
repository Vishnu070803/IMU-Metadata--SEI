# SEI Unified Video Pipeline - Testing Guide

This guide explains how to properly connect, test, and validate the three main components of the SEI Unified Video Pipeline: the **Server**, the **Client**, and the **Offline Extractor**.

## 1. Setting up the Connection

### Network Setup (USB CDC-NCM)
If you are connecting your host PC directly to the i.MX8MP board via USB (CDC-NCM Ethernet gadget):
1. **On the Board:** Run `server/scripts/setup_board.sh`. This assigns `10.0.0.1` to the board.
2. **On the Host PC:** Run `client/setup_pc.sh`. This assigns `10.0.0.2` to the PC.
3. Test the connection by pinging the board from your PC: `ping 10.0.0.1`

### Hardware (IMU) Setup
Ensure the ICM20948 IMU driver is loaded on the board. You should see `/dev/icm20948` present. If it is not found, the applications will seamlessly fall back to generating **simulated IMU data** for testing purposes.

---

## 2. Testing the Live RTSP Stream (Server + Client)

This test validates that the embedded board can capture video, inject IMU data in real-time, and stream it to a client that displays and records it.

### Step 2a: Start the Server (On the Board)
Navigate to the `server/` directory and run:
```bash
./scripts/build_and_run_server.sh --imu-hz 100
```
**Expected Behavior:**
- The server will print `100Hz IMU RTSP Server Active`.
- It will enter a "Wait Mode" waiting for connections. No stream or recording is active yet.
- Note: It runs continuously by default. You can test the auto-teardown by adding `--timeout 60` to force the session to stop after 60 seconds.

### Step 2b: Start the Client Display (On the PC)
Navigate to the `client/` directory and run:
```bash
./build_and_run_client.sh --ip 10.0.0.1 --csv live_data.csv
```
**Expected Behavior:**
- A video window will pop up showing the live camera feed with a **Cairo HUD Overlay** rendering the live IMU data (Accel, Gyro, Mag).
- **Console Logs:** You will see a live text dump of the SEI data updating in the terminal.
*(Note: The C application only displays the stream. To record it, see Step 2c).*

### Step 2c: Client Recording (Command-Line)
If you wish to save the pristine RTSP stream with SEI metadata intact to an MP4 file on the client PC, run one of the following GStreamer commands in a separate terminal:

**Variant A: Record + Display**
```bash
gst-launch-1.0 -e rtspsrc location=rtsp://10.0.0.1:8554/stream latency=1000 ! rtph264depay ! h264parse ! tee name=t t. ! queue ! avdec_h264 ! autovideosink sync=false t. ! queue ! mp4mux faststart=true ! filesink location=recorded_with_display.mp4
```

**Variant B: Record ONLY (Headless)**
```bash
gst-launch-1.0 -e rtspsrc location=rtsp://10.0.0.1:8554/stream latency=1000 ! rtph264depay ! h264parse ! video/x-h264,stream-format=avc,alignment=au ! mp4mux faststart=true ! filesink location=recorded_only.mp4
```

### Step 2d: Graceful Shutdown (Critical)
To finalize any `.mp4` recordings properly (writing the `moov` atom):
1. Press `Ctrl+C` on the Client terminal (or `gst-launch-1.0` terminal). It will send an EOS signal and write the `moov` atom.
2. The Server will detect the client disconnected, cleanly finalize its own un-overlayed recording, and return to "Wait Mode" for a new connection.
3. Press `Ctrl+C` on the Server terminal to shut down the server entirely.

---

## 3. Testing the Standalone Recorder (No Client)

If you only want to record high-speed IMU data locally on the board without the overhead of an RTSP server, use the 100Hz Recorder.

**On the Board:**
```bash
cd server/
./scripts/build_and_run_recorder_100hz.sh --timeout 30
```
**Expected Behavior:**
- The pipeline immediately starts capturing video and IMU data to a local `.mp4` file.
- Because we used `--timeout 30`, it will record for exactly 30 seconds.
- After 30 seconds, it automatically sends an `EOS` signal, finalizes the MP4 file, and gracefully exits.

---

## 4. Testing the Offline Extractor

You can extract data or play back recordings you made on the server. Copy a recorded `.mp4` file from the board to your PC.

**On the PC:**
```bash
cd extractor/
./build_and_run_extractor.sh --in my_recording.mp4 --csv extracted_data.csv --play
```
**Expected Behavior:**
- The `--play` flag will pop up a window playing back the raw video, actively decoding the embedded SEI data and drawing the HUD on the fly.
- It will generate `extracted_data.csv` containing all 100Hz IMU samples perfectly synced to their respective frames.

---

## 5. Troubleshooting & Logs to Observe

*   **"Uncertain or not enough buffers" / "Failed to probe pixel aspect ratio"**: 
    *   **Where:** Server console during startup.
    *   **Meaning:** These are benign warnings from the NXP VPU driver. **Ignore them**, they do not affect the pipeline.
*   **"System Link: WARNING — EOS timeout after 10s"**: 
    *   **Where:** Server or Recorder console during shutdown.
    *   **Meaning:** GStreamer failed to finalize the MP4 file (usually due to a pipeline hang). The fallback safety timer engaged and forced the app to quit. The resulting `.mp4` file may be unplayable in standard players.
*   **Missing or Stuttering HUD on the Client**:
    *   **Where:** Client display window.
    *   **Meaning:** If the network is heavily congested, UDP packets may be dropped, causing GStreamer to discard incomplete video frames containing the SEI NAL units.
