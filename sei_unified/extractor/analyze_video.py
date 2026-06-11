#!/usr/bin/env python3
import subprocess
import sys
import os

def analyze_video(filepath):
    if not os.path.exists(filepath):
        print(f"Error: File '{filepath}' does not exist.")
        sys.exit(1)

    print(f"Analyzing video: {filepath}")
    print("Extracting frame types (this may take a moment for large files)...")

    # Command to get picture type of every frame in the video stream
    # This reads the actual frames, ignoring container metadata that might lie
    cmd_frames = [
        "ffprobe",
        "-v", "error",
        "-select_streams", "v:0",
        "-show_entries", "frame=pict_type",
        "-of", "csv=p=0",
        filepath
    ]

    try:
        output = subprocess.check_output(cmd_frames, universal_newlines=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running ffprobe: {e}")
        sys.exit(1)
    except FileNotFoundError:
        print("Error: 'ffprobe' is not installed or not in PATH. Please install ffmpeg.")
        sys.exit(1)

    # Process frame types
    frames = output.strip().split('\n')
    frames = [f.strip() for f in frames if f.strip()]

    total_frames = len(frames)
    if total_frames == 0:
        print("No frames found in video.")
        sys.exit(0)

    count_I = frames.count('I')
    count_P = frames.count('P')
    count_B = frames.count('B')
    
    # Calculate GOP
    # Find indices of all I frames
    i_indices = [idx for idx, frame in enumerate(frames) if frame == 'I']
    
    gop_sizes = []
    for j in range(1, len(i_indices)):
        gop_sizes.append(i_indices[j] - i_indices[j-1])
    
    avg_gop = sum(gop_sizes) / len(gop_sizes) if gop_sizes else 0
    max_gop = max(gop_sizes) if gop_sizes else total_frames
    
    import json
    cmd_format = [
        "ffprobe",
        "-v", "error",
        "-print_format", "json",
        "-show_format",
        "-show_streams",
        filepath
    ]
    
    duration = 0.0
    bitrate = 0
    
    try:
        format_output = subprocess.check_output(cmd_format, universal_newlines=True)
        probe_data = json.loads(format_output)
        
        # Try to get duration
        if 'format' in probe_data and 'duration' in probe_data['format']:
            duration = float(probe_data['format']['duration'])
        elif 'streams' in probe_data and len(probe_data['streams']) > 0:
            stream = probe_data['streams'][0]
            if 'duration' in stream:
                duration = float(stream['duration'])
                
        # Try to get bitrate
        if 'format' in probe_data and 'bit_rate' in probe_data['format']:
            bitrate = int(probe_data['format']['bit_rate'])
        elif 'streams' in probe_data and len(probe_data['streams']) > 0:
            stream = probe_data['streams'][0]
            if 'bit_rate' in stream:
                bitrate = int(stream['bit_rate'])
                
    except Exception as e:
        print(f"Warning: Could not parse format info: {e}")

    # If bitrate metadata is missing (common in raw streams or fragmented MP4s), calculate it manually
    if bitrate == 0 and duration > 0:
        file_size_bits = os.path.getsize(filepath) * 8
        bitrate = int(file_size_bits / duration)

    # Count Custom SEI IMU Metadata
    # We search the binary file for the specific 16-byte UUID used in the C code
    # This avoids relying on ffprobe which might ignore unrecognized SEI user data.
    IMU_SEI_UUID = b'\xdc\x45\xe9\xbd\xe6\xd9\x48\xb7\x96\x2c\xd3\x9c\x24\x5a\xc8\x1a'
    try:
        with open(filepath, 'rb') as f:
            file_data = f.read()
            sei_count = file_data.count(IMU_SEI_UUID)
    except Exception as e:
        sei_count = 0
        print(f"Warning: Could not read binary file for SEI check: {e}")

    # Print Results
    print("\n" + "="*40)
    print(" VIDEO ANALYSIS RESULTS")
    print("="*40)
    print(f"Total Frames : {total_frames}")
    if duration > 0:
        print(f"Avg FPS      : {total_frames / duration:.2f} fps")
    print(f"Duration     : {duration:.2f} seconds")
    print(f"Bitrate      : {bitrate / 1000000:.2f} Mbps ({bitrate} bps)")
    print("-" * 40)
    print("FRAME TYPES (Actual Stream Scan)")
    print(f"  I-Frames   : {count_I}")
    print(f"  P-Frames   : {count_P}")
    print(f"  B-Frames   : {count_B}")
    
    if count_B > 0:
        print("  => Video CONTAINS B-frames!")
    else:
        print("  => Video DOES NOT contain B-frames.")
        
    print("-" * 40)
    print("GOP (Group of Pictures) STRUCTURE")
    if len(i_indices) == 0:
        print("  No I-frames found (Corrupted?)")
    elif len(i_indices) == 1:
        print(f"  Only 1 I-frame found. GOP size is {total_frames} (All-Intra or endless P)")
    else:
        print(f"  Average GOP size : {avg_gop:.1f} frames")
        print(f"  Maximum GOP size : {max_gop} frames")
        
        # Determine actual pattern of first GOP
        if len(i_indices) > 1:
            first_gop = frames[i_indices[0]:i_indices[1]]
            # Truncate pattern printing if it's too long
            pattern_str = "".join(first_gop)
            if len(pattern_str) > 60:
                pattern_str = pattern_str[:57] + "..."
            print(f"  First GOP Pattern: {pattern_str}")
            
    print("-" * 40)
    print("CUSTOM SEI METADATA (IMU Data)")
    print(f"  SEI Packets: {sei_count}")
    if sei_count > 0:
        print("  => Video CONTAINS embedded IMU metadata.")
        if total_frames > 0:
            avg_sei = sei_count / total_frames
            print(f"  => Average : {avg_sei:.2f} IMU samples per video frame.")
    else:
        print("  => NORMAL VIDEO: No embedded IMU metadata found.")
        
    print("="*40 + "\n")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_video.py <video_file.mp4>")
        sys.exit(1)
        
    analyze_video(sys.argv[1])
