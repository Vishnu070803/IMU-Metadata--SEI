#!/bin/bash
# install_dependencies.sh
# 
# This script installs the necessary libraries and tools for:
# 1. The Server (Board side)
# 2. The Client & Extractor (PC side)

set -e

echo "--- SEI Video Pipeline: Dependency Installer ---"

# Detect if we are on a Debian/Ubuntu based system
if [ -f /etc/debian_version ]; then
    echo "[1/2] Updating package list..."
    sudo apt-get update

    echo "[2/2] Installing dependencies..."
    
    # Core GStreamer and Compilation tools
    sudo apt-get install -y \
        gcc \
        pkg-config \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-bad1.0-dev

    # Server specific (RTSP Server)
    sudo apt-get install -y \
        libgstrtspserver-1.0-dev \
        gstreamer1.0-rtsp

    # Client/Extractor specific (Cairo & Video Plugins)
    sudo apt-get install -y \
        libcairo2-dev \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-libav \
        gstreamer1.0-x

    echo ""
    echo "✅ Dependencies installed successfully."
else
    echo "❌ Error: This script is intended for Debian/Ubuntu-based systems."
    echo "If you are using a different distribution, please install the equivalents of:"
    echo "  - gstreamer-1.0-dev"
    echo "  - gstreamer-rtsp-server-1.0-dev"
    echo "  - libcairo2-dev"
    exit 1
fi
