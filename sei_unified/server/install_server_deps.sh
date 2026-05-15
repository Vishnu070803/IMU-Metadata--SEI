#!/bin/bash
# install_server_deps.sh
set -e

echo "Installing Server Dependencies..."
sudo apt-get update
sudo apt-get install -y \
    gcc \
    pkg-config \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstrtspserver-1.0-dev \
    gstreamer1.0-rtsp \
    gstreamer1.0-plugins-good
