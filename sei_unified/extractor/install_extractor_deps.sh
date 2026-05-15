#!/bin/bash
# install_extractor_deps.sh
set -e

echo "Installing Extractor Dependencies..."
sudo apt-get update
sudo apt-get install -y \
    gcc \
    pkg-config \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libcairo2-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-libav
