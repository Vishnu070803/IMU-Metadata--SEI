#!/bin/bash
# install_client_deps.sh
set -e

echo "Installing Client Dependencies..."
sudo apt-get update
sudo apt-get install -y \
    gcc \
    pkg-config \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libcairo2-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-libav \
    gstreamer1.0-x
