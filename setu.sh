#!/bin/bash

# Unitree SDK2 Python Setup Script for Ubuntu Noble (24.04)
# Simplified version - builds cyclonedds directly without iceoryx
# Usage: bash setup_python.sh

set -e

echo "=== Unitree SDK2 Python Setup (Ubuntu Noble) ==="
echo ""

# Install all required dependencies
echo "[1/3] Installing system dependencies..."
sudo apt install -y python3-pip cmake g++ build-essential git bison flex

# Install cyclonedds directly
echo "[2/3] Installing cyclonedds..."
cd ~
if [ -d "cyclonedds" ]; then
    echo "cyclonedds directory exists, removing..."
    rm -rf cyclonedds
fi

git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds
mkdir -p build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_SHARED_LIBS=ON
cmake --build . --target install

# Install Python dependencies and unitree_sdk2_python
echo "[3/3] Installing Python SDK..."
pip3 install numpy opencv-python --break-system-packages

cd ~/robot_cpp/unitree_sdk2_python
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
pip3 install -e . --break-system-packages

echo ""
echo "=== Python SDK Setup Complete! ==="
echo ""
echo "IMPORTANT: Add this to your ~/.bashrc:"
echo "export CYCLONEDDS_HOME=\"\$HOME/cyclonedds/install\""
echo ""
echo "Run this now:"
echo "echo 'export CYCLONEDDS_HOME=\"\$HOME/cyclonedds/install\"' >> ~/.bashrc"
echo "source ~/.bashrc"
echo ""
echo "Examples are in: unitree_sdk2_python/example/"
echo ""