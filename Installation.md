# Unitree SDK Setup Guide

Setup guide for Unitree SDK2 (C++) and Python SDK on Ubuntu Noble (24.04).

## Prerequisites
- Ubuntu 24.04 (Noble)
- Python >= 3.8

## C++ SDK Setup

```bash
cd ~/robot_cpp/unitree_sdk2

# Install dependencies
sudo apt-get update
sudo apt-get install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)

# Optional: Install system-wide
sudo make install
```

Examples are in: `unitree_sdk2/build/example/`

## Python SDK Setup

```bash
cd ~/robot_cpp/unitree_sdk2_python

# Install dependencies
sudo apt install -y python3-pip cmake g++ build-essential git bison flex

# Build cyclonedds
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds
mkdir -p build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_SHARED_LIBS=ON
cmake --build . --target install

# Install Python packages
pip3 install numpy opencv-python --break-system-packages

# Install unitree_sdk2_python
cd ~/robot_cpp/unitree_sdk2_python
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
pip3 install -e . --break-system-packages

# Add to bashrc
echo 'export CYCLONEDDS_HOME="$HOME/cyclonedds/install"' >> ~/.bashrc
source ~/.bashrc
```

Examples are in: `unitree_sdk2_python/example/`

## Using Python Examples

First, find your network interface:
```bash
ip a
```

Then run examples (replace `YOUR_INTERFACE` with your interface name like `eth0` or `enp2s0`):

```bash
# Test DDS communication
python3 example/helloworld/publisher.py
python3 example/helloworld/subscriber.py

# Robot control (requires robot connection)
python3 example/high_level/read_highstate.py YOUR_INTERFACE
python3 example/high_level/sportmode_test.py YOUR_INTERFACE
python3 example/low_level/lowlevel_control.py YOUR_INTERFACE
python3 example/front_camera/camera_opencv.py YOUR_INTERFACE
```
