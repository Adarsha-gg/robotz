# Unitree SDK Setup Guide

Complete setup guide for Unitree SDK2 (C++), Python SDK, and ROS2 support on Ubuntu Noble (24.04).

## Prerequisites
- Ubuntu 24.04 (Noble)
- Python >= 3.8
- ROS2 Jazzy

---

## 1. C++ SDK Setup

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

---

## 2. Python SDK Setup

**Important: Deactivate any Python virtual environments before building**

```bash
deactivate  # if in venv

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

### Using Python Examples

```bash
# Find your network interface
ip a

# Test DDS communication
python3 example/helloworld/publisher.py
python3 example/helloworld/subscriber.py

# Robot control (replace YOUR_INTERFACE with your interface name)
python3 example/high_level/read_highstate.py YOUR_INTERFACE
python3 example/high_level/sportmode_test.py YOUR_INTERFACE
python3 example/low_level/lowlevel_control.py YOUR_INTERFACE
python3 example/front_camera/camera_opencv.py YOUR_INTERFACE
```

---

## 3. ROS2 Setup

**Important: Deactivate any Python virtual environments before building**

```bash
deactivate  # if in venv

cd ~/robot_cpp

# Install ROS2 dependencies
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp libyaml-cpp-dev python3-empy

# Clone unitree_ros2
git clone https://github.com/unitreerobotics/unitree_ros2

# Clean any old builds
cd unitree_ros2/cyclonedds_ws
rm -rf build install log

# Build unitree packages
source /opt/ros/jazzy/setup.bash
colcon build

# Build examples
cd ~/robot_cpp/unitree_ros2/example
rm -rf build install log
source /opt/ros/jazzy/setup.bash
source ~/robot_cpp/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build
```

### Configure Network Interface

Create setup scripts:

```bash
cd ~/robot_cpp/unitree_ros2

# For robot connection (replace enp0s8 with your interface from 'ip a')
cat > setup.sh << 'EOF'
#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/jazzy/setup.bash
source $HOME/robot_cpp/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp0s8" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
EOF

# For local testing (no robot)
cat > setup_local.sh << 'EOF'
#!/bin/bash
echo "Setup unitree ros2 environment (local)"
source /opt/ros/jazzy/setup.bash
source $HOME/robot_cpp/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
EOF

chmod +x setup.sh setup_local.sh
```

### Using ROS2

```bash
# Test locally (no robot)
source ~/robot_cpp/unitree_ros2/setup_local.sh
ros2 topic list

# With robot connected
source ~/robot_cpp/unitree_ros2/setup.sh
ros2 topic list
ros2 topic echo /sportmodestate

# Run examples
cd ~/robot_cpp/unitree_ros2/example
./install/unitree_ros2_example/bin/read_motion_state
./install/unitree_ros2_example/bin/read_low_state
./install/unitree_ros2_example/bin/read_wireless_controller
```

---

## Network Configuration for Robot

1. Connect robot via Ethernet
2. Find interface: `ip a`
3. Set static IP:
   - IP: `192.168.123.99`
   - Netmask: `255.255.255.0`
4. Update interface name in setup scripts

---

## Project Structure

```
robot_cpp/
├── unitree_sdk2/              # C++ SDK
│   └── build/example/         # C++ examples
├── unitree_sdk2_python/       # Python SDK
│   └── example/               # Python examples
├── unitree_ros2/              # ROS2 support
│   ├── cyclonedds_ws/         # ROS2 packages
│   ├── example/               # ROS2 examples
│   ├── setup.sh               # Robot connection
│   └── setup_local.sh         # Local testing
└── venv/                      # Python virtual env (excluded from builds)
```

---

## Important Notes

- **Always deactivate Python virtual environments before building ROS2 or cyclonedds**
- Use `deactivate` command before running any ROS2 build commands
- ROS2 requires system Python, not venv Python

---