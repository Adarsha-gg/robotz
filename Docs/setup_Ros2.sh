#!/bin/bash

# Unitree ROS2 Setup (ROS2 Jazzy already installed)
# Usage: bash setup_ros2.sh

set -e

echo "=== Unitree ROS2 Setup ==="
echo ""

# Make sure we're not in venv
unset VIRTUAL_ENV
unset PYTHONPATH
export PATH=$(echo $PATH | tr ':' '\n' | grep -v '/venv/' | tr '\n' ':')

# Install ROS2 dependencies
echo "[1/5] Installing ROS2 dependencies..."
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp libyaml-cpp-dev python3-empy

# Clone unitree_ros2 repo if not exists
echo "[2/5] Setting up unitree_ros2..."
cd ~/robot_cpp
if [ -d "unitree_ros2" ]; then
    echo "unitree_ros2 already exists"
else
    git clone https://github.com/unitreerobotics/unitree_ros2
fi

# Clean old build
echo "[3/5] Cleaning old build..."
cd ~/robot_cpp/unitree_ros2/cyclonedds_ws
rm -rf build install log

# Build unitree packages
echo "[4/5] Building unitree packages..."
source /opt/ros/jazzy/setup.bash
colcon build

# Build examples
echo "[5/5] Building examples..."
cd ~/robot_cpp/unitree_ros2/example
rm -rf build install log
source /opt/ros/jazzy/setup.bash
source ~/robot_cpp/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build

# Create setup scripts with enp0s8 interface
cd ~/robot_cpp/unitree_ros2

echo "#!/bin/bash
echo \"Setup unitree ros2 environment\"
source /opt/ros/jazzy/setup.bash
source \$HOME/robot_cpp/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name=\"enp0s8\" priority=\"default\" multicast=\"default\" />
                        </Interfaces></General></Domain></CycloneDDS>'" > setup.sh

echo "#!/bin/bash
echo \"Setup unitree ros2 environment (local)\"
source /opt/ros/jazzy/setup.bash
source \$HOME/robot_cpp/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name=\"lo\" priority=\"default\" multicast=\"default\" />
                        </Interfaces></General></Domain></CycloneDDS>'" > setup_local.sh

chmod +x setup.sh setup_local.sh

echo ""
echo "=== Setup Complete! ==="
echo ""
echo "Network interface set to: enp0s8"
echo ""
echo "To use (with robot connected):"
echo "  source ~/robot_cpp/unitree_ros2/setup.sh"
echo "  ros2 topic list"
echo ""
echo "To test locally (no robot):"
echo "  source ~/robot_cpp/unitree_ros2/setup_local.sh"
echo ""
echo "Run examples:"
echo "  cd ~/robot_cpp/unitree_ros2/example"
echo "  ./install/unitree_ros2_example/bin/read_motion_state"
echo ""