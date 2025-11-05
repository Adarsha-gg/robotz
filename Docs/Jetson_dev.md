# G1 Jetson Orin Setup Guide

Guide for developing directly on the G1's internal Jetson Orin computer via SSH.

## Initial Connection

### 1. Connect to Robot Network

Connect your laptop to the G1's network (Ethernet or WiFi).

### 2. Find Jetson IP Address

The Jetson IP is typically: `192.168.123.164` or check robot documentation.

### 3. SSH Into Jetson

```bash
ssh unitree@192.168.123.164
# Password: typically '123' or check documentation
```

## First Time Setup on Jetson

Once SSH'd in, run these commands:

### 1. Check What's Already Installed

```bash
# Check ROS2 version
ls /opt/ros
# Should show: foxy

# Check if unitree SDK is installed
ls ~/

# Check current ROS2 topics (robot should be running)
source /opt/ros/foxy/setup.bash
ros2 topic list
```

### 2. Install Development Tools

```bash
# Update system
sudo apt update

# Install build tools
sudo apt install -y git cmake build-essential python3-pip

```

### 3. Set Up Your Workspace

```bash
# Create workspace
mkdir -p ~/dev/robot_code
cd ~/dev

# Clone your repos (when ready)
# git clone <your-repo>
```

### 4. Configure ROS2 Environment

Add to `~/.bashrc`:

```bash
# Add these lines
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

## Using VS Code Remote SSH (Recommended)

### On Your Laptop:

1. Install VS Code
2. Install "Remote - SSH" extension
3. Press F1 → "Remote-SSH: Connect to Host"
4. Enter: `unitree@192.168.123.161`
5. Enter password
6. Open folder: `/home/unitree/dev/robot_code`

Now you code in VS Code on your laptop, but everything runs on the Jetson!

## Development Workflow

### Option 1: Develop Directly on Jetson

```bash
# SSH in
ssh unitree@192.168.123.161

# Navigate to your code
cd ~/dev/robot_code

# Edit code
vim my_program.cpp  # or use VS Code Remote

# Build
mkdir build && cd build
cmake ..
make

# Run
./my_program
```

### Option 2: Use ROS2 Packages

```bash
# Create ROS2 workspace on Jetson
cd ~/dev
mkdir -p ros2_ws/src
cd ros2_ws/src

# Create your package
ros2 pkg create --build-type ament_cmake my_robot_controller

# Build
cd ~/dev/ros2_ws
colcon build

# Source and run
source install/setup.bash
ros2 run my_robot_controller my_node
```

## Copying Files from Laptop to Jetson

If you want to develop on laptop and copy to Jetson:

```bash
# From your laptop
scp -r my_project/ unitree@192.168.123.161:~/dev/

# Or use rsync
rsync -avz my_project/ unitree@192.168.123.161:~/dev/my_project/
```

## Important Notes

### SDK Locations on Jetson

The G1 should already have:
- Unitree SDK2 installed
- ROS2 Foxy with unitree packages
- All necessary drivers

Check these locations:
```bash
ls /opt/ros/foxy/share/ | grep unitree
ls ~/unitree_sdk2 # or wherever it's installed
```

### Network Configuration

The Jetson uses interface: **typically `eth0` or `enp0s0`**

To check:
```bash
ip a
```

### Running Code at Startup

To run your code automatically when robot boots:

1. Create systemd service:
```bash
sudo nano /etc/systemd/system/my_robot_code.service
```

2. Add:
```ini
[Unit]
Description=My Robot Code
After=network.target

[Service]
Type=simple
User=unitree
WorkingDirectory=/home/unitree/dev/robot_code
ExecStart=/home/unitree/dev/robot_code/build/my_program
Restart=always

[Install]
WantedBy=multi-user.target
```

3. Enable:
```bash
sudo systemctl enable my_robot_code.service
sudo systemctl start my_robot_code.service
```

## Testing Without Robot Movement

```bash
# Read sensor data only
ros2 topic echo /lowstate
ros2 topic echo /sportmodestate
ros2 topic echo /imu

# Don't send commands until you're ready!
```

## Useful Commands

```bash
# Check running ROS2 nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor topic
ros2 topic echo /topic_name

# Check CPU/GPU usage on Jetson
htop
tegrastats  # Jetson-specific monitoring

# Check disk space
df -h
```

## Safety Notes

⚠️ **IMPORTANT:**
- Always have emergency stop ready
- Test in safe open area first
- Start with reading data before sending commands
- The robot will respond to control commands immediately
- Make sure sport mode is off when testing low-level control

## Troubleshooting

### Can't SSH In
- Check robot is powered on
- Verify network connection
- Try default IP: `192.168.123.161`
- Check robot documentation for correct IP/password

### ROS2 Topics Not Showing
```bash
# Make sure ROS2 environment is sourced
source /opt/ros/foxy/setup.bash

# Check if robot's ROS2 nodes are running
ros2 node list
```

### Permission Denied
```bash
# Add your user to necessary groups
sudo usermod -aG dialout $USER
sudo usermod -aG video $USER
# Logout and login again
```

## Next Steps When You Get the Robot

1. ✅ SSH into Jetson
2. ✅ Verify ROS2 Foxy is working
3. ✅ Check what's already installed
4. ✅ Set up your development environment
5. ✅ Test reading sensor data
6. ✅ Write and test your first program
7. ✅ Gradually add control features

---

**You're ready to code on the Jetson when the robot arrives!**