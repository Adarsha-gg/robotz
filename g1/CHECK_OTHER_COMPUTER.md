# Things to Check on Working Computer

Run these commands on your OTHER computer (where it works) and compare:

## 1. ROS2 Environment Variables
```bash
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI: $CYCLONEDDS_URI"
env | grep ROS
env | grep DDS
```

## 2. Network Interface
```bash
# What network interface is being used?
ip addr show
ifconfig
# Check which interface connects to the robot
```

## 3. Which Setup Script Are You Using?
```bash
# Check your .bashrc or startup script
cat ~/.bashrc | grep -E "(source|unitree|ros)"
# What setup.sh are you sourcing?
```

## 4. Robot Network Configuration
```bash
# Is there a robot IP configured anywhere?
grep -r "192.168" ~/robot_cpp/g1/
cat /etc/hosts | grep -i unitree
# Can you ping the robot?
ping <robot_ip>
```

## 5. Python Venv Packages
```bash
source ~/robot_cpp/venv/bin/activate  # or wherever your venv is
pip list | grep -E "(rclpy|unitree|groq|cyclone)"
python3 --version
```

## 6. CycloneDDS Config Files
```bash
# Check for any DDS config files
find ~ -name "cyclonedds.xml" 2>/dev/null
find ~ -name "*dds*.xml" 2>/dev/null
cat $CYCLONEDDS_URI 2>/dev/null || echo "Not a file"
```

## 7. ROS2 Topics When Working
```bash
# When the voice node is running successfully:
ros2 topic list
ros2 topic echo /audio_msg
ros2 topic info /api/voice/request
```

## 8. Are You SSH'd Into the Robot?
```bash
hostname  # Is this the robot itself or your laptop?
uname -a
# Are you running ON the robot or connecting TO the robot?
```

## 9. Git Config (if it's a repo)
```bash
cd ~/robot_cpp/g1
git remote -v  # Any remote repos?
git log -1     # Last commit info
git diff       # Any local changes?
```

## 10. Startup Command
```bash
# What exact command do you run to start it?
history | grep "python.*main.py"
# Check if there's an alias or script
```

## Most Likely Issues:
- **ROS_DOMAIN_ID mismatch** (robot expects 1, you have 0)
- **Network interface** (setup_local.sh uses 'lo', should use actual ethernet/wifi)
- **Not SSH'd into robot** (running locally when should be on robot)
- **Robot's ROS2 services not running** (need to start robot's software first)
