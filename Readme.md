# G1 Robot Control System

Custom voice control system for Unitree G1 humanoid robot using ROS2 Jazzy, GPT integration, and English TTS.

## Features

- Voice-controlled commands via robot microphone
- GPT-powered conversational AI
- English TTS using Piper
- Advanced LED patterns and animations
- Modular Python architecture

## Requirements

- Ubuntu 24.04 LTS
- ROS2 Jazzy
- Python 3.12+
- Ethernet connection to G1 robot

## Installation

### 1. Install ROS2 Jazzy
```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rosidl-generator-dds-idl
```

### 2. Clone Repository
```bash
git clone --recurse-submodules https://github.com/YOUR_USERNAME/robot_cpp.git
cd robot_cpp
```

### 3. Build Unitree ROS2 Packages
```bash
cd unitree_ros2/cyclonedds_ws/src
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x

cd rmw_cyclonedds
git checkout jazzy
cd ../..

source /opt/ros/jazzy/setup.bash
colcon build

cd ../example
source /opt/ros/jazzy/setup.bash
source ../cyclonedds_ws/install/setup.bash
colcon build
```

### 4. Install Python Dependencies
```bash
cd g1
python3 -m venv venv
source venv/bin/activate
pip install groq python-dotenv piper-tts
sudo apt install ffmpeg
```

### 5. Download Piper Voice Model
```bash
mkdir -p ~/piper_voices
cd ~/piper_voices
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json
```

### 6. Configure Environment

Create `.env` file in `g1/` directory:
```
GROQ_API_KEY=your_api_key_here
```

### 7. Configure Network

Edit `unitree_ros2/setup.sh` and `unitree_ros2/setup_local.sh`:
- Change `foxy` to `jazzy`
- Set your network interface name in setup.sh
- Use `lo` interface in setup_local.sh

## Project Structure
```
robot_cpp/
├── g1/                      # Main control code
│   ├── main.py
│   ├── .env
│   └── audio/               # Voice, TTS, LED control modules
├── unitree_ros2/            # Unitree ROS2 SDK
└── unitree_sdk2/            # Unitree SDK2
```

## Usage

### Connect to Robot
```bash
sudo ip addr add 192.168.123.99/24 dev YOUR_INTERFACE
sudo ip link set YOUR_INTERFACE up
ping 192.168.123.164
```

### Run System
```bash
cd ~/robot_cpp/g1
source ../unitree_ros2/setup.sh
python3 main.py
```

### Voice Commands

- **LED:** "rainbow", "breathe", "pulse", "party mode", "lights red/blue/off"
- **Volume:** "volume up/down", "set volume to 50"
- **Chat:** Any other text goes to GPT

### Test Without Voice
```bash
ros2 topic pub --once /gpt_command std_msgs/msg/String "data: 'rainbow'"
```

## Troubleshooting

**No robot topics?**
- Check connection: `ping 192.168.123.164`
- Use fresh terminal, source setup.sh again
- Verify: `ros2 topic list`

**Import errors?**
- Activate venv: `source venv/bin/activate`
- Reinstall: `pip install groq python-dotenv piper-tts`

**Piper not found?**
- Check: `piper --help`
- Reinstall: `pip install --force-reinstall piper-tts`

## API Reference

### Known API IDs
- 1001: TTS
- 1003: Play Audio
- 1004: Stop Audio
- 1005: Get Volume
- 1006: Set Volume
- 1010: LED Control

### Audio Format Requirements
- Sample rate: 16kHz
- Channels: Mono (1 channel)
- Format: 16-bit PCM WAV

## Development

Add features in `g1/audio/`, update `__init__.py`, integrate in `main.py` or `command_parser.py`.

## License

MIT License