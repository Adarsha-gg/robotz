# Unitree G1 MuJoCo Dance Simulator - Setup Guide

## Overview
Complete guide for setting up the Unitree MuJoCo simulator with the G1 humanoid robot for motion capture-based dancing.

---

## How to Build

### Step 1: Create Project Structure
```bash
mkdir -p ~/robot_cpp
cd ~/robot_cpp
python3 -m venv venv
source venv/bin/activate
```

### Step 2: Clone Repositories
```bash
cd ~/robot_cpp
git clone https://github.com/unitreerobotics/unitree_mujoco.git
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
```

### Step 3: Install MuJoCo and Dependencies
```bash
pip install mujoco numpy matplotlib pygame
```

### Step 4: Build and Install CycloneDDS
```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds
cd cyclonedds
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/cyclonedds/install -DENABLE_SHM=OFF
cmake --build . --target install
```

### Step 5: Set Environment Variable
```bash
export CYCLONEDDS_HOME=$HOME/cyclonedds/install
echo "export CYCLONEDDS_HOME=$HOME/cyclonedds/install" >> ~/.bashrc
source ~/.bashrc
```

### Step 6: Install Unitree SDK2 Python
```bash
cd ~/robot_cpp/unitree_sdk2_python
source ~/robot_cpp/venv/bin/activate
pip install -e .
```

### Step 7: Configure for G1 Robot
```bash
cd ~/robot_cpp/unitree_mujoco/simulate_python
nano config.py
```

Change these lines:
```python
ROBOT = "g1"  # Changed from "go2"
USE_JOYSTICK = 0  # Disable gamepad requirement
```

### Step 8: Run Simulator
```bash
cd ~/robot_cpp/unitree_mujoco/simulate_python
source ~/robot_cpp/venv/bin/activate
python3 unitree_mujoco.py
```

---

## Problems / Troubleshooting

### Error 1: CycloneDDS Library Not Found
```
CycloneDDSLoaderException: Failed to load CycloneDDS library from /home/pass/cyclonedds/install/lib/libddsc.so
```

**Fix:**
Rebuild CycloneDDS properly:
```bash
cd ~/cyclonedds
rm -rf build install
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/cyclonedds/install -DENABLE_SHM=OFF
cmake --build . --target install
export CYCLONEDDS_HOME=$HOME/cyclonedds/install
```

### Error 2: Missing Iceoryx Library
```
OSError: libiceoryx_binding_c.so: cannot open shared object file: No such file or directory
```

**Fix:**
This happens when CycloneDDS is built with shared memory support. Rebuild with SHM disabled:
```bash
cd ~/cyclonedds
rm -rf build install
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/cyclonedds/install -DENABLE_SHM=OFF
cmake --build . --target install
```

The key flag is `-DENABLE_SHM=OFF` which removes the Iceoryx dependency.

### Error 3: Wrong Robot Loaded (Go2 instead of G1)
**Symptom:** Quadruped Go2 robot appears instead of humanoid G1.

**Fix:**
Edit `/home/pass/robot_cpp/unitree_mujoco/simulate_python/config.py`:
```python
ROBOT = "g1"  # Change from "go2"
```

### Error 4: Multiple SDK Installations Conflict
**Symptom:** Import errors or module not found issues.

**Fix:**
Use single editable installation:
```bash
source ~/robot_cpp/venv/bin/activate
pip uninstall unitree-sdk2py -y
cd ~/robot_cpp/unitree_sdk2_python
pip install -e .
```

### Warning: Display/Wayland Messages (Not Errors)
```
libdecor-gtk-WARNING: Failed to initialize GTK
GLFWError: Wayland: The platform does not provide the window position
```

**Fix:**
These are harmless warnings on Wayland systems. Ignore them or use X11:
```bash
export GDK_BACKEND=x11
python3 unitree_mujoco.py
```

### Warning: "No gamepad detected" (Not an Error)
**Fix:**
Set `USE_JOYSTICK = 0` in `config.py`. Gamepad is optional.

---

## Verification Commands
```bash
# Check CycloneDDS installation
ls -la ~/cyclonedds/install/lib/libddsc.so

# Verify environment variable
echo $CYCLONEDDS_HOME

# Test SDK import
python -c "import unitree_sdk2py; print('SDK OK')"

# Run simulator
cd ~/robot_cpp/unitree_mujoco/simulate_python && source ~/robot_cpp/venv/bin/activate && python3 unitree_mujoco.py
```

---

## Directory Structure After Setup
```
~/robot_cpp/
├── venv/
├── unitree_mujoco/
│   ├── simulate_python/
│   │   ├── config.py (edited for G1)
│   │   └── unitree_mujoco.py
│   └── unitree_robots/
│       └── g1/
└── unitree_sdk2_python/

~/cyclonedds/
├── build/
└── install/
    └── lib/
        └── libddsc.so
```

---

## Summary of Issues Fixed

| Issue | Root Cause | Solution |
|-------|------------|----------|
| CycloneDDS not loading | Library missing/wrong path | Built from source with correct prefix |
| Iceoryx dependency error | SHM enabled by default | Rebuilt with `-DENABLE_SHM=OFF` |
| Wrong robot (Go2 vs G1) | Default config for Go2 | Changed `ROBOT = "g1"` |
| SDK conflicts | Multiple installations | Single editable install |

---

**Status: ✅ G1 Robot Successfully Loaded in MuJoCo Simulator**

Ready for dance programming!