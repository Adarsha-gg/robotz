# G1 Arm Actions & Gestures Reference

Quick reference for controlling G1 robot arm actions and body gestures.

---

## Two Types of Actions

### 1. Sport API Gestures (Work Over Network)
These use the standard sport service - should work from your laptop over network.

### 2. Arm Action API (May Need Jetson)  
G1-specific arm gestures - might need to run on Jetson (Error 3102 if service not available).

---

## Sport API Gestures

### Basic Setup

```bash
# Connect to robot network (enp0s8)
source ~/robot_cpp/unitree_ros2/setup.sh
ros2 topic list
```

### Command Format

```bash
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: YOUR_ID, api_id: API_ID}},
  parameter: 'PARAMETERS',
  binary: []
}"
```

### Available Gestures

#### Basic Control
```bash
# Stand up - API 1004
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 1, api_id: 1004}},
  parameter: '',
  binary: []
}"

# Stand down (lie down) - API 1005
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 2, api_id: 1005}},
  parameter: '',
  binary: []
}"

# Sit - API 1009
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 3, api_id: 1009}},
  parameter: '',
  binary: []
}"

# Rise from sit - API 1010
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 4, api_id: 1010}},
  parameter: '',
  binary: []
}"

# Stop moving - API 1003
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 5, api_id: 1003}},
  parameter: '',
  binary: []
}"

# Recovery stand - API 1006
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 6, api_id: 1006}},
  parameter: '',
  binary: []
}"
```

#### Fun Gestures
```bash
# Wave hello - API 1016
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 10, api_id: 1016}},
  parameter: '',
  binary: []
}"

# Stretch - API 1017
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 11, api_id: 1017}},
  parameter: '',
  binary: []
}"

# Heart gesture - API 1036
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 12, api_id: 1036}},
  parameter: '',
  binary: []
}"

# Dance 1 - API 1022
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 13, api_id: 1022}},
  parameter: '',
  binary: []
}"

# Dance 2 - API 1023
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 14, api_id: 1023}},
  parameter: '',
  binary: []
}"

# Scrape - API 1029
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 15, api_id: 1029}},
  parameter: '',
  binary: []
}"

# Content - API 1020
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 16, api_id: 1020}},
  parameter: '',
  binary: []
}"
```

#### Advanced Moves
```bash
# Front flip - API 1030
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 20, api_id: 1030}},
  parameter: '',
  binary: []
}"

# Front jump - API 1031
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 21, api_id: 1031}},
  parameter: '',
  binary: []
}"

# Front pounce - API 1032
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 22, api_id: 1032}},
  parameter: '',
  binary: []
}"

# Left flip - API 2041
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 23, api_id: 2041}},
  parameter: '',
  binary: []
}"

# Back flip - API 2043
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 24, api_id: 2043}},
  parameter: '',
  binary: []
}"
```

### Complete API ID List

```
Basic Control:
1001 - Damp
1002 - BalanceStand
1003 - StopMove
1004 - StandUp
1005 - StandDown
1006 - RecoveryStand
1009 - Sit
1010 - RiseSit

Gestures:
1016 - Hello (wave)
1017 - Stretch
1020 - Content
1022 - Dance1
1023 - Dance2
1029 - Scrape
1036 - Heart

Advanced:
1030 - FrontFlip
1031 - FrontJump
1032 - FrontPounce
2041 - LeftFlip
2043 - BackFlip
2044 - HandStand

Walking:
1061 - StaticWalk
1062 - TrotRun
1063 - EconomicGait
2045 - FreeWalk
2049 - ClassicWalk
2050 - WalkUpright
2051 - CrossStep
```

---

## Arm Action API (G1 Specific)

### Using Python SDK

```bash
cd ~/robot_cpp/unitree_sdk2_python/example/g1/high_level
python3 g1_arm_action_example.py enp0s8
```

### Using C++ (on Jetson)

```bash
# SSH into Jetson
ssh unitree@192.168.123.161

# Navigate to examples
cd ~/unitree_sdk2/build/example
./g1_arm_action_example
```

### Common Arm Action Errors

#### Error 3102: Service Not Available
**Cause**: Arm action service not running or not accessible from laptop

**Solution**: 
- Run code on Jetson via SSH
- Or use Sport API gestures instead

#### Error 7404/7303: Invalid FSM State
**Cause**: Robot not in correct FSM state for arm actions

**Solution**: Set FSM to 500, 501, or 801
```bash
# Set FSM to 500
ros2 topic pub --once /api/loco/request unitree_api/msg/Request "{
  header: {identity: {id: 100, api_id: 7101}},
  parameter: '{\"data\": 500}',
  binary: []
}"
```

#### Error 7303: Action Not Supported in Current FSM
**Cause**: Some arm actions only work in specific FSM states

**Solutions**:
- Try FSM 500 for basic arm actions
- Try FSM 501 for walking + arm actions  
- Try FSM 801 for advanced modes

---

## Quick Test Script

Save this as `test_gestures.sh`:

```bash
#!/bin/bash

echo "Testing G1 Gestures"
source ~/robot_cpp/unitree_ros2/setup.sh

# Stand up
echo "Standing up..."
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 1, api_id: 1004}},
  parameter: '',
  binary: []
}"
sleep 3

# Wave hello
echo "Waving hello..."
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 2, api_id: 1016}},
  parameter: '',
  binary: []
}"
sleep 3

# Heart gesture
echo "Making heart..."
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 3, api_id: 1036}},
  parameter: '',
  binary: []
}"
sleep 3

# Sit down
echo "Sitting down..."
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 4, api_id: 1009}},
  parameter: '',
  binary: []
}"

echo "Done!"
```

Run with: `bash test_gestures.sh`

---

## Troubleshooting

### Gestures Not Working?

1. **Check robot state:**
```bash
ros2 topic echo /sportmodestate
```

2. **Check if topics exist:**
```bash
ros2 topic list | grep api
```

3. **Try emergency stop first:**
```bash
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{
  header: {identity: {id: 999, api_id: 1003}},
  parameter: '',
  binary: []
}"
```

4. **Make sure network is correct:**
```bash
ping 192.168.123.161
export | grep CYCLONEDDS
```

### If Sport API Works but Arm Actions Don't

This is normal! Arm actions need special service that might only be available on Jetson.

**Options:**
1. Use Sport API gestures (they work fine)
2. SSH into Jetson and run arm action code there
3. Check if arm service is running: `ros2 service list`

---

## Notes

- **Sport API gestures** are safe and work over network
- **Arm Action API** might require running code on Jetson
- Always have emergency stop ready (API 1003)
- Test in safe open area
- Start with simple gestures before trying flips

---

## When You Get the Robot

**Day 1 Checklist:**
- [ ] Connect to network (enp0s8)
- [ ] Source setup: `source ~/robot_cpp/unitree_ros2/setup.sh`
- [ ] Test topics: `ros2 topic list`
- [ ] Stand up: API 1004
- [ ] Sit down: API 1009
- [ ] Wave hello: API 1016
- [ ] Emergency stop: API 1003

**If something goes wrong:** API 1003 (StopMove) is your friend!

---

## Quick Reference

```bash
# Source environment
source ~/robot_cpp/unitree_ros2/setup.sh

# Emergency stop
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{header: {identity: {id: 999, api_id: 1003}}, parameter: '', binary: []}"

# Stand up
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{header: {identity: {id: 1, api_id: 1004}}, parameter: '', binary: []}"

# Sit down  
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{header: {identity: {id: 2, api_id: 1009}}, parameter: '', binary: []}"

# Wave
ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{header: {identity: {id: 3, api_id: 1016}}, parameter: '', binary: []}"
```

Good luck! 🤖✋