#!/usr/bin/env python3
"""
Standalone Balance Script for Unitree G1 with State Monitoring
Checks FSM mode to ensure proper balance
"""

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState
import json
import time


class BalanceScriptWithMonitoring(Node):
    def __init__(self):
        super().__init__('balance_script')
        
        # Publisher for loco commands
        self.loco_pub = self.create_publisher(
            Request,
            '/api/loco/request',
            10
        )
        
        # Subscribe to robot state
        self.mode_state = None
        self.create_subscription(
            SportModeState,
            '/lf/sportmodestate',
            self.mode_callback,
            10
        )
        
        # Subscribe to loco responses
        self.create_subscription(
            Response,
            '/api/loco/response',
            self.response_callback,
            10
        )
        
        self.current_fsm = None
        
        # Wait for connections
        time.sleep(2)
        
        print("\n" + "="*60)
        print("🤖 UNITREE G1 HANGER BOOT SEQUENCE (WITH STATE MONITORING)")
        print("="*60)
        print("Proper sequence: Damp → Stand-up → Height sweep → Balance → Start")
        print("="*60 + "\n")
        
        # Run the sequence
        self.run_boot_sequence()
    
    def mode_callback(self, msg):
        """Monitor robot's FSM mode"""
        self.mode_state = msg.mode
        # Mode meanings:
        # 0 = Feet loaded, static stand
        # 1 = Feet loaded, dynamic/gait active
        # 2 = Feet unloaded (hanging)
    
    def response_callback(self, msg):
        """Monitor responses from loco API"""
        try:
            if msg.parameter:
                data = json.loads(msg.parameter)
                if 'data' in data:
                    self.current_fsm = data.get('data')
        except:
            pass
    
    def _create_request(self, api_id, parameter):
        """Create a Request message"""
        request = Request()
        request.header.identity.id = int(time.time() * 1000000)
        request.header.identity.api_id = api_id
        request.header.lease.id = 0
        request.header.policy.priority = 0
        request.header.policy.noreply = False
        request.parameter = parameter
        request.binary = []
        return request
    
    def set_fsm_id(self, fsm_id):
        """Set FSM state"""
        request = self._create_request(
            7101,  # API_SET_FSM_ID
            json.dumps({"data": fsm_id})
        )
        self.loco_pub.publish(request)
        self.current_fsm = fsm_id
        print(f"   → FSM ID set to {fsm_id}")
    
    def set_stand_height(self, height):
        """Set standing height"""
        request = self._create_request(
            7104,  # API_SET_STAND_HEIGHT
            json.dumps({"data": height})
        )
        self.loco_pub.publish(request)
    
    def set_balance_mode(self, mode):
        """Set balance mode (0=static, 1=continuous gait)"""
        request = self._create_request(
            7102,  # API_SET_BALANCE_MODE
            json.dumps({"data": mode})
        )
        self.loco_pub.publish(request)
    
    def wait_for_mode(self, target_mode, timeout=5):
        """Wait for robot to reach specific mode"""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.mode_state == target_mode:
                return True
        return False
    
    def run_boot_sequence(self):
        """Execute the full boot sequence with monitoring"""
        
        # Step 1: Damp mode (FSM 1)
        print("📍 Step 1: Damp mode (FSM 1)...")
        self.set_fsm_id(1)
        time.sleep(2)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f"   → Current mode: {self.mode_state}")
        
        # Step 2: Stand-up (FSM 4)
        print("\n📍 Step 2: Stand-up command (FSM 4)...")
        self.set_fsm_id(4)
        time.sleep(2)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f"   → Current mode: {self.mode_state}")
        
        # Step 3: Height sweep
        print("\n📏 Step 3: Height sweep (extending legs)...")
        print("   Watching for feet to touch ground (mode 2 → 0)...")
        
        max_height = 0.5
        step = 0.02
        height = 0.0
        
        while height < max_height:
            height += step
            self.set_stand_height(height)
            
            # Spin to get updates
            for _ in range(2):
                rclpy.spin_once(self, timeout_sec=0.05)
            
            # Show progress every 5 steps
            if int(height / step) % 5 == 0:
                mode_str = "loaded" if self.mode_state == 0 else "unloaded" if self.mode_state == 2 else "dynamic"
                print(f"   → Height: {height:.2f} m, mode: {self.mode_state} ({mode_str})")
            
            # Break early if feet touch ground
            if self.mode_state == 0 and height > 0.2:
                print(f"   ✅ Feet loaded at {height:.2f} m!")
                break
        
        if self.mode_state != 0:
            print(f"\n   ⚠️  WARNING: Feet still unloaded (mode={self.mode_state})")
            print(f"   Robot might be hanging or hanger too high/low")
            print(f"   Current height: {height:.2f} m\n")
        
        time.sleep(1)
        
        # Step 4: Balance stand (mode 0)
        print("\n⚖️  Step 4: Balance stand (mode 0)...")
        self.set_balance_mode(0)
        time.sleep(1)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f"   → Balance mode set, current mode: {self.mode_state}")
        
        # Step 5: Start balance controller (FSM 200)
        print("\n🚀 Step 5: Starting balance controller (FSM 200)...")
        self.set_fsm_id(200)
        time.sleep(2)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        print("\n" + "="*60)
        if self.mode_state == 0 or self.mode_state == 1:
            print("✅ ROBOT IS IN BALANCED STAND!")
            print(f"   FSM: 200, Mode: {self.mode_state} (feet loaded)")
        else:
            print("⚠️  BALANCE SEQUENCE COMPLETE BUT CHECK ROBOT STATE")
            print(f"   FSM: 200, Mode: {self.mode_state}")
            if self.mode_state == 2:
                print("   Mode 2 = Feet unloaded (might still be hanging)")
        print("="*60)
        print("\nYou can now:")
        print("  - Try gestures with voice commands")
        print("  - Press Ctrl+C to exit this script")
        print("="*60 + "\n")


def main():
    rclpy.init()
    
    try:
        node = BalanceScriptWithMonitoring()
        
        print("⏳ Monitoring robot state... (Press Ctrl+C to exit)\n")
        
        # Keep spinning to monitor state
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            
    except KeyboardInterrupt:
        print("\n⚠️  Exiting balance script")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()