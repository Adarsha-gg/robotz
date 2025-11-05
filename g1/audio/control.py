#!/usr/bin/env python3
"""
Monitor what happens when controller buttons are pressed
"""
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response
from std_msgs.msg import String
from unitree_go.msg import WirelessController
import json
import time

class ControllerMonitor(Node):
    def __init__(self):
        super().__init__('controller_monitor')
        
        # Monitor controller inputs
        self.create_subscription(
            WirelessController, 
            '/wirelesscontroller', 
            self.controller_cb, 
            10
        )
        
        # Monitor voice API
        self.create_subscription(Request, '/api/voice/request', self.voice_req_cb, 10)
        self.create_subscription(Response, '/api/voice/response', self.voice_res_cb, 10)
        
        # Monitor GPT API
        self.create_subscription(Request, '/api/gpt/request', self.gpt_req_cb, 10)
        self.create_subscription(Response, '/api/gpt/response', self.gpt_res_cb, 10)
        
        # Monitor GPT state and commands
        self.create_subscription(String, '/gpt_state', self.state_cb, 10)
        self.create_subscription(String, '/gpt_cmd', self.cmd_cb, 10)
        
        # Monitor audio
        self.create_subscription(String, '/audio_msg', self.audio_cb, 10)
        
        print("\n" + "=" * 70)
        print("🎮 CONTROLLER BUTTON MONITOR")
        print("=" * 70)
        print("Press and HOLD the mic buttons on controller, then speak")
        print("Release the buttons when done")
        print("=" * 70 + "\n")
        
        self.last_controller_state = None
    
    def controller_cb(self, msg):
        # Print any button changes
        state = str(msg)
        if state != self.last_controller_state:
            print(f"\n🎮 CONTROLLER STATE CHANGED:")
            print(f"{msg}")
            self.last_controller_state = state
    
    def voice_req_cb(self, msg):
        print(f"\n📤 VOICE REQUEST:")
        print(f"   API ID: {msg.header.identity.api_id}")
        print(f"   Params: {msg.parameter}")
        if msg.binary:
            print(f"   Binary: {len(msg.binary)} bytes")
    
    def voice_res_cb(self, msg):
        print(f"\n📥 VOICE RESPONSE:")
        print(f"   API ID: {msg.header.identity.api_id}")
        print(f"   Status: {msg.header.status.code}")
        print(f"   Data: {msg.data}")
    
    def gpt_req_cb(self, msg):
        print(f"\n📤 GPT REQUEST:")
        print(f"   API ID: {msg.header.identity.api_id}")
        print(f"   Params: {msg.parameter}")
    
    def gpt_res_cb(self, msg):
        print(f"\n📥 GPT RESPONSE:")
        print(f"   API ID: {msg.header.identity.api_id}")
        print(f"   Status: {msg.header.status.code}")
        print(f"   Data: {msg.data}")
    
    def state_cb(self, msg):
        print(f"\n🤖 GPT STATE: {msg.data}")
    
    def cmd_cb(self, msg):
        print(f"\n💬 GPT CMD: {msg.data}")
    
    def audio_cb(self, msg):
        try:
            data = json.loads(msg.data)
            text = data.get('text', '')
            if text:
                print(f"\n🎤 AUDIO TRANSCRIBED: '{text}'")
        except:
            pass

def main():
    rclpy.init()
    node = ControllerMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nStopped monitoring")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()