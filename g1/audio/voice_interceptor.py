#!/usr/bin/env python3
"""
Voice Command Interceptor
Captures robot's speech recognition and routes through custom GPT+TTS
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class VoiceInterceptor(Node):
    def __init__(self, gpt_handler):
        super().__init__('voice_interceptor')
        self.gpt_handler = gpt_handler
        
        # Subscribe to robot's speech recognition output
        self.asr_sub = self.create_subscription(
            String,
            '/audio_msg',
            self.voice_command_callback,
            10
        )
        
        self.get_logger().info("Voice interceptor active - listening to robot mic...")
    
    def voice_command_callback(self, msg):
        """Process voice commands from robot's microphone"""
        try:
            # Parse JSON data
            audio_data = json.loads(msg.data)
            
            # Extract the transcribed text
            text = audio_data.get('text', '').strip()
            
            if not text:
                return
            
            self.get_logger().info(f"🎤 Robot heard: '{text}'")
            
            # Process through YOUR GPT system instead of robot's
            self.gpt_handler.process_text(text)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse audio message: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing voice command: {e}")

def main():
    rclpy.init()
    rclpy.shutdown()

if __name__ == '__main__':
    main()