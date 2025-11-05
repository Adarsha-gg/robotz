#!/usr/bin/env python3
"""
G1 Robot Main Entry Point
With keyboard stop control
"""
import rclpy
from rclpy.executors import MultiThreadedExecutor
from audio.voice_gpt_node import VoiceGPTNode
import sys
import select
import threading

def main():
    rclpy.init()
    
    try:
        # Create main GPT node
        voice_gpt_node = VoiceGPTNode()
        
        # Setup executor
        executor = MultiThreadedExecutor()
        executor.add_node(voice_gpt_node)
        executor.add_node(voice_gpt_node.audio_client)
        
        print("\n" + "=" * 60)
        print("🤖 G1 ROBOT SYSTEM ONLINE")
        print("=" * 60)
        print("✨ Features:")
        print("  • Robot microphone → YOUR custom GPT (Groq)")
        print("  • Voice responses → Piper TTS")
        print("  • Voice commands → Volume, LED control, etc.")
        print("\n⌨️  KEYBOARD CONTROLS:")
        print("  • Type 's' + ENTER to stop speaking")
        print("  • Say 'stop' to the robot")
        print("  • Press Ctrl+C to shutdown")
        print("\n🎤 Speak to the robot!")
        print("=" * 60 + "\n")
        
        # Keyboard listener thread
        def keyboard_listener():
            while rclpy.ok():
                try:
                    if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                        line = sys.stdin.readline().strip().lower()
                        if line == 's':
                            print("\n🛑 Stopping audio...\n")
                            voice_gpt_node.audio_client.stop_audio("audio_player")
                except:
                    pass
        
        kb_thread = threading.Thread(target=keyboard_listener, daemon=True)
        kb_thread.start()
        
        executor.spin()
        
    except KeyboardInterrupt:
        print("\n\n🛑 Shutting down gracefully...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()