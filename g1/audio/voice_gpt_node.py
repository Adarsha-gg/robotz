#!/usr/bin/env python3
"""
Voice GPT Node - ROS2 node integrating audio, TTS, and GPT
NOW WITH MICROPHONE ACTIVATION

ARCHITECTURE OVERVIEW:
1. This node initializes and connects to the robot's internal ROS2 network.
2. It subscribes to the robot's audio stream (/audio_msg) to "hear" what the robot hears.
3. It disables the robot's built-in GPT so it doesn't conflict with our custom one.
4. When audio is received:
   a. It checks if the text is English.
   b. It sends the text to the CommandParser to see if it's a command (e.g. "turn red").
   c. If it's a command, it executes it immediately via AudioClient.
   d. If it's not a command, it sends it to GPTHandler to get a conversational response.
   e. The response is sent to TTSEngine to be spoken aloud.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_go.msg import AudioData
from unitree_api.msg import Request, Response
import tempfile
import wave
import math
import os
import time
import json
import threading
from groq import Groq
from dotenv import load_dotenv

# Import helper classes that handle specific subsystems
from .audio_client import G1AudioClient
from .tts_engine import TTSEngine
from .gpt_handler import GPTHandler
from audio.led_controller import LEDController
from audio.command_parser import CommandParser
from unitree_go.msg import WirelessController

class VoiceGPTNode(Node):
    """ROS2 node for voice-controlled GPT conversations with microphone activation"""
    
    def __init__(self):
        # Initialize the ROS2 node with name 'voice_gpt_node'
        super().__init__('voice_gpt_node')
        
        # Load environment variables (API keys, etc.) from .env file
        load_dotenv()
        
        # --- SUBSYSTEM INITIALIZATION ---
        # 1. Audio Client: Handles low-level communication with the robot (sending requests)
        self.audio_client = G1AudioClient()
        
        # 2. TTS Engine: Handles converting text to speech (using Piper)
        self.tts_engine = TTSEngine(self.audio_client)
        
        # Don't use LED controller - keep it simple for now
        # self.led_controller = LEDController(self.audio_client)

        self.is_speaking = False

        # 3. Groq Client: The connection to the LLM (Llama 3)
        self.groq_client = Groq(api_key=os.getenv('GROQ_API_KEY'))
        
        # 4. Command Parser: Analyzes text to find commands (like "turn red")
        self.command_parser = CommandParser(self.audio_client, groq_client=self.groq_client)
        
        # 5. GPT Handler: Manages the conversation history and persona
        self.gpt_handler = GPTHandler(
            self.tts_engine,
            command_parser=self.command_parser,
            max_history=10,
            system_prompt=None,
            groq_client=self.groq_client
        )
        
        # === MICROPHONE INTERCEPTION (CRITICAL) ===
        # This is where we "hook" into the robot's ears.
        # The robot publishes transcribed text to '/audio_msg'.
        # We subscribe to it so we receive every word it hears.
        self.robot_audio_sub = self.create_subscription(
            String,
            '/audio_msg',  # Topic where robot publishes transcribed speech
            self.robot_audio_callback, # Function called when message arrives
            10
        )
        
        # === MANUAL INPUT CHANNELS (Optional) ===
        # These allow us to inject audio or text manually for testing
        self.audio_sub = self.create_subscription(
            AudioData, '/gpt_audio_input', self.audio_callback, 10)
        self.text_sub = self.create_subscription(
            String, '/gpt_command', self.text_callback, 10)
        
        
        # === OUTPUT ===
        # We publish our responses here so other nodes can see them
        self.text_pub = self.create_publisher(String, '/gpt_response', 10)
        
        # === VOICE CONTROL (for microphone activation) ===
        # Used to send API commands to enable/disable the mic
        self.voice_control_pub = self.create_publisher(
            Request,
            '/api/voice/request',
            10
        )
        
        # === GPT CONTROL (for disabling robot's GPT) ===
        # Used to send API commands to kill the built-in chatbot
        self.gpt_control_pub = self.create_publisher(
            Request,
            '/api/gpt/request',
            10
        )
        
        self.get_logger().info('Voice GPT Node initialized!')
        
        # Set initial state: Volume 50%, LED Green
        time.sleep(0.5)
        self.audio_client.set_volume(50)
        self.audio_client.led_control(0, 255, 0)  # Green = ready
        
        # === CONTROLLER MONITORING ===
        # Listen to the wireless controller to detect button presses
        self.create_subscription(
            WirelessController,
            '/wirelesscontroller',
            self.controller_callback,
            10
        )
        self.controller_active = False
        self.last_keys = 0
        self.breathing_active = False 
        
        
        # === ENABLE MICROPHONE ON STARTUP ===
        # Immediately try to take control of the microphone
        self.enable_microphone()
    
    def enable_microphone(self, mode=2):
        """
        Enable robot's microphone using API 1008
        AND disable robot's built-in GPT
        
        Args:
            mode: 1=always_listening, 2=push_to_talk, 3=close_interaction
        """
        def send_enable():
            # Wait a bit for connections to establish
            time.sleep(2)
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("🎤 Activating robot microphone...")
            self.get_logger().info("🚫 Disabling robot's built-in GPT...")
            self.get_logger().info("=" * 60)
            
            # FIRST: Disable robot's GPT service
            # We send multiple disable commands to be sure it's dead.
            # If we don't do this, the robot will answer AND we will answer (double talk).
            disable_attempts = [
                {"action": "disable"},
                {"state": False},
                {"enable": False},
            ]
            
            for params in disable_attempts:
                gpt_disable = Request()
                gpt_disable.header.identity.id = 8001009
                gpt_disable.header.identity.api_id = 1  # GPT control API
                gpt_disable.header.lease.id = 0
                gpt_disable.header.policy.priority = 0
                gpt_disable.header.policy.noreply = False
                gpt_disable.parameter = json.dumps(params)
                gpt_disable.binary = []
                
                self.gpt_control_pub.publish(gpt_disable)
                time.sleep(0.5)
            
            self.get_logger().info("✅ GPT disable commands sent")
            
            # SECOND: Enable microphone (API 1008)
            # This tells the robot to start listening and transcribing.
            mic_enable = Request()
            mic_enable.header.identity.id = 8001007
            mic_enable.header.identity.api_id = 1008  # Voice mode control API
            mic_enable.header.lease.id = 0
            mic_enable.header.policy.priority = 0
            mic_enable.header.policy.noreply = False
            mic_enable.parameter = json.dumps({"mode": mode})
            mic_enable.binary = []
            
            self.voice_control_pub.publish(mic_enable)
            
            mode_names = {1: "always listening", 2: "push-to-talk", 3: "close interaction"}
            mode_name = mode_names.get(mode, "unknown")
            self.get_logger().info(f"✅ Microphone activated (mode={mode} [{mode_name}])")
            self.get_logger().info("✅ Robot's GPT disabled - using YOUR custom GPT only!")
            self.get_logger().info("🗣️  Speak to the robot...")
        
        # Run this in a background thread so we don't block the main loop
        threading.Thread(target=send_enable, daemon=True).start()
    
    
    
    def robot_audio_callback(self, msg):
        """
        MAIN VOICE INPUT HANDLER
        This function triggers whenever the robot hears something.
        
        Flow:
        1. Receive JSON string from robot.
        2. Parse JSON to get text.
        3. Filter out non-English noise.
        4. Send to CommandParser -> Is it a command?
        5. If Command -> Execute it.
        6. If Not Command -> Send to GPT -> Speak response.
        """
        try:
            # 1. Parse JSON
            audio_data = json.loads(msg.data)
            text = audio_data.get('text', '').strip()
            confidence = audio_data.get('confidence', 0.0)
            
            if not text:
                return
            
            # 2. Filter Noise (Non-English check)
            # The robot sometimes hallucinates Chinese characters from noise.
            # We check if the text is mostly ASCII (English).
            ascii_ratio = sum(1 for c in text if ord(c) < 128) / len(text)
            
            if ascii_ratio < 0.7:
                self.get_logger().warn(f"❌ Non-English rejected: {text} (ASCII: {ascii_ratio:.1%})")
                return  # Just ignore it, don't speak
            
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info(f"🎤 HEARD: '{text}' (confidence: {confidence:.2f})")
            self.get_logger().info("=" * 60)
            
        except json.JSONDecodeError:
            self.get_logger().error("JSON parse failed")
            return
        
        # 3. Process the text
        try:
            # Step A: Check for Commands
            # "is_command" will be True if the user said something like "turn red"
            is_command, cmd_response = self.command_parser.parse(text)
            
            if is_command:
                # It WAS a command!
                self.get_logger().info(f"⚙️ Command: {cmd_response}")
                # Speak confirmation (e.g., "Turning lights red")
                self.tts_engine.speak(cmd_response)
                reply = cmd_response
            else:
                # It was NOT a command, so it's a conversation.
                self.get_logger().info("🤖 Processing with GPT...")
                
                # Step B: Ask GPT
                # This sends the text + history to Groq
                reply = self.gpt_handler.process_text(text)
                
                if reply:
                    self.get_logger().info("🔊 Speaking...")
                    # Step C: Speak the response
                    self.tts_engine.speak(reply)
            
            # 4. Publish the response (for logging/UI)
            if reply:
                msg = String()
                msg.data = reply
                self.text_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            import traceback
            traceback.print_exc()
    
    def text_callback(self, msg):
        """
        Handle manual text command input
        Useful for testing without speaking.
        """
        self.get_logger().info(f'📝 Text command: {msg.data}')
        
        # Process through GPT (command parser is inside gpt_handler too)
        reply = self.gpt_handler.process_text(msg.data)
        
        if reply:
            # Speak response
            self.tts_engine.speak(reply)
            
            # Publish response
            response_msg = String()
            response_msg.data = reply
            self.text_pub.publish(response_msg)
    
    def audio_callback(self, msg):
        """
        Handle manual audio input for transcription (via /gpt_audio_input topic)
        This is for sending raw audio bytes if you aren't using the robot's mic.
        """
        self.get_logger().info('🎵 Processing manual audio input...')
        
        try:
            # 1. Save audio to temporary WAV file
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_wav:
                with wave.open(temp_wav.name, 'wb') as wav:
                    wav.setnchannels(1)
                    wav.setsampwidth(2)
                    wav.setframerate(16000)
                    wav.writeframes(bytes(msg.data))
                temp_path = temp_wav.name
            
            # 2. Transcribe using Groq Whisper
            with open(temp_path, 'rb') as audio_file:
                transcript = self.groq_client.audio.transcriptions.create(
                    file=(temp_path, audio_file.read()),
                    model="whisper-large-v3-turbo",
                    response_format="text"
                )
                
            
            text = transcript.strip()
            self.get_logger().info(f'Transcribed: {text}')
            
            # 3. Process through GPT
            reply = self.gpt_handler.process_text(text)
            
            # 4. Publish response
            if reply:
                response_msg = String()
                response_msg.data = reply
                self.text_pub.publish(response_msg)
            
            # Cleanup
            os.unlink(temp_path)
            
        except Exception as e:
            self.get_logger().error(f'Audio processing error: {e}')

    def controller_callback(self, msg):
        """
        Monitor controller button presses
        keys=40 means mic buttons are HELD (listening mode)
        keys=0 or 32 means buttons released
        """
        current_keys = msg.keys
        
        # Detect button press (transition to keys=40)
        if current_keys == 40 and not self.controller_active:
            self.controller_active = True
            self.get_logger().info("🎮 Controller mic buttons PRESSED - listening active...")
            # Start breathing purple effect
            self.start_breathing_purple()
            
        # Detect button release (transition away from 40)
        elif current_keys != 40 and self.controller_active:
            self.controller_active = False
            self.get_logger().info("🎮 Controller mic buttons RELEASED")
            # Stop breathing and return to solid green
            self.stop_breathing()
            self.audio_client.led_control(0, 255, 0)  # Green = ready
        
        self.last_keys = current_keys

    def start_breathing_purple(self):
        """Start breathing purple LED effect"""
        self.breathing_active = True
        threading.Thread(target=self._breathing_loop, daemon=True).start()

    def stop_breathing(self):
        """Stop breathing LED effect"""
        self.breathing_active = False

    def _breathing_loop(self):
        """
        Ultra-smooth breathing with ease-in-out - purple color
        Runs in a separate thread to not block main logic.
        """
        
        cycle_duration = 3.5  # Slower cycle
        update_rate = 60
    
        def ease_in_out(t):
            """Smooth easing function"""
            return (math.cos(math.pi * t) + 1) / 2
        
            start_time = time.time()
            
            while self.breathing_active:
                elapsed = time.time() - start_time
                # Normalized time (0 to 1) within cycle
                t = (elapsed % cycle_duration) / cycle_duration
                
                # Apply easing for smooth acceleration/deceleration
                brightness = 0.25 + 0.75 * ease_in_out(t)
                
                # Rich purple color
                r = int(138 * brightness)
                g = int(43 * brightness)
                b = int(226 * brightness)
                
                self.audio_client.led_control(r, g, b)
                
                time.sleep(1.0 / update_rate)           


def main():
    rclpy.init()
    node = VoiceGPTNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()