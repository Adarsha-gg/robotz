#!/usr/bin/env python3
"""
G1 Audio Client - Core audio API wrapper for Unitree G1 robot
Handles TTS, audio playback, volume control, and LED control
"""

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response
import json
import time
import wave


class G1AudioClient(Node):
    """Core audio client for G1 robot communication"""
    
    def __init__(self):
        super().__init__('g1_audio_client')
        
        # Publishers
        self.voice_publisher = self.create_publisher(Request, '/api/voice/request', 10)
        self.loco_publisher = self.create_publisher(Request, '/api/loco/request', 10)  # ADD THIS
        
        # Subscribers for responses
        self.voice_response_sub = self.create_subscription(
            Response, '/api/voice/response', self.voice_response_callback, 10)
        
        self.get_logger().info("G1 Audio Client initialized")

    def execute_gesture(self, action_id):
        """
        Execute arm gesture action
        
        Args:
            action_id: 0=wave, 1=turn&wave, 2=handshake, 3=release
            
        Returns:
            0 on success, -1 on failure
        """
        try:
            params = json.dumps({"data": action_id})
            request = self._create_request(7106, params)
            self.loco_publisher.publish(request)  # Use loco publisher!
            self.get_logger().info(f"Gesture action {action_id}")
            return 0
        except Exception as e:
            self.get_logger().error(f"Gesture failed: {e}")
            return -1    
            
    def _create_request(self, api_id, parameter="", binary_data=None):
        """
        Helper to create properly formatted Request messages
        
        Args:
            api_id: API function ID
            parameter: JSON string parameter
            binary_data: Binary data as bytes
            
        Returns:
            Formatted Request message
        """
        request = Request()
        
        # Set header fields
        request.header.identity.id = int(time.time() * 1000000)
        request.header.identity.api_id = api_id
        request.header.lease.id = 0
        request.header.policy.priority = 0
        request.header.policy.noreply = False
        
        # Set data
        request.parameter = parameter
        request.binary = list(binary_data) if binary_data else []
        
        return request
        
    def voice_response_callback(self, msg):
        """Handle voice API responses"""
        self.get_logger().debug(f"Voice API response: {msg}")
    
    def tts_maker(self, text, voice_id=1):
        """
        Text-to-speech using G1's built-in TTS
        
        Args:
            text: Text to speak
            voice_id: 0=Chinese, 1=English
            
        Returns:
            0 on success, -1 on failure
        """
        try:
            params = {
                "index": 0,
                "speaker_id": voice_id,
                "text": text
            }
            
            request = self._create_request(1001, json.dumps(params))
            self.voice_publisher.publish(request)
            self.get_logger().info(f"TTS: {text}")
            return 0
        except Exception as e:
            self.get_logger().error(f"TTS failed: {e}")
            return -1
    
    def get_volume(self):
        """Get current volume level"""
        try:
            request = self._create_request(1005)
            self.voice_publisher.publish(request)
            return 0
        except Exception as e:
            self.get_logger().error(f"Get volume failed: {e}")
            return -1
    
    def set_volume(self, volume_level):
        """
        Set volume level
        
        Args:
            volume_level: Volume 0-100
        """
        try:
            params = json.dumps({"volume": volume_level})
            request = self._create_request(1006, params)
            self.voice_publisher.publish(request)
            self.get_logger().info(f"Volume set to {volume_level}%")
            return 0
        except Exception as e:
            self.get_logger().error(f"Set volume failed: {e}")
            return -1
    
    def play_audio_file(self, filepath):
        """
        Play audio file (must be 16kHz, mono, 16-bit WAV)
        
        Args:
            filepath: Path to WAV file
        """
        try:
            # Read WAV file
            with wave.open(filepath, 'rb') as wav_file:
                sample_rate = wav_file.getframerate()
                num_channels = wav_file.getnchannels()
                sample_width = wav_file.getsampwidth()
                
                if sample_width != 2:
                    self.get_logger().error("Must be 16-bit audio")
                    return -1
                
                if sample_rate != 16000 or num_channels != 1:
                    self.get_logger().error(f"Wrong format: {sample_rate}Hz, {num_channels}ch")
                    return -1
                
                pcm_data = wav_file.readframes(wav_file.getnframes())
            
            # Send to G1
            stream_id = str(int(time.time() * 1000))
            params = json.dumps({
                "app_name": "audio_player",
                "stream_id": stream_id
            })
            
            request = self._create_request(1003, params, pcm_data)
            self.voice_publisher.publish(request)
            self.get_logger().info(f"Playing audio: {len(pcm_data)} bytes")
            return 0
            
        except Exception as e:
            self.get_logger().error(f"Audio playback failed: {e}")
            return -1
    
    def stop_audio(self, audio_id="audio_player"):
        """Stop audio playback"""
        try:
            params = json.dumps({"app_name": audio_id})
            request = self._create_request(1004, params)
            self.voice_publisher.publish(request)
            return 0
        except Exception as e:
            self.get_logger().error(f"Stop audio failed: {e}")
            return -1
    
    def led_control(self, red, green, blue):
        """
        Control robot LED colors
        
        Args:
            red, green, blue: RGB values 0-255
        """
        try:
            params = json.dumps({"R": red, "G": green, "B": blue})
            request = self._create_request(1010, params)
            self.voice_publisher.publish(request)
            self.get_logger().debug(f"LED: RGB({red}, {green}, {blue})")  # Changed to debug
            return 0
        except Exception as e:
            self.get_logger().error(f"LED control failed: {e}")
            return -1