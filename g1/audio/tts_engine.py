#!/usr/bin/env python3
"""
TTS Engine for G1 Robot
Optimized for speed
"""
import subprocess
import tempfile
import os
import time
import rclpy.logging
from pathlib import Path

class TTSEngine:
    """Text-to-speech engine using Piper"""
    
    def __init__(self, audio_client, voice_model_path=None):
        """Initialize TTS engine with Piper."""
        self.audio_client = audio_client
        self.logger = rclpy.logging.get_logger('tts_engine')
        
        # Check if piper is available
        try:
            subprocess.run(['piper', '--help'], capture_output=True, check=True, timeout=2)
            self.logger.info('Piper TTS found and ready')
        except Exception as e:
            self.logger.error(f"Piper check failed: {e}")
            raise RuntimeError("Piper TTS not found")
        
        # Set voice model path
        self.voice_model_path = voice_model_path or '/home/pass/robot_cpp/piper_voices/en_US-lessac-medium.onnx'
        
        # Verify model exists
        if not Path(self.voice_model_path).exists():
            self.logger.error(f'Voice model not found at {self.voice_model_path}')
            raise RuntimeError(f"Voice model not found: {self.voice_model_path}")
        
        self.logger.info(f"Using voice model: {self.voice_model_path}")
    
    def speak(self, text):
        """
        Convert text to speech and play on robot (OPTIMIZED)
        
        Args:
            text: Text to speak
        
        Returns:
            0 on success, -1 on failure
        """
        if not text:
            self.logger.warn("Empty text, nothing to speak")
            return -1
            
        try:
            self.logger.info(f"Starting TTS for: {text[:50]}...")
            start_time = time.time()
            
            # Generate unique temp files
            temp_base = f'/tmp/speech_{int(time.time() * 1000)}'
            piper_wav = f'{temp_base}_piper.wav'
            final_wav = f'{temp_base}_final.wav'
            
            # Run Piper TTS
            self.logger.info("Running Piper...")
            piper_result = subprocess.run([
                'piper',
                '-m', self.voice_model_path,
                '-f', piper_wav
            ], input=text.encode(), check=True, capture_output=True, timeout=10)
            
            piper_time = time.time() - start_time
            self.logger.info(f"Piper completed in {piper_time:.2f}s")
            
            # Check if Piper output exists
            if not os.path.exists(piper_wav):
                self.logger.error(f"Piper didn't create output file: {piper_wav}")
                return -1
            
            # Convert to G1 format
            self.logger.info("Converting audio format...")
            ffmpeg_start = time.time()
            subprocess.run([
                'ffmpeg', '-y',
                '-i', piper_wav,
                '-ar', '16000',
                '-ac', '1',
                '-c:a', 'pcm_s16le',
                '-loglevel', 'error',
                final_wav
            ], check=True, capture_output=True, timeout=5)
            
            ffmpeg_time = time.time() - ffmpeg_start
            self.logger.info(f"Conversion completed in {ffmpeg_time:.2f}s")
            
            # Check if conversion output exists
            if not os.path.exists(final_wav):
                self.logger.error(f"ffmpeg didn't create output: {final_wav}")
                return -1
            
            # Play on robot
            self.logger.info("Sending audio to robot...")
            play_result = self.audio_client.play_audio_file(final_wav)
            
            # Cleanup
            try:
                os.unlink(piper_wav)
                os.unlink(final_wav)
            except Exception as e:
                self.logger.warn(f"Cleanup failed: {e}")
            
            total_time = time.time() - start_time
            self.logger.info(f"✅ TTS complete in {total_time:.2f}s")
            
            return play_result
            
        except subprocess.TimeoutExpired as e:
            self.logger.error(f'TTS timeout: {e}')
            return -1
        except subprocess.CalledProcessError as e:
            self.logger.error(f'TTS command failed: {e}')
            self.logger.error(f'stderr: {e.stderr.decode() if e.stderr else "none"}')
            return -1
        except Exception as e:
            self.logger.error(f'TTS error: {e}')
            return -1
    
    def speak_builtin(self, text, voice_id=1):
        """
        Use G1's built-in TTS (faster but lower quality)
        
        Args:
            text: Text to speak
            voice_id: 0=Chinese, 1=English
        """
        return self.audio_client.tts_maker(text, voice_id)
