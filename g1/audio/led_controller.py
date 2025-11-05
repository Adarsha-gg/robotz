#!/usr/bin/env python3
"""
LED Controller - Fancy LED patterns and effects for G1
"""

import time
import math
import threading


class LEDController:
    """Advanced LED control with smooth animations"""
    
    def __init__(self, audio_client):
        self.audio_client = audio_client
        self.running = False
        self.current_thread = None
    
    def stop(self):
        """Stop any running animation"""
        self.running = False
        if self.current_thread:
            self.current_thread.join()
    
    def set_color(self, r, g, b):
        """Set solid color"""
        self.audio_client.led_control(r, g, b)
    
    def smooth_transition(self, start_color, end_color, duration=2, steps=30):
        """Smooth color transition"""
        start_r, start_g, start_b = start_color
        end_r, end_g, end_b = end_color
        
        step_delay = duration / steps
        
        for step in range(steps + 1):
            factor = step / steps
            
            r = int(start_r + (end_r - start_r) * factor)
            g = int(start_g + (end_g - start_g) * factor)
            b = int(start_b + (end_b - start_b) * factor)
            
            self.set_color(r, g, b)
            time.sleep(step_delay)
    
    def pulse(self, color=(0, 255, 0), duration=5, speed=1):
        """Pulsing effect"""
        def _pulse():
            self.running = True
            start_time = time.time()
            base_r, base_g, base_b = color
            
            while self.running and (time.time() - start_time) < duration:
                # Sine wave for smooth pulsing
                factor = (math.sin(time.time() * speed * 2 * math.pi) + 1) / 2
                
                r = int(base_r * factor)
                g = int(base_g * factor)
                b = int(base_b * factor)
                
                self.set_color(r, g, b)
                time.sleep(0.05)
            
            self.set_color(0, 0, 0)
            self.running = False
        
        self.stop()
        self.current_thread = threading.Thread(target=_pulse)
        self.current_thread.start()
    
    def rainbow_cycle(self, duration=10, speed=1):
        """Rainbow color cycle"""
        def _rainbow():
            self.running = True
            start_time = time.time()
            
            while self.running and (time.time() - start_time) < duration:
                hue = (time.time() * speed * 50) % 360
                r, g, b = self.hsv_to_rgb(hue, 1.0, 1.0)
                
                self.set_color(r, g, b)
                time.sleep(0.05)
            
            self.set_color(0, 0, 0)
            self.running = False
        
        self.stop()
        self.current_thread = threading.Thread(target=_rainbow)
        self.current_thread.start()
    
    def strobe(self, color=(255, 255, 255), duration=5, frequency=5):
        """Strobe effect"""
        def _strobe():
            self.running = True
            start_time = time.time()
            interval = 1.0 / frequency / 2  # On/off intervals
            
            while self.running and (time.time() - start_time) < duration:
                self.set_color(*color)
                time.sleep(interval)
                self.set_color(0, 0, 0)
                time.sleep(interval)
            
            self.set_color(0, 0, 0)
            self.running = False
        
        self.stop()
        self.current_thread = threading.Thread(target=_strobe)
        self.current_thread.start()
    
    def breathing(self, color=(0, 0, 255), duration=10, breath_duration=3):
        """Breathing effect (slow pulse)"""
        def _breathe():
            self.running = True
            start_time = time.time()
            base_r, base_g, base_b = color
            
            while self.running and (time.time() - start_time) < duration:
                elapsed = time.time() - start_time
                # Smooth breathing using sine wave
                factor = (math.sin((elapsed / breath_duration) * 2 * math.pi) + 1) / 2
                factor = factor ** 2  # Make it smoother
                
                r = int(base_r * factor)
                g = int(base_g * factor)
                b = int(base_b * factor)
                
                self.set_color(r, g, b)
                time.sleep(0.05)
            
            self.set_color(0, 0, 0)
            self.running = False
        
        self.stop()
        self.current_thread = threading.Thread(target=_breathe)
        self.current_thread.start()
    
    def party_mode(self, duration=10):
        """Random flashing colors"""
        def _party():
            self.running = True
            start_time = time.time()
            
            import random
            
            while self.running and (time.time() - start_time) < duration:
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                
                self.set_color(r, g, b)
                time.sleep(0.2)
            
            self.set_color(0, 0, 0)
            self.running = False
        
        self.stop()
        self.current_thread = threading.Thread(target=_party)
        self.current_thread.start()
    
    def police_lights(self, duration=5):
        """Red/blue alternating"""
        def _police():
            self.running = True
            start_time = time.time()
            
            while self.running and (time.time() - start_time) < duration:
                self.set_color(255, 0, 0)  # Red
                time.sleep(0.3)
                self.set_color(0, 0, 255)  # Blue
                time.sleep(0.3)
            
            self.set_color(0, 0, 0)
            self.running = False
        
        self.stop()
        self.current_thread = threading.Thread(target=_police)
        self.current_thread.start()
    
    def thinking_animation(self):
        """Thinking indicator - cyan pulse"""
        self.pulse(color=(0, 255, 255), duration=999, speed=2)
    
    def success_flash(self):
        """Quick green flash"""
        self.set_color(0, 255, 0)
        time.sleep(0.5)
        self.set_color(0, 0, 0)
    
    def error_flash(self):
        """Quick red flash"""
        self.set_color(255, 0, 0)
        time.sleep(0.5)
        self.set_color(0, 0, 0)
    
    def hsv_to_rgb(self, h, s, v):
        """Convert HSV to RGB (h: 0-360, s/v: 0-1)"""
        h = h / 60.0
        i = int(h)
        f = h - i
        
        p = v * (1.0 - s)
        q = v * (1.0 - s * f)
        t = v * (1.0 - s * (1.0 - f))
        
        i = i % 6
        
        if i == 0:
            r, g, b = v, t, p
        elif i == 1:
            r, g, b = q, v, p
        elif i == 2:
            r, g, b = p, v, t
        elif i == 3:
            r, g, b = p, q, v
        elif i == 4:
            r, g, b = t, p, v
        else:
            r, g, b = v, p, q
        
        return int(r * 255), int(g * 255), int(b * 255)