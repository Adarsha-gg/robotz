#!/usr/bin/env python3
"""
Command Parser with LLM Intent Understanding (Groq API)

ARCHITECTURE OVERVIEW:
1. This module receives raw text from the user (e.g., "make the lights red").
2. It constructs a prompt for the LLM (Llama 3) that explains the robot's capabilities.
3. It asks the LLM to return a JSON object describing the action.
   - Example: {"action_type": "led", "action": "red"}
4. It parses the JSON and executes the corresponding function in AudioClient.
"""

import json
import os
import threading
import time
from groq import Groq


class CommandParser:
    """Parse voice commands for robot actions using LLM interpretation"""

    def __init__(self, audio_client, groq_client=None):
        # We need the audio_client to actually EXECUTE the commands (move, light up, etc.)
        self.audio_client = audio_client
        
        # We use Groq (Llama 3) to understand natural language
        self.groq_client = groq_client or Groq(api_key=os.getenv("GROQ_API_KEY"))

        # === Gesture & LED Mappings ===
        # These are the internal IDs the robot understands
        self.gestures = {
            "hello": 0,       # Wave
            "turn": 1,        # Turn around
            "goodbye": 1,     # turn back and wave(for now)
            "shake_hand": 2,  # Extend hand
            "stop": 3,        # Stop moving
            "release": 3,     # Release hand
        }

        self.led_colors = {
            "red": (255, 0, 0),
            "blue": (0, 0, 255),
            "green": (0, 255, 0),
            "purple": (255, 0, 255),
            "yellow": (255, 255, 0),
            "cyan": (0, 255, 255),
            "white": (255, 255, 255),
        }

    # --------------------------------------------------
    # Primary Parse Function
    # --------------------------------------------------
    def parse(self, text):
        """
        Main entry point for command parsing.
        
        Args:
            text: The user's spoken text (e.g. "turn red")
            
        Returns:
            (is_command, response_text)
            - is_command: True if an action was taken, False if it's just chat.
            - response_text: What the robot should say back.
        """
        # Step 1: Ask the LLM what this text means
        result = self.llm_interpret(text)
        
        # Safety: handle if LLM returns a list (it shouldn't, but AI is unpredictable)
        if isinstance(result, list):
            print(f"⚠️ Got list in parse(), taking first item")
            result = result[0] if result else {"action_type": "unknown"}
        
        # Safety: ensure result is valid
        if not isinstance(result, dict):
            print(f"❌ Invalid result type in parse(): {type(result)}")
            return False, "Invalid command format"
        
        if "action_type" not in result:
            print(f"❌ Missing action_type in parse(). Got: {result}")
            return False, "Invalid command format"
        
        # Step 2: Execute the action described by the LLM
        return self._execute_single_action(result)

    # --------------------------------------------------
    # LLM Interpretation (Groq)
    # --------------------------------------------------
    def llm_interpret(self, text):
        """
        Use LLM to interpret user's intent and map to a known action.
        
        This is the "Brain" of the command system. It takes vague human speech
        and converts it into precise JSON instructions.
        """
        try:
            # Construct the prompt. We tell the LLM exactly what the robot can do.
            prompt = f"""You are a robot command interpreter. Analyze this command and return a JSON response.

Available capabilities:
- Gestures: hello, turn, goodbye, shake_hand, stop, release
- LED colors: red, blue, green, purple, yellow, cyan, white, off
- Volume: 0-100 (can be "up", "down", or specific number)

IMPORTANT: If the user requests MULTIPLE actions in sequence (like "turn red then blue"), 
return ONLY the FIRST action. The user can give another command for the next action.

Examples:
- "wave hello" → {{"action_type": "gesture", "action": "hello"}}
- "make the lights blue" → {{"action_type": "led", "action": "blue"}}
- "set lights to red and then blue" → {{"action_type": "led", "action": "red"}}
- "turn off the lights" → {{"action_type": "led", "action": "off"}}
- "volume up" → {{"action_type": "volume", "action": "up"}}
- "set volume to 75" → {{"action_type": "volume", "action": "set", "parameters": {{"volume": 75}}}}

User command: "{text}"

Respond with ONLY valid JSON in this format (single action only):
{{
    "action_type": "gesture|led|volume|unknown",
    "action": "specific_action_name",
    "parameters": {{}} (optional, for volume numbers)
}}"""

            # Send to Groq API
            response = self.groq_client.chat.completions.create(
                model="llama-3.3-70b-versatile",
                messages=[
                    {"role": "system", "content": "You interpret natural language into robot control commands. Always return valid JSON for a SINGLE action only."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1, # Low temperature = more deterministic/precise
                response_format={"type": "json_object"} # Force JSON output
            )

            content = response.choices[0].message.content
            print(f"🔍 Raw LLM response: {content}")
            
            result = json.loads(content)
            print(f"🔍 Parsed result: {result}")
            print(f"🔍 Type: {type(result)}")
            
            # Safety check: if it's a list, take the first item
            if isinstance(result, list):
                print(f"⚠️ LLM returned list, taking first action")
                result = result[0] if result else {"action_type": "unknown", "action": None}
            
            # Safety check: ensure it's a dict with required keys
            if not isinstance(result, dict):
                print(f"❌ Result is not a dict: {result}")
                return {"action_type": "unknown", "action": None}
            
            if "action_type" not in result:
                print(f"❌ Missing action_type in result: {result}")
                return {"action_type": "unknown", "action": None}
            
            return result

        except json.JSONDecodeError as e:
            print(f"❌ JSON decode error: {e}")
            print(f"   Content was: {content if 'content' in locals() else 'N/A'}")
            return {"action_type": "unknown", "action": None}
        except Exception as e:
            print(f"❌ LLM interpret error: {e}")
            import traceback
            traceback.print_exc()
            return {"action_type": "unknown", "action": None}

    # --------------------------------------------------
    # Action Execution
    # --------------------------------------------------
    def _execute_single_action(self, result):
        """
        Execute a single parsed action.
        Dispatches to the appropriate handler based on 'action_type'.
        """
        # Final safety check
        if not isinstance(result, dict):
            print(f"❌ Invalid action type in _execute_single_action: {type(result)}")
            return False, "Invalid action format"
        
        if "action_type" not in result:
            print(f"❌ Missing action_type in _execute_single_action: {result}")
            return False, "Invalid action format"
        
        action_type = result.get("action_type")
        
        # Dispatcher
        if action_type == "gesture":
            return self.handle_gesture(result)
        
        elif action_type == "led":
            return self.handle_led(result)
        
        elif action_type == "volume":
            return self.handle_volume(result)
        
        elif action_type == "unknown":
            # If LLM says "unknown", it means it's probably just conversation
            return False, "Command not recognized"
        
        return False, "Command not understood"

    # --------------------------------------------------
    # Action Handlers
    # --------------------------------------------------
    def handle_gesture(self, result):
        """Handle gesture commands (wave, handshake, etc.)"""
        action = result.get("action", "").lower()
        gesture_id = self.gestures.get(action)
        
        if gesture_id is not None:
            # Call the hardware client to trigger the move
            success = self.trigger_gesture(gesture_id)
            if success:
                return True, f"{action.replace('_', ' ').title()}"
            else:
                return True, "Gesture failed"
        
        return False, "Unknown gesture"

    def handle_led(self, result):
        """Handle LED color commands"""
        action = result.get("action", "").lower()
        
        if action == "off":
            self.audio_client.led_control(0, 0, 0)
            return True, "Lights turned off"
        
        rgb = self.led_colors.get(action)
        if rgb:
            # Call hardware client to set color
            self.audio_client.led_control(*rgb)
            return True, f"{action.title()} lights on"
        
        return False, "Unknown color"

    def handle_volume(self, result):
        """Handle volume commands"""
        action = result.get("action", "").lower()
        
        if action == "up":
            self.audio_client.set_volume(100)
            return True, "Volume set to maximum"
        
        elif action == "down":
            self.audio_client.set_volume(30)
            return True, "Volume lowered"
        
        elif action == "set":
            vol = result.get("parameters", {}).get("volume", 50)
            vol = max(0, min(100, vol))  # Clamp between 0-100
            self.audio_client.set_volume(vol)
            return True, f"Volume set to {vol}"
        
        return False, "Unknown volume command"

    # --------------------------------------------------
    # Gesture Trigger
    # --------------------------------------------------
    def trigger_gesture(self, action_id):
        """
        Send gesture action via loco_publisher.
        This constructs the specific API packet for the robot's locomotion controller.
        """
        try:
            # API 7106 is for "Sport Mode" actions
            req = self.audio_client._create_request(7106, json.dumps({"data": action_id}))
            self.audio_client.loco_publisher.publish(req)

            # Special case: Handshake needs to be released after a few seconds
            if action_id == 2:
                threading.Thread(target=self._delayed_release, daemon=True).start()

            return True
        except Exception as e:
            print(f"❌ Gesture failed: {e}")
            return False

    def _delayed_release(self):
        """Auto-release handshake after 5 seconds"""
        time.sleep(5)
        try:
            # Action 3 is "Stand/Release"
            req = self.audio_client._create_request(7106, json.dumps({"data": 3}))
            self.audio_client.loco_publisher.publish(req)
            print("✅ Auto-released handshake")
        except Exception as e:
            print(f"❌ Auto-release failed: {e}")