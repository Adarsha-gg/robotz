#!/usr/bin/env python3
"""
GPT Handler for G1 Robot
Manages conversations with GPT

ARCHITECTURE OVERVIEW:
1. This module manages the "personality" and "memory" of the robot.
2. It maintains a list of previous messages (conversation_history) so the robot remembers what you just said.
3. It sends the entire history + the new message to Groq (Llama 3) to get a response.
4. It also checks for commands FIRST before asking GPT, to ensure fast reaction times.
"""
import os
from groq import Groq
from dotenv import load_dotenv

class GPTHandler:
    """Handles GPT conversations with context memory"""
    
    def __init__(self, tts_engine, system_prompt=None, max_history=10, command_parser=None, groq_client=None):
        """
        Initialize GPT handler
        
        Args:
            tts_engine: TTSEngine instance for speaking responses (not used directly here, but good for reference)
            system_prompt: Custom system prompt (the "persona" of the robot)
            max_history: How many previous messages to keep in memory (prevents token overflow)
        """
        self.tts_engine = tts_engine
        self.max_history = max_history
        self.command_parser = command_parser
        
        # Load environment variables
        load_dotenv()
        
        # Initialize Groq client
        api_key = os.getenv('GROQ_API_KEY')
        if not api_key:
            raise ValueError('GROQ_API_KEY not found in .env file!')
        
        self.client = groq_client or Groq(api_key=os.getenv("GROQ_API_KEY"))
        
        # Conversation history
        # This list grows as we talk. We must trim it occasionally.
        self.conversation_history = []
        
        # System prompt
        # This tells the AI who it is and how to behave.
        self.system_prompt = system_prompt or (
            "Your name is Any. You are a robot for William Paterson University. "
            "You do not answer in more than 2 sentences."
            "You are speaking not texting."
            "You dont have to mention William Paterson University everytime "
        )
    
    def process_text(self, text):
        """
        Process text input through GPT and return response
        
        Flow:
        1. Check if the text is actually a command (e.g. "stop").
        2. If it is, execute it and return the result.
        3. If not, append the text to conversation history.
        4. Send history to Groq.
        5. Receive response, append to history, and return it.
        
        Args:
            text: User input text
        
        Returns:
            GPT response text
        """
        # Step 1: Check for special commands first
        # We do this here as a backup, although voice_gpt_node usually handles it.
        if self.command_parser:
            is_command, response = self.command_parser.parse(text)
            if is_command:
                # Don't speak here - let voice_gpt_node handle it
                return response
        
        # Step 2: Otherwise process through GPT
        try:
            # Add user message to history
            self.conversation_history.append({
                "role": "user",
                "content": text
            })
            
            # Trim history if it gets too long
            # We keep the last N messages to maintain context without using too many tokens.
            if len(self.conversation_history) > self.max_history:
                self.conversation_history = self.conversation_history[-self.max_history:]
            
            # Step 3: Get GPT response
            # We send the System Prompt + History to the model.
            response = self.client.chat.completions.create(
                model="llama-3.3-70b-versatile",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    *self.conversation_history
                ]
            )
            
            reply = response.choices[0].message.content
            
            # Step 4: Add assistant response to history
            self.conversation_history.append({
                "role": "assistant",
                "content": reply
            })
            
            print(f'GPT: {reply}')
            
            # IMPORTANT: Just return the text, don't speak!
            # The calling node (voice_gpt_node) handles the actual speaking.
            return reply
            
        except Exception as e:
            print(f'GPT error: {e}')
            return None
    
    def clear_history(self):
        """Clear conversation history (reset memory)"""
        self.conversation_history = []
    
    def set_system_prompt(self, prompt):
        """Update system prompt (change persona)"""
        self.system_prompt = prompt
        self.clear_history()