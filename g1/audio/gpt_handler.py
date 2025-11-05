#!/usr/bin/env python3
"""
GPT Handler for G1 Robot
Manages conversations with GPT
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
            tts_engine: TTSEngine instance for speaking responses
            system_prompt: Custom system prompt
            max_history: Maximum conversation history to keep
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
        self.conversation_history = []
        
        # System prompt
        self.system_prompt = system_prompt or (
            "Your name is Any. You are a robot for William Paterson University. "
            "You do not answer in more than 2 sentences."
            "You are speaking not texting."
            "You dont have to mention William Paterson University everytime "
        )
    
    def process_text(self, text):
        """
        Process text input through GPT and return response
        
        Args:
            text: User input text
        
        Returns:
            GPT response text
        """
        # Check for special commands first
        if self.command_parser:
            is_command, response = self.command_parser.parse(text)
            if is_command:
                # Don't speak here - let voice_gpt_node handle it
                return response
        
        # Otherwise process through GPT
        try:
            # Add user message
            self.conversation_history.append({
                "role": "user",
                "content": text
            })
            
            # Trim history
            if len(self.conversation_history) > self.max_history:
                self.conversation_history = self.conversation_history[-self.max_history:]
            
            # Get GPT response
            response = self.client.chat.completions.create(
                model="llama-3.3-70b-versatile",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    *self.conversation_history
                ]
            )
            
            reply = response.choices[0].message.content
            
            # Add to history
            self.conversation_history.append({
                "role": "assistant",
                "content": reply
            })
            
            print(f'GPT: {reply}')
            
            # IMPORTANT: Just return, don't speak!
            return reply
            
        except Exception as e:
            print(f'GPT error: {e}')
            return None
    
    def clear_history(self):
        """Clear conversation history"""
        self.conversation_history = []
    
    def set_system_prompt(self, prompt):
        """Update system prompt"""
        self.system_prompt = prompt
        self.clear_history()