"""Robot Configuration"""
import os
from dotenv import load_dotenv

load_dotenv()

class Config:
    # API Keys
    GROQ_API_KEY = os.getenv('GROQ_API_KEY')
    
    # Paths
    PIPER_MODEL = '/home/pass/robot_cpp/piper_voices/en_US-lessac-medium.onnx'
    
    # Audio Settings
    AUDIO_SAMPLE_RATE = 16000
    AUDIO_CHANNELS = 1
    
    # GPT Settings
    GPT_MODEL = "llama-3.3-70b-versatile"
    GPT_MAX_HISTORY = 10
    SYSTEM_PROMPT = "Your name is Willy. Keep responses under 2 sentences."
    
    # Robot Settings
    DEFAULT_VOLUME = 50
    DEFAULT_VOICE_ID = 1  # English
    
    # API IDs
    API_TTS = 1001
    API_PLAY_AUDIO = 1003
    API_STOP_AUDIO = 1004
    API_GET_VOLUME = 1005
    API_SET_VOLUME = 1006
    API_VOICE_STATUS = 1007  # Voice status query
    API_VOICE_MODE = 1008    # Voice mode control
    API_LED = 1010
    
    # Voice Modes
    VOICE_MODE_WAKE_WORD = 1
    VOICE_MODE_CLOSE_INTERACTION = 3  # Continuous listening