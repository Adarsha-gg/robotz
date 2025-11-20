# G1 Robot Voice & GPT Architecture

This document provides a high-level overview of the G1 Robot's voice interaction system. The system integrates the robot's hardware capabilities (microphone, speaker, LEDs, gestures) with an external LLM (Groq) for intelligent conversation and command execution.

## System Overview

The core of the system is a ROS 2 node (`VoiceGPTNode`) that acts as the central orchestrator. It intercepts audio from the robot, processes it to determine if it's a command or a conversation, and then executes the appropriate action (speaking back or performing a physical action).

### Key Components

1.  **VoiceGPTNode** (`audio/voice_gpt_node.py`)
    *   **Role**: The main brain of the application.
    *   **Function**: Initializes all subsystems, manages the main event loop, handles ROS 2 topics, and coordinates between audio input, LLM processing, and output.
    *   **Key Feature**: It actively manages the robot's microphone mode (switching between "always listening" and "push-to-talk") and disables the robot's built-in GPT to replace it with your custom one.

2.  **G1AudioClient** (`audio/audio_client.py`)
    *   **Role**: The hardware abstraction layer.
    *   **Function**: Wraps the Unitree API to send low-level requests to the robot.
    *   **Capabilities**:
        *   **Voice**: TTS, Volume Control, Audio Playback.
        *   **Locomotion**: Triggering gestures (wave, handshake, etc.).
        *   **LEDs**: Controlling the robot's face lights.

3.  **GPTHandler** (`audio/gpt_handler.py`)
    *   **Role**: The conversational engine.
    *   **Function**: Manages the session with the Groq LLM (Llama 3).
    *   **Features**: Maintains conversation history (context) and manages the system prompt (persona).

4.  **CommandParser** (`audio/command_parser.py`)
    *   **Role**: The intent classifier.
    *   **Function**: Uses a specialized LLM prompt to analyze user text and determine if it's a command (e.g., "turn red", "wave hello") or just chat.
    *   **Output**: Returns structured JSON actions (`gesture`, `led`, `volume`) if a command is detected.

5.  **TTSEngine** (`audio/tts_engine.py`)
    *   **Role**: The voice synthesizer.
    *   **Function**: Converts text responses into audio using **Piper TTS** (running locally).
    *   **Process**: Generates a WAV file, converts it to the specific format required by the G1 robot (16kHz, 16-bit, mono), and sends it via `G1AudioClient`.

6.  **LEDController** (`audio/led_controller.py`)
    *   **Role**: Visual feedback.
    *   **Function**: Provides advanced LED animations like breathing, pulsing, and rainbow effects to make the robot feel "alive".

## Architecture Diagram

```mermaid
graph TD
    subgraph Robot Hardware
        Mic[Microphone]
        Speaker[Speaker]
        LEDs[Face LEDs]
        Motors[Arm Motors]
    end

    subgraph ROS2 System
        MsgAudio[/audio_msg/]
        MsgController[/wirelesscontroller/]
        
        Node[VoiceGPTNode]
        
        Client[G1AudioClient]
        TTS[TTSEngine]
        GPT[GPTHandler]
        Parser[CommandParser]
    end

    subgraph External Services
        Groq[Groq API (Llama 3)]
    end

    %% Input Flow
    Mic --> MsgAudio
    MsgAudio --> Node
    MsgController --> Node
    
    %% Processing Flow
    Node --> Parser
    Parser -- "Is Command?" --> Groq
    Groq -- "JSON Action" --> Parser
    
    Parser -- "Command Found" --> Client
    Parser -- "No Command" --> GPT
    
    GPT -- "Conversation" --> Groq
    Groq -- "Text Response" --> GPT
    GPT --> Node
    
    %% Output Flow
    Node -- "Text to Speak" --> TTS
    TTS -- "Generate WAV" --> Piper[Piper Local]
    Piper --> Client
    
    Client -- "API 1003 (Audio)" --> Speaker
    Client -- "API 1010 (LED)" --> LEDs
    Client -- "API 7106 (Gesture)" --> Motors
```

## Data Flow Description

1.  **Input**:
    *   The robot's microphone captures speech and publishes transcribed text to `/audio_msg`.
    *   `VoiceGPTNode` receives this text.

2.  **Decision**:
    *   The text is passed to `CommandParser`.
    *   The parser asks Groq: "Is this a command?"
    *   **If Command**: The parser extracts the action (e.g., `{"action": "wave"}`) and `G1AudioClient` executes it immediately.
    *   **If Chat**: The text is passed to `GPTHandler`, which sends it to Groq (with conversation history) to get a verbal response.

3.  **Output**:
    *   The text response from `GPTHandler` is sent to `TTSEngine`.
    *   `TTSEngine` generates audio and sends it to the robot's speaker.
    *   Simultaneously, `VoiceGPTNode` might trigger LED effects (like a "breathing" purple light) to indicate listening or processing states.

## File Map

| File | Purpose |
|------|---------|
| `main.py` | Entry point. Sets up the ROS node and keyboard control. |
| `audio/voice_gpt_node.py` | Main logic. Connects all pieces together. |
| `audio/audio_client.py` | Unitree API wrapper. Handles all robot communication. |
| `audio/gpt_handler.py` | Chatbot logic. Manages LLM context. |
| `audio/command_parser.py` | Command understanding. Maps text to robot actions. |
| `audio/tts_engine.py` | Text-to-Speech generation (Piper). |
| `audio/led_controller.py` | Fancy LED patterns. |
