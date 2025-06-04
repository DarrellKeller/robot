# Robot Control Project

This project controls a robot with autonomous navigation capabilities, voice interaction, and AI-driven decision-making.

## Overview

The robot uses an ESP32 microcontroller for low-level motor control and sensor readings (Time-of-Flight distance sensors). A Python application running on a host computer (a mac mini) handles higher-level logic, including:

*   Serial communication with the ESP32.
*   Wake word detection and speech-to-text using local models (Piper TTS, MLX Whisper).
*   Image capture and analysis using local vision models (e.g., MLX-VLM, Moondream via Ollama).
*   Decision-making using a local large language model (LLM via Ollama, e.g., Gemma3).
*   Executing actions based on LLM decisions (e.g., navigation, speaking, surveying).

## Core Components

*   `autonomous_control.py`: The main Python script that orchestrates all robot functions and manages its state.
*   `robot_actions.py`: Module for sending specific commands (move, turn, stop, etc.) to the ESP32.
*   `tts_module.py`: Handles Text-to-Speech using Piper.
*   `vision_module.py`: Handles image capture and analysis using MLX-VLM or a similar local vision model.
*   `thinking_module.py`: Interacts with a local LLM (via Ollama) to get decisions based on sensory input or user commands.
*   `wakeword_server.py`: A separate process that continuously listens for a wake word, transcribes user speech, and communicates with `autonomous_control.py` via flag files.
*   `motor_control/motor_control.ino`: Arduino sketch for the ESP32, managing motors, sensors, and PID control for basic obstacle avoidance.
*   `example_code/`: Contains various standalone scripts for testing individual components (TTS, vision, ESP32 direct control, etc.).

## Setup

1.  **Hardware:**
    *   ESP32 microcontroller.
    *   Robot chassis with motors and motor driver.
    *   Time-of-Flight (ToF) distance sensors (e.g., VL53L0X) connected via an I2C multiplexer (e.g., TCA9548A).
    *   Webcam connected to the host computer.
    *   Microphone connected to the host computer.
    *   Speakers connected to the host computer.

2.  **ESP32 Firmware:**
    *   Flash `motor_control/motor_control.ino` to your ESP32 using the Arduino IDE.
    *   Ensure all necessary libraries (e.g., `Wire`, `VL53L0X`, `PID_v1`) are installed in your Arduino environment.

3.  **Python Environment (Host Computer):**
    *   Create a Python virtual environment (recommended).
    *   Install required Python packages:
        ```bash
        pip install -r requirements.txt
        ```
    *   Ensure Ollama is installed and running, and that you have pulled the necessary models (e.g., `ollama pull gemma3`, `ollama pull moondream`). Refer to Ollama's documentation for setup.
    *   Ensure MLX and its related vision/audio models are set up if you intend to use them directly. Refer to MLX documentation.
    *   Download TTS voice models (e.g., `en_US-ryan-low.onnx` and its `.json` config file) and place them in the `robot/` directory or `robot/example_code/` for the example script. The `example_code/download_voice.py` script can assist with this.

4.  **Configuration:**
    *   In `autonomous_control.py`:
        *   Update `SERIAL_PORT` to match your ESP32's serial port.
    *   In `tts_module.py`, `vision_module.py`, `thinking_module.py`, and `wakeword_server.py`:
        *   Verify model names and paths if you are using different local models or configurations.
        *   The vision module (`vision_module.py`) and thinking module (`thinking_module.py`) are configured to use MLX-VLM and Ollama respectively. Adjust if using different backends.
    *   The `wakeword_server.py` and `autonomous_control.py` use flag files for inter-process communication. Ensure write permissions for these files in the project directory.

## Running the Robot

1.  **Start Ollama Service:** Ensure your Ollama service is running with the required models downloaded.
2.  **Start Wake Word Server:**
    ```bash
    python robot/wakeword_server.py
    ```
3.  **Start Main Control Script:** In a new terminal:
    ```bash
    python robot/autonomous_control.py
    ```

The robot should initialize, and if configured to start autonomously, it will begin navigating. You can interact with it using the defined wake word.

## ESP32 Serial Commands (for direct control via `example_code/esp32_control.py` or other serial tools)

*   `w`: Move forward
*   `s`: Move backward
*   `a`: Turn left
*   `d`: Turn right
*   `x`: Stop motors
*   `p`: Toggle PID autonomous mode on/off
*   `0`-`9`: Set base speed (0 for max speed ~100%, 9 for min speed ~10%)

## Project Structure

```
robot/
├── autonomous_control.py       # Main control logic
├── robot_actions.py            # Functions to send commands to ESP32
├── tts_module.py               # Text-to-Speech handling
├── vision_module.py            # Image capture and analysis
├── thinking_module.py          # LLM interaction for decisions
├── wakeword_server.py          # Wake word detection and speech capture server
├── requirements.txt            # Python dependencies
├── README.md                   # This file
├── example_code/               # Directory for standalone example scripts
│   ├── esp32_control.py
│   ├── mlx_webcam_vision.py
│   ├── piper_with_playback.py
│   ├── scene_analyzer.py
│   ├── test_original_format.py
│   ├── download_voice.py
│   ├── wakeword_detector.py
│   ├── webcam_vision_moondream_local.py
│   └── webcam_vision.py
├── motor_control/
│   └── motor_control.ino       # ESP32 firmware
├── en_US-ryan-low.onnx         # Example TTS voice model
└── en_US-ryan-low.onnx.json    # Example TTS voice model config
```

## Notes

*   The system relies heavily on local AI models. Performance will vary based on the host computer's hardware.
*   Paths to models and serial ports may need adjustment based on your specific setup.
*   The inter-process communication via flag files is a simple mechanism; for more complex applications, a more robust IPC method (like sockets or message queues) might be considered. 