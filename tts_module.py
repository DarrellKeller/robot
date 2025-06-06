import wave
import os
from piper.voice import PiperVoice
import re # Import the regular expression module

# TTS setup
MODEL_PATH = "en_US-ryan-high.onnx" # Consider making this configurable or passed in
VOICE = None

def initialize_tts():
    global VOICE
    if not os.path.exists(MODEL_PATH):
        print(f"TTS model not found at {MODEL_PATH}.")
        # Return a status or raise an exception
        return False
    try:
        VOICE = PiperVoice.load(MODEL_PATH)
        print("TTS model loaded successfully.")
        return True
    except Exception as e:
        print(f"Error loading TTS model: {e}")
        return False

def speak(text):
    if VOICE is None:
        print("TTS not initialized. Call initialize_tts() first.")
        return

    # Sanitize the text
    # Keep Latin alphabet (a-z, A-Z), numbers (0-9), spaces, periods, and exclamation marks.
    # Remove everything else.
    sanitized_text = re.sub(r'[^a-zA-Z0-9 .!\']', '', text)

    # print(f"TTS (Sanitized): {sanitized_text}")
    output_path = "output.wav" # Ensure this path is writable
    try:
        with wave.open(output_path, "w") as wav_file:
            wav_file.setnchannels(1)
            # PiperVoice.synthesize will set sample_width and sample_rate based on the model
            # We only need to ensure the channels.
            # sample_width_bytes = VOICE.config.sample_width # typically 2 for 16-bit
            # sample_rate_hz = VOICE.config.sample_rate
            # wav_file.setsampwidth(sample_width_bytes)
            # wav_file.setframerate(sample_rate_hz)
            
            # The synthesize method handles wave file parameters.
            VOICE.synthesize(sanitized_text, wav_file)
        
        # Use a platform-independent way to play sound if possible, or keep afplay for macOS
        # For broader compatibility, consider libraries like 'sounddevice' or 'playsound'
        os.system(f"afplay {output_path}") # afplay is macOS specific
        os.remove(output_path)
    except AttributeError as ae:
        if "config" in str(ae).lower() or "sample_rate" in str(ae).lower():
             print(f"TTS Error: VOICE object might not be fully initialized or is missing attributes like 'config'. Voice: {VOICE}")
        else:
            print(f"An attribute error occurred during TTS: {ae}")

    except Exception as e:
        print(f"Error during TTS: {e}")

if __name__ == '__main__':
    # Example usage:
    if initialize_tts():
        speak("Hello, this is a test of the text to speech system.")
        speak("I am a sassy robot.") 