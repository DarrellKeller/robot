import wave
import os
import sys
from piper.voice import PiperVoice

# Path to the ONNX voice model
# Update this path to where your voice model is located
MODEL_PATH = "en_US-ryan-low.onnx" # Example model, replace if needed

def ensure_model_exists(model_path):
    """Check if the model file exists, print error and exit if not."""
    if not os.path.exists(model_path):
        print(f"Error: TTS model not found at '{model_path}'.")
        print("Please ensure the model is downloaded and the path is correct.")
        # You might want to add instructions on how to download it, e.g.:
        # print("You can download models from: https://huggingface.co/rhasspy/piper-voices/tree/main")
        sys.exit(1)

def initialize_tts(model_path):
    """Initialize and return the PiperVoice TTS engine."""
    try:
        voice = PiperVoice.load(model_path)
        print(f"TTS model '{model_path}' loaded successfully.")
        return voice
    except Exception as e:
        print(f"Error loading TTS model: {e}")
        sys.exit(1)

def speak_text(voice_engine, text_to_speak, output_filename="output.wav"):
    """Synthesize text to speech and save to a WAV file, then play it."""
    if not text_to_speak.strip():
        print("No text provided to speak.")
        return

    print(f"Synthesizing: '{text_to_speak}'")
    try:
        with wave.open(output_filename, "w") as wav_file:
            # Configure WAV file settings from the voice model
            wav_file.setnchannels(voice_engine.config.num_channels)
            wav_file.setsampwidth(voice_engine.config.sample_width) # Bytes per sample
            wav_file.setframerate(voice_engine.config.sample_rate)
            
            # Synthesize and write to file
            voice_engine.synthesize(text_to_speak, wav_file)
        
        print(f"Speech saved to {output_filename}")

        # Play the generated WAV file
        # Using afplay for macOS. For other systems, you might need a different command:
        # Linux: aplay output.wav or paplay output.wav
        # Windows: start output.wav (might open in default media player)
        if sys.platform == "darwin": # macOS
            os.system(f"afplay {output_filename}")
        elif sys.platform == "linux":
            # Try aplay, then paplay if aplay is not found
            if os.system(f"aplay {output_filename}") != 0:
                os.system(f"paplay {output_filename}")
        else:
            print(f"Playback not automatically supported on this OS. Please play {output_filename} manually.")
            
        # Clean up the temporary WAV file
        # os.remove(output_filename) # Optional: remove after playing
        print(f"You can delete {output_filename} manually if no longer needed.")

    except Exception as e:
        print(f"Error during TTS synthesis or playback: {e}")

if __name__ == "__main__":
    ensure_model_exists(MODEL_PATH)
    tts_engine = initialize_tts(MODEL_PATH)
    
    text = "Hello, this is a test of the Piper text to speech system with audio playback."
    speak_text(tts_engine, text)

    text2 = "I hope you can hear me loud and clear!"
    speak_text(tts_engine, text2, output_filename="output2.wav") 