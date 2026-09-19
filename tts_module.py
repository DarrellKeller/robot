import wave
import os
import subprocess
import shutil
import sys
import importlib.util
from piper.voice import PiperVoice
import re # Import the regular expression module

try:
    import pyaudio
except Exception:
    pyaudio = None

ALLOW_SAY_FALLBACK = os.getenv("TTS_ALLOW_SAY_FALLBACK") == "1"

# TTS setup
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "en_US-ryan-high.onnx") # Consider making this configurable or passed in
VOICE = None

def initialize_tts():
    global VOICE
    espeakbridge_ok = importlib.util.find_spec("piper.espeakbridge") is not None
    if not espeakbridge_ok:
        print("TTS Warning: piper.espeakbridge not found in this Python environment.")
        print(f"TTS Debug: python={sys.executable}")
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
        if ALLOW_SAY_FALLBACK and shutil.which("say"):
            return subprocess.run(["say", text], check=False).returncode == 0
        return False

    # Sanitize the text
    # Keep Latin alphabet (a-z, A-Z), numbers (0-9), spaces, periods, and exclamation marks.
    # Remove everything else.
    sanitized_text = re.sub(r'[^a-zA-Z0-9 .!\']', '', text)

    # print(f"TTS (Sanitized): {sanitized_text}")
    output_path = os.path.join(SCRIPT_DIR, "output.wav") # Ensure this path is writable
    keep_output = os.getenv("KEEP_TTS_OUTPUT") == "1"
    try:
        with wave.open(output_path, "w") as wav_file:
            num_channels = getattr(VOICE.config, "num_channels", None) or 1
            sample_width = getattr(VOICE.config, "sample_width", None) or getattr(VOICE.config, "sample_width_bytes", None)
            sample_rate = getattr(VOICE.config, "sample_rate", None) or getattr(VOICE.config, "sample_rate_hz", None)
            if not sample_width:
                sample_width = 2
                print("TTS Warning: sample_width missing; defaulting to 2 bytes.")
            if not sample_rate:
                sample_rate = 22050
                print("TTS Warning: sample_rate missing; defaulting to 22050 Hz.")
            wav_file.setnchannels(num_channels)
            wav_file.setsampwidth(sample_width)
            wav_file.setframerate(sample_rate)

            try:
                for chunk in VOICE.synthesize(sanitized_text):
                    if hasattr(chunk, "audio_int16_bytes"):
                        if not sample_rate:
                            sample_rate = getattr(chunk, "sample_rate", None) or sample_rate
                            if sample_rate:
                                wav_file.setframerate(sample_rate)
                        wav_file.writeframes(chunk.audio_int16_bytes)
                    else:
                        wav_file.writeframes(chunk)
            except Exception as synth_error:
                if ALLOW_SAY_FALLBACK and shutil.which("say"):
                    print(f"TTS synth failed: {synth_error}. Falling back to system voice.")
                    return subprocess.run(["say", sanitized_text], check=False).returncode == 0
                else:
                    print(f"TTS synth failed: {synth_error}. No fallback enabled.")
                    print(f"TTS Debug: python={sys.executable}")
                return False
        # Optionally keep output.wav for debugging
        
        # Playback: try pyaudio first (cross-platform), then OS commands.
        if pyaudio:
            try:
                with wave.open(output_path, "rb") as wav_file:
                    print("Audio playback: pyaudio")
                    p = pyaudio.PyAudio()
                    stream = p.open(
                        format=p.get_format_from_width(wav_file.getsampwidth()),
                        channels=wav_file.getnchannels(),
                        rate=wav_file.getframerate(),
                        output=True,
                    )
                    chunk = 1024
                    data = wav_file.readframes(chunk)
                    while data:
                        stream.write(data)
                        data = wav_file.readframes(chunk)
                    stream.stop_stream()
                    stream.close()
                    p.terminate()
            except Exception as e:
                print(f"Audio playback via pyaudio failed: {e}")
            else:
                if os.path.exists(output_path) and not keep_output:
                    os.remove(output_path)
                return True

        if shutil.which("afplay"):
            print("Audio playback: afplay")
            result = subprocess.run(["afplay", output_path], check=False)
            if result.returncode != 0:
                print(f"Audio playback failed: afplay exited with {result.returncode}. File kept at {output_path}.")
                return False
        elif shutil.which("aplay"):
            print("Audio playback: aplay")
            result = subprocess.run(["aplay", output_path], check=False)
            if result.returncode != 0:
                print(f"Audio playback failed: aplay exited with {result.returncode}. File kept at {output_path}.")
                return False
        elif shutil.which("paplay"):
            print("Audio playback: paplay")
            result = subprocess.run(["paplay", output_path], check=False)
            if result.returncode != 0:
                print(f"Audio playback failed: paplay exited with {result.returncode}. File kept at {output_path}.")
                return False
        else:
            print(f"Audio playback failed: no known playback command available. File kept at {output_path}.")
            return False
        if os.path.exists(output_path) and not keep_output:
            os.remove(output_path)
        return True
    except AttributeError as ae:
        if "config" in str(ae).lower() or "sample_rate" in str(ae).lower():
             print(f"TTS Error: VOICE object might not be fully initialized or is missing attributes like 'config'. Voice: {VOICE}")
        else:
            print(f"An attribute error occurred during TTS: {ae}")

    except Exception as e:
        print(f"Error during TTS: {e}")

    return False

if __name__ == '__main__':
    # Example usage:
    if initialize_tts():
        speak("Hello, this is a test of the text to speech system.")
        speak("I am a sassy robot.") 