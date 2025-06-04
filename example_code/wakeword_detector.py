import time
import wave
import os
import numpy as np
import pyaudio
# import torch # No longer needed
# import torchaudio # No longer needed
from piper.voice import PiperVoice
import mlx_whisper # Using mlx-whisper directly

# TTS setup
model_path = "en_US-ryan-low.onnx"
if not os.path.exists(model_path):
    print(f"TTS model not found at {model_path}.")
    print("Consider running the download_voice.py script in this directory.")
    # Attempt to find it in the parent directory if not in current.
    if os.path.exists(os.path.join("..", model_path)):
        model_path = os.path.join("..", model_path)
        print(f"Found TTS model at {model_path}")
    else:
        exit()

try:
    VOICE = PiperVoice.load(model_path)
    print("TTS model loaded successfully.")
except Exception as e:
    print(f"Error loading TTS model: {e}")
    exit()

def speak(text):
    print(f"TTS: {text}")
    try:
        with wave.open("output.wav", "w") as wav_file:
            wav_file.setnchannels(1)
            sample_width_bytes = 2 
            sample_rate_hz = VOICE.config.sample_rate
            wav_file.setsampwidth(sample_width_bytes)
            wav_file.setframerate(sample_rate_hz)
            VOICE.synthesize(text, wav_file)
        os.system("afplay output.wav") # macOS specific
        # For other OS, you might need:
        # Linux: os.system("aplay output.wav") or os.system("paplay output.wav")
        # Windows: os.system("start output.wav")
        os.remove("output.wav")
    except Exception as e:
        print(f"Error during TTS: {e}")

# Wake words
WAKE_WORDS = ["alfred", "robot", "guy"]

# Audio settings
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000  # Whisper model expects 16kHz
CHUNK_DURATION_MS = 250  # Read audio in smaller chunks (e.g., 250ms)
CHUNK_SAMPLES = int(RATE * CHUNK_DURATION_MS / 1000)
PROCESS_INTERVAL_SECONDS = 1.0 # Process 1 second of audio at a time
PROCESS_SAMPLES = int(RATE * PROCESS_INTERVAL_SECONDS)

# VAD Settings
VAD_THRESHOLD = 0.008  # RMS energy threshold for silence detection (needs tuning)

# MLX Whisper Model
ASR_MODEL_NAME = "mlx-community/whisper-base.en-mlx"
LANGUAGE = "en"

print(f"Using MLX Whisper model: {ASR_MODEL_NAME}")
# Model will be loaded by the first call to mlx_whisper.transcribe

print("Listening for wake words...")
print(f"Wake words: {WAKE_WORDS}")
print(f"VAD RMS Threshold: {VAD_THRESHOLD}")

audio_interface = pyaudio.PyAudio()
stream = audio_interface.open(format=FORMAT,
                              channels=CHANNELS,
                              rate=RATE,
                              input=True,
                              frames_per_buffer=CHUNK_SAMPLES)

audio_buffer_np = np.array([], dtype=np.float32)

try:
    while True:
        audio_chunk_bytes = stream.read(CHUNK_SAMPLES, exception_on_overflow=False)
        chunk_np = np.frombuffer(audio_chunk_bytes, dtype=np.int16).astype(np.float32) / 32768.0
        
        audio_buffer_np = np.concatenate((audio_buffer_np, chunk_np))

        if len(audio_buffer_np) >= PROCESS_SAMPLES:
            # Process the latest PROCESS_SAMPLES from the buffer
            process_segment_np = audio_buffer_np[-PROCESS_SAMPLES:]

            # Simple VAD: Calculate RMS energy
            rms = np.sqrt(np.mean(process_segment_np**2))
            # print(f"Processing audio segment: {len(process_segment_np)} samples, RMS: {rms:.4f}") # Verbose

            if rms < VAD_THRESHOLD:
                # print("Silence detected (RMS below threshold), skipping transcription.") # Verbose
                pass
            else:
                print(f"Sufficient audio detected (RMS: {rms:.4f}), transcribing...")
                try:
                    # Transcribe the 1-second segment
                    result = mlx_whisper.transcribe(
                        audio=process_segment_np,
                        path_or_hf_repo=ASR_MODEL_NAME,
                        language=LANGUAGE,
                        # verbose=False # Optional: set to True for more detailed Whisper logs
                    )
                    transcribed_text = result['text'].strip().lower()
                    print(f"Transcribed: {transcribed_text}")

                    if transcribed_text:
                        for wake_word in WAKE_WORDS:
                            if wake_word in transcribed_text:
                                print(f"Wake word '{wake_word}' detected!")
                                speak("I hear you.")
                                # Clear buffer after wake word to avoid re-triggering immediately
                                audio_buffer_np = np.array([], dtype=np.float32) 
                                break 
                except Exception as e:
                    print(f"Error during transcription: {e}")
            
            # Trim the buffer: keep only the part that wasn't processed 
            # or, for non-overlapping windows, clear up to the processed segment.
            # We take the most recent samples for processing.
            # To make it a rolling buffer of roughly PROCESS_SAMPLES:
            if len(audio_buffer_np) > PROCESS_SAMPLES:
                 audio_buffer_np = audio_buffer_np[-PROCESS_SAMPLES:] # Keep last second as a simple rolling buffer
            # Or, to ensure non-overlapping chunks are mostly processed:
            # audio_buffer_np = audio_buffer_np[PROCESS_SAMPLES:] # If CHUNK_SAMPLES divides PROCESS_SAMPLES cleanly
            # Given CHUNK_DURATION_MS = 250 and PROCESS_INTERVAL_SECONDS = 1.0, four chunks make a segment.
            # So, after processing PROCESS_SAMPLES, we should remove that many from the start of the buffer.
            # However, we took the *last* PROCESS_SAMPLES.
            # A better way for rolling buffer to ensure we don't miss anything at seams:
            # Keep a buffer slightly larger than PROCESS_SAMPLES and slide the window.
            # For now, the current approach of processing the tail and then trimming should be okay for basic wake word.
            # Let's try a more explicit trim:
            # audio_buffer_np = audio_buffer_np[CHUNK_SAMPLES:] # Slide buffer by one CHUNK_SAMPLES after each read and potential process
            # Corrected sliding window: remove the oldest CHUNK_SAMPLES to make space for the new one
            if len(audio_buffer_np) >= CHUNK_SAMPLES:
                audio_buffer_np = audio_buffer_np[CHUNK_SAMPLES:]


        # Prevent busy-waiting if buffer is not full enough
        # The stream.read is blocking, so this might not be strictly necessary unless CHUNK_SAMPLES is tiny
        # time.sleep(0.01) 

except KeyboardInterrupt:
    print("Stopping...")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    if 'stream' in locals() and stream.is_active():
        stream.stop_stream()
        stream.close()
    if 'audio_interface' in locals():
        audio_interface.terminate()
    print("Audio stream closed.") 