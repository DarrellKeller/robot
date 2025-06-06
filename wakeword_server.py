import time
import os
import numpy as np
import pyaudio
import mlx_whisper # Restored mlx-whisper
import random # Added for random choice of acknowledgements
import importlib

# Import speak function from your tts_module
from tts_module import initialize_tts, speak # Assuming tts_module.py is in the same directory or PYTHONPATH

# Wake words
WAKE_WORDS = ["robot", "guy", "toad", "spinny", "mauricio", "maurices", "maurice", "mauricey", "maricio"] # Keep your desired wake words

# Acknowledgement phrases
ACKNOWLEDGEMENTS = ["Hey!", "What's up?", "What?", "Need something?", "Yes?", "I'm here.", "I'm listening."]

# Audio settings
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000  # Whisper model expects 16kHz
CHUNK_DURATION_MS = 250
CHUNK_SAMPLES = int(RATE * CHUNK_DURATION_MS / 1000)
PROCESS_INTERVAL_SECONDS = 1.0 # Duration of audio segment processed for wake words
PROCESS_SAMPLES = int(RATE * PROCESS_INTERVAL_SECONDS)

# VAD Settings
VAD_THRESHOLD = 0.02 # RMS energy threshold - may need tuning

# MLX Whisper Model
ASR_MODEL_NAME = "mlx-community/whisper-base.en-mlx"
LANGUAGE = "en"

# Transcription Quality Filters
MAX_WORDS_PER_INTERVAL = 15  # Max plausible words for PROCESS_INTERVAL_SECONDS
MAX_CHARS_PER_INTERVAL = 75  # Max plausible characters for PROCESS_INTERVAL_SECONDS

# IPC Flag File
WAKE_WORD_FLAG_FILE = "WAKE_WORD_DETECTED.flag"
LISTENING_COMPLETE_FLAG_FILE = "LISTENING_COMPLETE.flag"
USER_SPEECH_FILE = "user_speech.txt"

# Global variable to control the main listening loop of the server
server_running = True

def create_flag_file(filepath):
    try:
        with open(filepath, 'w') as f:
            f.write(str(time.time())) # Write timestamp to it
        print(f"Created flag file: {filepath}")
    except IOError as e:
        print(f"Error creating flag file {filepath}: {e}")

def clear_flag_file(filepath):
    if os.path.exists(filepath):
        try:
            os.remove(filepath)
            print(f"Cleared flag file: {filepath}")
        except OSError as e:
            print(f"Error clearing flag file {filepath}: {e}")

def do_listen_and_transcribe(audio_stream, play_acknowledgment=False, acknowledgment_text=""):
    """
    Centralized function to listen for user speech, transcribe it, and create result files.
    
    Args:
        audio_stream: The pyaudio stream object
        play_acknowledgment: Whether to play an initial acknowledgment sound
        acknowledgment_text: What to say for acknowledgment (if play_acknowledgment is True)
    """
    if play_acknowledgment and acknowledgment_text:
        speak(acknowledgment_text)
    
    print("Listening for user command...")
    command_audio_buffer = np.array([], dtype=np.float32)
    silence_start_time = None
    MIN_SPEECH_DURATION = 0.5 # seconds - user must speak for at least this long
    MAX_SILENCE_AFTER_SPEECH = 1.5 # seconds - how long to wait for more speech
    MAX_LISTENING_TIME = 15.0 # seconds - maximum time to listen before timeout

    speech_detected_once = False
    listening_start_time = time.time()

    while server_running:
        # Check for timeout
        if time.time() - listening_start_time > MAX_LISTENING_TIME:
            print("Listening timeout reached.")
            break

        try:
            audio_chunk_bytes = audio_stream.read(CHUNK_SAMPLES, exception_on_overflow=False)
            chunk_np = np.frombuffer(audio_chunk_bytes, dtype=np.int16).astype(np.float32) / 32768.0
            command_audio_buffer = np.concatenate((command_audio_buffer, chunk_np))

            # Keep buffer to a manageable size (e.g., last 10-15 seconds for a command)
            max_command_buffer_samples = RATE * 15
            if len(command_audio_buffer) > max_command_buffer_samples:
                command_audio_buffer = command_audio_buffer[-max_command_buffer_samples:]

            # VAD on the latest segment of the command buffer (e.g., last 0.5s)
            vad_segment_samples = int(RATE * 0.5)
            if len(command_audio_buffer) >= vad_segment_samples:
                current_segment_for_vad = command_audio_buffer[-vad_segment_samples:]
                rms = np.sqrt(np.mean(current_segment_for_vad**2))
            else:
                rms = 0 # Not enough audio for VAD yet
            
            print(f"Listening... RMS: {rms:.4f} (Buffer: {len(command_audio_buffer)/RATE:.1f}s)")

            if rms > VAD_THRESHOLD:
                silence_start_time = None # Reset silence timer if speech detected
                speech_detected_once = True
                print("Speech detected...")
            elif speech_detected_once:
                if silence_start_time is None:
                    silence_start_time = time.time()
                    print("Potential end of speech, starting silence timer...")
                
                if time.time() - silence_start_time > MAX_SILENCE_AFTER_SPEECH:
                    print("User finished speaking (silence threshold met).")
                    break # Exit the listening loop

        except Exception as e:
            print(f"Error during command listening: {e}")
            break
    
    # Process the captured audio
    if speech_detected_once and len(command_audio_buffer) / RATE >= MIN_SPEECH_DURATION:
        print(f"Transcribing command (buffer size: {len(command_audio_buffer)} samples)")
        try:
            result = mlx_whisper.transcribe(
                audio=command_audio_buffer,
                path_or_hf_repo=ASR_MODEL_NAME,
                language=LANGUAGE
            )
            raw_transcribed_text = result['text'].strip()
            
            # Apply filters to the command transcription as well
            valid_command_transcription = False
            if raw_transcribed_text:
                words = raw_transcribed_text.lower().split()
                num_words = len(words)
                num_chars = len(raw_transcribed_text)

                passes_length_check = True
                # For commands, MAX_WORDS_PER_INTERVAL might be too short.
                # Let's define different limits or make them more generous for commands.
                # Using slightly more generous limits for commands as they can be longer than a single wake word phrase.
                MAX_COMMAND_WORDS = 30 
                MAX_COMMAND_CHARS = 200 
                if num_words > MAX_COMMAND_WORDS or num_chars > MAX_COMMAND_CHARS:
                    print(f"Command Filtered (too long/chars)")
                    passes_length_check = False
                
                if passes_length_check:
                    valid_command_transcription = True
                    print(f"Command Transcribed: '{raw_transcribed_text}'")
            
            if valid_command_transcription:
                transcribed_command_text = raw_transcribed_text # Use the filtered, raw text
                try:
                    with open(USER_SPEECH_FILE, 'w') as f:
                        f.write(transcribed_command_text)
                    create_flag_file(LISTENING_COMPLETE_FLAG_FILE)
                    print(f"Created {USER_SPEECH_FILE} and {LISTENING_COMPLETE_FLAG_FILE}")
                except IOError as e:
                    print(f"Error writing user speech file: {e}")
            else:
                print("No valid command transcribed after filtering or original was empty.")

        except Exception as e:
            print(f"Error during command transcription: {e}")
    else:
        print("No valid speech detected or command too short.")

def listen_for_command(audio_stream, audio_interface_ref):
    """Deprecated function - replaced by do_listen_and_transcribe"""
    # This function is now just a wrapper for backward compatibility if needed
    acknowledgment = random.choice(ACKNOWLEDGEMENTS)
    do_listen_and_transcribe(audio_stream, play_acknowledgment=True, acknowledgment_text=acknowledgment)

def run_wakeword_server():
    global server_running
    consecutive_filtered_wake_word_checks = 0

    if not initialize_tts():
        print("Failed to initialize TTS. Wakeword server might not speak.")
        # Decide if you want to exit or continue without TTS feedback from server

    print(f"Using MLX Whisper model for ASR: {ASR_MODEL_NAME}")
    print(f"Listening for wake words: {WAKE_WORDS}")
    print(f"VAD Threshold: {VAD_THRESHOLD}")
    print("Wakeword server started. Press Ctrl+C to stop.")

    audio_interface = pyaudio.PyAudio()
    stream = audio_interface.open(format=FORMAT,
                                  channels=CHANNELS,
                                  rate=RATE,
                                  input=True,
                                  frames_per_buffer=CHUNK_SAMPLES)

    # Clear any pre-existing flags on start
    clear_flag_file(WAKE_WORD_FLAG_FILE)
    clear_flag_file(LISTENING_COMPLETE_FLAG_FILE)
    # clear_flag_file(REQUEST_AUDIO_CAPTURE_FLAG) # Already removed
    if os.path.exists(USER_SPEECH_FILE):
        try: os.remove(USER_SPEECH_FILE)
        except OSError as e: print(f"Error removing stale user speech file: {e}")

    audio_buffer_np = np.array([], dtype=np.float32)

    try:
        while server_running:
            # Check for robot-initiated listening request was removed in prior steps, so this is correct.

            # Normal wake word detection
            audio_chunk_bytes = stream.read(CHUNK_SAMPLES, exception_on_overflow=False)
            chunk_np = np.frombuffer(audio_chunk_bytes, dtype=np.int16).astype(np.float32) / 32768.0
            audio_buffer_np = np.concatenate((audio_buffer_np, chunk_np))

            if len(audio_buffer_np) >= PROCESS_SAMPLES:
                process_segment_np = audio_buffer_np[-PROCESS_SAMPLES:]
                rms = np.sqrt(np.mean(process_segment_np**2))
                # print(f"RMS: {rms:.4f}") # Logging for VAD tuning

                if rms >= VAD_THRESHOLD:
                    # print(f"Sufficient audio for wake word check (RMS: {rms:.4f}), transcribing...")
                    try:
                        result = mlx_whisper.transcribe(
                            audio=process_segment_np,
                            path_or_hf_repo=ASR_MODEL_NAME,
                            language=LANGUAGE,
                            initial_prompt="robot mauricio dance" # Bias towards wake words if helpful
                        )
                        raw_transcribed_text = result['text'].strip()
                        
                        valid_transcription_for_wakeword = False
                        if raw_transcribed_text:
                            words = raw_transcribed_text.lower().split()
                            num_words = len(words)
                            num_chars = len(raw_transcribed_text) # Using raw length for char count

                            passes_length_check = True
                            if num_words > MAX_WORDS_PER_INTERVAL or num_chars > MAX_CHARS_PER_INTERVAL:
                                print(f"Filtered (too long/chars)")
                                passes_length_check = False
                            
                            if passes_length_check:
                                consecutive_filtered_wake_word_checks = 0
                                valid_transcription_for_wakeword = True
                                print(f"Wakeword check: '{raw_transcribed_text}'")
                            else:
                                consecutive_filtered_wake_word_checks += 1
                        else:
                            consecutive_filtered_wake_word_checks = 0

                        if consecutive_filtered_wake_word_checks >= 3:
                            print("Model produced filtered results 3 times in a row. Restarting model by reloading module...")
                            try:
                                importlib.reload(mlx_whisper)
                                print("mlx_whisper module reloaded.")
                            except Exception as e:
                                print(f"Failed to reload mlx_whisper: {e}")
                            consecutive_filtered_wake_word_checks = 0 # Reset after reloading
                        
                        if valid_transcription_for_wakeword:
                            transcribed_text_lower = raw_transcribed_text.lower()
                            for wake_word in WAKE_WORDS:
                                if wake_word in transcribed_text_lower:
                                    print(f"Wake word '{wake_word}' detected!")
                                    create_flag_file(WAKE_WORD_FLAG_FILE)
                                    
                                    # Play acknowledgment and listen for command
                                    acknowledgment = random.choice(ACKNOWLEDGEMENTS)
                                    do_listen_and_transcribe(stream, play_acknowledgment=True, acknowledgment_text=acknowledgment)
                                    audio_buffer_np = np.array([], dtype=np.float32) # Clear buffer after listening
                                    break # Stop checking other wake words
                            
                    except Exception as e:
                        print(f"Error during wake word transcription: {e}")
                
                # Trim buffer for wake word detection
                audio_buffer_np = audio_buffer_np[CHUNK_SAMPLES:]
            else:
                 # If buffer is not full enough for processing, wait a bit.
                 # The stream.read is blocking for CHUNK_SAMPLES, so this helps avoid tight loop on very small chunks
                 # if CHUNK_SAMPLES is significantly smaller than PROCESS_SAMPLES.
                 # However, for CHUNK_DURATION_MS=250ms, this is probably fine without extra sleep.
                 pass # time.sleep(0.05)

    except KeyboardInterrupt:
        print("Wakeword server stopping by user interrupt...")
    except Exception as e:
        print(f"An unexpected error occurred in wakeword server: {e}")
    finally:
        server_running = False
        if 'stream' in locals() and stream.is_active():
            stream.stop_stream()
            stream.close()
        if 'audio_interface' in locals():
            audio_interface.terminate()
        # Clear flags on exit
        clear_flag_file(WAKE_WORD_FLAG_FILE)
        clear_flag_file(LISTENING_COMPLETE_FLAG_FILE)
        # clear_flag_file(REQUEST_AUDIO_CAPTURE_FLAG) # Already removed
        if os.path.exists(USER_SPEECH_FILE):
            try: os.remove(USER_SPEECH_FILE); print(f"Cleaned up {USER_SPEECH_FILE}")
            except OSError as e: print(f"Error cleaning up user speech file {USER_SPEECH_FILE}: {e}")
        print("Wakeword server audio stream closed. Exiting.")

if __name__ == "__main__":
    run_wakeword_server() 