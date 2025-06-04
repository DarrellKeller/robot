import serial
import time
import signal
import sys
import csv
import os
import json

# Robot specific module imports
import tts_module
import vision_module
import thinking_module
import robot_actions

# --- Configuration ---
SERIAL_PORT = '/dev/tty.usbserial-0001'  # CHANGE THIS to your ESP32's serial port
BAUD_RATE = 115200
DATA_TIMEOUT = 1.0  # Seconds to wait for serial data
EXPECTED_COLUMNS = 17 # Increased from 16 to accommodate PID_Active_ESP

# IPC Flag Files (must match wakeword_server.py)
WAKE_WORD_FLAG_FILE = "WAKE_WORD_DETECTED.flag"
LISTENING_COMPLETE_FLAG_FILE = "LISTENING_COMPLETE.flag"
USER_SPEECH_FILE = "user_speech.txt"
# REQUEST_AUDIO_CAPTURE_FLAG = "REQUEST_AUDIO_CAPTURE.flag" # New flag for robot-initiated listening -- REMOVED

# Global serial object
ser = None

# Robot States
STATE_IDLE = "IDLE" # Doing nothing, waiting for wakeword or initial start
STATE_AUTONOMOUS_NAV = "AUTONOMOUS_NAV" # Moving based on ESP32 PID
STATE_CRITICAL_OBSTACLE_HANDLER = "CRITICAL_OBSTACLE_HANDLER" # ESP32 stopped due to obstacle, Python taking over
STATE_SURVEY_MODE = "SURVEY_MODE" # Taking pictures (front, left, right)
STATE_AWAITING_LLM_DECISION = "AWAITING_LLM_DECISION" # Sent info to LLM, waiting for JSON response
STATE_PROCESSING_USER_COMMAND = "PROCESSING_USER_COMMAND" # Wakeword heard, user speech captured, sending to LLM
STATE_EXECUTING_LLM_DECISION = "EXECUTING_LLM_DECISION" # Performing actions from LLM JSON

current_robot_state = STATE_IDLE
previous_robot_data = None # To store the last complete robot data packet
last_llm_decision = None # To store the last decision from the LLM
last_action_description_for_llm = None # To store a description of the last action for the LLM context
current_directive = "Explore the environment, describe what you see, and await further instructions." # Initial directive

def cleanup_and_exit(sig=None, frame=None):
    """Gracefully close the serial port and exit."""
    global ser, current_robot_state
    print("\nCleaning up and exiting...")
    current_robot_state = STATE_IDLE # Stop any ongoing processes
    if ser and ser.is_open:
        try:
            robot_actions.stop_robot(ser) # Send a final stop command
            ser.close()
            print("Serial port closed.")
        except Exception as e:
            print(f"Error during serial cleanup: {e}")
    # Clear IPC flags (optional, server should also do this on its exit)
    if os.path.exists(WAKE_WORD_FLAG_FILE):
        try: os.remove(WAKE_WORD_FLAG_FILE)
        except OSError as e: print(f"Error removing flag file {WAKE_WORD_FLAG_FILE}: {e}")
    if os.path.exists(LISTENING_COMPLETE_FLAG_FILE):
        try: os.remove(LISTENING_COMPLETE_FLAG_FILE)
        except OSError as e: print(f"Error removing flag file {LISTENING_COMPLETE_FLAG_FILE}: {e}")
    if os.path.exists(USER_SPEECH_FILE):
        try: os.remove(USER_SPEECH_FILE)
        except OSError as e: print(f"Error removing speech file {USER_SPEECH_FILE}: {e}")
    # REMOVED REQUEST_AUDIO_CAPTURE_FLAG cleanup
    # if os.path.exists(REQUEST_AUDIO_CAPTURE_FLAG):
    #     try: os.remove(REQUEST_AUDIO_CAPTURE_FLAG)
    #     except OSError as e: print(f"Error removing flag file {REQUEST_AUDIO_CAPTURE_FLAG}: {e}")
    sys.exit(0)

def clear_flag_file(flag_path):
    """Safely remove a flag file."""
    if os.path.exists(flag_path):
        try:
            os.remove(flag_path)
            print(f"IPC: Cleared flag file {flag_path}")
        except OSError as e:
            print(f"Error removing flag file {flag_path}: {e}")

def connect_serial():
    """Establish serial connection with the ESP32."""
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=DATA_TIMEOUT)
        print(f"Successfully connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        time.sleep(2)  # Wait for ESP32 to reset
        ser.reset_input_buffer()
        return True
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        return False

def parse_data_line(line):
    """Parse a CSV line of sensor data from ESP32."""
    try:
        if isinstance(line, bytes):
            line = line.decode('utf-8').strip()
        else:
            line = line.strip()
        if not line: return None

        values = list(csv.reader([line]))[0]
        if len(values) != EXPECTED_COLUMNS:
            # print(f"Warning: Malformed data. Expected {EXPECTED_COLUMNS}, got {len(values)}. Data: '{line}'")
            return None
        
        data = {
            "L90": int(values[0]), "L45": int(values[1]), "F": int(values[2]), "R45": int(values[3]), "R90": int(values[4]),
            "sW_L90": float(values[5]), "sW_L45": float(values[6]), "sW_F": float(values[7]), "sW_R45": float(values[8]), "sW_R90": float(values[9]),
            "SteeringIn": float(values[10]), "PID_Out": float(values[11]),
            "L_Speed": int(values[12]), "R_Speed": int(values[13]),
            "BaseSpeed": int(values[14]), "CurSpeed": int(values[15]),
            "PID_Active_ESP": bool(int(values[16])) # Parsed from ESP32
        }
        # Actual PID active status can be inferred if CurSpeed > 0 and no manual command was just sent,
        # or if ESP32 sends it. For now, this is a simplification.
        # We can check if CurSpeed is 0 as an indicator of PID stopping or manual stop.
        return data
    except ValueError as e:
        # print(f"Error converting data: {e}. Line: '{line}'")
        return None
    except Exception as e:
        # print(f"An unexpected error during parsing: {e}. Line: '{line}'")
        return None


def initialize_systems():
    """Initialize TTS and Vision models."""
    print("Initializing Text-to-Speech system...")
    if not tts_module.initialize_tts():
        print("CRITICAL: Failed to initialize TTS. Speech functions will not work.")
        # Potentially exit or run in a degraded mode
    
    print("Initializing Vision system...")
    if not vision_module.initialize_vision_model():
        print("CRITICAL: Failed to initialize Vision model. Survey functions will not work.")
        # Potentially exit or run in a degraded mode
    
    print("All external systems initialized (or attempted).")

# --- Main Application Logic ---
def main_loop():
    global ser, current_robot_state, previous_robot_data, last_llm_decision, current_directive, last_action_description_for_llm
    
    print(f"\nStarting autonomous control loop. Initial state: {current_robot_state}")
    print("Press Ctrl+C to exit.")

    # Set initial ESP32 mode to autonomous if desired
    # robot_actions.set_autonomous_mode(ser, True) # Start in autonomous
    # current_robot_state = STATE_AUTONOMOUS_NAV # If starting autonomously

    last_data_print_time = time.time()
    last_wake_word_check_time = time.time()
    awaiting_speech_start_time = None # Timer for robot-requested speech

    while True:
        try:
            # --- Read Serial Data from ESP32 ---
            raw_line = None
            if ser and ser.in_waiting > 0:
                raw_line = ser.readline()
                # print(f"Raw from ESP32: {raw_line.strip()}") # Debug raw data
                robot_data = parse_data_line(raw_line)
                if robot_data:
                    previous_robot_data = robot_data # Always store the latest valid data
                    # Optional: Print data periodically
                    if time.time() - last_data_print_time > 2.0: # Print data every 2s
                        print(f"State: {current_robot_state} | ESP32 Data: F={robot_data['F']} L45={robot_data['L45']} R45={robot_data['R45']} | Speed L={robot_data['L_Speed']} R={robot_data['R_Speed']} Cur={robot_data['CurSpeed']}")
                        last_data_print_time = time.time()
                
                # Handle non-data messages (like PID Active status from ESP32 if implemented)
                elif raw_line:
                    try:
                        decoded_line = raw_line.decode('utf-8').strip()
                        if "BaseSpeed:" in decoded_line or "PID Active:" in decoded_line or "initialized." in decoded_line or "Failed to detect" in decoded_line or "TIMEOUT" in decoded_line:
                            print(f"ESP32 MSG: {decoded_line}")
                    except UnicodeDecodeError:
                        pass # Already handled by parse_data_line somewhat
            
            # --- IPC: Check for Wake Word Server Signals ---
            if time.time() - last_wake_word_check_time > 0.25: # Check every 250ms
                if os.path.exists(LISTENING_COMPLETE_FLAG_FILE):
                    print("IPC: LISTENING_COMPLETE_FLAG_FILE detected.")
                    if current_robot_state not in [STATE_AWAITING_LLM_DECISION, STATE_EXECUTING_LLM_DECISION]: # Avoid interrupting critical LLM tasks
                        robot_actions.stop_robot(ser) # Stop robot movement
                        current_robot_state = STATE_PROCESSING_USER_COMMAND
                    else:
                        print(f"IPC: In state {current_robot_state}, deferring user command processing.")
                    # Flag will be cleared after processing the command

                elif os.path.exists(WAKE_WORD_FLAG_FILE):
                    print("IPC: WAKE_WORD_FLAG_FILE detected.")
                    if current_robot_state not in [STATE_AWAITING_LLM_DECISION, STATE_EXECUTING_LLM_DECISION, STATE_PROCESSING_USER_COMMAND, STATE_SURVEY_MODE, STATE_CRITICAL_OBSTACLE_HANDLER]:
                        print("IPC: Wake word acknowledged. Robot stopping. Server is now listening for command.")
                        robot_actions.stop_robot(ser) # Stop the robot
                        current_robot_state = STATE_IDLE # Or a new state like AWAITING_USER_SPEECH
                    else:
                        print(f"IPC: In state {current_robot_state}, wake word ignored by main brain for now.")
                    # Wakeword server clears its own WAKE_WORD_FLAG_FILE after starting to listen for command.
                    # Or we can clear it here if the server logic changes.
                    # For now, assume server handles it.
                    clear_flag_file(WAKE_WORD_FLAG_FILE) # Main brain now clears the flag
                last_wake_word_check_time = time.time()

            # --- Robot State Machine ---
            if current_robot_state == STATE_IDLE:
                # Waiting for a wake word or an event to trigger another state.
                # Robot will now stay idle unless an IPC event or other logic changes its state.
                # The proactive survey from IDLE has been removed.
                pass # Stay idle until an IPC event or other state change occurs

            elif current_robot_state == STATE_AUTONOMOUS_NAV:
                if previous_robot_data:
                    # Check for critical stop condition triggered by ESP32's TOF logic
                    # ESP32's CurrentSpeed becomes 0 when it stops due to obstacle (and PID was active)
                    # We need to ensure this wasn't a manual 'x' stop or PID toggle.
                    # This logic assumes ESP32 sets CurSpeed to 0 when its internal avoidance stops it.
                    if previous_robot_data["CurSpeed"] == 0 and previous_robot_data.get("PID_Active_ESP", True):
                        # How to differentiate from a manual 'x' command or 'p' toggle from Python side?
                        # For now, assume if Python didn't just send 'x' or 'p', and CurSpeed is 0, ESP32 stopped itself.
                        # This needs robust check. Let's assume for now this is the ESP32's autonomous stop.
                        print("State AUTONOMOUS_NAV: ESP32 reported CurSpeed = 0. Obstacle detected by ESP32.")
                        current_robot_state = STATE_CRITICAL_OBSTACLE_HANDLER
                # If wake word detected, state will change via IPC check above.

            elif current_robot_state == STATE_CRITICAL_OBSTACLE_HANDLER:
                print("State CRITICAL_OBSTACLE_HANDLER: Initiating backup and survey.")
                robot_actions.backup_robot(ser, duration_seconds=1.5)
                # backup_robot already sends a stop command after backup.
                current_robot_state = STATE_SURVEY_MODE
                # Clear any pending LLM decision from a previous cycle if any
                last_llm_decision = None 
                last_action_description_for_llm = "I was stuck and just backed up."

            elif current_robot_state == STATE_SURVEY_MODE:
                print("State SURVEY_MODE: Starting visual survey.")
                descriptions = {"front": "Error during front view", "left": "Error during left view", "right": "Error during right view"}

                # 1. Front View
                tts_module.speak("Looking around")
                img_front = vision_module.capture_image()
                if img_front:
                    res_front = vision_module.analyze_image(img_front, prompt="Describe the scene in 3 sentences.")
                    descriptions["front"] = res_front.get('description', descriptions["front"])
                    print(f"Survey - Front: {descriptions['front']}")
                else: print("Survey - Front: Failed to capture image.")
                time.sleep(0.5)

                # 2. Left View
                # tts_module.speak("Now, to my left.")
                robot_actions.turn_robot(ser, 'left', duration_seconds=2.0)
                img_left = vision_module.capture_image()
                if img_left:
                    res_left = vision_module.analyze_image(img_left, prompt="Describe the scene in 3 sentences.")
                    descriptions["left"] = res_left.get('description', descriptions["left"])
                    print(f"Survey - Left: {descriptions['left']}")
                else: print("Survey - Left: Failed to capture image.")
                time.sleep(0.5)

                # 3. Right View 
                # tts_module.speak("And finally, to my right.")
                robot_actions.turn_robot(ser, 'right', duration_seconds=4) # Turn right
                img_right = vision_module.capture_image()
                if img_right:
                    res_right = vision_module.analyze_image(img_right, prompt="Describe the scene in 3 sentences.")
                    descriptions["right"] = res_right.get('description', descriptions["right"])
                    print(f"Survey - Right: {descriptions['right']}")
                else: print("Survey - Right: Failed to capture image.")
                time.sleep(0.5)
                
                # Return to roughly center (optional, or let LLM decide next turn)
                # tts_module.speak("Okay, I've had a good look around.")
                robot_actions.turn_robot(ser, 'left', duration_seconds=2) # Attempt to re-center
                robot_actions.stop_robot(ser)

                print("State SURVEY_MODE: Survey complete. Requesting LLM decision.")
                
                # Use the last action description if available
                action_context_for_this_survey = last_action_description_for_llm
                last_action_description_for_llm = None # Clear after use

                last_llm_decision = thinking_module.get_decision_for_survey(
                    descriptions["front"],
                    descriptions["left"],
                    descriptions["right"],
                    last_action_context=action_context_for_this_survey, # Pass it here
                    # current_directive=current_directive # REMOVE: thinking_module manages its own directive
                )
                current_robot_state = STATE_EXECUTING_LLM_DECISION

            elif current_robot_state == STATE_PROCESSING_USER_COMMAND:
                print("State PROCESSING_USER_COMMAND: Reading user speech.")
                user_speech_text = ""
                try:
                    with open(USER_SPEECH_FILE, 'r') as f:
                        user_speech_text = f.read().strip()
                    os.remove(USER_SPEECH_FILE) # Clean up file
                except Exception as e:
                    print(f"Error reading or deleting user speech file: {e}")
                    tts_module.speak("I had trouble understanding what you said.")
                    clear_flag_file(LISTENING_COMPLETE_FLAG_FILE) # Ensure flag is cleared before changing state
                    current_robot_state = STATE_IDLE
                    continue

                if user_speech_text:
                    # The formatted_command in autonomous_control.py was already quite good at providing context.
                    # thinking_module.get_decision_for_user_command also appends its own directive context.
                    # We'll let thinking_module handle adding its internal directive to the prompt for consistency.
                    formatted_command = f"A voice addressing you has said \"{user_speech_text}\". How do you respond?"
                    print(f"Sending to LLM for user command: {formatted_command}")
                    last_llm_decision = thinking_module.get_decision_for_user_command(
                        formatted_command
                        # current_directive=current_directive # REMOVE: thinking_module manages its own directive
                    )
                    current_robot_state = STATE_EXECUTING_LLM_DECISION
                else:
                    print("User speech was empty.")
                    tts_module.speak("I didn't catch that, please try again after the wake word.")
                    current_robot_state = STATE_IDLE
                
                clear_flag_file(LISTENING_COMPLETE_FLAG_FILE) # Always clear the flag after attempting to process
            
            elif current_robot_state == STATE_EXECUTING_LLM_DECISION:
                print(f"State EXECUTING_LLM_DECISION: Processing decision: {json.dumps(last_llm_decision, indent=2)}")
                if not last_llm_decision or last_llm_decision.get('error'): # More robust check for error
                    speak_message = "I had a problem with my thinking process. I'll just stop for now."
                    if last_llm_decision and last_llm_decision.get("speak"): # Check if last_llm_decision is not None before .get()
                         speak_message = last_llm_decision["speak"]
                    tts_module.speak(speak_message)
                    robot_actions.stop_robot(ser)
                    current_robot_state = STATE_IDLE
                    last_llm_decision = None
                    continue

                action_sets_specific_next_state = False
                any_action_other_than_speak_or_directive_change = False
                next_state_after_actions = STATE_IDLE # Default next state if no movement or survey command
                
                survey_or_autonav_chosen_this_cycle = False # Flag for mutual exclusivity

                actions_to_execute = list(last_llm_decision.keys())

                for action_key in actions_to_execute:
                    action_value = last_llm_decision.get(action_key)

                    if action_key == "speak" and action_value:
                        tts_module.speak(action_value)
                        time.sleep(0.2)
                        
                    elif action_key == "change_directive" and action_value:
                        new_directive = action_value
                        print(f"LLM Command: Change Directive to '{new_directive}'")
                        # autonomous_control.py keeps its own copy of current_directive updated if needed for its own logic/logging.
                        # The primary update happens within thinking_module.py's process_llm_response.
                        current_directive = new_directive 
                        last_action_description_for_llm = f"My directive was just updated to: {current_directive}."

                    elif action_key == "stop" and action_value is True:
                        print("LLM Command: Stop")
                        robot_actions.stop_robot(ser)
                        current_robot_state = STATE_IDLE
                        action_sets_specific_next_state = True
                        any_action_other_than_speak_or_directive_change = True
                        last_action_description_for_llm = "I was told to stop."
                        break 

                    elif action_key == "survey" and action_value is True:
                        if not survey_or_autonav_chosen_this_cycle:
                            print("LLM Command: Survey")
                            current_robot_state = STATE_SURVEY_MODE
                            action_sets_specific_next_state = True
                            any_action_other_than_speak_or_directive_change = True
                            last_action_description_for_llm = "I am initiating a survey based on LLM request."
                            survey_or_autonav_chosen_this_cycle = True
                            break 
                        else:
                            print("LLM Command: Survey requested, but move_forward_autonomously already chosen this cycle. Ignoring survey.")

                    elif action_key == "move_forward_autonomously" and action_value is True:
                        if not survey_or_autonav_chosen_this_cycle:
                            print("LLM Command: Move Forward Autonomously")
                            robot_actions.set_autonomous_mode(ser, True)
                            current_robot_state = STATE_AUTONOMOUS_NAV
                            action_sets_specific_next_state = True
                            any_action_other_than_speak_or_directive_change = True
                            last_action_description_for_llm = "I am now moving forward autonomously."
                            survey_or_autonav_chosen_this_cycle = True
                            break
                        else:
                            print("LLM Command: move_forward_autonomously requested, but survey already chosen this cycle. Ignoring.")

                    elif action_key == "turn_left" and action_value is True:
                        print("LLM Command: Turn Left")
                        robot_actions.turn_robot(ser, 'left', 1.0)
                        any_action_other_than_speak_or_directive_change = True
                        last_action_description_for_llm = "I just turned left."
                        next_state_after_actions = STATE_SURVEY_MODE
                    elif action_key == "turn_right" and action_value is True:
                        print("LLM Command: Turn Right")
                        robot_actions.turn_robot(ser, 'right', 1.0)
                        any_action_other_than_speak_or_directive_change = True
                        last_action_description_for_llm = "I just turned right."
                        next_state_after_actions = STATE_SURVEY_MODE
                        
                    elif action_key == "dance" and action_value is True:
                        print("LLM Command: Dance!")
                        robot_actions.dance_silly(ser)
                        any_action_other_than_speak_or_directive_change = True
                        last_action_description_for_llm = "I just performed a dance."
                        next_state_after_actions = STATE_SURVEY_MODE

                current_llm_speak_output_was_present = "speak" in last_llm_decision
                last_llm_decision = None 

                if not action_sets_specific_next_state: 
                    if any_action_other_than_speak_or_directive_change:
                        print(f"Discrete actions completed. Transitioning to {next_state_after_actions}.")
                        current_robot_state = next_state_after_actions 
                    else:
                        print("No movement or major state-changing action from LLM. Going to IDLE.")
                        current_robot_state = STATE_IDLE
                        if not last_action_description_for_llm: 
                           last_action_description_for_llm = "I just spoke or had my directive updated based on LLM guidance." if current_llm_speak_output_was_present or ("change_directive" in actions_to_execute) else "The LLM gave no specific action, so I am now idle."
                
                continue

            # Small delay to prevent high CPU usage if no blocking calls are made
            time.sleep(0.05)

        except serial.SerialException as e:
            print(f"Serial error: {e}. Attempting to reconnect...")
            if ser and ser.is_open: ser.close()
            time.sleep(3)
            if not connect_serial():
                print("Reconnect failed. Exiting.")
                cleanup_and_exit()
            else:
                if ser: ser.reset_input_buffer()
        except KeyboardInterrupt:
            print("Main loop interrupted by user.")
            break
        except Exception as e:
            print(f"An error occurred in main_loop: {e}")
            # Consider which state to revert to on general error, maybe IDLE
            # current_robot_state = STATE_IDLE
            # tts_module.speak("Oh dear, something went wrong with my main functions.")
            time.sleep(1) # Avoid rapid error logging

if __name__ == "__main__":
    signal.signal(signal.SIGINT, cleanup_and_exit)
    signal.signal(signal.SIGTERM, cleanup_and_exit)

    # Clear any lingering IPC files from a previous run
    for f in [WAKE_WORD_FLAG_FILE, LISTENING_COMPLETE_FLAG_FILE, USER_SPEECH_FILE]: #, REQUEST_AUDIO_CAPTURE_FLAG]:
        if os.path.exists(f):
            try: os.remove(f)
            except OSError as e: print(f"Could not remove old IPC file {f}: {e}")

    initialize_systems() # Initialize TTS, Vision

    if connect_serial():
        # Start in IDLE state, it will transition to AUTONOMOUS_NAV if conditions are met
        current_robot_state = STATE_IDLE 
        # Or uncomment below to start directly in autonomous mode:
        # robot_actions.set_autonomous_mode(ser, True)
        # current_robot_state = STATE_AUTONOMOUS_NAV
        main_loop()
    
    cleanup_and_exit() 