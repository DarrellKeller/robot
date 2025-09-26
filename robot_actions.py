import time
import math

# This module assumes that a 'ser' (serial connection object)
# is available and passed to its functions from the main control script.

def send_command_to_robot(ser, command_char):
    """Sends a single character command to the robot via serial.
    
    Args:
        ser: The initialized serial.Serial object.
        command_char: The character command to send (e.g., 'w', 'x', 'a').
    """
    if ser and ser.is_open:
        print(f"Sending command to ESP32: {command_char}")
        ser.write(command_char.encode('utf-8'))
        ser.flush() # Ensure data is sent immediately
        time.sleep(0.1) # Small delay for ESP32 to process
    else:
        print("Serial port not available or not open.")

def backup_robot(ser, duration_seconds=1.5):
    """Commands the robot to move backward for a specified duration."""
    print(f"Action: Backing up for {duration_seconds} seconds.")
    send_command_to_robot(ser, 's') # 's' for reverse
    time.sleep(duration_seconds)
    send_command_to_robot(ser, 'x') # 'x' to stop after backing up
    print("Action: Backup complete.")

DEFAULT_IMU_TURN_ANGLE = 75.0


def turn_robot(ser, direction, angle_degrees=DEFAULT_IMU_TURN_ANGLE, turn_speed_level=None):
    """Commands the ESP32 to perform IMU-based turns in multiples of the default angle."""
    if direction not in ('left', 'right'):
        print(f"Invalid turn direction: {direction}")
        return

    if angle_degrees <= 0:
        print("Angle must be positive for turn_robot.")
        return

    turns_needed = max(1, int(math.ceil(angle_degrees / DEFAULT_IMU_TURN_ANGLE)))
    command_char = 'a' if direction == 'left' else 'd'

    print(f"Action: Turning {direction} for {angle_degrees} degrees (approx {turns_needed} pulses).")

    # Stop any current motion before initiating the turn
    send_command_to_robot(ser, 'x')
    time.sleep(0.15)

    if turn_speed_level is not None:
        set_robot_speed(ser, turn_speed_level)
        time.sleep(0.1)

    estimated_time_per_turn = 2.2  # seconds, tuned to IMU turn duration on ESP32

    for i in range(turns_needed):
        print(f"Action: Turn pulse {i+1}/{turns_needed} ({direction}).")
        send_command_to_robot(ser, command_char)
        time.sleep(estimated_time_per_turn)
        send_command_to_robot(ser, 'x')
        time.sleep(0.2)

    print("Action: Turn sequence complete.")

def stop_robot(ser):
    """Commands the robot to stop all motor functions."""
    print("Action: Stopping robot.")
    send_command_to_robot(ser, 'x')

def set_autonomous_mode(ser, enable=True):
    """Toggles the PID (autonomous) mode on the ESP32."""
    if enable:
        print("Action: Enabling autonomous (PID) mode.")
    else:
        print("Action: Disabling autonomous (PID) mode. Robot will stop.")
    send_command_to_robot(ser, 'p') # 'p' toggles PID
    # If disabling, the ESP32 side automatically stops motors.

def set_robot_speed(ser, speed_level):
    """Sets the robot's base speed.

    Args:
        ser: The initialized serial.Serial object.
        speed_level: An integer from 0 (max speed) to 9 (10% speed).
                     Maps to '0' through '9' character commands.
    """
    if 0 <= speed_level <= 9:
        command_char = str(speed_level)
        print(f"Action: Setting speed to level {speed_level}.")
        send_command_to_robot(ser, command_char)
    else:
        print(f"Invalid speed level: {speed_level}. Must be 0-9.")

def dance_silly(ser):
    """Commands the robot to perform a short spin and wiggle dance."""
    print("Action: Time to dance! LET'S GOOO!")
    
    # Spin: Assuming 'a' makes it turn left (spin on axis)
    # You might need a dedicated spin command from ESP32 if 'a' is not a zero-radius turn
    # Or adjust durations if 'a' and 'd' are tank turns.
    send_command_to_robot(ser, 'a') # Start spin left
    time.sleep(1.5) # Spin for 1.5 seconds
    send_command_to_robot(ser, 'x') # Stop spin
    time.sleep(0.3)

    # Wiggle
    print("Action: Wiggle wiggle!")
    send_command_to_robot(ser, 'a') # Turn left briefly
    time.sleep(0.5)
    send_command_to_robot(ser, 'd') # Turn right briefly
    time.sleep(0.5)
    send_command_to_robot(ser, 'a') # Turn left briefly
    time.sleep(0.5)
    send_command_to_robot(ser, 'd') # Turn right briefly
    time.sleep(0.5)
    send_command_to_robot(ser, 'x') # Stop wiggle
    
    print("Action: Dance complete.")

if __name__ == '__main__':
    # This module is not meant to be run directly as it needs a serial object.
    # The following is for conceptual testing if you mock the serial object.
    class MockSerial:
        def __init__(self):
            self.is_open = True
            print("MockSerial initialized.")
        def write(self, data):
            print(f"MockSerial write: {data}")
        def flush(self):
            print("MockSerial flush")
        def close(self):
            print("MockSerial close")
            self.is_open = False

    mock_ser = MockSerial()
    print("\n--- Testing robot actions with MockSerial ---")
    
    print("\nTesting backup...")
    backup_robot(mock_ser, 0.5)
    
    print("\nTesting turn left...")
    turn_robot(mock_ser, 'left', 0.7)

    print("\nTesting turn right...")
    turn_robot(mock_ser, 'right', 0.6)

    print("\nTesting stop...")
    stop_robot(mock_ser)

    print("\nTesting set autonomous mode ON...")
    set_autonomous_mode(mock_ser, True)

    print("\nTesting set autonomous mode OFF...")
    set_autonomous_mode(mock_ser, False)

    print("\nTesting set speed...")
    set_robot_speed(mock_ser, 5) # 50% speed
    set_robot_speed(mock_ser, 0) # Max speed
    set_robot_speed(mock_ser, 9) # 10% speed
    set_robot_speed(mock_ser, 10) # Invalid

    print("\nTesting dance...")
    dance_silly(mock_ser)

    mock_ser.close() 