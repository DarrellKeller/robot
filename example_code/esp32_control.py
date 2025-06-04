import serial
import time
import sys
import signal

# Global serial object to be used by the cleanup function
ser = None

def cleanup_and_exit(sig=None, frame=None):
    """Gracefully close the serial port and exit."""
    global ser
    print("\nCleaning up and exiting...")
    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")
    sys.exit(0)

def main():
    global ser  # Declare ser as global to modify it
    # Set up clean exit on Ctrl+C
    signal.signal(signal.SIGINT, cleanup_and_exit)
    
    # Configure serial port
    try:
        ser = serial.Serial(
            port='/dev/tty.usbserial-0001',  # This might need to be changed based on your system
            baudrate=115200,
            timeout=1
        )
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        print("Please check the port name and make sure the ESP32 is connected")
        sys.exit(1)
    
    print("Control the bot with WASD keys:")
    print("W - Forward")
    print("S - Reverse")
    print("A - Left Turn")
    print("D - Right Turn")
    print("X - Stop")
    print("P - Toggle PID Autonomous Mode")
    print("0-9 - Set Base Speed (0=Max, 9=Min)")
    print("Press Ctrl+C to exit")

    try:
        while True:
            command = input("Enter command (W/A/S/D/X/P/0-9): ").strip().lower()
            
            if command in ['w', 'a', 's', 'd', 'x', 'p', '1','2','3','4','5','6','7','8','9','0']:
                ser.write(command.encode())
                print(f"Sent command: {command}")
            else:
                print("Invalid input")
                continue
            
            time.sleep(0.1)  # Small delay to prevent overwhelming the serial port

    except Exception as e:
        print(f"\nAn error occurred: {e}")
        cleanup_and_exit()

if __name__ == "__main__":
    main() 