import serial

# Replace with your COM port and baud rate
PORT = "COM5"  # e.g., /dev/ttyUSB0 on Linux or COM3 on Windows
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT} at {BAUD} baud\n")
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print(line)
except serial.SerialException as e:
    print("Serial connection failed:", e)
except KeyboardInterrupt:
    print("\nStopped by user")

