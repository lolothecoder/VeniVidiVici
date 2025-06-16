# raw_one_byte.py
import serial

PORT = "/dev/ttyACM0"   # ← change if your device node is actually /dev/ttyUSB0, etc.
BAUD = 57600

try:
    ser = serial.Serial(PORT, baudrate=BAUD, timeout=1)
    print(f"✅ Opened {PORT} @ {BAUD} baud. Reading one byte at a time… (Ctrl+C to quit)")
except Exception as e:
    print(f"❌ Could not open {PORT}: {e}")
    exit(1)

try:
    while True:
        b = ser.read(1)     # read a single byte
        if not b:
            continue       # no data in the last second
        print(f"Got byte: {b!r}  (int={b[0]})")
except KeyboardInterrupt:
    print("\nExiting.")
finally:
    ser.close()
