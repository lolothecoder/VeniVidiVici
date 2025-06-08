#!/usr/bin/env python3

import serial
import time
import sys
import termios
import tty

# ─── CONFIG ────────────────────────────────────────────────────────────────────
PORT = "/dev/ttyACM0"
BAUD = 57600
ARDUINO_BOOT_DELAY = 2.0      # seconds to wait after opening serial for device boot
MESSAGE = "o 50 50"         # command to send
TERMINATORS = ["\r", "\n", "\r\n", ""]  # try various line endings
RETRIES = 15                   # retries per terminator
RETRY_DELAY = 0.1             # seconds between retries
READ_TIMEOUT = 1              # seconds to wait for response

# ─── MAIN ─────────────────────────────────────────────────────────────────────
def main():
    # Open serial port and toggle DTR
    try:
        ser = serial.Serial(PORT, baudrate=BAUD, timeout=0.5, write_timeout=0.5)
    except Exception as e:
        print(f"❌ Could not open {PORT} @ {BAUD}: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"✅ Opened {PORT} @ {BAUD}. Toggling DTR to reset device…")
    ser.setDTR(False)
    time.sleep(0.1)
    ser.setDTR(True)

    print(f"Waiting {ARDUINO_BOOT_DELAY}s for device boot…")
    time.sleep(ARDUINO_BOOT_DELAY)

    # Discard any initial data
    while ser.in_waiting:
        _ = ser.read(ser.in_waiting)

    # Send commands with various terminators
    for term in TERMINATORS:
        full_cmd = MESSAGE + term
        for attempt in range(1, RETRIES + 1):
            print(f"→ Sending '{MESSAGE}' + terminator {repr(term)} (attempt {attempt}/{RETRIES})")
            try:
                ser.write(full_cmd.encode('ascii'))
                ser.flush()
            except Exception as e:
                print(f"✖️ Write error: {e}")
            time.sleep(RETRY_DELAY)

    # Read any responses
    print(f"⏱ Reading responses for {READ_TIMEOUT}s…")
    end_time = time.time() + READ_TIMEOUT
    resp = b""
    while time.time() < end_time:
        if ser.in_waiting:
            resp += ser.read(ser.in_waiting)
    if resp:
        try:
            print(f"📥 Received: {resp.decode('ascii', errors='replace')}")
        except:
            print(f"📥 Received raw: {resp}")
    else:
        print("⚠️ No response received.")

    # Keep port open until user quits
    print("\nPress 'q' to quit and close the serial port.")
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch.lower() == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    ser.close()
    print("✅ Serial port closed. Done.")

if __name__ == '__main__':
    main()
