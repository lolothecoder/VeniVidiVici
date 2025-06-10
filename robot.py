#!/usr/bin/env python3
import serial
import sys
import termios
import tty
import time
import select

# ─── CONFIG ────────────────────────────────────────────────────────────────────

PORT  = "/dev/ttyACM0"   # change if your Arduino is on a different device
BAUD  = 57600

# Speeds (adjusted as requested)
FORWARD_SPEED   = 120   # forward wheels at +120
BACKWARD_SPEED  = -120  # backward wheels at -120
TURN_SPEED      = 80    # pivot turn at ±80
STOP_SPEED      = 0

# How long to wait after opening serial for the Arduino to be ready
ARDUINO_BOOT_DELAY = 2.0  # seconds

# How long to wait (in seconds) for a keypress before sending stop
IDLE_TIMEOUT = 0.1

# Interval (in seconds) to resend "h 0 1" to keep door open (must be < 2s)
DOOR_KEEPALIVE_INTERVAL = 1.0

# Interval (in seconds) to resend "g -100" to keep grabber running
GRABBER_KEEPALIVE_INTERVAL = 1.0

# Interval (in seconds) to resend "h 1 1" to keep ramp extended (must be < 2s)
RAMP_KEEPALIVE_INTERVAL = 1.0

# ─── HELPERS ──────────────────────────────────────────────────────────────────

def _get_key_nonblocking(timeout=IDLE_TIMEOUT):
    """
    Wait up to timeout seconds for one keypress.
    Returns:
      - A single-character string if a key was pressed,
      - '' (empty string) if timeout elapsed with no key.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            return ch
        else:
            return ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def send_drive_command(ser, left_val, right_val):
    """
    Send exactly: b"o <left_val> <right_val>\r"
    over the already-opened Serial object ser.
    """
    left_val = left_val * 1.038
    cmd = f"o {left_val} {right_val}\r"
    ser.write(cmd.encode("ascii"))
    ser.flush()

def send_open_door(ser):
    """
    Send exactly: b"h 0 1\r" to open (keep open) the door.
    """
    cmd = "h 0 1\r"
    ser.write(cmd.encode("ascii"))
    ser.flush()

def send_close_door(ser):
    """
    Send exactly: b"h 0 0\r" to close the door.
    """
    cmd = "h 0 0\r"
    ser.write(cmd.encode("ascii"))
    ser.flush()

def send_grabber_command(ser, value):
    """
    Send exactly: b"g <value>\r" to control the grabber motor.
    """
    cmd = f"g {value}\r"
    ser.write(cmd.encode("ascii"))
    ser.flush()

def send_ramp_extend(ser):
    """
    Send exactly: b"h 1 1\r" to extend (keep extended) the ramp.
    """
    cmd = "h 1 1\r"
    ser.write(cmd.encode("ascii"))
    ser.flush()

def send_ramp_close(ser):
    """
    Send exactly: b"h 1 0\r" to retract/close the ramp.
    """
    cmd = "h 1 0\r"
    ser.write(cmd.encode("ascii"))
    ser.flush()

# ─── MAIN TELEOP LOOP ──────────────────────────────────────────────────────────

def main():
    # 1) Open serial port
    try:
        ser = serial.Serial(PORT, baudrate=BAUD, timeout=0.1)
    except Exception as e:
        print(f"❌ Could not open {PORT} @ {BAUD}: {e}")
        return

    print(f"✅ Opened {PORT} @ {BAUD}. Waiting {ARDUINO_BOOT_DELAY}s for Arduino…")
    time.sleep(ARDUINO_BOOT_DELAY)

    # 2) Discard any initial bootloader/sketch messages
    while ser.in_waiting:
        ser.read(ser.in_waiting)

    # 3) Print instructions
    print(
        "Teleop started. Use keys (no Enter required):\n"
        "  W = forward   S = backward\n"
        "  A = turn left D = turn right\n"
        "  SPACE = stop\n"
        "  O = open door   P = close door\n"
        "  K or ; = grabber on  L = grabber off\n"
        "  M = ramp extend  , = ramp close\n"
        "  Q = quit\n"
    )

    last_drive_state  = (None, None)  # last wheel command sent
    door_open         = False         # whether door is currently open
    last_door_time    = 0.0           # timestamp of last "h 0 1" sent

    grabber_active    = False         # whether grabber is running
    last_grabber_time = 0.0           # timestamp of last "g -100" sent

    ramp_extended     = False         # whether ramp is currently extended
    last_ramp_time    = 0.0           # timestamp of last "h 1 1" sent

    try:
        while True:
            current_time = time.time()
            ch = _get_key_nonblocking(IDLE_TIMEOUT).lower()

            # ─── HANDLE KEY PRESSES ───────────────────────────────────────
            if ch == "w":
                # Drive forward
                state = (FORWARD_SPEED, FORWARD_SPEED)
                send_drive_command(ser, *state)
                print(f"→ forward {state[0]},{state[1]}")
                last_drive_state = state

            elif ch == "s":
                # Drive backward
                state = (BACKWARD_SPEED, BACKWARD_SPEED)
                send_drive_command(ser, *state)
                print(f"→ backward {state[0]},{state[1]}")
                last_drive_state = state

            elif ch == "a":
                # Turn left
                state = (-TURN_SPEED, TURN_SPEED)
                send_drive_command(ser, *state)
                print(f"→ turn left {state[0]},{state[1]}")
                last_drive_state = state

            elif ch == "d":
                # Turn right
                state = (TURN_SPEED, -TURN_SPEED)
                send_drive_command(ser, *state)
                print(f"→ turn right {state[0]},{state[1]}")
                last_drive_state = state

            elif ch == " ":
                # Immediate stop
                state = (STOP_SPEED, STOP_SPEED)
                send_drive_command(ser, *state)
                print("→ stop")
                last_drive_state = state

            elif ch == "o":
                # Open door: send immediately and start keepalive
                send_open_door(ser)
                print("→ door open (sent 'h 0 1')")
                door_open = True
                last_door_time = current_time

            elif ch == "p":
                # Close door
                send_close_door(ser)
                print("→ door closed (sent 'h 0 0')")
                door_open = False

            elif ch in ("k", ";"):
                # Activate grabber: send immediately and start keepalive
                send_grabber_command(ser, 200)
                print("→ grabber on (sent 'g 150')")
                grabber_active = True
                last_grabber_time = current_time

            elif ch == "l":
                # Deactivate grabber
                send_grabber_command(ser, 0)
                print("→ grabber off (sent 'g 0')")
                grabber_active = False

            elif ch == "m":
                # Extend ramp
                send_ramp_extend(ser)
                print("→ ramp extend (sent 'h 1 1')")
                ramp_extended = True
                last_ramp_time = current_time

            elif ch == ",":
                # Close ramp
                send_ramp_close(ser)
                print("→ ramp close (sent 'h 1 0')")
                ramp_extended = False

            elif ch == "q":
                # Stop wheels, close door, stop grabber, retract ramp, then quit
                send_drive_command(ser, STOP_SPEED, STOP_SPEED)
                send_close_door(ser)
                send_grabber_command(ser, 0)
                send_ramp_close(ser)
                print("→ quit: stopped wheels, closed door, stopped grabber, and closed ramp")
                break

            elif ch == "":
                # No key pressed for IDLE_TIMEOUT seconds:
                # 1) If wheels weren’t already stopped, send stop
                if last_drive_state != (STOP_SPEED, STOP_SPEED):
                    send_drive_command(ser, STOP_SPEED, STOP_SPEED)
                    print("→ idle → stop")
                    last_drive_state = (STOP_SPEED, STOP_SPEED)

                # Door, grabber, and ramp keepalive handled below

            else:
                # Some other key – ignore entirely
                pass

            # ─── DOOR KEEPALIVE LOGIC ───────────────────────────────────────
            if door_open and (current_time - last_door_time >= DOOR_KEEPALIVE_INTERVAL):
                send_open_door(ser)
                last_door_time = current_time

            # ─── GRABBER KEEPALIVE LOGIC ─────────────────────────────────────
            if grabber_active and (current_time - last_grabber_time >= GRABBER_KEEPALIVE_INTERVAL):
                send_grabber_command(ser, 200)
                last_grabber_time = current_time

            # ─── RAMP KEEPALIVE LOGIC ───────────────────────────────────────
            if ramp_extended and (current_time - last_ramp_time >= RAMP_KEEPALIVE_INTERVAL):
                send_ramp_extend(ser)
                last_ramp_time = current_time

    except KeyboardInterrupt:
        # On Ctrl+C, stop wheels, close door, stop grabber, retract ramp before exiting
        send_drive_command(ser, STOP_SPEED, STOP_SPEED)
        send_close_door(ser)
        send_grabber_command(ser, 0)
        send_ramp_close(ser)
        print("\n→ Interrupted. Stopping motors, closing door, stopping grabber, and closing ramp.")

    finally:
        ser.close()

if __name__ == "__main__":
    main()


