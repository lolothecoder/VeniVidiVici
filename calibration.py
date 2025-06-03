#!/usr/bin/env python3
import serial
import time

# ─── USER PARAMETERS ───────────────────────────────────────────────────────────
PORT       = "/dev/ttyACM0"   # adjust as needed
BAUD       = 57600
TEST_PWMS  = [50,  60,  80,  100, 120, 140]  # PWM steps to test
DRIVE_TIME = 2.0             # seconds to drive each test

# Replace with your drive‐command function (left and right the same)
def send_drive_command(ser, left_val, right_val):
    left_val = left_val * 1.038
    cmd = f"o {left_val} {right_val}\r"
    ser.write(cmd.encode("ascii"))
    ser.flush()

def run_test(pwm):
    """
    Sends PWM forward for DRIVE_TIME, then stops.
    Returns the elapsed time (DRIVE_TIME) so you can manually measure distance.
    """
    ser = serial.Serial(PORT, baudrate=BAUD, timeout=0.1)
    time.sleep(2.0)  # let Arduino finish booting
    while ser.in_waiting:
        ser.read(ser.in_waiting)

    print(f"=== Testing PWM = {pwm} ===")
    input("  Press ENTER, then have someone measure distance over {} s...".format(DRIVE_TIME))
    # start driving
    send_drive_command(ser, pwm, pwm)
    t0 = time.time()
    time.sleep(DRIVE_TIME)
    # stop
    send_drive_command(ser, 0, 0)
    t1 = time.time()
    ser.close()
    actual_time = t1 - t0
    print(f"  Drove for {actual_time:.2f}s.  Stop robot, measure distance traveled, then press ENTER.")
    input("  (Measure distance now) ")
    distance = float(input("  Enter measured distance (in meters): "))
    speed = distance / actual_time
    print(f"  → PWM={pwm}, speed={speed:.3f} m/s\n")
    return pwm, speed

def main():
    results = []
    for pwm in TEST_PWMS:
        pwm_val, speed = run_test(pwm)
        results.append((pwm_val, speed))
    print("\nCalibration results (PWM → speed):")
    for p, s in results:
        print(f"  PWM={p:3d} → {s:.3f} m/s")
    print("\nNow you can interpolate which PWM gives 0.200 m/s.\n")

if __name__ == "__main__":
    main()
