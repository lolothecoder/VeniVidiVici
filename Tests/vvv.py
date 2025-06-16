#!/usr/bin/env python3
import serial
import time
import sys
from rplidar import RPLidar
import statistics

# ─── CONFIG ────────────────────────────────────────────────────────────────────

# Serial port for Arduino drive commands
PORT = '/dev/ttyACM0'  # adjust as needed
BAUD = 57600

# Serial drive commands
SPEED_CMD = 'o 80 80\r'  # ~0.2 m/s
STOP_CMD  = 'o 0 0\r'

# LIDAR port
LIDAR_PORT = '/dev/ttyUSB0'  # adjust to your RPLIDAR device
LIDAR_BAUD = 115200

# Distance to travel (m)
DISTANCE_M = 1.0
# Angular window around front direction (degrees)
ANGLE_WINDOW_DEG = 5.0
# How many scans to take for initial reading
INITIAL_SCANS = 5
# Delay between drive commands
DRIVE_LOOP_DELAY = 0.1  # seconds

# ─── HELPERS ──────────────────────────────────────────────────────────────────

def get_front_distance(scan):
    """
    Extract distances (in mm) from scan within ANGLE_WINDOW_DEG of 0° (front).
    """
    front = [point[2] for point in scan
             if point[1] < ANGLE_WINDOW_DEG or point[1] > (360 - ANGLE_WINDOW_DEG)]
    return front

# ─── MAIN SCRIPT ───────────────────────────────────────────────────────────────

def main():
    # 1) Open LIDAR
    try:
        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD)
    except Exception as e:
        print(f"❌ Could not open LIDAR on {LIDAR_PORT}: {e}")
        sys.exit(1)

    # allow LIDAR to spin up
    time.sleep(1.0)

    # 2) Take initial scans and compute median front distance
    print("→ Taking initial scans from LiDAR...")
    scans = []
    for scan in lidar.iter_scans(max_buf_meas=200):
        front = get_front_distance(scan)
        if front:
            scans.append(statistics.median(front))
        if len(scans) >= INITIAL_SCANS:
            break
    if not scans:
        print("❌ No valid front measurements from LiDAR; exiting.")
        lidar.stop(); lidar.disconnect()
        sys.exit(1)

    initial_mm = statistics.median(scans)
    target_mm = initial_mm - (DISTANCE_M * 1000)
    print(f"→ Initial front distance: {initial_mm:.0f} mm")
    print(f"→ Driving until front distance is <= {target_mm:.0f} mm (~{DISTANCE_M:.2f} m traveled)")

    # 3) Open serial for driving
    try:
        ser = serial.Serial(PORT, baudrate=BAUD, timeout=0.1)
    except Exception as e:
        print(f"❌ Could not open {PORT}@{BAUD}: {e}")
        lidar.stop(); lidar.disconnect()
        sys.exit(1)

    time.sleep(2.0)
    while ser.in_waiting:
        ser.read(ser.in_waiting)

    # 4) Drive loop
    try:
        while True:
            # grab one scan
            scan = next(lidar.iter_scans(max_buf_meas=200))
            front = get_front_distance(scan)
            if not front:
                print("⚠️  No front data in this scan; retrying...")
                continue
            current_mm = statistics.median(front)
            print(f"   current front: {current_mm:.0f} mm", end='\r', flush=True)

            if current_mm <= target_mm:
                print("\n→ Reached target distance.")
                break

            # send drive command
            ser.write(SPEED_CMD.encode('ascii'))
            ser.flush()
            time.sleep(DRIVE_LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        # Stop robot and clean up
        print("→ Stopping robot.")
        ser.write(STOP_CMD.encode('ascii'))
        ser.flush()
        ser.close()
        lidar.stop()
        lidar.disconnect()
        print("→ Done.")


if __name__ == '__main__':
    main()
