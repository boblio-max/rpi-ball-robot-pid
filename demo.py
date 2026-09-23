"""
ORBS Ball Robot - Demo Script
Raspberry Pi Zero 2W + MPU6050 IMU + Adafruit Motor HAT (MotorKit)
3x DC motors in T-shape:
        M1 (motor1)
        |
M0 (motor1?) ---+--- M2
Actually:
  motor1 = M0 (left of T bar)
  motor2 = M1 (stem of T)
  motor3 = M2 (right of T bar)

Mapping (from delete.py / code.or):
  M0/M2 = roll axis  (left/right)
  M1    = pitch axis (forward/backward)

Demo sequence:
  forward -> stop -> backward -> stop -> left -> stop -> right -> stop

Run on Pi with: python3 demo.py
Optional: python3 demo.py --speed 0.5 --move-time 1.5
"""

import time
import argparse

# Hardware imports - will fail on non-Pi, so allow dry-run
try:
    import board
    import busio
    from adafruit_motorkit import MotorKit
    HAS_HAT = True
except ImportError:
    HAS_HAT = False
    print("[WARN] Motor HAT libraries not found - running in dry-run print mode")

try:
    import smbus2
    HAS_IMU = True
except ImportError:
    HAS_IMU = False

MPU_ADDR = 0x68

# --- IMU (optional, for telemetry only) ---
imu_bus = None
if HAS_IMU:
    try:
        imu_bus = smbus2.SMBus(1)
        # wake up MPU6050
        imu_bus.write_byte_data(MPU_ADDR, 0x6B, 0x00)
    except Exception as e:
        print(f"[WARN] IMU init failed: {e}")
        imu_bus = None
        HAS_IMU = False

def read_imu_accel():
    """Return ax, ay, az raw or None if no IMU."""
    if not imu_bus:
        return None
    try:
        import struct
        raw = imu_bus.read_i2c_block_data(MPU_ADDR, 0x3B, 6)
        ax = struct.unpack('>h', bytes(raw[0:2]))[0]
        ay = struct.unpack('>h', bytes(raw[2:4]))[0]
        az = struct.unpack('>h', bytes(raw[4:6]))[0]
        return ax, ay, az
    except Exception as e:
        print(f"[WARN] IMU read failed: {e}")
        return None

# --- Motors ---
kit = None
if HAS_HAT:
    i2c = busio.I2C(board.SCL, board.SDA)
    kit = MotorKit(i2c=i2c)

def set_motors(m0, m1, m2):
    """Set throttles clamped to [-1.0, 1.0]. M0=motor1, M1=motor2, M2=motor3."""
    m0 = max(-1.0, min(1.0, m0))
    m1 = max(-1.0, min(1.0, m1))
    m2 = max(-1.0, min(1.0, m2))
    if kit:
        kit.motor1.throttle = m0
        kit.motor2.throttle = m1
        kit.motor3.throttle = m2
    print(f"motors -> M0:{m0:+.2f} M1:{m1:+.2f} M2:{m2:+.2f} | IMU:{read_imu_accel()}")

def stop():
    set_motors(0.0, 0.0, 0.0)

# --- Direction primitives (T-shape mixing) ---
def forward(speed):
    # pitch forward = +M1
    set_motors(0.0, speed, 0.0)

def backward(speed):
    # pitch backward = -M1
    set_motors(0.0, -speed, 0.0)

def left(speed):
    # roll left = +M0, -M2
    set_motors(speed, 0.0, -speed)

def right(speed):
    # roll right = -M0, +M2
    set_motors(-speed, 0.0, speed)

def do_move(name, func, speed, move_time, stop_time):
    print(f"\n== {name} for {move_time}s @ {speed} ==")
    func(speed)
    time.sleep(move_time)
    print("-- STOP --")
    stop()
    time.sleep(stop_time)

def main():
    parser = argparse.ArgumentParser(description="ORBS T-ball demo")
    parser.add_argument("--speed", type=float, default=0.5, help="0.0-1.0 throttle")
    parser.add_argument("--move-time", type=float, default=1.5, help="seconds per direction")
    parser.add_argument("--stop-time", type=float, default=0.7, help="pause between moves")
    parser.add_argument("--loop", action="store_true", help="repeat until Ctrl+C")
    args = parser.parse_args()

    speed = max(0.0, min(1.0, args.speed))

    print("ORBS Ball Robot Demo - Ctrl+C to stop")
    print(f"T-layout: M1=forward/back, M0/M2=left/right | speed={speed}")
    time.sleep(1.0)

    try:
        while True:
            do_move("FORWARD", forward, speed, args.move_time, args.stop_time)
            do_move("BACKWARD", backward, speed, args.move_time, args.stop_time)
            do_move("LEFT", left, speed, args.move_time, args.stop_time)
            do_move("RIGHT", right, speed, args.move_time, args.stop_time)

            if not args.loop:
                break

        print("\nDemo complete!")
    except KeyboardInterrupt:
        print("\nInterrupted!")
    finally:
        stop()
        print("Motors stopped.")

if __name__ == "__main__":
    main()
