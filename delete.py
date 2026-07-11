import time
import struct
import math
import board
import busio
from adafruit_motorkit import MotorKit
import smbus2

# ──────────────────────────────────────────────
# MPU6050
# ──────────────────────────────────────────────
MPU_ADDR = 0x68
bus = smbus2.SMBus(1)

def mpu_init():
    bus.write_byte_data(MPU_ADDR, 0x6B, 0x00)
    bus.write_byte_data(MPU_ADDR, 0x1B, 0x00)
    bus.write_byte_data(MPU_ADDR, 0x1C, 0x00)

def mpu_read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)
    val = (high << 8) | low
    if val >= 0x8000:
        val -= 0x10000
    return val

def mpu_read_accel():
    raw = bus.read_i2c_block_data(MPU_ADDR, 0x3B, 6)
    ax = struct.unpack('>h', bytes(raw[0:2]))[0]
    ay = struct.unpack('>h', bytes(raw[2:4]))[0]
    az = struct.unpack('>h', bytes(raw[4:6]))[0]
    return ax, ay, az

def mpu_read_gyro():
    raw = bus.read_i2c_block_data(MPU_ADDR, 0x43, 6)
    gx = struct.unpack('>h', bytes(raw[0:2]))[0]
    gy = struct.unpack('>h', bytes(raw[2:4]))[0]
    gz = struct.unpack('>h', bytes(raw[4:6]))[0]
    return gx, gy, gz

# ──────────────────────────────────────────────
# Complementary filter
# ──────────────────────────────────────────────
angle_x = 0.0
angle_y = 0.0
alpha = 0.98

def update_angles(dt):
    global angle_x, angle_y
    ax, ay, az = mpu_read_accel()
    gx, gy, gz = mpu_read_gyro()

    accel_x = math.atan2(ay, math.sqrt(ax * ax + az * az)) * 180.0 / math.pi
    accel_y = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi

    gyro_x = gy / 131.0
    gyro_y = -gx / 131.0

    angle_x = alpha * (angle_x + gyro_x * dt) + (1 - alpha) * accel_x
    angle_y = alpha * (angle_y + gyro_y * dt) + (1 - alpha) * accel_y

    return angle_x, angle_y

# ──────────────────────────────────────────────
# PID + Feedforward
# ──────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, ff=0.0, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ff = ff
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.monotonic()

    def compute(self, setpoint, measurement):
        now = time.monotonic()
        dt = now - self.prev_time
        if dt <= 0:
            dt = 1e-3
        self.prev_time = now

        error = setpoint - measurement

        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = (self.kp * error
                  + self.ki * self.integral
                  + self.kd * derivative
                  + self.ff * setpoint)

        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.monotonic()

# ──────────────────────────────────────────────
# Motor mixing — T shape
#         M1
#         |
#  M0 ---+--- M2
#
# M0/M2 = horizontal bar (roll axis)
# M1    = vertical bar   (pitch axis)
# ──────────────────────────────────────────────
i2c = busio.I2C(board.SCL, board.SDA)
kit = MotorKit(i2c=i2c)

def set_motors(m0, m1, m2):
    kit.motor1.throttle = max(-1.0, min(1.0, m0))
    kit.motor2.throttle = max(-1.0, min(1.0, m1))
    kit.motor3.throttle = max(-1.0, min(1.0, m2))

def motor_mix(pitch_out, roll_out):
    m0 = roll_out
    m1 = pitch_out
    m2 = -roll_out
    set_motors(m0, m1, m2)

# ──────────────────────────────────────────────
# Main loop
# ──────────────────────────────────────────────
def main():
    mpu_init()
    time.sleep(0.1)

    pid_roll = PID(kp=0.010, ki=0.001, kd=0.002, ff=0.005, output_limit=1.0)
    pid_pitch = PID(kp=0.010, ki=0.001, kd=0.002, ff=0.005, output_limit=1.0)

    setpoint_x = 0.0
    setpoint_y = 0.0
    loop_hz = 100
    loop_dt = 1.0 / loop_hz

    print("Ball robot starting — Ctrl+C to stop")
    time.sleep(1.0)

    try:
        while True:
            t0 = time.monotonic()

            roll, pitch = update_angles(loop_dt)

            roll_out = pid_roll.compute(setpoint_x, roll)
            pitch_out = pid_pitch.compute(setpoint_y, pitch)

            motor_mix(pitch_out, roll_out)

            elapsed = time.monotonic() - t0
            sleep_time = loop_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        set_motors(0.0, 0.0, 0.0)
        print("\nStopped")

if __name__ == "__main__":
    main()
