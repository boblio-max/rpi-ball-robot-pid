# ORBS Ball Robot

a robot that balances (and drives) on top of a ball. Pi Zero 2W + MPU6050 IMU + Adafruit Motor HAT pushing 3 DC motors in a T-layout, holding the whole thing upright at 100 Hz. it's the classic inverted-pendulum problem except the pendulum is a ball and the floor is lava (not really, but it feels like it when it falls).

## how it actually works

- complementary filter (α=0.98) fuses gyro + accel into clean pitch/roll at 100 Hz
- per-axis PID loops compute corrections, mixed as `M1=pitch, M0=+roll, M2=−roll` across the three T-layout motors
- `delete.py` is the Python reference controller, `code.or` is the same controller ported to Origin (yes, the robot runs on my programming language, that's the flex)
- `demo.py` runs scripted moves, `laptop_controller.py` is a WASD teleop server, `pi_receiver.py` is the failsafe client on the Pi (link dies → motors stop)

```bash
pip install websockets smbus2 adafruit-circuitpython-motorkit
python laptop_controller.py  # laptop side, WASD to drive
```

## tuning + safety

gains are in the repo but every ball/floor/motor combo drifts differently — start with P low and work up. test with the ball chocked before free-balancing, and keep fingers clear of the motors. failsafe exists but physics doesn't care about your code.

## stack

Python (+ Origin port), Pi Zero 2W, MPU6050 over I2C, Motor HAT, websockets teleop.
