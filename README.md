# ORBS Ball Robot

A ball-balancing robot: Raspberry Pi Zero 2W + MPU6050 IMU + Adafruit Motor HAT
driving 3x DC motors in a T-layout to balance (and drive) on a ball.

```
        M1 (pitch: forward / backward)
        |
M0 -----+----- M2   (roll: left / right)
```

Mixing: `M1 = pitch`, `M0 = +roll`, `M2 = -roll`, throttles clamped to [-1, 1].

## Files

| File | What it is |
|---|---|
| `code.or` | Full controller in the [Origin language](https://github.com/boblio-max/origin-dev): complementary filter + PID classes + 100 Hz main loop, with `py {}` blocks for I2C/motor hardware access. Run with `origin i code.or`. |
| `delete.py` | Pure-Python mirror of the same controller (filter + PID + mixing). Working reference for the Origin port. |
| `demo.py` | Scripted movement demo: forward → backward → left → right with stops. Runs on the Pi, dry-runs (prints) on a laptop. |
| `laptop_controller.py` | WASD teleop server (websockets, 20 Hz). Run on the laptop; the Pi connects to it. |
| `pi_receiver.py` | Pi-side client: receives joystick packets, mixes to motors at 20 Hz. Failsafe: link age > 1 s or disarmed → motors stop. |

## Quick start

```bash
# Dry-run the demo on any machine (no hardware needed)
python demo.py
python demo.py --speed 0.5 --move-time 1.5 --loop

# Teleop: terminal 1 (laptop), terminal 2 (Pi)
pip install websockets
python laptop_controller.py --port 8766
python3 pi_receiver.py --server <LAPTOP_IP> --port 8766
```

Teleop keys (laptop window focused): `W` forward / `S` backward / `A` left /
`D` right, `R`/`F` speed up/down, `P` arm-disarm, `SPACE` e-stop, `X` quit.

On the Pi also install: `adafruit-circuitpython-motorkit smbus2 websockets`.

## Control approach

- **Attitude**: complementary filter (`ALPHA = 0.98`) fusing accelerometer tilt
  (atan2) with integrated gyro rates at 100 Hz.
- **Balance**: one PID per axis (`kp = 0.010, ki = 0.001, kd = 0.002`,
  feed-forward `0.005`, output limited to ±1.0) holding 0° setpoint.
- **Origin port**: `code.or` keeps the filter math, PID logic, and motor mixing
  in native Origin; only raw I2C reads and motor writes live in `py {}` blocks.

## Safety

Test with the ball chocked or the robot tethered first. E-stop (`SPACE`) cuts
and disarms. The receiver failsafe stops the motors if the link drops.
