"""
ORBS Ball Robot - Pi Receiver
Run: python3 pi_receiver.py --server <LAPTOP_IP> --port 8766
Laptop: python laptop_controller.py --port 8766

Drives 3x DC motors (T-shape) via Adafruit Motor HAT:
  M1 (motor2) = pitch axis -> forward/backward
  M0 (motor1) / M2 (motor3) = roll axis -> left/right

Mixing (same as demo.py):
  forward  (pitch=+1): M1=+speed
  backward (pitch=-1): M1=-speed
  left     (roll=-1):  M0=-speed, M2=+speed
  right    (roll=+1):  M0=+speed, M2=-speed

Failsafe: link age >1s or disarmed -> motors stop.
Requires on Pi: pip install websockets adafruit-circuitpython-motorkit smbus2
"""
import argparse
import asyncio
import json
import time
import threading

try:
    import board
    import busio
    from adafruit_motorkit import MotorKit
    HAS_HAT = True
except ImportError:
    HAS_HAT = False
    print("[WARN] Motor HAT libs missing - dry-run print mode")

try:
    import smbus2
    HAS_IMU = True
except ImportError:
    HAS_IMU = False

import websockets

MPU_ADDR = 0x68
FAILSAFE_S = 1.0
RECONNECT_DELAY = 1.0

cmd_lock = threading.Lock()
cmd = {"pitch": 0.0, "roll": 0.0, "speed": 0.5, "armed": False, "ts": 0.0}
last_rx = time.monotonic()
telemetry_out = {"cmd": "telemetry", "m0": 0.0, "m1": 0.0, "m2": 0.0,
                 "armed": False, "status": "SAFE"}
stop_flag = False

# --- Motors ---
kit = None
if HAS_HAT:
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        kit = MotorKit(i2c=i2c)
        print("[MOTORS] Motor HAT ready")
    except Exception as e:
        print(f"[WARN] Motor HAT init failed: {e}")
        kit = None
        HAS_HAT = False

# --- IMU (telemetry only) ---
imu_bus = None
if HAS_IMU:
    try:
        imu_bus = smbus2.SMBus(1)
        imu_bus.write_byte_data(MPU_ADDR, 0x6B, 0x00)
    except Exception as e:
        print(f"[WARN] IMU init failed: {e}")
        imu_bus = None


def set_motors(m0, m1, m2):
    m0 = max(-1.0, min(1.0, m0))
    m1 = max(-1.0, min(1.0, m1))
    m2 = max(-1.0, min(1.0, m2))
    if kit:
        kit.motor1.throttle = m0
        kit.motor2.throttle = m1
        kit.motor3.throttle = m2
    telemetry_out.update({"m0": round(m0, 2), "m1": round(m1, 2), "m2": round(m2, 2)})
    print(f"motors M0:{m0:+.2f} M1:{m1:+.2f} M2:{m2:+.2f}")


def stop_motors():
    set_motors(0.0, 0.0, 0.0)


def drive_loop():
    """20Hz loop: read latest joystick cmd -> mix -> motors."""
    global telemetry_out
    dt = 1.0 / 20.0
    print("[DRIVE] loop live (20Hz)")
    while not stop_flag:
        t0 = time.monotonic()
        with cmd_lock:
            c = dict(cmd)
            age = time.monotonic() - last_rx
        if (not c["armed"]) or (age > FAILSAFE_S):
            stop_motors()
            telemetry_out.update({
                "armed": False,
                "status": "SAFE" if not c["armed"] else "LINK-LOST",
            })
            time.sleep(dt)
            continue
        speed = max(0.0, min(1.0, float(c.get("speed", 0.5))))
        pitch = max(-1.0, min(1.0, float(c.get("pitch", 0.0))))
        roll = max(-1.0, min(1.0, float(c.get("roll", 0.0))))
        # T-mix (matches demo.py)
        m1 = pitch * speed
        m0 = roll * speed
        m2 = -roll * speed
        set_motors(m0, m1, m2)
        names = []
        if pitch > 0:
            names.append("FWD")
        elif pitch < 0:
            names.append("BACK")
        if roll > 0:
            names.append("RIGHT")
        elif roll < 0:
            names.append("LEFT")
        telemetry_out.update({
            "armed": True,
            "status": "FLY:" + ("+".join(names) if names else "HOLD"),
        })
        el = time.monotonic() - t0
        if dt - el > 0:
            time.sleep(dt - el)
    stop_motors()


async def net_client(uri):
    global last_rx
    while not stop_flag:
        print(f"[PI] connecting to {uri} ...")
        try:
            async with websockets.connect(uri, ping_interval=20, ping_timeout=20) as ws:
                print("[PI] connected.")
                await ws.send(json.dumps({"cmd": "hello", "name": "orbs_pi", "type": "pi_client"}))
                try:
                    ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))
                    if ack.get("ok"):
                        print("[PI] server hello-ack.")
                except Exception:
                    pass
                last_rx = time.monotonic()

                async def rx_loop():
                    global last_rx
                    async for raw in ws:
                        try:
                            m = json.loads(raw)
                        except json.JSONDecodeError:
                            continue
                        if m.get("cmd") == "joystick":
                            with cmd_lock:
                                for k in ("pitch", "roll", "speed", "armed"):
                                    if k in m:
                                        cmd[k] = m[k]
                                cmd["ts"] = m.get("ts", time.time())
                            last_rx = time.monotonic()

                async def tx_loop():
                    while True:
                        try:
                            await ws.send(json.dumps(telemetry_out))
                        except websockets.exceptions.ConnectionClosed:
                            break
                        await asyncio.sleep(0.5)

                await asyncio.gather(rx_loop(), tx_loop())
        except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError, OSError) as e:
            print(f"[PI] link {e}, retry {RECONNECT_DELAY}s...")
            await asyncio.sleep(RECONNECT_DELAY)


def main():
    ap = argparse.ArgumentParser(description="ORBS pi receiver - websocket -> motors")
    ap.add_argument("--server", required=True, help="laptop IP")
    ap.add_argument("--port", type=int, default=8766)
    a = ap.parse_args()
    t = threading.Thread(target=drive_loop, daemon=True)
    t.start()
    try:
        asyncio.run(net_client(f"ws://{a.server}:{a.port}"))
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        globals()["stop_flag"] = True


if __name__ == "__main__":
    main()
