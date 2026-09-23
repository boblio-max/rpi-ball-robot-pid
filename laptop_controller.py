"""
ORBS Ball Robot - Laptop Controller
Run: python laptop_controller.py --port 8766
Pi : python3 pi_receiver.py --server <LAPTOP_IP> --port 8766

Controls (hold keys, window focused):
  W forward / S backward / A left / D right
  R speed+ / F speed- / SPACE e-stop / P arm-disarm / X quit

Requires: pip install websockets
"""
import argparse
import asyncio
import json
import time
from collections import deque

import websockets

try:
    import msvcrt
    HAS_MSVCRT = True
except ImportError:
    HAS_MSVCRT = False

HOST = "0.0.0.0"
SEND_HZ = 20
HOLD_TIME = 0.35

connected = set()
client_types = {}
state = {"speed": 0.5, "armed": False}
telem = {"m0": 0.0, "m1": 0.0, "m2": 0.0, "status": "NO-LINK"}
key_expiry = {}
logs = deque(maxlen=8)
quit_flag = False


def log(msg):
    logs.append(f"{time.strftime('%H:%M:%S')} {msg}")
    print(msg)


def press(k):
    key_expiry[k.lower()] = time.monotonic() + HOLD_TIME


def active(k):
    return time.monotonic() < key_expiry.get(k.lower(), 0)


def keyboard_poll():
    if not HAS_MSVCRT:
        print("[KEYS] no msvcrt (not Windows) - type w/a/s/d + Enter, x to quit")
        while True:
            try:
                s = input().strip().lower()
                if not s:
                    continue
                if s[0] == " ":
                    press("space")
                else:
                    press(s[0])
                if s[0] == "x":
                    break
            except EOFError:
                break
        return
    print("[KEYS] W fwd / S back / A left / D right / R-F speed / P arm / SPACE stop / X quit")
    while True:
        if quit_flag:
            break
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in (b"\x00", b"\xe0"):
                try:
                    msvcrt.getch()
                except Exception:
                    pass
                continue
            if ch == b" ":
                press("space")
            elif ch == b"\x03":
                press("x")
            else:
                try:
                    press(ch.decode("utf-8", errors="ignore"))
                except Exception:
                    pass
        time.sleep(0.01)


async def handler(ws):
    connected.add(ws)
    log(f"[+] {ws.remote_address} connected")
    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                await ws.send(json.dumps({"error": "invalid json"}))
                continue
            if msg.get("cmd") == "hello":
                client_types[ws] = msg.get("type", "?")
                log(f"'{msg.get('name')}' joined")
                await ws.send(json.dumps({"ack": "hello", "ok": True}))
            elif msg.get("cmd") == "telemetry":
                telem.update(msg)
            else:
                await ws.send(json.dumps({"error": "unknown cmd"}))
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected.discard(ws)
        client_types.pop(ws, None)
        log("[-] client left")


async def broadcast():
    global quit_flag
    while True:
        if active("x"):
            log("quit")
            quit_flag = True
            for ws in list(connected):
                try:
                    await ws.close()
                except Exception:
                    pass
            asyncio.get_event_loop().stop()
            return
        if active("p"):
            state["armed"] = not state["armed"]
            log("ARMED" if state["armed"] else "DISARMED")
            key_expiry.clear()
            await asyncio.sleep(0.3)
            continue
        if active("space"):
            state["armed"] = False
            log("[E-STOP] disarmed + stop")
            key_expiry.clear()
            await asyncio.sleep(0.3)
            continue
        if active("r"):
            state["speed"] = min(1.0, round(state["speed"] + 0.05, 2))
            key_expiry.pop("r", None)
            log(f"speed {state['speed']:.2f}")
        if active("f"):
            state["speed"] = max(0.1, round(state["speed"] - 0.05, 2))
            key_expiry.pop("f", None)
            log(f"speed {state['speed']:.2f}")

        # T-ball axes: pitch = W-S (M1), roll = D-A (M0/M2)
        pitch = float(active("w") - active("s"))
        roll = float(active("d") - active("a"))
        pkt = {
            "cmd": "joystick",
            "pitch": pitch,
            "roll": roll,
            "speed": state["speed"],
            "armed": state["armed"],
            "ts": time.time(),
        }
        targets = [c for c in connected if client_types.get(c) == "pi_client"]
        if targets:
            data = json.dumps(pkt)
            await asyncio.gather(*(c.send(data) for c in targets), return_exceptions=True)

        # simple status line
        n_pi = len(targets)
        print(f"\r{'ARMED' if state['armed'] else 'SAFE '} spd:{state['speed']:.2f} "
              f"WASD:[{'W' if pitch>0 else ' '}{'S' if pitch<0 else ' '}{'A' if roll<0 else ' '}{'D' if roll>0 else ' '}] "
              f"pi:{n_pi} {telem.get('status','?')}   ", end="", flush=True)
        await asyncio.sleep(1.0 / SEND_HZ)


async def main(port):
    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, keyboard_poll)
    log(f"listening ws://0.0.0.0:{port} | pi: --server <LAPTOP_IP> --port {port}")
    async with websockets.serve(handler, HOST, port, compression=None,
                                max_size=2**20, ping_interval=20,
                                ping_timeout=20):
        await asyncio.Future()  # run until X / Ctrl+C


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="ORBS laptop WASD controller")
    ap.add_argument("--port", type=int, default=8766)
    a = ap.parse_args()
    try:
        asyncio.run(main(a.port))
    except KeyboardInterrupt:
        print("\nbye")
