#!/usr/bin/env python3
"""A fake rosbridge — just enough of the protocol to test the off-cell clients without the cell.

`tools/test_two_arm_switch.py` constructs the real pad client, which connects to rosbridge before it
does anything, so without the Xavier on the LAN the suite cannot even start. This is a WebSocket that
speaks rosbridge v2 (advertise / subscribe / unsubscribe / publish) and simulates two arms:

  - publishes <ns>/joint_states at 20 Hz for every topic subscribed to
  - when a client publishes <ns>/joint_targets the arm "follows": the next readings come back as
    those targets, moving at most 6 deg per tick like a real servo

    uv run --with websockets python tools/fake_rosbridge.py --port 9099
    SDL_VIDEODRIVER=dummy uv run --with pygame --with roslibpy python tools/test_two_arm_switch.py \
        --host 127.0.0.1 --port 9099

What it proves: the client's wiring, its namespace (it must ask for /arm1/joint_states and publish
/arm1/joint_targets), the A/B/X/Y rules and the arming pose. What it cannot prove: the torque HTTP
proxies and anything about the real servos — the LAN check in the suite stays red by design.
"""

import argparse
import asyncio
import json
import math
import time

import websockets

STATE = {}          # namespace -> {"pos": [6 floats], "goal": [6 floats]}
SUBS = {}           # topic -> set(websocket)
HZ = 20.0


def ns_of(topic):
    return topic.strip("/").split("/")[0] if topic.strip("/").count("/") else ""


def ensure(ns):
    STATE.setdefault(ns, {"pos": [0.0, -10.0, 20.0, 0.0, 0.0, 10.0],
                          "goal": [0.0, -10.0, 20.0, 0.0, 0.0, 10.0]})


async def pump():
    k = 0
    while True:
        k += 1
        for topic, clients in list(SUBS.items()):
            if not clients or not topic.endswith("/joint_states"):
                continue
            ns = ns_of(topic)
            ensure(ns)
            st = STATE[ns]
            for i in range(6):
                d = st["goal"][i] - st["pos"][i]
                st["pos"][i] += max(-6.0, min(6.0, d))
            msg = {"op": "publish", "topic": topic,
                   "msg": {"header": {"stamp": {"sec": int(time.time()), "nanosec": k * 50000000},
                                      "frame_id": ""},
                           "name": ["shoulder_pan", "shoulder_lift", "elbow_flex",
                                    "wrist_flex", "wrist_roll", "gripper"],
                           "position": [round(v, 3) for v in st["pos"]],
                           "velocity": [], "effort": []}}
            raw = json.dumps(msg)
            for ws in list(clients):
                try:
                    await ws.send(raw)
                except Exception:
                    clients.discard(ws)
        await asyncio.sleep(1.0 / HZ)


async def handler(ws):
    print("client connected", flush=True)
    async for raw in ws:
        try:
            m = json.loads(raw)
        except json.JSONDecodeError:
            continue
        op, topic = m.get("op"), m.get("topic", "")
        if op == "subscribe":
            SUBS.setdefault(topic, set()).add(ws)
            print(f"subscribe {topic}", flush=True)
        elif op == "unsubscribe":
            SUBS.get(topic, set()).discard(ws)
        elif op == "advertise":
            print(f"advertise {topic}", flush=True)
        elif op == "publish":
            data = (m.get("msg") or {}).get("data")
            if topic.endswith("/joint_targets") and data:
                ns = ns_of(topic)
                ensure(ns)
                STATE[ns]["goal"] = [float(v) for v in data][:6]
                print(f"targets {topic} <- {[round(v, 1) for v in data]}", flush=True)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9099)
    a = ap.parse_args()
    async with websockets.serve(handler, "127.0.0.1", a.port, max_size=None):
        print(f"fake rosbridge on ws://127.0.0.1:{a.port}", flush=True)
        await pump()


if __name__ == "__main__":
    asyncio.run(main())
