#!/usr/bin/env python3
"""Headless test of the F710 client's two-arm switch (Plan Phase 5) — no pad needed.

Loads `windows/f710_teleop_ws.py`, fakes the A button and drives `handle_a()` at the real tick
rate against the live cell over rosbridge. Checks the four rules that matter: a tap does nothing,
5 s switches, holding longer does not fire twice, a release re-arms — and a switch to an arm whose
states have gone stale is refused.

    SDL_VIDEODRIVER=dummy uv run --with pygame --with roslibpy python tools/test_two_arm_switch.py --host 192.168.1.185

Exit code 0 means every check passed; it prints one line per check either way.
"""

import argparse
import importlib.util
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TELEOP = os.path.join(HERE, "windows", "f710_teleop_ws.py")

fails = []


def check(name, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + f"{name}: {got!r} (expected {want!r})")
    if not ok:
        fails.append(name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.185", help="the Xavier's LAN address")
    ap.add_argument("--port", type=int, default=9090)
    args = ap.parse_args()

    spec = importlib.util.spec_from_file_location("teleop", TELEOP)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["teleop"] = mod
    spec.loader.exec_module(mod)

    t = mod.Teleop(args.host, args.port)
    a_down = {"v": False}
    t._button = lambda b: a_down["v"] if b == 0 else False

    def hold(seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            t.handle_a()
            time.sleep(0.02)

    check("starts on arm 1", t.active, 0)

    print("A for 0.4 s (a tap)")
    a_down["v"] = True
    hold(0.4)
    check("a tap does not switch", t.active, 0)

    print("keep holding past 5 s")
    hold(5.0)
    check("holding 5 s switches to arm 2", t.active, 1)

    print("6 more seconds with A still down")
    hold(6.0)
    check("does not fire twice without a release", t.active, 1)

    a_down["v"] = False
    t.handle_a()
    print("release, then hold again")
    a_down["v"] = True
    hold(5.4)
    check("a second hold switches back to arm 1", t.active, 0)

    print("switch while the far arm is silent")
    a_down["v"] = False
    t.handle_a()
    t.arm_state_t[1] = time.time() - 10.0
    t.switch_arm()
    check("refuses an arm that stopped answering", t.active, 0)

    for p in t.pubs:
        p.unadvertise()
    t.client.terminate()
    print("\nRESULT:", "all ok" if not fails else f"{len(fails)} failed: {fails}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
