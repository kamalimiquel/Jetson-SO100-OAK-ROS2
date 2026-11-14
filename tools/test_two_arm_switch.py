#!/usr/bin/env python3
"""Headless test of the F710 client's torque (A) and arm switch (Y) — no pad needed.

Loads `windows/f710_teleop_ws.py`, fakes the buttons and the two HTTP calls (so nothing on the real
arm is energised), and checks the rules the operator feels:

  switch (Y)  starts on arm 1 · a tap does nothing · 5 s switches · no double fire without a release
              · a second hold switches back · a silent arm is refused
  torque (A)  5 s arms with two buzzes and publishes the arm's REAL pose (no jump) · 5 s again
              releases with one buzz · a stale target buffer still resolves to the real pose
              · a stale pose refuses to arm with a long buzz

    SDL_VIDEODRIVER=dummy uv run --with pygame --with roslibpy python tools/test_two_arm_switch.py

Exit code 0 means every check passed.
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


def close(name, got, want, tol=0.05):
    ok = len(got) == len(want) and all(abs(a - b) <= tol for a, b in zip(got, want))
    print(("  ok   " if ok else "  FAIL ") + f"{name}: {[round(v, 2) for v in got]}"
          + ("" if ok else f" (expected {[round(v, 2) for v in want]})"))
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

    # Before anything is faked: the real HTTP path must work. `roslibpy` 2.x has no `.host` on the
    # Ros object, and reading it there turned every torque call into an exception that looked
    # exactly like a dead proxy ("torque: cannot read state").
    check("reads a real arm's torque over the LAN", t.read_arm_torque(0) is not None, True)

    buttons = {"a": False, "y": False}
    t._button = lambda b: buttons["a"] if b == 0 else (buttons["y"] if b == 3 else False)

    buzzes = []
    t.rumble = lambda kind, count=1: buzzes.append((kind, count))

    # The torque HTTP pair is faked: this test must not energise the real arm.
    fake = {"on": False}
    t.read_arm_torque = lambda arm: fake["on"]
    t.write_arm_torque = lambda arm, want: fake.update(on=want) or want
    sent = []
    t.send = lambda: sent.append(list(t.targets))

    def tick_for(seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            t.handle_a()
            t.handle_y()
            time.sleep(0.02)

    # ---------------------------------------------------------------- switch, on Y
    print("== switch (Y)")
    check("starts on arm 1", t.active, 0)
    buttons["y"] = True
    tick_for(0.4)
    check("a tap does not switch", t.active, 0)
    tick_for(5.0)
    check("holding 5 s switches to arm 2", t.active, 1)
    tick_for(6.0)
    check("does not fire twice without a release", t.active, 1)
    buttons["y"] = False
    t.handle_y()
    buttons["y"] = True
    tick_for(5.4)
    check("a second hold switches back to arm 1", t.active, 0)
    buttons["y"] = False
    t.handle_y()
    t.arm_state_t[1] = time.time() - 10.0
    t.switch_arm()
    check("refuses a silent arm", t.active, 0)

    # ---------------------------------------------------------------- torque, on A
    print("== torque (A)")
    buzzes.clear()
    sent.clear()
    real = list(t.arm_targets[t.active])
    t.targets = [0.0] * 6                      # stale: whatever was left in the buffer
    fake["on"] = False
    buttons["a"] = True
    tick_for(5.2)
    check("5 s arms the active arm", fake["on"], True)
    check("two buzzes when arming", buzzes, [("short", 2)])
    close("armed holding the real pose, not the stale buffer", t.targets, real)
    check("published exactly one target", len(sent), 1)
    close("and that target is the real pose", sent[0] if sent else [], real)
    tick_for(4.0)
    check("does not fire twice while A stays down", len(buzzes), 1)

    buttons["a"] = False
    t.handle_a()
    buttons["a"] = True
    buzzes.clear()
    tick_for(5.2)
    check("5 s again releases it", fake["on"], False)
    check("one buzz when releasing", buzzes, [("short", 1)])

    buttons["a"] = False
    t.handle_a()
    t.targets = [-33.0, 12.0, -80.0, 40.0, -70.0, 5.0]   # an old command, far from reality
    real2 = list(t.arm_targets[t.active])
    buttons["a"] = True
    tick_for(5.2)
    close("a later arm cycle still holds the real pose", t.targets, real2)

    # a silent arm refuses to arm
    buttons["a"] = False
    t.handle_a()
    fake["on"] = False
    buzzes.clear()
    t.arm_state_t[t.active] = time.time() - 10.0
    t.toggle_torque()
    check("refuses to arm without a fresh pose", fake["on"], False)
    check("long buzz when refusing", buzzes, [("long", 1)])

    for p in t.pubs:
        p.unadvertise()
    t.client.terminate()
    print("\nRESULT:", "all ok" if not fails else f"{len(fails)} failed: {fails}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
