#!/usr/bin/env python3
"""Headless test of the F710 client's torque (B) and arm switch (A) — no pad needed.

Loads `windows/f710_teleop_ws.py`, fakes the buttons **and the two torque HTTP calls**, so nothing on
the real arm is ever energised, then checks the rules the operator feels:

  start-up   both arms end up torque-off without being asked
  switch (A) starts on arm 1 · a tap does nothing · 5 s switches · no double fire without a release
             · a second hold switches back · a silent arm is refused · **A never buzzes**
  torque (B) 5 s arms · one buzz now and the second 3 s later · the commanded pose is the one read
             with torque on (no jump from a stale buffer) · 5 s again releases with one buzz ·
             a stale pose refuses to arm with a long buzz

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

    # Fake the torque HTTP pair at CLASS level, before construction: the client now writes
    # torque-off to both arms while it starts, and this test must not touch the real servos.
    fake = {"on": True}
    real_read = mod.Teleop.read_arm_torque                    # kept for the LAN regression check
    mod.Teleop.read_arm_torque = lambda self, arm: fake["on"]
    mod.Teleop.write_arm_torque = lambda self, arm, want: fake.update(on=want) or want

    t = mod.Teleop(args.host, args.port)

    check("start-up leaves both arms torque-off", t.torque_on, [False, False])
    check("start-up really wrote torque off", fake["on"], False)

    # Before anything else is faked: the real HTTP path must work. `roslibpy` 2.x has no `.host`
    # on the Ros object, and reading it there turned every torque call into an exception that
    # looked exactly like a dead proxy ("torque: cannot read state").
    check("reads a real arm's torque over the LAN", real_read(t, 0) is not None, True)

    buttons = {"a": False, "b": False}
    t._button = lambda b: buttons["a"] if b == 0 else (buttons["b"] if b == 1 else False)

    buzzes = []
    t.rumble = lambda kind, count=1, **kw: buzzes.append((kind, count))   # `quiet` only silences the log
    said = []
    t.say = lambda text, key=None: said.append(text)
    # The measurement lines are plain print() on purpose (no dedupe), so capture those too.
    mod.print = lambda *a, **k: said.append(" ".join(str(x) for x in a))
    sent = []
    t.send = lambda: sent.append(list(t.targets))

    def tick_for(seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            t.service_buzz()
            t.handle_a()
            t.handle_b()
            time.sleep(0.02)

    print("== the client tracks the real pose on every message")
    t.on_state({"position": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]}, 1)
    first = list(t.arm_targets[1])
    t.on_state({"position": [7.0, 8.0, 9.0, 10.0, 11.0, 12.0]}, 1)
    check("a second message updates the stored pose", list(t.arm_targets[1]),
          [7.0, 8.0, 9.0, 10.0, 11.0, 12.0])
    check("and it is not stuck on the first one", first, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    t.arm_state_t[1] = 0.0

    # ---------------------------------------------------------------- switch, on A (silent)
    print("== switch (A)")
    check("starts on arm 1", t.active, 0)
    buzzes.clear()
    buttons["a"] = True
    tick_for(0.4)
    check("a tap does not switch", t.active, 0)
    tick_for(5.0)
    check("holding 5 s switches to arm 2", t.active, 1)
    tick_for(6.0)
    check("does not fire twice without a release", t.active, 1)
    buttons["a"] = False
    t.handle_a()
    buttons["a"] = True
    tick_for(5.4)
    check("a second hold switches back to arm 1", t.active, 0)
    buttons["a"] = False
    t.handle_a()
    t.arm_state_t[1] = time.time() - 10.0
    t.switch_arm()
    check("refuses a silent arm", t.active, 0)
    check("A never buzzes", buzzes, [])

    # ---------------------------------------------------------------- torque, on B
    print("== torque (B)")
    buzzes.clear()
    sent.clear()
    t.arm_state_t[t.active] = time.time()          # a fresh pose for the arming read
    t.targets = [0.0] * 6                          # stale: whatever was left in the buffer
    real = list(t.arm_targets[t.active])
    fake["on"] = False
    buttons["b"] = True
    tick_for(5.2)
    check("5 s arms the active arm", fake["on"], True)
    check("one buzz at the arm moment", buzzes, [("short", 1)])
    gap = (t.buzz_at or 0) - time.time()
    check("the second buzz is scheduled ~1 s later", 0.5 < gap < 1.5, True)
    close("commands the pose read with torque on, not the stale buffer", t.targets, real)
    check("published exactly one target", len(sent), 1)
    close("and that target is the read pose", sent[0] if sent else [], real)
    tick_for(1.3)
    check("the second buzz fires after 1 s", buzzes, [("short", 1), ("short", 1)])
    check("says it armed, without dumping the pose",
          any("torque: ARMED" in x for x in said)
          and not any("read before" in x or "read after" in x for x in said), True)
    check("and does not report a pose later",
          any("3 s after" in x for x in said), False)
    check("no report is left pending", getattr(t, "report_at", None), None)
    tick_for(4.0)
    check("does not fire again while B stays down", len(buzzes), 2)

    buttons["b"] = False
    t.handle_b()
    buttons["b"] = True
    buzzes.clear()
    tick_for(5.2)
    check("5 s again releases it", fake["on"], False)
    check("one buzz when releasing", buzzes, [("short", 1)])

    check("the release says so, and prints no pose",
          any("torque: RELEASED" in x for x in said)
          and not any("just before releasing" in x for x in said), True)

    buttons["b"] = False
    t.handle_b()
    t.targets = [-33.0, 12.0, -80.0, 40.0, -70.0, 5.0]   # an old command, far from reality
    t.arm_state_t[t.active] = time.time()
    real2 = list(t.arm_targets[t.active])
    buttons["b"] = True
    tick_for(5.2)
    close("a later arm cycle still takes the pose it reads", t.targets, real2)

    # a silent arm refuses to arm
    buttons["b"] = False
    t.handle_b()
    fake["on"] = False
    buzzes.clear()
    t.arm_state_t[t.active] = time.time() - 10.0
    t.toggle_torque()
    check("refuses to arm without a fresh pose", fake["on"], False)
    check("long buzz when refusing", buzzes, [("long", 1)])

    print("== pad X records, pad Y replays (the keyboard no longer prints anything)")
    fake["on"] = True
    t.torque_on[t.active] = True
    check("print_pose is gone", hasattr(t, "print_pose"), False)
    said.clear()
    t._button = lambda b: b == 2          # pad X
    t.prev.clear(); t.tick()
    check("pad X starts a recording", t.recording, True)
    check("and says so", any("RECORDING started" in x for x in said), True)
    t.record = [[1.0] * 6, [2.0] * 6]
    t._button = lambda b: b == 3          # pad Y
    t.prev.clear(); t.tick()
    check("pad Y starts the replay", t.replaying, True)
    t.replaying = False
    t.recording = False
    t._button = lambda b: False

    for p in t.pubs:
        p.unadvertise()
    t.client.terminate()
    print("\nRESULT:", "all ok" if not fails else f"{len(fails)} failed: {fails}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
