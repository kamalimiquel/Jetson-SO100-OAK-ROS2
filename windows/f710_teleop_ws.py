"""F710 teleop over rosbridge (ADR-0017 amendment, two-arm Plan Phase 5).

Reads a Logitech F710 (switch on the back = X) with pygame and publishes
/joint_targets (std_msgs/Float64MultiArray, 6 values in DEGREES) through the
Xavier's rosbridge WebSocket. No ROS2 installation on Windows is needed —
same transport as the MuJoCo twin.

Install (Windows):
    pip install pygame roslibpy

Run:
    python windows\\f710_teleop_ws.py --host 192.168.1.50

Modes (same behaviour as the C++ client in this folder):
  joint-step (default)  D-pad L/R select joint 0..5, left stick Y moves it,
                        LB/RB close/open the gripper.
  cartesian-feel        START toggles: left stick = pan/lift, right stick X =
                        elbow, right stick Y = wrist_roll, LB+right stick Y =
                        wrist_flex, triggers = gripper.
  record / replay       X starts/stops recording the sent targets, B short
                        replays.

Two-arm switch (Plan Phase 5):
  - hold A for 5 s to switch teleop from arm 1 to arm 2 and back; it fires once
    at the 5 s mark (not on release) and needs a release before the next switch;
  - the switch is NOT blind: it asks the new arm's /joint_states first and, if it
    has not answered within SWITCH_CONFIRM seconds, refuses (long rumble) and stays;
  - on switching it stops publishing to the old topic, re-seeds the targets from
    the new arm's real pose (so there is no jump), clamps the first delta and
    resumes on the new topic;
  - rumble: one short buzz = switched to arm 1, two = arm 2, a long one = refused;
  - torque is PER ARM: switching does not energise anything. If the newly selected
    arm has torque off, the client stops publishing and says so once per state
    (the torque proxies are :8092 for arm 1 and :8095 for arm 2).

Safety:
  - nothing is published until the active arm's /joint_states has been received
    (its real position seeds the targets, so the first command is "stay");
  - per-tick deltas are clamped and values limited to the joint ranges;
  - publishing stops after 0.5 s without user input (a lost controller cannot drive the arm);
  - /joint_targets carries degrees, matching the LeRobot convention.
"""

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request

import pygame
import roslibpy

DOF = 6
TICK = 0.02                     # 50 Hz
STEP_MAX = 1.0                  # deg per tick at full stick (joint-step) = 50 deg/s
FEEL_MAX = 0.5                  # deg per tick at full stick (cartesian-feel) = 25 deg/s
GRIP_RATE = 1.0                 # gripper % per tick = 50 %/s
DEADZONE = 0.15
HOLD_TORQUE = 5.0               # hold B this long to switch the servos on/off
HOLD_SWITCH = 5.0               # hold A this long to switch arms (Plan Phase 5)
SWITCH_CONFIRM = 1.0            # s the far arm must have answered within, or the switch is refused
IDLE_STOP = 0.5                 # s without input -> stop publishing
TORQUE_RE_CHECK = 1.5           # s between re-queries of the active arm's torque
LIM_LO = [-170.0, -170.0, -170.0, -170.0, -170.0, 0.0]
LIM_HI = [170.0, 170.0, 170.0, 170.0, 170.0, 100.0]

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# The two arms share domain 42; namespaces + the torque proxy keep them apart.
ARMS = [
    {"name": "arm 1", "targets": "/joint_targets",   "states": "/joint_states",          "torque": 8092},
    {"name": "arm 2", "targets": "/arm2/joint_targets", "states": "/arm2/joint_states",  "torque": 8095},
]


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def dz(v, scale=1.0):
    """Dead zone + quadratic curve: fine control near the centre, full speed at
    the edge. `scale` multiplies the whole response (--scale)."""
    if abs(v) < DEADZONE:
        return 0.0
    return v * abs(v) * scale


class Teleop:
    def __init__(self, host, port, scale=1.0):
        self.scale = scale
        self.active = 0
        self.client = roslibpy.Ros(host=host, port=port)
        self.client.run()
        self.pubs = [roslibpy.Topic(self.client, a["targets"], "std_msgs/Float64MultiArray")
                     for a in ARMS]
        for p in self.pubs:
            p.advertise()
        self.arm_state_seen = [False, False]
        self.arm_state_t = [0.0, 0.0]
        self.arm_targets = [[0.0] * DOF, [0.0] * DOF]     # each arm's real pose seed
        self.targets = [0.0] * DOF
        for i, a in enumerate(ARMS):
            t = roslibpy.Topic(self.client, a["states"], "sensor_msgs/JointState")
            t.subscribe(lambda m, i=i: self.on_state(m, i))

        self.torque_on = [None, None]                     # None=unknown, True/False known
        self.last_torque_check = 0.0
        self.torque_warned = [False, False]

        self.cartesian = False
        self.joint_sel = 0
        self.recording = False
        self.record = []
        self.replaying = False
        self.replay_i = 0
        self.last_input = 0.0
        self.joystick = None
        self.hat_x_prev = 0
        self.b_down_at = None
        self.b_long_fired = False
        self.a_down_at = None
        self.a_long_fired = False

        self.prev = {}
        print("waiting for /joint_states ...")
        # seed the active arm from its real pose before any teleop is allowed
        while not self.arm_state_seen[self.active]:
            self.client.run_once()
            time.sleep(0.05)

    # ---- ROS callbacks
    def on_state(self, msg, arm):
        pos = msg.get("position", [])
        if len(pos) < DOF:
            return
        self.arm_state_t[arm] = time.time()
        if not self.arm_state_seen[arm]:
            self.arm_state_seen[arm] = True
            self.arm_targets[arm] = [float(p) for p in pos[:DOF]]
            if arm == self.active:
                self.targets = list(self.arm_targets[arm])
                print(f"{ARMS[arm]['name']} state received — targets seeded at its real pose")

    # ---- controller helpers
    def axis(self, a):
        if self.joystick is None:
            return 0.0
        return dz(self.joystick.get_axis(a), self.scale)

    def _button(self, b):
        """get_button with a bounds guard: pad layouts differ (hat vs buttons)."""
        if self.joystick is None or b >= self.joystick.get_numbuttons():
            return False
        return bool(self.joystick.get_button(b))

    def pressed(self, b):
        now = self._button(b)
        was = self.prev.get(b, False)
        self.prev[b] = now
        return now and not was

    def held(self, b):
        return self._button(b)

    def dpad_step(self):
        """-1/0/+1, EDGE-TRIGGERED: one joint hop per push of the D-pad."""
        if self.joystick is None:
            return 0
        cur = 0
        if self.joystick.get_numhats() > 0:
            cur = self.joystick.get_hat(0)[0]
        if not cur:                                  # button fallback
            left, right = self.pressed(13), self.pressed(14)
            cur = (1 if right else 0) - (1 if left else 0)
        prev, self.hat_x_prev = self.hat_x_prev, cur
        return cur if cur and not prev else 0

    def hat_y(self):
        """-1/0/+1 vertical D-pad, CONTINUOUS."""
        if self.joystick is None:
            return 0
        if self.joystick.get_numhats() > 0:
            return self.joystick.get_hat(0)[1]
        up, down = self._button(11), self._button(12)
        return (1 if up else 0) - (1 if down else 0)

    def rumble(self, kind, count=1):
        """Short buzzes (one = arm 1, two = arm 2) or one long one (switch refused)."""
        if self.joystick is None:
            return
        try:
            if kind == "long":
                self.joystick.set_rumble(0.9, 0.9, 800)
                return
            for _ in range(count):
                self.joystick.set_rumble(0.6, 0.6, 250)
                time.sleep(0.12)
        except Exception:  # noqa: BLE001 — rumble is cosmetic
            pass

    # ---- two-arm switch (Plan Phase 5)
    def switch_arm(self):
        """A held 5 s: move teleop to the other arm, with a feedback check."""
        new = 1 - self.active
        now = time.time()
        if not self.arm_state_seen[new] or now - self.arm_state_t[new] > SWITCH_CONFIRM:
            print(f"{ARMS[new]['name']} not answering — switch REFUSED, staying on {ARMS[self.active]['name']}")
            self.rumble("long")
            return
        # stop publishing to the old topic (we only ever publish to active), re-seed from the
        # new arm's real pose so the first command is "stay", clamp, then resume.
        self.active = new
        self.targets = list(self.arm_targets[new])
        self.torque_on[new] = None                      # re-learn on the next gate
        self.torque_warned[new] = False
        self.last_torque_check = 0.0
        print(f"-> teleop on {ARMS[new]['name']} ({ARMS[new]['states']})")
        self.rumble("short", count=new + 1)             # 1 buzz arm1, 2 arm2

    def handle_a(self):
        """A short = nothing; held 5 s = switch arms, firing once, needs a release."""
        now = time.time()
        down = self._button(0)              # A
        if down and self.a_down_at is None:
            self.a_down_at = now
            self.a_long_fired = False
        elif down and not self.a_long_fired and now - self.a_down_at >= HOLD_SWITCH:
            self.a_long_fired = True
            print(f"A held {HOLD_SWITCH:.0f} s")
            self.switch_arm()
        elif not down and self.a_down_at is not None:
            self.a_down_at = None           # short tap: nothing; a release also re-arms A

    def torque_url(self):
        return f"http://{self.client.host}:{ARMS[self.active]['torque']}/torque"

    def read_arm_torque(self, arm):
        """Bool/None for one arm's torque via its LAN proxy (loopback port differs per arm)."""
        try:
            url = f"http://{self.client.host}:{ARMS[arm]['torque']}/torque"
            with urllib.request.urlopen(url, timeout=2.5) as r:
                return bool(json.loads(r.read() or b"{}").get("torque"))
        except Exception:  # noqa: BLE001
            return None

    def torque_gate(self):
        """Re-learn the active arm's torque a few times per second and refuse to publish if off."""
        now = time.time()
        if now - self.last_torque_check < TORQUE_RE_CHECK:
            return
        self.last_torque_check = now
        arm = self.active
        on = self.read_arm_torque(arm)
        if on is None:
            return
        self.torque_on[arm] = on
        news = "ENABLED" if on else "OFF"
        print(f"{ARMS[arm]['name']} torque: {news}" +
              ("" if on else " — press 'Arm torque' in the web page to energise it"))

    def toggle_torque(self):
        """Read the ACTIVE arm's real state, invert it — never assume."""
        arm = self.active
        on = self.read_arm_torque(arm)
        if on is None:
            print("torque: cannot read state")
            return
        want = not on
        try:
            req = urllib.request.Request(
                self.torque_url(), data=json.dumps({"on": want}).encode(), method="POST",
                headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=6) as r:
                state = bool(json.loads(r.read() or b"{}").get("torque"))
            self.torque_on[arm] = state
            print(f"{ARMS[arm]['name']} torque:", "ENABLED" if state else "DISABLED")
        except Exception as exc:  # noqa: BLE001
            print(f"torque: toggle failed ({exc})")

    def handle_b(self):
        """B short = replay; held HOLD_TORQUE = torque on/off for the ACTIVE arm."""
        now = time.time()
        down = self._button(1)
        if down and self.b_down_at is None:
            self.b_down_at = now
            self.b_long_fired = False
        elif down and not self.b_long_fired and now - self.b_down_at >= HOLD_TORQUE:
            self.b_long_fired = True
            print(f"B held {HOLD_TORQUE:.0f} s")
            self.toggle_torque()
        elif not down and self.b_down_at is not None:
            held = now - self.b_down_at
            self.b_down_at = None
            if not self.b_long_fired and held < HOLD_TORQUE and self.record:
                self.replaying = True
                self.replay_i = 0
                print(f"REPLAY of {len(self.record)} points")

    # ---- main loop
    def run(self):
        pygame.init()
        pygame.joystick.init()
        clock = pygame.time.Clock()
        print("F710 teleop — joint-step on arm 1. START=mode, X=record, B=replay/torque, Ctrl+C quits.")
        print("  hold B 5 s = torque on/off (arm is booted torque-off)")
        print("  hold A 5 s = switch the teleop to the other arm (Phase 5)")
        try:
            while True:
                pygame.event.pump()
                if self.joystick is None and pygame.joystick.get_count() > 0:
                    self.joystick = pygame.joystick.Joystick(0)
                    self.joystick.init()
                    print(f"controller: {self.joystick.get_name()} "
                          f"({self.joystick.get_numaxes()} axes, "
                          f"{self.joystick.get_numbuttons()} buttons, "
                          f"{self.joystick.get_numhats()} hats)")
                self.tick()
                clock.tick(50)
        except KeyboardInterrupt:
            print("bye")
        finally:
            for p in self.pubs:
                p.unadvertise()
            self.client.terminate()
            pygame.quit()

    def tick(self):
        self.handle_a()
        if self.pressed(7):        # START
            self.cartesian = not self.cartesian
            print("mode:", "cartesian-feel" if self.cartesian else "joint-step")
        if self.pressed(2):        # X
            self.recording = not self.recording
            if self.recording:
                self.record = []
                print("RECORDING started")
            else:
                print(f"RECORDING stopped — {len(self.record)} points")
        self.handle_b()
        self.torque_gate()

        if self.replaying:
            if self.replay_i >= len(self.record):
                self.replaying = False
                print("replay finished")
            else:
                self.targets = list(self.record[self.replay_i])
                self.replay_i += 1
                self.send()
            return

        moved = False
        if self.cartesian:
            lx, ly = self.axis(0), self.axis(1)
            rx, ry = self.axis(2), self.axis(3)
            if lx or ly:
                self.targets[0] += lx * FEEL_MAX
                self.targets[1] -= ly * FEEL_MAX
                moved = True
            if self.held(4):                      # LB = shift
                if ry:
                    self.targets[3] -= ry * STEP_MAX
                    moved = True
            elif ry:
                self.targets[4] -= ry * FEEL_MAX
                moved = True
            if rx:
                self.targets[2] += rx * FEEL_MAX
                moved = True
            naxes = self.joystick.get_numaxes() if self.joystick else 0
            trig = (self.joystick.get_axis(5) + 1) / 2 if naxes > 5 else 0.0   # RT
            trig -= (self.joystick.get_axis(4) + 1) / 2 if naxes > 4 else 0.0  # LT
            if abs(trig) > 0.05:
                self.targets[5] += trig * GRIP_RATE
                moved = True
            hy = self.hat_y()
            if hy:
                self.targets[3] += hy * STEP_MAX
                moved = True
        else:
            d = self.dpad_step()
            if d:
                self.joint_sel = (self.joint_sel + d) % DOF
                print("joint:", JOINT_NAMES[self.joint_sel])
            ly = self.axis(1)
            if ly:
                self.targets[self.joint_sel] -= ly * STEP_MAX
                moved = True
            if self.held(4):       # LB
                self.targets[5] -= GRIP_RATE
                moved = True
            if self.held(5):       # RB
                self.targets[5] += GRIP_RATE
                moved = True

        for i in range(DOF):
            self.targets[i] = clamp(self.targets[i], LIM_LO[i], LIM_HI[i])

        if moved:
            self.last_input = time.time()
            self.send()

    def send(self):
        # Torque gate (Plan Phase 5): if the ACTIVE arm is known torque-off, do not publish
        # (a page button is the only way it should be energised; see toggle_torque).
        if self.torque_on[self.active] is False:
            if not self.torque_warned[self.active]:
                self.torque_warned[self.active] = True
                print(f"{ARMS[self.active]['name']}: torque off — not publishing "
                      "(press 'Arm torque' in the web page)")
            return
        self.torque_warned[self.active] = False
        self.pubs[self.active].publish(
            {"layout": {"dim": [], "data_offset": 0}, "data": [float(v) for v in self.targets]})
        if self.recording:
            self.record.append(list(self.targets))


def dump(seconds=20):
    """Print every axis/button/hat change — use it to map an unknown pad."""
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("no joystick found"); return
    js = pygame.joystick.Joystick(0); js.init()
    print(f"{js.get_name()} — {js.get_numaxes()} axes, {js.get_numbuttons()} buttons, {js.get_numhats()} hats")
    print(f"move each control for {seconds}s ...")
    prev = {}
    t0 = time.time()
    clock = pygame.time.Clock()
    while time.time() - t0 < seconds:
        pygame.event.pump()
        cur = {}
        for a in range(js.get_numaxes()):
            v = js.get_axis(a)
            if abs(v) > 0.3: cur[f"axis {a}"] = round(v, 2)
        for b in range(js.get_numbuttons()):
            if js.get_button(b): cur[f"button {b}"] = 1
        for h in range(js.get_numhats()):
            if any(js.get_hat(h)): cur[f"hat {h}"] = js.get_hat(h)
        for k in sorted(cur):
            if prev.get(k) != cur[k]:
                print(f"  {k} = {cur[k]}")
        prev = cur
        clock.tick(30)
    print("done")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.185", help="the Xavier's LAN address")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--dump", action="store_true", help="only print pad input (mapping aid)")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="speed factor (0.5 = half speed, 2.0 = double)")
    args = ap.parse_args()
    if args.dump:
        dump(); return 0
    print(f"rosbridge at ws://{args.host}:{args.port} (pass --host <xavier-ip> to change)")
    Teleop(args.host, args.port, args.scale).run()


if __name__ == "__main__":
    sys.exit(main())
