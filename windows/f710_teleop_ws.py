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
  record / replay       pad X starts/stops recording the sent targets, pad Y
                        replays them.
  arm / torque          pad A (held 5 s) = change arm 1/2; pad B (held 5 s) =
                        torque ON/OFF for the active arm.
  keyboard              Q (or Esc) quits. The small window is what lets SDL
                        deliver that key at all.

Torque and the two-arm switch:
  - hold B for 5 s to energise / release the ACTIVE arm (a tap does nothing).
    Arming first re-seeds the targets from the arm's real pose, so the servos take
    hold exactly where the arm is — never at an old value from the last command;
  - rumble: two short buzzes = armed, one = released, a long one = the arm was not
    answering and nothing changed;
  - hold A for 5 s to switch teleop from arm 1 to arm 2 and back; it fires once
    at the 5 s mark (not on release) and needs a release before the next switch;
  - the switch is NOT blind: it asks the new arm's /joint_states first and, if it
    has not answered within SWITCH_CONFIRM seconds, refuses (long rumble) and stays;
  - on switching it stops publishing to the old topic, re-seeds the targets from
    the new arm's real pose (so there is no jump), clamps the first delta and
    resumes on the new topic;
  - rumble: one short buzz = switched to arm 1, two = arm 2, a long one = refused;
  - torque is PER ARM: switching does not energise anything, and it never moves the
    arm you leave. If the newly selected arm has torque off, the client stops
    publishing and says so once per state (the torque proxies are :8092 for arm 1
    and :8095 for arm 2).

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
HOLD_TORQUE = 5.0               # hold A this long to energise / release the active arm
HOLD_SWITCH = 5.0               # hold Y this long to switch arms (Plan Phase 5)
ARM_BUZZ_GAP = 1.0              # s between the two buzzes that mean "armed"
ARM_SETTLE = 1.0                # s to wait for a reading with torque on before commanding it
SWITCH_CONFIRM = 1.0            # s the far arm must have answered within, or the switch is refused
IDLE_STOP = 0.5                 # s without input -> stop publishing
SEED_TIMEOUT = 20.0             # s to wait for the active arm's first /joint_states
TORQUE_RE_CHECK = 1.5           # s between re-queries of the active arm's torque
LIM_LO = [-170.0, -170.0, -170.0, -170.0, -170.0, 0.0]
LIM_HI = [170.0, 170.0, 170.0, 170.0, 170.0, 100.0]

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# The two arms share domain 42; namespaces + the torque proxy keep them apart.
ARMS = [
    {"name": "arm 1", "targets": "/joint_targets",   "states": "/joint_states",          "torque": 8092},
    {"name": "arm 2", "targets": "/arm2/joint_targets", "states": "/arm2/joint_states",  "torque": 8095},
]


def fmt(vec, nd=1):
    """Joint vector for the log: one line, fixed width, no numpy formatting surprises."""
    return "[" + ", ".join(f"{v:.{nd}f}" for v in vec) + "]"


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
        self.host = host            # kept here: roslibpy 2.x drops .host from the Ros object
        self.client = roslibpy.Ros(host=host, port=port)
        try:
            self.client.run()          # non-blocking: starts the event loop in its own thread
        except Exception as exc:  # noqa: BLE001 — roslibpy raises RosTimeoutError
            raise SystemExit(
                f"cannot reach rosbridge at ws://{host}:{port} ({exc}) — is the cell up, "
                f"and is {host} the Xavier's current address? (`--host` changes it)")
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

        self.state_seq = [0, 0]                           # /joint_states messages seen per arm
        self.buzz_at = None                               # pending second buzz (arming)
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
        self.y_down_at = None
        self.y_long_fired = False

        self.prev = {}
        self.said = {}                       # last message per key: identical repeats stay silent

        print(f"waiting for {ARMS[self.active]['states']} ...")
        # Seed the active arm from its real pose before any teleop is allowed. roslibpy's
        # `run()` is non-blocking (it starts its own event-loop thread), so the callbacks land
        # while this waits — there is no `run_once()` to pump. Waiting forever is worse than
        # failing: it looks like a hang when the real problem is a container that never came up.
        t0 = time.time()
        while not self.arm_state_seen[self.active]:
            if time.time() - t0 > SEED_TIMEOUT:
                raise SystemExit(
                    f"no {ARMS[self.active]['states']} after {SEED_TIMEOUT:.0f} s — "
                    f"is {ARMS[self.active]['name']}'s container up? (curl :80xx/status)")
            time.sleep(0.05)

        # Every start leaves both arms loose: whatever the servos were holding, the operator
        # decides when they go stiff. An explicit write, not an assumption about how it booted.
        self.force_torque_off()

    def force_torque_off(self):
        """Write torque off to both arms at start-up and report what actually came back."""
        for arm in (0, 1):
            state = self.write_arm_torque(arm, False)
            if state is None:
                self.torque_on[arm] = False
                self.say(f"{ARMS[arm]['name']} torque: OFF requested, proxy "
                         f":{ARMS[arm]['torque']} unreachable", key=f"start{arm}")
            else:
                self.torque_on[arm] = state
                self.say(f"{ARMS[arm]['name']} torque: "
                         f"{'OFF' if not state else 'COULD NOT TURN OFF'} at start", key=f"start{arm}")

    # ---- ROS callbacks
    def on_state(self, msg, arm):
        pos = msg.get("position", [])
        if len(pos) < DOF:
            return
        self.arm_state_t[arm] = time.time()
        self.state_seq[arm] += 1
        # EVERY message updates the real pose. This used to happen only on the first one, which
        # left `arm_targets` frozen at the pose the arm had when the client started — so arming
        # commanded that startup pose and the arm went back to it, exactly like an old command.
        self.arm_targets[arm] = [float(p) for p in pos[:DOF]]
        if not self.arm_state_seen[arm]:
            self.arm_state_seen[arm] = True
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

    def say(self, text, key=None):
        """Print only when the message actually changes.

        The status lines are polled several times a second (torque gate) and the buzzes come in
        pairs, so a plain print() floods the console with the same sentence. One line per new
        message; the key says which message is being compared.
        """
        key = key or text
        if self.said.get(key) == text:
            return
        self.said[key] = text
        print(text)

    def rumble(self, kind, count=1, quiet=False):
        """Short buzzes (one = arm 1 / released, two = arm 2 / armed) or one long one (refused).

        The log line matters as much as the buzz: SDL silently ignores rumble on pads and
        drivers that do not implement it, and then "it does not vibrate" has no other clue.
        """
        if self.joystick is None:
            if not quiet:
                self.say("buzz: no controller — nothing to rumble")
            return
        # pygame 2 exposes both names depending on version; take whichever exists.
        buzz = getattr(self.joystick, "rumble", None) or getattr(self.joystick, "set_rumble", None)
        if buzz is None:
            if not quiet:
                self.say("buzz: this pygame has no rumble API")
            return
        try:
            if kind == "long":
                ok = buzz(0.9, 0.9, 900)
            else:
                ok = buzz(0.7, 0.7, 320)
                for _ in range(count - 1):
                    time.sleep(0.26)                # gap, so two buzzes read as two
                    ok = buzz(0.7, 0.7, 320)
            if not quiet:
                self.say(f"buzz {kind} x{count}: " + ("ok" if ok is not False else
                         "NOT SUPPORTED by this pad or its driver"))
        except Exception as exc:  # noqa: BLE001 — rumble is cosmetic, never fatal
            if not quiet:
                self.say(f"buzz failed: {exc}")

    def service_buzz(self):
        """Fire the pending second "armed" buzz from the main loop — a sleep(3) here would
        freeze the teleop for three seconds, which on a live arm is not acceptable."""
        if self.buzz_at is not None and time.time() >= self.buzz_at:
            self.buzz_at = None
            self.rumble("short", count=1, quiet=True)   # same message as the first one

    # ---- two-arm switch (Plan Phase 5)
    def switch_arm(self):
        """A held 5 s: move teleop to the other arm, with a feedback check."""
        new = 1 - self.active
        if not self.fresh_pose(new):
            print(f"{ARMS[new]['name']} not answering — switch REFUSED, staying on {ARMS[self.active]['name']}")
            return
        # stop publishing to the old topic (we only ever publish to active), re-seed from the
        # new arm's real pose so the first command is "stay", clamp, then resume.
        self.active = new
        self.targets = list(self.arm_targets[new])
        self.torque_on[new] = None                      # re-learn on the next gate
        self.torque_warned[new] = False
        self.last_torque_check = 0.0
        print(f"-> teleop on {ARMS[new]['name']} ({ARMS[new]['states']}) — the switch is silent by design")

    # ---- torque (B) and arm switch (A)
    def fresh_pose(self, arm):
        """True when that arm published /joint_states recently enough to be trusted."""
        return self.arm_state_seen[arm] and (time.time() - self.arm_state_t[arm]) <= SWITCH_CONFIRM

    def torque_url(self, arm=None):
        return f"http://{self.host}:{ARMS[self.active if arm is None else arm]['torque']}/torque"

    @staticmethod
    def _opener():
        """Bypass any system proxy: one on Windows turns these LAN calls into timeouts."""
        return urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def read_arm_torque(self, arm):
        """Bool/None for one arm's torque via its LAN proxy (the port differs per arm)."""
        try:
            with self._opener().open(self.torque_url(arm), timeout=5) as r:
                return bool(json.loads(r.read() or b"{}").get("torque"))
        except Exception:  # noqa: BLE001
            return None

    def write_arm_torque(self, arm, want):
        """Set an arm's torque; returns what the server reports, or None on failure."""
        try:
            req = urllib.request.Request(
                self.torque_url(arm), data=json.dumps({"on": want}).encode(), method="POST",
                headers={"Content-Type": "application/json"})
            with self._opener().open(req, timeout=8) as r:
                return bool(json.loads(r.read() or b"{}").get("torque"))
        except Exception as exc:  # noqa: BLE001
            print(f"torque: POST failed ({exc})")
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
        self.say(f"{ARMS[arm]['name']} torque: {'ENABLED' if on else 'OFF'}" +
                 ("" if on else " — hold B 5 s to arm it"), key=f"gate{arm}")

    def handle_a(self):
        """A short = nothing; held 5 s = switch arms, firing once, needs a release."""
        now = time.time()
        down = self._button(0)              # A
        if down and self.a_down_at is None:
            self.a_down_at = now
            self.a_long_fired = False
        elif down and not self.a_long_fired and now - self.a_down_at >= HOLD_SWITCH:
            self.a_long_fired = True
            print(f"A held {HOLD_SWITCH:.0f} s = change arm")
            self.switch_arm()
        elif not down and self.a_down_at is not None:
            self.a_down_at = None           # a tap does nothing; releasing re-arms A

    def toggle_torque(self):
        """Arm or release the active arm WITHOUT moving it, and say which with the rumble."""
        arm = self.active
        on = self.read_arm_torque(arm)
        if on is None:
            self.say(f"{ARMS[arm]['name']}: torque state unreadable (proxy :{ARMS[arm]['torque']}) "
                     "— nothing changed", key=f"torque{arm}")
            self.rumble("long")
            return

        if on:                                          # ---- release
            fresh = self.fresh_pose(arm)
            state = self.write_arm_torque(arm, False)
            if state is None:
                self.rumble("long")
                return
            self.torque_on[arm] = False
            self.torque_warned[arm] = False
            if fresh:
                self.targets = list(self.arm_targets[arm])
            print(f"{ARMS[arm]['name']} torque: RELEASED")
            self.rumble("short", count=1)               # one buzz = loose
            return

        # ---- arm. Read the joints BEFORE energising, energise, then command exactly what the
        # arm reads with torque on. That is the only way the servos cannot snap to an old value:
        # the first target of a live arm is the pose it is standing in.
        if not self.fresh_pose(arm):
            self.say(f"{ARMS[arm]['name']}: no fresh pose — refusing to arm (it would jump)",
                     key=f"torque{arm}")
            self.rumble("long")
            return
        seq = self.state_seq[arm]
        if self.replaying:
            # A replay in progress publishes the recorded pose on the next tick and would drag the
            # arm there the moment the servos go live — an "old position" by definition. Arming
            # always wins over a replay.
            self.replaying = False
            self.replay_i = 0
            print(f"{ARMS[arm]['name']}: replay cancelled on arming")
        state = self.write_arm_torque(arm, True)
        if state is None:
            self.rumble("long")
            return
        self.torque_on[arm] = True
        self.torque_warned[arm] = False
        self.last_torque_check = time.time()

        # Wait for the first reading taken WITH torque on, then command that one.
        deadline = time.time() + ARM_SETTLE
        while self.state_seq[arm] == seq and time.time() < deadline:
            time.sleep(0.01)
        pose_after = list(self.arm_targets[arm])
        self.targets = pose_after
        # Only command it if the reading is real: the server already holds the arm at the pose it
        # read, so a stale vector here would be the only way left to make it jump.
        if self.fresh_pose(arm):
            self.send()
        else:
            self.say(f"{ARMS[arm]['name']}: armed, but no fresh pose to confirm — not publishing",
                     key=f"torque{arm}")
        print(f"{ARMS[arm]['name']} torque: ARMED at the pose it was standing in")
        self.rumble("short", count=1)                   # buzz 1 of 2 = live
        self.buzz_at = time.time() + ARM_BUZZ_GAP       # buzz 2, one second later

    def start_replay(self):
        """Pad Y: replay what was recorded, at the tick rate it was captured with."""
        if not self.record:
            print("replay: nothing recorded yet (pad X starts a recording)")
            return
        if self.torque_on[self.active] is False:
            print(f"replay: {ARMS[self.active]['name']} has no torque — arming it first")
            return
        self.replaying = True
        self.replay_i = 0
        print(f"REPLAY of {len(self.record)} points")

    def handle_b(self):
        """B held 5 s = torque on/off for the ACTIVE arm (a tap does nothing; Y replays)."""
        now = time.time()
        down = self._button(1)
        if down and self.b_down_at is None:
            self.b_down_at = now
            self.b_long_fired = False
        elif down and not self.b_long_fired and now - self.b_down_at >= HOLD_TORQUE:
            self.b_long_fired = True
            print(f"B held {HOLD_TORQUE:.0f} s = torque")
            self.toggle_torque()
        elif not down and self.b_down_at is not None:
            self.b_down_at = None

    # ---- main loop
    def run(self):
        pygame.init()
        pygame.joystick.init()
        clock = pygame.time.Clock()
        # A small window is the only way to read the keyboard: without a display surface SDL
        # delivers no key events at all, and Q is how you leave cleanly.
        screen = None
        try:
            screen = pygame.display.set_mode((620, 150))
            pygame.display.set_caption("F710 teleop — Q = quit")
        except Exception as exc:  # noqa: BLE001
            print(f"no window ({exc}) — keyboard shortcuts disabled, the pad still works")
        print("F710 teleop — joint-step on arm 1. Both arms start torque-off.")
        print("  keyboard Q / Ctrl+C = quit")
        print("  pad A (5 s) = change arm 1/2, pad B (5 s) = torque ON/OFF, "
              "pad X = record start/stop, pad Y = replay, START = mode")
        try:
            while True:
                for ev in pygame.event.get():
                    if ev.type == pygame.QUIT:
                        raise KeyboardInterrupt
                    if ev.type == pygame.KEYDOWN:
                        if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                            raise KeyboardInterrupt
                if screen is not None and not self.draw_status(screen):
                    raise KeyboardInterrupt
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
            try:
                self.client.terminate()
            except Exception as exc:                           # noqa: BLE001
                # roslibpy 2.x raises AttributeError here when the loop never started; a traceback on
                # exit reads like a crash and hides the real one.
                print(f"(rosbridge connection closed: {type(exc).__name__})")
            pygame.quit()

    def draw_status(self, screen):
        """Cheap status panel in the window: what the pad is driving and the last pose seen."""
        try:
            screen.fill((18, 18, 20))
            font = pygame.font.SysFont("consolas,couriernew", 15)
            small = pygame.font.SysFont("consolas,couriernew", 13)
            arm = self.active
            state = {True: "TORQUE ON", False: "TORQUE OFF", None: "torque ?"}[self.torque_on[arm]]
            lines = [
                (f"{ARMS[arm]['name']} · {state} · {'RECORDING' if self.recording else 'idle'}"
                 + (f" · replay {self.replay_i}" if self.replaying else ""), font, (235, 235, 235)),
                ("joints: " + fmt(self.arm_targets[arm]), small, (150, 220, 150)),
                ("X = print joints   Q = quit", small, (150, 150, 160)),
            ]
            y = 10
            for text, f, colour in lines:
                screen.blit(f.render(text, True, colour), (12, y))
                y += 22 if f is font else 20
            pygame.display.flip()
            return True
        except Exception:  # noqa: BLE001 — a broken window must not stop the teleop
            return False

    def tick(self):
        self.service_buzz()
        self.handle_a()
        if self.pressed(7):        # START
            self.cartesian = not self.cartesian
            print("mode:", "cartesian-feel" if self.cartesian else "joint-step")
        if self.pressed(2):        # pad X
            self.recording = not self.recording
            if self.recording:
                self.record = []
                print("RECORDING started")
            else:
                print(f"RECORDING stopped — {len(self.record)} points")
        if self.pressed(3):        # pad Y
            self.start_replay()
        self.handle_b()
        self.torque_gate()

        # Released arm: it cannot move on its own, so keep the targets glued to the real pose.
        # Arming later then commands "stay" instead of whatever value was left in the buffer.
        if self.torque_on[self.active] is False and self.fresh_pose(self.active):
            self.targets = list(self.arm_targets[self.active])

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
                      "(hold B 5 s, or 'Arm torque' in the web page)")
            return
        self.torque_warned[self.active] = False
        self.pubs[self.active].publish(
            {"layout": {"dim": [], "data_offset": 0}, "data": [float(v) for v in self.targets]})
        if self.recording:
            self.record.append(list(self.targets))


def rumble_test():
    """Buzz the pad twice and say whether SDL accepted it — run this before blaming the code."""
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("no joystick found")
        return 1
    js = pygame.joystick.Joystick(0)
    js.init()
    buzz = getattr(js, "rumble", None) or getattr(js, "set_rumble", None)
    print(f"{js.get_name()}  guid={js.get_guid()}  pygame {pygame.version.ver}")
    print("rumble API:", "yes" if buzz else "NO — this build cannot buzz the pad")
    if buzz is None:
        return 1
    for i in range(2):
        ok = buzz(0.8, 0.8, 400)
        print(f"  buzz {i + 1}: {ok}  (False means the driver refused it)")
        time.sleep(0.45)
    pygame.quit()
    print("if you felt nothing, the pad or its driver has no rumble (F710 over XInput usually does)")
    return 0


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
    ap.add_argument("--rumble-test", action="store_true",
                    help="buzz the pad twice and exit (checks whether SDL rumble works)")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="speed factor (0.5 = half speed, 2.0 = double)")
    args = ap.parse_args()
    if args.rumble_test:
        return rumble_test()
    if args.dump:
        dump(); return 0
    print(f"rosbridge at ws://{args.host}:{args.port} (pass --host <xavier-ip> to change)")
    Teleop(args.host, args.port, args.scale).run()


if __name__ == "__main__":
    sys.exit(main())
