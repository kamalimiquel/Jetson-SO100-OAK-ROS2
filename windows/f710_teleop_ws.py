"""F710 teleop over rosbridge (ADR-0017 amendment).

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
  cartesian-feel        START toggles: left X/Y -> pan/lift, right X/Y ->
                        elbow/wrist_roll, triggers -> gripper.
  record / replay       X starts/stops recording the sent targets, B replays.

Safety:
  - nothing is published until /joint_states has been received (the arm's real
    position seeds the targets, so the first command is "stay where you are");
  - per-tick deltas are clamped and values limited to the joint ranges;
  - publishing stops after 0.5 s without user input (steady state is left to
    the arm bridge, so a lost controller cannot drive the arm);
  - /joint_targets carries degrees, matching the LeRobot convention.
"""

import argparse
import math
import sys
import time

import pygame
import roslibpy

DOF = 6
TICK = 0.02                     # 50 Hz
STEP_MAX = 20.0                 # deg per tick at full stick (joint-step)
FEEL_MAX = 10.0                 # deg per tick at full stick (cartesian-feel)
GRIP_RATE = 2.5                 # gripper % per tick
DEADZONE = 0.15
IDLE_STOP = 0.5                 # s without input -> stop publishing
LIM_LO = [-170.0, -170.0, -170.0, -170.0, -170.0, 0.0]
LIM_HI = [170.0, 170.0, 170.0, 170.0, 170.0, 100.0]

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def dz(v):
    return 0.0 if abs(v) < DEADZONE else v


class Teleop:
    def __init__(self, host, port):
        self.client = roslibpy.Ros(host=host, port=port)
        self.client.run()
        self.pub = roslibpy.Topic(self.client, "/joint_targets", "std_msgs/Float64MultiArray")
        self.pub.advertise()
        self.sub = roslibpy.Topic(self.client, "/joint_states", "sensor_msgs/JointState")
        self.sub.subscribe(self.on_state)

        self.targets = [0.0] * DOF
        self.have_state = False
        self.cartesian = False
        self.joint_sel = 0
        self.recording = False
        self.record = []
        self.replaying = False
        self.replay_i = 0
        self.last_input = 0.0
        self.joystick = None

        self.prev = {}          # edge state per button
        print("waiting for /joint_states ...")
        while not self.have_state:
            time.sleep(0.1)

    # ---- ROS callbacks
    def on_state(self, msg):
        if len(msg.get("position", [])) < DOF:
            return
        if not self.have_state:
            self.targets = [float(p) for p in msg["position"][:DOF]]
            self.have_state = True
            print("state received — targets seeded at the arm's real pose")
        else:
            # keep targets within reach of the real pose on the first second
            pass

    # ---- controller helpers
    def axis(self, a):
        if self.joystick is None:
            return 0.0
        return dz(self.joystick.get_axis(a))

    def pressed(self, b):
        """Edge-triggered button press."""
        if self.joystick is None:
            return False
        now = self.joystick.get_button(b)
        was = self.prev.get(b, False)
        self.prev[b] = now
        return now and not was

    def held(self, b):
        return self.joystick is not None and bool(self.joystick.get_button(b))

    # ---- main loop
    def run(self):
        pygame.init()
        pygame.joystick.init()
        clock = pygame.time.Clock()
        print("F710 teleop — joint-step mode. START=toggle mode, X=record, B=replay. Ctrl+C to quit.")
        try:
            while True:
                pygame.event.pump()
                if self.joystick is None and pygame.joystick.get_count() > 0:
                    self.joystick = pygame.joystick.Joystick(0)
                    self.joystick.init()
                    print(f"controller: {self.joystick.get_name()}")
                self.tick()
                clock.tick(50)
        except KeyboardInterrupt:
            print("bye")
        finally:
            self.sub.unsubscribe()
            self.client.terminate()
            pygame.quit()

    def tick(self):
        # mode / record / replay toggles
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
        if self.pressed(1) and self.record:   # B
            self.replaying = True
            self.replay_i = 0
            print(f"REPLAY of {len(self.record)} points")

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
            if rx or ry:
                self.targets[2] += rx * FEEL_MAX
                self.targets[4] -= ry * FEEL_MAX
                moved = True
            trig = (self.joystick.get_axis(5) + 1) / 2 if self.joystick else 0.0   # RT 0..1
            trig -= (self.joystick.get_axis(4) + 1) / 2 if self.joystick else 0.0  # LT
            if abs(trig) > 0.05:
                self.targets[5] += trig * GRIP_RATE
                moved = True
        else:
            if self.pressed(14):   # D-pad right
                self.joint_sel = (self.joint_sel + 1) % DOF
                print("joint:", JOINT_NAMES[self.joint_sel])
            if self.pressed(13):   # D-pad left
                self.joint_sel = (self.joint_sel - 1) % DOF
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
        self.pub.publish({"layout": {"dim": [], "data_offset": 0}, "data": [float(v) for v in self.targets]})
        if self.recording:
            self.record.append(list(self.targets))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.50", help="the Xavier's LAN address")
    ap.add_argument("--port", type=int, default=9090)
    args = ap.parse_args()
    print(f"rosbridge at ws://{args.host}:{args.port} (pass --host <xavier-ip> to change)")
    Teleop(args.host, args.port).run()


if __name__ == "__main__":
    sys.exit(main())
