#!/usr/bin/env python3
"""Headless test of the episode recorder — no ROS, no cameras, no disk of consequence.

What is being pinned is the arithmetic an experiment depends on: every observation keeps the
message's own stamp, the age is computed against it, a stale sample is counted (not hidden), the
manifest totals match the files, and the recorder listens to arm-specific state — never to the
merged `/cell/joint_states_rad`, whose stamp is the last arrival's and would make a dead arm look
current.

    python3 tools/test_episode_recorder.py
"""

import importlib.util
import json
import os
import shutil
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.join(os.path.dirname(HERE), "docker", "ws")
LIB = os.path.join(WS, "lib")

fails = []


def check(name, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + "%s: %r" % (name, got) +
          ("" if ok else " (expected %r)" % (want,)))
    if not ok:
        fails.append(name)


class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Stamp:
    def __init__(self, sec, nanosec=0):
        self.sec, self.nanosec = sec, nanosec


class Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    def warn(self, m):
        self.lines.append(m)

    def error(self, m):
        self.lines.append(m)


class FakeNode:
    def __init__(self, name):
        self.node_name = name
        self.subs, self.pubs, self.timers = {}, {}, []

    def create_publisher(self, cls, name, qos):
        self.pubs[name] = Pub()
        return self.pubs[name]

    def create_subscription(self, cls, name, cb, qos):
        self.subs[name] = cb
        return object()

    def create_timer(self, period, cb):
        self.timers.append((period, cb))
        return object()

    def get_logger(self):
        return Log()

    def destroy_node(self):
        pass


def load_modules():
    rclpy = types.ModuleType("rclpy")
    for fn in ("init", "spin", "shutdown"):
        setattr(rclpy, fn, lambda *a, **k: None)
    node = types.ModuleType("rclpy.node")
    node.Node = FakeNode
    sensor = types.ModuleType("sensor_msgs")
    sensor_msg = types.ModuleType("sensor_msgs.msg")

    def js_init(self):
        self.header = Msg(stamp=None, frame_id="")
        self.name, self.position, self.velocity, self.effort = [], [], [], []

    def ci_init(self):
        self.header = Msg(stamp=None, frame_id="")
        self.data = b""

    sensor_msg.JointState = type("JointState", (Msg,), {"__init__": js_init})
    sensor_msg.CompressedImage = type("CompressedImage", (Msg,), {"__init__": ci_init})
    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.Float64MultiArray = type("Float64MultiArray", (Msg,), {})
    std_msg.String = type("String", (Msg,), {})
    for name, mod in {"rclpy": rclpy, "rclpy.node": node, "sensor_msgs": sensor,
                      "sensor_msgs.msg": sensor_msg, "std_msgs": std,
                      "std_msgs.msg": std_msg}.items():
        sys.modules[name] = mod
    sys.modules["rclpy"].node = node
    sys.modules["sensor_msgs"].msg = sensor_msg
    sys.modules["std_msgs"].msg = std_msg
    sys.path.insert(0, LIB)
    mods = {}
    for key, path in (("episode_log", os.path.join(LIB, "episode_log.py")),
                      ("recorder", os.path.join(WS, "control", "recorder.py"))):
        spec = importlib.util.spec_from_file_location(key, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[key] = mod
        spec.loader.exec_module(mod)
        mods[key] = mod
    return mods["episode_log"], mods["recorder"]


def main():
    el, rec = load_modules()
    root = tempfile.mkdtemp(prefix="episode-test-")
    try:
        clock = {"wall": 1_800_000_000.0, "mono": 50.0}
        log = el.EpisodeLog(root, "Put the yellow block in the red tray",
                            clock=lambda: clock["wall"], mono=lambda: clock["mono"])

        print("one observation, two clocks")
        clock["wall"] += 0.25
        clock["mono"] += 0.25
        row = log.observe("/arm1/joint_states", stamp_wall=1_800_000_000.0,
                          payload={"arm1_shoulder_pan": 12.0})
        check("the stamp is the message's own", row["stamp"], 1_800_000_000.0)
        check("the age is receipt minus stamp", row["age_s"], 0.25)
        check("the monotonic receipt is recorded too", row["received"], 50.25)

        clock["wall"] += 0.9
        clock["mono"] += 0.9
        log.observe("/arm2/joint_states", stamp_wall=clock["wall"] - 0.9, payload={})
        check("a sample older than the window is counted as stale",
              log.manifest["stale"], {"/arm2/joint_states": 1})
        check("and the worst age per topic is kept", log.manifest["max_age_s"]["/arm2/joint_states"],
              0.9)
        check("per-topic counts", log.manifest["observations"],
              {"/arm1/joint_states": 1, "/arm2/joint_states": 1})

        print("a frame is a file plus a reference")
        row = log.observe("/arm1/gripper/image/compressed", stamp_wall=clock["wall"], frame=b"jpeg")
        path = os.path.join(log.dir, row["file"])
        check("the jpeg is on disk", open(path, "rb").read(), b"jpeg")
        check("named in the observation line", row["file"].startswith("frames/"), True)

        print("a message with no stamp has no age, and says so")
        row = log.observe("/oak/scene/color/compressed", stamp_wall=None, frame=b"x")
        check("age stays None rather than invented", row["age_s"], None)
        check("and is not counted as stale", "/oak/scene/color/compressed" in log.manifest["stale"],
              False)

        print("commands")
        log.command("arm1", {"shoulder_pan": 1.0}, accepted=True)
        log.command("arm1", {"shoulder_pan": 1.0}, accepted=False, reason="outside -170..170")
        check("accepted commands are counted", log.manifest["commands"], 1)

        print("the file, read back")
        episode_dir = log.close()
        manifest = json.load(open(os.path.join(episode_dir, "manifest.json")))
        obs = [json.loads(line) for line in open(os.path.join(episode_dir, "observations.jsonl"))]
        cmds = [json.loads(line) for line in open(os.path.join(episode_dir, "commands.jsonl"))]
        check("one line per observation", len(obs), 4)
        check("one line per command, refused ones included", len(cmds), 2)
        check("the instruction is verbatim", manifest["instruction"],
              "Put the yellow block in the red tray")
        check("the jaw is declared uncalibrated in the manifest", manifest["jaw"]["calibrated"], False)
        check("duration is on the monotonic clock", manifest["duration_s"], 1.15)
        check("counts agree with the lines", manifest["observations"]["/arm1/joint_states"], 1)
        check("the summary mentions the worst age", "worst age 0.90s" in log.summary(), True)

        print("two episodes in one second do not share a directory")
        a = el.EpisodeLog(root, clock=lambda: 1_800_000_100.0, mono=lambda: 1.0)
        b = el.EpisodeLog(root, clock=lambda: 1_800_000_100.0, mono=lambda: 2.0)
        check("second one gets its own directory", a.dir == b.dir, False)
        a.close()
        b.close()

        print("the node: arm-specific sources only")
        node = rec.Recorder(root, "instruction", hz=5.0)
        check("subscribes to both arms' own state", sorted(t for t in node.subs if "joint_states" in t),
              ["/arm1/joint_states", "/arm2/joint_states"])
        check("never to the merged cell topic", "/cell/joint_states_rad" in node.subs, False)
        check("reads arm 1's wrist and the scene colour",
              ("/arm1/gripper/image/compressed" in node.subs,
               "/oak/scene/color/compressed" in node.subs), (True, True))
        check("watches both arms' targets, to record what was asked",
              ("/arm1/joint_targets" in node.subs, "/arm2/joint_targets" in node.subs), (True, True))

        st = rec.JointState()
        st.header = Msg(stamp=Stamp(1_800_000_200, 0), frame_id="")
        st.name = ["shoulder_pan", "gripper"]
        st.position = [1.0, 42.0]
        st.velocity, st.effort = [], []
        node.on_msg("/arm1/joint_states", st)
        img = rec.CompressedImage()
        img.header = Msg(stamp=Stamp(1_800_000_200, 0), frame_id="arm1_gripper_color_optical_frame")
        img.data = b"\xff\xd8jpeg"
        node.on_msg("/arm1/gripper/image/compressed", img)
        node.tick()
        check("the state is recorded by joint name",
              node.log.manifest["observations"].get("/arm1/joint_states"), 1)
        check("a frame is written once and not offered again on the next tick",
              (node.log.manifest["observations"].get("/arm1/gripper/image/compressed"), node.tick() or
               node.log.manifest["observations"].get("/arm1/gripper/image/compressed")), (1, 1))

        cmd = rec.Float64MultiArray()
        cmd.data = [1.0, 2.0, 3.0, 4.0, 5.0]
        node.on_command("arm1", cmd)
        node.log.close()
        rows = [json.loads(line) for line in
                open(os.path.join(node.log.dir, "commands.jsonl")).readlines()]
        check("a five-value command is recorded with its refusal, not obeyed",
              "NaN" in rows[-1]["reason"], True)
        check("and never counted as an accepted command",
              rows[-1]["accepted"], False)
    finally:
        shutil.rmtree(root, ignore_errors=True)

    print("\n%s" % ("all checks pass" if not fails else "%d check(s) FAILED" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
