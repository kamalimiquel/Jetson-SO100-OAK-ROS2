#!/usr/bin/env python3
"""Episode recorder — per-arm state, the frames the policy would see, and the commands.

What it deliberately is not: a bag of the whole graph. It subscribes to arm-specific sources only
(`/armN/joint_states`, never the merged `/cell/joint_states_rad`, whose stamp is the last arrival's
and therefore hides a dead arm), keeps each message's own acquisition stamp, and writes the age
against it. The files are described in `docker/ws/lib/episode_log.py`.

Sampling: images and named arm states at RECORD_HZ (5 by default), because a 25 Hz joint topic plus
two 10 Hz cameras written raw is a disk full of duplicates, and a comparison episode does not need
every sample. What it does need is the *pairing*: each tick writes one observation line per source
with the message stamp it actually came from, so the age is visible per source rather than implied.

Env:
  RECORD_DIR         where episodes land (default /episodes). Unset -> the node refuses to run.
  RECORD_HZ          samples per second (default 5)
  RECORD_INSTRUCTION the operator's sentence, recorded verbatim
  RECORD_SECONDS     stop after this long (default 0 = until SIGINT)

Scene caveat, written into every manifest by this node: the scene camera's base extrinsic failed
its holdout checks, so the recorded RGB-D is evidence of *what was seen*, not a metric measurement
of where it is. Frames are recorded through the same topic the policy gets, not re-encoded.
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "lib"))
import episode_log  # noqa: E402
import command_contract as cc  # noqa: E402

STATES = ("/arm1/joint_states", "/arm2/joint_states")
WRIST = "/arm1/gripper/image/compressed"       # the only wrist camera focused at 10-15 cm
SCENE = "/oak/scene/color/compressed"


def stamp_seconds(header):
    """A message stamp as wall seconds, or None when the header carries none (a zero stamp)."""
    s = getattr(header, "stamp", None)
    if s is None or (s.sec == 0 and s.nanosec == 0):
        return None
    return s.sec + s.nanosec * 1e-9


class Recorder(Node):
    def __init__(self, root, instruction="", hz=5.0, seconds=0.0, log=None):
        super().__init__("episode_recorder")
        self.log = log or episode_log.EpisodeLog(root, instruction)
        self.log.note("scene color recorded without metric pose: the base extrinsic failed its "
                      "holdout checks, so this is what was seen, not where it is")
        self.log.note("jaw is the LeRobot percentage; the TF angle is a proxy, not a measurement")
        self.latest = {}          # topic -> (stamp_wall, payload)
        self.published = {}       # image topic -> JPEG bytes, one per tick
        self.log.command("system", {}, accepted=True,
                         reason="recorder started: %s" % self.log.manifest["episode"])
        for topic in STATES + (WRIST, SCENE):
            self.create_subscription(JointState if topic in STATES else CompressedImage, topic,
                                     lambda m, t=topic: self.on_msg(t, m), 10)
        for ns in ("arm1", "arm2"):
            self.create_subscription(Float64MultiArray, "/%s/joint_targets" % ns,
                                     lambda m, n=ns: self.on_command(n, m), 10)
        self.create_timer(1.0 / hz, self.tick)
        self.deadline = time.monotonic() + seconds if seconds else None
        self.get_logger().info("recording %s at %.1f Hz -> %s" % (", ".join(STATES + (WRIST, SCENE)),
                                                                 hz, self.log.dir))

    def on_msg(self, topic, msg):
        if isinstance(msg, JointState):
            # by NAME, never by index: the array order is not a contract anywhere in this cell
            values = {n: round(float(p), 2) for n, p in zip(msg.name, msg.position)}
            self.latest[topic] = (stamp_seconds(msg.header), values)
        else:
            self.latest[topic] = (stamp_seconds(msg.header), None)
            self.published[topic] = bytes(msg.data)

    def on_command(self, ns, msg):
        try:
            driven, held, _ = cc.validate_topic(list(msg.data))
            self.log.command(ns, driven, accepted=False,
                             reason="requested on the topic by some writer; the arbiter decides "
                                    "what reaches the bus" + (" (hold %s)" % sorted(held) if held else ""))
        except cc.CommandError as exc:
            self.log.command(ns, list(msg.data), accepted=False, reason=str(exc))

    def tick(self):
        for topic, (stamp, values) in list(self.latest.items()):
            frame = self.published.get(topic) if topic in (WRIST, SCENE) else None
            # An image is only written when one has arrived since the last tick (identity is not
            # tracked; the stamp says it, and a repeated frame is dropped rather than duplicated).
            if topic in (WRIST, SCENE):
                if frame is None:
                    continue
                self.log.observe(topic, stamp_wall=stamp, frame=frame)
                self.published.pop(topic, None)
            else:
                self.log.observe(topic, stamp_wall=stamp, payload=values)
        if self.deadline and time.monotonic() > self.deadline:
            self.finish()

    def finish(self):
        # Closing from inside a timer callback: raising SystemExit here is swallowed by the
        # executor ("Exception in callback") and the node keeps recording into a closed file, so
        # the shutdown is explicit and the close happens once.
        if getattr(self, "finished", False):
            return
        self.finished = True
        self.get_logger().info(self.log.summary())
        self.get_logger().info("episode written to %s" % self.log.close())
        rclpy.shutdown()


def main():
    root = os.environ.get("RECORD_DIR", "")
    if not root:
        print("[recorder] RECORD_DIR is not set — nothing to do (this node never records by "
              "accident)", flush=True)
        return 0
    rclpy.init()
    node = Recorder(root,
                    instruction=os.environ.get("RECORD_INSTRUCTION", ""),
                    hz=float(os.environ.get("RECORD_HZ", "5")),
                    seconds=float(os.environ.get("RECORD_SECONDS", "0")))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
