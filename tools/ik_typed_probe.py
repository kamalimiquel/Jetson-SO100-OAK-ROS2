#!/usr/bin/env python3
"""Send typed IK requests and read the answers back — the correlation is the point.

    python3 tools/ik_typed_probe.py arm1          # both paths: unreachable target and deleted frame
    python3 tools/ik_typed_probe.py arm1 frame    # only the frame refusal (the case that matters most)

Needs the built interface package: run it inside the control container, as
`docker exec so100-control bash -lc 'source /opt/iface_cache/install/setup.bash && python3 …'`.

Send them ONE AT A TIME. Two requests published back-to-back arrive as one result on this DDS setup,
which looks like a broken node and is not: the node logs both (`ik: req=…`) while the probe receives
one. The frame case alone is the clean check.

Two requests, neither of which moves anything (IK_APPLY=0 on the bench): one with a reachable id and an
unreachable target, so the answer must be ok=false *with the same id*; and one with a bad frame, which the
node has to refuse with a sentence instead of solving.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from so100_cell_interfaces.msg import SolveRequest, SolveResult

NS = sys.argv[1] if len(sys.argv) > 1 else "arm1"


class Probe(Node):
    def __init__(self):
        super().__init__("ik_probe")
        self.pub = self.create_publisher(SolveRequest, "/%s/ik/solve" % NS, 10)
        self.got = []
        self.create_subscription(SolveResult, "/%s/ik/result" % NS, lambda m: self.got.append(m), 10)

    def ask(self, req_id, x, frame):
        m = SolveRequest()
        m.request_id = req_id
        m.target = PoseStamped()
        m.target.header.frame_id = frame
        m.target.header.stamp = self.get_clock().now().to_msg()
        m.target.pose.position.x = x
        m.target.pose.position.y = 0.0
        m.target.pose.position.z = 0.15
        m.target.pose.orientation.w = 1.0
        m.max_pos_err_mm = 1.0
        m.max_ori_err_deg = 2.0
        self.pub.publish(m)


def main():
    rclpy.init()
    p = Probe()
    time.sleep(2.0)                      # let discovery settle
    # Re-send while discovery settles: a single publish 2 s after start is lost often enough to make a
    # passing node look broken (this probe did exactly that the first time).
    deadline = time.time() + 25
    while time.time() < deadline and len({m.request_id for m in p.got}) < 2:
        only = sys.argv[2] if len(sys.argv) > 2 else ""
        if only != "frame":
            p.ask("probe-far", 2.0, "%s_base" % NS)   # outside any reach
        if only != "far":
            p.ask("probe-frame", 0.15, "base")        # a frame this cell deleted (ADR-0023)
        for _ in range(5):
            rclpy.spin_once(p, timeout_sec=0.1)
    rclpy.spin_once(p, timeout_sec=1.0)
    for m in p.got:
        print("RESULT id=%r ok=%s pos_err=%.1f ori_err=%.1f iters=%d | %s"
              % (m.request_id, m.ok, m.pos_err_mm, m.ori_err_deg, m.iterations, m.detail))
    ids = [m.request_id for m in p.got]
    want = {"probe-frame"} if (len(sys.argv) > 2 and sys.argv[2] == "frame") else \
           ({"probe-far"} if (len(sys.argv) > 2 and sys.argv[2] == "far") else {"probe-far", "probe-frame"})
    ok = want <= set(ids) and all(not m.ok for m in p.got)
    print("CORRELATION %s" % ("OK" if ok else "FAILED (ids=%s)" % ids))
    p.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
