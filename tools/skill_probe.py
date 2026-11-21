#!/usr/bin/env python3
"""Ask the supervisor for two skills and print what it answered. Nothing moves with torque off.

    python3 tools/skill_probe.py            # inside the control container, interfaces sourced
"""
import sys
import time

import rclpy
from rclpy.node import Node
from so100_cell_interfaces.srv import SkillRequest

CASES = [
    ("move_joint", "probe-1", '{"arm": "so100_cell", "angles": [0,0,0,0,0,50]}'),
    ("move_joint", "probe-2", '{"arm": "nope", "angles": [0,0,0,0,0,50]}'),
    ("open_jaw", "probe-3", '{"arm": "so100_cell", "mm": 40}'),
]


class Probe(Node):
    def __init__(self):
        super().__init__("skill_probe")
        self.cli = self.create_client(SkillRequest, "/cell/skill")
        if not self.cli.wait_for_service(timeout_sec=10.0):
            print("no /cell/skill: is the supervisor running?")
            raise SystemExit(2)


def main():
    rclpy.init()
    p = Probe()
    bad = 0
    for skill, mission, args in CASES:
        req = SkillRequest.Request()
        req.skill, req.mission_id, req.args_json = skill, mission, args
        fut = p.cli.call_async(req)
        rclpy.spin_until_future_complete(p, fut, timeout_sec=20.0)
        r = fut.result()
        if r is None:
            print("%-11s %-8s NO ANSWER" % (skill, mission))
            bad += 1
            continue
        print("%-11s %-8s accepted=%-5s ticket=%-14s %s"
              % (skill, mission, r.accepted, r.ticket or "-", r.reason[:90]))
    p.destroy_node()
    rclpy.shutdown()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
