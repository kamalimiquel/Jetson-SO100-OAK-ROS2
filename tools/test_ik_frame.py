"""The IK base frame must belong to the arm that is solving, and a blank frame must be refused.

Both were wrong: `BASE_FRAME = "arm2_base" if NS else "base"` gave arm 1's solver arm 2's base (a
30 cm error in every answer), and `if msg.header.frame_id and ...` accepted an empty frame. Neither
shows up as an exception, so here are the cases as a test. No rclpy: the module is imported with the
ROS imports stubbed out, the same trick tools/test_compat_bridge.py uses.

    python3 tools/test_ik_frame.py
"""

import importlib.util
import sys
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
failures = []


def check(name, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + "%s: %r" % (name, got))
    if not ok:
        failures.append("%s: got %r, wanted %r" % (name, got, want))


def stubs():
    """ik_node imports rclpy and three message packages at module level; stub them out."""
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    class _Any(object):
        def __init__(self, *a, **k):
            pass

        def __setattr__(self, k, v):
            object.__setattr__(self, k, v)

        def __getattr__(self, k):
            return _Any()

    for pkg in ("rclpy", "rclpy.node", "geometry_msgs", "geometry_msgs.msg",
                "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg"):
        mod(pkg, Node=_Any, PoseStamped=_Any, JointState=_Any,
            Float64MultiArray=_Any, String=_Any, init=lambda *a, **k: None,
            spin=lambda *a, **k: None, ok=lambda: True)


def load(ns):
    stubs()
    import os
    os.environ["ARM_NS"] = ns
    spec = importlib.util.spec_from_file_location("ik_node_" + (ns or "bare"),
                                                  HERE.parent / "docker/ws/control/ik_node.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


print("each arm solves in its own base")
for ns, want in (("arm1", "arm1_base"), ("arm2", "arm2_base"), ("", "arm1_base")):
    m = load(ns)
    check("ARM_NS=%r -> BASE_FRAME" % ns, m.BASE_FRAME, want)

print("a blank or foreign frame is refused, the right one is accepted")
m = load("arm1")
check("arm1_base accepted", m.accepts_frame("arm1_base", "arm1_base"), True)
check("arm2_base refused", m.accepts_frame("arm2_base", "arm1_base"), False)
check("empty refused", m.accepts_frame("", "arm1_base"), False)
check("None refused", m.accepts_frame(None, "arm1_base"), False)
check("the deleted 'base' refused", m.accepts_frame("base", "arm1_base"), False)
m2 = load("arm2")
check("arm2 accepts arm2_base", m2.accepts_frame("arm2_base", "arm2_base"), True)
check("arm2 refuses arm1_base", m2.accepts_frame("arm1_base", "arm2_base"), False)

print()
if failures:
    print("%d FAILED" % len(failures))
    for f in failures:
        print("  - " + f)
    raise SystemExit(1)
print("all checks pass")
