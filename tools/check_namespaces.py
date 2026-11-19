#!/usr/bin/env python3
"""Refuse to ship a bare topic or frame — the invariant of ADR-0023, checked instead of remembered.

R2 is the failure that has no symptom: one unprefixed frame and RViz silently draws arm 1's shoulder
attached to arm 2's base. So the rule is mechanical and this script is it:

  frames   every <link> and <joint> of so101_cell.urdf is arm1_* or arm2_*, and no static transform
           published by the containers uses a bare frame id
  topics   every ROS topic literal in the cell's code is under /arm1, /arm2, /oak, /cell or /ik —
           the bare 1.0 names may appear ONLY in compat_bridge.py (that is its whole job)
  cameras  both wrist cameras are /armN/gripper/* and their frame is armN_gripper_color_optical_frame
  clients  the pad and the MuJoCo twin read /arm1/*, not /joint_states

    python3 tools/check_namespaces.py          # exit 0 = the invariant holds
"""

import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CELL_URDF = os.path.join(ROOT, "docker/ws/arm/urdf/so101_cell.urdf")

PREFIXES = ("/arm1", "/arm2", "/oak", "/cell", "/ik")
# The released 1.0 surface, allowed in exactly one file.
LEGACY = {"/joint_states", "/joint_states_rad", "/joint_targets", "/robot_description"}
LEGACY_OWNER = "docker/ws/arm/compat_bridge.py"
# Files whose string literals are mostly HTTP routes (/read, /write, /stereo, /snapshot...), not ROS
# topics: their ROS names are asserted explicitly further down instead of guessed from literals.
HTTP_SERVERS = {
    "docker/ws/arm/lerobot_joint_server.py", "docker/ws/arm/torque_api.py",
    "docker/ws/cams/mjpeg_view.py", "docker/ws/cams/cloud_view.py",
    "docker/web/wrist_focus.py", "docker/web/shutdown_server.py",
    "docker/web/wrist_pair.py", "docker/web/xavier_temp.py",
    "docker/ws/arm/arm_bridge.py", "tools/camfocus.py",
    # The arbiter's literals are HTTP routes on the loopback ports (/write, /claim, /owner); its
    # ROS topics are asserted explicitly in check_command_surface() below, the same way the
    # bridge's are.
    "docker/ws/control/arbiter.py",
}
# The invariant's own fixtures, and the figure that draws the chain for the docs.
SELF = {"tools/check_namespaces.py", "tools/test_compat_bridge.py",
        # fake topics and fake HTTP answers on purpose: no cell publisher lives here
        "tools/test_arbiter.py", "tools/test_joint_merge.py",
        "tools/test_command_contract.py", "tools/test_episode_recorder.py",
        "tools/test_joint_server.py",
        # the fake rosbridge matches topic SUFFIXES ("/joint_states") for any namespace by design,
        # so it must contain those strings while never being a cell publisher itself
        "tools/fake_rosbridge.py"}
# Not topics: device nodes, mounts, URLs, namespaces of the host filesystem.
NOT_A_TOPIC = re.compile(r"^/(dev|tmp|opt|srv|home|usr|etc|var|proc|sys|run|root)\b|^//|^/cell$")
SCAN_EXT = (".py", ".sh", ".html", ".rviz", ".yaml")
TOPIC_LITERAL = re.compile(r'["\'](/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*)["\']')
FRAME_LITERAL = re.compile(r'--frame-id\s+"?\$\{?FP\}?[^"\s]*"?|--child-frame-id\s+"?[^"\s]+"?')

fails = []


def bad(msg):
    fails.append(msg)
    print("  FAIL " + msg)


def ok(msg):
    print("  ok   " + msg)


def check_urdf():
    print("frames — one URDF, every name prefixed")
    root = ET.parse(CELL_URDF).getroot()
    names = [n.get("name") for tag in ("link", "joint") for n in root.findall(tag)]
    bare = [n for n in names if not n.startswith(("arm1_", "arm2_"))]
    if bare:
        bad(f"{CELL_URDF}: {len(bare)} unprefixed: {bare[:5]}")
    else:
        ok(f"{len(names)} links+joints, all arm1_*/arm2_*")
    mount = [j for j in root.findall("joint") if j.get("name") == "arm1_mount"]
    if not mount:
        bad("no arm1_mount joint: the 30 cm would have no writer")
    else:
        parent = mount[0].find("parent").get("link")
        child = mount[0].find("child").get("link")
        xyz = mount[0].find("origin").get("xyz")
        if (parent, child) == ("arm1_base", "arm2_base"):
            ok(f"the mount joint is {parent} -> {child} at {xyz} m")
        else:
            bad(f"mount joint is {parent} -> {child}, expected arm1_base -> arm2_base")


def check_static_frames():
    print("frames — nothing publishes a bare frame id")
    entry = os.path.join(ROOT, "docker/ws/arm/entry.sh")
    text = open(entry, encoding="utf-8").read()
    # The ONLY static transform left in the arm container is the wrist one, and it is prefixed.
    frames = re.findall(r'--(?:child-)?frame-id\s+"?([^"\s\\]+)', text)
    bare = [f for f in frames if f.startswith("base") or f.startswith("moving_jaw")]
    if bare:
        bad(f"entry.sh publishes bare frame(s): {bare}")
    else:
        ok(f"entry.sh static frames: {frames or 'none'}")


def check_frame_ids():
    print("frame ids — no message is stamped in a frame that no longer exists")
    stale = re.compile(r'frame_id\s*=\s*["\'](base|moving_jaw_so101_v1|gripper)["\']')
    hits = []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames
                       if d not in (".git", "node_modules", "dist", "__pycache__", "docs")]
        for fn in filenames:
            if not fn.endswith((".py", ".sh", ".js")):
                continue
            path = os.path.join(dirpath, fn)
            rel = Path(os.path.relpath(path, ROOT)).as_posix()
            text = open(path, encoding="utf-8", errors="ignore").read()
            for line_no, line in enumerate(text.splitlines(), 1):
                if stale.search(line):
                    hits.append(f"{rel}:{line_no}: {line.strip()[:70]}")
    if hits:
        for h in hits:
            bad(f"frame id from before ADR-0023 — {h}")
    else:
        ok("no message names a deleted frame (arm1_base is the root)")


def check_topics():
    print("topics — everything namespaced, the 1.0 names only in the bridge")
    seen, legacy_outside = 0, []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames
                       if d not in (".git", "node_modules", "dist", "__pycache__", "urdf", "docs")]
        for fn in filenames:
            if not fn.endswith(SCAN_EXT):
                continue
            path = os.path.join(dirpath, fn)
            # as_posix() because os.path.relpath returns backslashes on the operator's Windows PC,
            # where these slash-delimited exclusions then match nothing and the checker reported
            # 48 violations of its own fixtures.
            rel = Path(os.path.relpath(path, ROOT)).as_posix()
            if rel in HTTP_SERVERS or rel in SELF:
                continue
            try:
                text = open(path, encoding="utf-8", errors="ignore").read()
            except OSError:
                continue
            for lit in set(TOPIC_LITERAL.findall(text)):
                if NOT_A_TOPIC.match(lit) or "." in lit:
                    continue
                seen += 1
                if lit.startswith(PREFIXES):
                    continue
                if lit in LEGACY:
                    if rel != LEGACY_OWNER:
                        legacy_outside.append(f"{rel}: {lit}")
                    continue
                bad(f"{rel}: topic {lit} is not under {' '.join(PREFIXES)}")
    if legacy_outside:
        for line in legacy_outside:
            bad(f"1.0 name outside the bridge — {line}")
    else:
        ok(f"{seen} topic literals scanned; legacy names only in {LEGACY_OWNER}")


def check_cameras_and_clients():
    print("cameras and clients")
    cam = open(os.path.join(ROOT, "docker/ws/cams/wrist_cam_node.py"), encoding="utf-8").read()
    for needle, why in (
        ('WRIST_TOPIC_NS", "/arm1/gripper"', "arm 1's camera defaults to /arm1/gripper"),
        ('WRIST_FRAME", "arm1_gripper_color_optical_frame"', "and to a prefixed frame"),
    ):
        (ok if needle in cam else lambda m: bad(m))(why)
    mjpeg = open(os.path.join(ROOT, "docker/ws/cams/mjpeg_view.py"), encoding="utf-8").read()
    (ok if '"/arm1/gripper/image/compressed"' in mjpeg else lambda m: bad(m))(
        "the MJPEG viewer's gripper pane reads /arm1/gripper")
    focus = open(os.path.join(ROOT, "docker/web/wrist_focus.py"), encoding="utf-8").read()
    (ok if '1: "/arm1/gripper/image/compressed"' in focus else lambda m: bad(m))(
        "the focus test scores arm 1 from /arm1/gripper")
    pair = open(os.path.join(ROOT, "docker/web/wrist_pair.py"), encoding="utf-8").read()
    for needle, why in (
        ('"/arm1/gripper/image/compressed"', "the pair reads arm 1 from /arm1/gripper"),
        ('"/cell/gripper/cameras/compressed"', "and publishes the pair under /cell"),
    ):
        (ok if needle in pair else lambda m: bad(m))(why)
    pad = open(os.path.join(ROOT, "windows/f710_teleop_ws.py"), encoding="utf-8").read()
    twin = open(os.path.join(ROOT, "windows/mujoco_twin.py"), encoding="utf-8").read()
    (ok if '"/arm1/joint_targets"' in pad and '"/arm1/joint_states"' in pad
     else lambda m: bad(m))("the pad speaks /arm1/*")
    (ok if '"/arm1/joint_states"' in twin else lambda m: bad(m))("the twin reads /arm1/joint_states")
    rviz = open(os.path.join(ROOT, "docker/web/rviz.rviz"), encoding="utf-8").read()
    (ok if "Fixed Frame: arm1_base" in rviz else lambda m: bad(m))("RViz fixes on arm1_base")


def check_command_surface():
    """The command path's wiring and its two silent-failure guards, asserted by name."""
    print("command surface — one writer, six values, honest stamps")
    arb = open(os.path.join(ROOT, "docker/ws/control/arbiter.py"), encoding="utf-8").read()
    for needle, why in (
        ('"/%s/joint_targets" % ns', "the arbiter reads each arm's own targets"),
        ('"/cell/arbiter"', "and reports on /cell"),
        ("def decide(", "the command policy is a pure function"),
        ("ARBITER_ENFORCE", "and enforcement is a switch, off by default"),
    ):
        (ok if needle in arb else lambda m: bad(m))(why)
    rec = open(os.path.join(ROOT, "docker/ws/control/recorder.py"), encoding="utf-8").read()
    for needle, why, want in (
        ('"/arm1/joint_states"', "the recorder reads arm 1's own state", True),
        ('"/arm2/joint_states"', "and arm 2's", True),
        ('"/cell/joint_states_rad"', "never the merged state (its stamp is the last arrival's)", False),
    ):
        got = needle in rec
        (ok if got == want else lambda m: bad(m))(why)
    server = open(os.path.join(ROOT, "docker/ws/arm/lerobot_joint_server.py"), encoding="utf-8").read()
    for needle, why, want in (
        ('bus.sync_write("Goal_Position", driven)', "one packet for all six motors", True),
        ("for name, value in zip(names, angles)", "not one write per motor (partial poses)", False),
        ("cc.validate_body(body)", "the command contract is enforced at the port", True),
        ('"/claim"', "and ownership is taken at the port, not on the topic", True),
    ):
        got = needle in server
        (ok if got == want else lambda m: bad(m))(why)
    bridge = open(os.path.join(ROOT, "docker/ws/arm/arm_bridge.py"), encoding="utf-8").read()
    (ok if "ARM_WRITE_MODE" in bridge else lambda m: bad(m))(
        "the bridge can yield the write path to the arbiter")
    merge = open(os.path.join(ROOT, "docker/ws/arm/joint_merge.py"), encoding="utf-8").read()
    (ok if "MERGE_STALE_S" in merge and '"/cell/joint_states_rad"' in merge else lambda m: bad(m))(
        "joint_merge drops a silent arm instead of disguising it")


def main():
    print("namespace invariant (ADR-0023)\n")
    check_urdf()
    check_static_frames()
    check_frame_ids()
    check_topics()
    check_cameras_and_clients()
    check_command_surface()
    print("\n%s" % ("the invariant holds" if not fails else "%d violation(s)" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
