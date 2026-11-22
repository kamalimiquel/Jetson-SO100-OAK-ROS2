#!/usr/bin/env python3
"""Checks for the manifest an agent sees — no ROS, no cell, no endpoint.

Each case is one that would otherwise be believed silently: an epoch that survives a restart, an arm dropped
from the manifest because it was offline, a deadband reported as 0 when it was never measured.

    python3 tools/test_manifest.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mcp"))
sys.path.insert(0, os.path.join(HERE, "..", "docker", "ws", "lib"))
import joint_scale as js  # noqa: E402
import manifest as mf  # noqa: E402

ok = fail = 0


def check(label, got, want):
    global ok, fail
    if got == want:
        ok += 1
        print("ok   %-58s %r" % (label, got))
    else:
        fail += 1
        print("FAIL %-58s got %r want %r" % (label, got, want))


def raises(fn, label):
    try:
        fn()
    except (mf.ManifestError, js.ScaleError):
        check(label, True, True)
        return
    check(label, False, True)


T1 = js.load(js.path_for("so100_cell"))
T2 = js.load(js.path_for("so100_cell_b"))
tables = {"so100_cell": T1, "so100_cell_b": T2}

# ---- the epoch must not survive a restart, and must move with the calibration
e1 = mf.epoch("boot-aaa", tables)
check("the epoch is stable for the same inputs", e1, mf.epoch("boot-aaa", tables))
check("a new boot gives a new epoch", e1 != mf.epoch("boot-bbb", tables), True)
check("a recalibrated arm gives a new epoch",
      e1 != mf.epoch("boot-aaa", {"so100_cell": dict(T1, sha256="deadbeef")}), True)
raises(lambda: mf.epoch("", tables), "no boot id is refused")

ARM1 = {"arm": "so100_cell", "reachable": True, "owner": None, "torque": False, "table": T1}
ARM2 = {"arm": "so100_cell_b", "reachable": False, "owner": None, "torque": None, "table": T2,
        "error": "unreachable ([Errno 111] Connection refused)"}
ID = {"robot": "bench-02", "serial": "so100_cell", "profile": "robot"}

m = mf.build(ID, [ARM1, ARM2], e1, images_lock="abc123")
check("the schema is declared", m["schema"], mf.SCHEMA)
check("both arms are present", [a["arm"] for a in m["arms"]], ["so100_cell", "so100_cell_b"])
check("an unreachable arm is reported, not dropped", m["arms"][1]["reachable"], False)
check("...with its reason", "Connection refused" in m["arms"][1]["error"], True)
check("a reachable arm says so", m["arms"][0]["reachable"], True)
check("arm 1's measured deadband is in the manifest", m["arms"][0]["joints"]["shoulder_pan"]["deadband_deg"], 1.0)
check("arm 2's is its own", m["arms"][1]["joints"]["shoulder_pan"]["deadband_deg"], 2.0)
check("an unmeasured joint says null, not 0", m["arms"][0]["joints"]["elbow_flex"]["deadband_deg"], None)
check("the scale travels with the joint", m["arms"][0]["joints"]["wrist_roll"]["scale"], 0.877)
check("the jaw has a curve, not a scale", "curve" in m["arms"][0]["joints"]["gripper"], True)
check("the calibration digest is truncated but present", len(m["arms"][0]["calibration_sha256"]), 16)
check("the skills are the closed list", m["skills"], list(mf.SKILLS))

raises(lambda: mf.build(dict(ID, robot=""), [ARM1], e1), "a manifest without a robot is refused")
raises(lambda: mf.build(dict(ID, profile="bench"), [ARM1], e1), "an unknown profile is refused")
raises(lambda: mf.build(ID, [], e1), "a cell with no arms is refused")
raises(lambda: mf.build(ID, [{"arm": "so100_cell"}], e1), "an arm without its table is refused")
raises(lambda: mf.build(ID, [ARM1], ""), "a manifest without an epoch is refused")

# the sim profile must be a different declaration, not the same one renamed
sim = mf.build(dict(ID, profile="sim"), [ARM1], e1)
check("sim is a declared profile, not a fallback", sim["identity"]["profile"], "sim")
check("and it is visible in the document", "sim" in mf.to_json(sim), True)

print("\n%s — %d checks, %d failed" % ("PASS" if not fail else "FAIL", ok + fail, fail))
sys.exit(1 if fail else 0)
