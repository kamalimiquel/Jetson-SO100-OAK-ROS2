#!/usr/bin/env python3
"""Headless test of the unit manifest — no ROS, no SDK, no arm.

The manifest is what stops the quiet failure: the same number meaning degrees in one configuration
and a −100…100 fraction in another, with nothing raising anywhere. These cases are the ways it can
go wrong, and each must produce a sentence a human can act on.

    python3 tools/test_units_manifest.py
"""

import importlib.util
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
LIB = os.path.join(os.path.dirname(HERE), "docker", "ws", "lib", "units_manifest.py")

fails = []


def check(name, got, want):
    ok = got == want
    print(("  ok   " if ok else "  FAIL ") + "%s: %r" % (name, got) +
          ("" if ok else " (expected %r)" % (want,)))
    if not ok:
        fails.append(name)


_spec = importlib.util.spec_from_file_location("units_manifest", LIB)
um = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(um)


def manifest(use_degrees=True, joints=None, signatures=None, version="0.7.0"):
    joints = joints if joints is not None else [
        {"name": n, "id": i, "model": "sts3215",
         "norm": "RANGE_0_100" if n == "gripper" else "DEGREES"}
        for n, i, _m, _n in um.JOINTS]
    return {"sdk_version": version, "use_degrees": use_degrees, "joints": joints,
            "signatures": signatures if signatures is not None else
            {"read": ["normalize"], "write": ["normalize"], "sync_write": ["normalize"]}}


def main():
    print("the expected configuration")
    errors, warnings = um.check(manifest())
    check("a correct manifest has no errors", errors, [])
    check("and a matching SDK version raises no warning", warnings, [])

    print("the failure that has no symptom: wrong unit system")
    errors, _ = um.check(manifest(use_degrees=False))
    check("use_degrees False is refused", any("use_degrees" in e for e in errors), True)
    check("and the sentence says what it would become", "fraction" in errors[0], True)

    bad = manifest()
    bad["joints"][2]["norm"] = "RANGE_M100_100"          # elbow
    errors, _ = um.check(bad)
    check("a body joint in the wrong mode is refused", any("elbow_flex" in e for e in errors), True)
    check("naming the unit as the problem", any("unit of this joint is wrong" in e for e in errors),
          True)

    bad = manifest()
    bad["joints"][5]["norm"] = "DEGREES"                 # jaw
    errors, _ = um.check(bad)
    check("a jaw reported in degrees is refused", any("gripper" in e for e in errors), True)

    print("a bus map that moved")
    bad = manifest()
    bad["joints"] = bad["joints"][:5] + [
        {"name": "extra_a", "id": 7, "model": "sts3215", "norm": "DEGREES"},
        {"name": "extra_b", "id": 8, "model": "sts3215", "norm": "DEGREES"}]
    errors, _ = um.check(bad)
    check("a missing motor is refused", any("gripper" in e and "missing" in e for e in errors), True)
    check("an extra motor changes the count", any("motors reported" in e for e in errors), True)

    bad = manifest()
    bad["joints"][0], bad["joints"][1] = bad["joints"][1], bad["joints"][0]
    errors, _ = um.check(bad)
    check("a reordered map is refused (the array has no names)",
          any("wire order" in e for e in errors), True)

    bad = manifest()
    bad["joints"][0]["id"] = 9
    errors, _ = um.check(bad)
    check("a changed bus id is refused", any("bus id" in e for e in errors), True)

    print("raw ticks must be reachable at all")
    bad = manifest(signatures={"read": ["data_name", "motor"], "write": ["normalize"],
                               "sync_write": ["normalize"]})
    errors, _ = um.check(bad)
    check("read() without normalize is refused", any("read()" in e for e in errors), True)
    _, warnings_ = um.check(manifest(signatures={}))
    check("an uninspected signature is a warning, not a stop",
          any("signature not inspected" in w for w in warnings_), True)

    print("version drift is recorded, not enforced")
    _, warnings_ = um.check(manifest(version="0.8.1"))
    check("a newer SDK warns", any("re-run the unit table" in w for w in warnings_), True)
    _, warnings_ = um.check(manifest(version=None))
    check("an unknown version warns", any("version unknown" in w for w in warnings_), True)

    print("reading the manifest out of an object graph")
    arm = types.SimpleNamespace(bus=types.SimpleNamespace(motors={
        n: types.SimpleNamespace(id=i, model="sts3215",
                                 norm_mode=types.SimpleNamespace(
                                     name="RANGE_0_100" if n == "gripper" else "DEGREES"))
        for n, i, _m, _n in um.JOINTS}))
    observed = um.observed_from(arm, config=types.SimpleNamespace(use_degrees=True),
                                sdk_version="0.7.0",
                                signatures={"read": ["normalize"], "write": ["normalize"],
                                            "sync_write": ["normalize"]})
    check("observed_from() reads ids, models and modes", [j["name"] for j in observed["joints"]],
          [j[0] for j in um.JOINTS])
    check("and it passes the check", um.check(observed)[0], [])
    check("describe() gives one line per joint", len(um.describe(observed)), 6)

    print("\n%s" % ("all checks pass" if not fails else "%d check(s) FAILED" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
