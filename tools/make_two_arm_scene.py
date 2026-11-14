#!/usr/bin/env python3
"""Generate a two-arm MuJoCo scene from a single-arm model (two-arm Plan, Phase 6).

MuJoCo has no "instance": the same <body> cannot appear twice with the same names. This takes the
arm model XML (the file whose worldbody holds the SO-101 — `mujoco_menagerie/robotstudio_so101/scene.xml`,
which pulls the arm in through `<include file="so101.xml"/>`) and emits one scene with the arm twice:

  - the first arm unchanged, at the origin;
  - a deep copy with every id prefixed `arm2_`, rooted at (0, +0.30, 0) — the same 30 cm as the
    static `base -> arm2_base` in the cell (README §3.6 R3), so the twin and RViz agree.

What gets renamed is the **ids** (body/joint/geom/site/camera) and the **actuators** that drive that
arm's joints. What is deliberately *not* renamed: `class`/`childclass` (both arms share the same
default classes), `mesh`/`material` references (the assets are shared, which MuJoCo allows) and
anything inside `<asset>`.

`<include>` files are resolved first: a scene that only includes the arm has no body of its own to
copy, which is why the naive version of this tool failed on the real model.

Usage (run where the model lives — the Windows operator PC, or any box with mujoco):
    python tools/make_two_arm_scene.py mujoco_menagerie/robotstudio_so101/scene.xml --check
    # -> scene_two_arms.xml next to it; --out to change, --offset-y to change the gap

`--check` compiles the result with mujoco (when importable) and prints the joint/body counts plus the
measured distance between the two arms' gripper bodies, which must read the requested gap.
"""

import argparse
import copy
import os
import sys
import xml.etree.ElementTree as ET

PREFIX = "arm2_"

# Elements whose `name` is an id in the compiled model and must therefore be unique.
ID_TAGS = {"body", "joint", "geom", "site", "camera", "light", "frame", "tendon", "equality",
           "exclude", "sensor", "motor", "position", "velocity", "general", "intvelocity",
           "damper", "cylinder", "muscle", "adhesion", "plugin"}

# MuJoCo's schema wants these sections in this order.
SECTION_ORDER = ["compiler", "option", "size", "visual", "statistic", "asset", "default",
                 "worldbody", "contact", "tendon", "actuator", "sensor", "keyframe", "plugin"]


def _q(tag):
    """Local tag name without an XML namespace."""
    return tag.rsplit("}", 1)[-1]


def _insert_section(root, section):
    """Insert <section> at its schema position (after every section that must precede it)."""
    rank = SECTION_ORDER.index(_q(section.tag)) if _q(section.tag) in SECTION_ORDER else len(SECTION_ORDER)
    for i, child in enumerate(list(root)):
        ctag = _q(child.tag)
        crand = SECTION_ORDER.index(ctag) if ctag in SECTION_ORDER else len(SECTION_ORDER)
        if crand > rank:
            root.insert(i, section)
            return
    root.append(section)


def resolve_includes(path, _depth=0):
    """Parse `path` and inline every <include>, recursively. Returns the merged root element."""
    if _depth > 5:
        raise SystemExit("include nesting too deep at %s" % path)
    base = os.path.dirname(os.path.abspath(path))
    root = ET.parse(path).getroot()

    for include in [c for c in list(root) if _q(c.tag) == "include"]:
        root.remove(include)
        inc_root = resolve_includes(os.path.join(base, include.get("file")), _depth + 1)
        for child in list(inc_root):
            tag = _q(child.tag)
            if tag == "worldbody":
                wb = root.find("worldbody")
                if wb is None:
                    wb = ET.Element("worldbody")
                    _insert_section(root, wb)
                for item in list(child):
                    wb.append(item)
            elif tag in ("asset", "default", "actuator", "sensor", "contact", "tendon", "equality") \
                    and root.find(tag) is not None:
                for item in list(child):
                    root.find(tag).append(item)
            else:
                _insert_section(root, child)
    return root


def find_arm_body(worldbody):
    """The arm is the first top-level body that actually carries a joint."""
    for body in worldbody.findall("body"):
        if body.find(".//joint") is not None:
            return body
    return None


def prefix_ids(elem):
    """Prefix the ids in this subtree, leaving asset references and default classes alone."""
    if _q(elem.tag) in ID_TAGS:
        name = elem.get("name")
        if name:
            elem.set("name", PREFIX + name)
    for child in elem:
        prefix_ids(child)


def joint_names(body):
    return {j.get("name") for j in body.iter("joint") if j.get("name")}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model", help="path to the single-arm model XML (e.g. .../robotstudio_so101/scene.xml)")
    ap.add_argument("--out", default=None, help="output path (default: <model dir>/scene_two_arms.xml)")
    ap.add_argument("--offset-y", type=float, default=0.30,
                    help="gap in metres (default 0.30, matches base->arm2_base)")
    ap.add_argument("--check", action="store_true", help="compile with mujoco and report the geometry")
    args = ap.parse_args()

    root = resolve_includes(args.model)
    worldbody = root.find("worldbody")
    if worldbody is None:
        print("error: no <worldbody> in model", file=sys.stderr)
        return 1
    arm1 = find_arm_body(worldbody)
    if arm1 is None:
        print("error: no body with a <joint> under <worldbody>", file=sys.stderr)
        return 1

    arm2 = copy.deepcopy(arm1)
    prefix_ids(arm2)
    arm2.set("pos", "0 %g 0" % args.offset_y)

    other = [c for c in worldbody if _q(c.tag) != "body"]
    worldbody[:] = other + [arm1, arm2]

    # The actuators driving this arm's joints need a second, prefixed copy aimed at arm2's joints.
    actuator = root.find("actuator")
    if actuator is not None:
        names = joint_names(arm1)
        for act in [a for a in actuator if a.get("joint") in names]:
            twin = copy.deepcopy(act)
            twin.set("name", PREFIX + (act.get("name") or act.get("joint")))
            twin.set("joint", PREFIX + act.get("joint"))
            actuator.append(twin)

    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.model)), "scene_two_arms.xml")
    ET.ElementTree(root).write(out, encoding="utf-8", xml_declaration=True)
    print("wrote %s" % out)
    print("  arm 1: unchanged, at the origin")
    print("  arm 2: %s-prefixed, at (0, %g, 0)" % (PREFIX, args.offset_y))

    if args.check:
        try:
            import mujoco
        except ImportError:
            print("check: mujoco not importable here, skipping")
            return 0
        model = mujoco.MjModel.from_xml_path(out)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        g1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        g2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, PREFIX + "gripper")
        if min(g1, g2) < 0:
            print("check: gripper bodies not found", file=sys.stderr)
            return 1
        d = data.xpos[g2] - data.xpos[g1]
        print("check: nq=%d njnt=%d nbody=%d nu=%d" % (model.nq, model.njnt, model.nbody, model.nu))
        print("check: gripper centres differ by (%.4f, %.4f, %.4f) m -> %.4f m apart"
              % (d[0], d[1], d[2], (d @ d) ** 0.5))
        if abs(d[1] - args.offset_y) > 1e-6:
            print("check: FAILED — the y gap is not the requested %.3f m" % args.offset_y, file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
