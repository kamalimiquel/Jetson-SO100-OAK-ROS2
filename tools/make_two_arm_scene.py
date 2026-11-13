#!/usr/bin/env python3
"""Generate a two-arm MuJoCo scene from a single-arm model (two-arm Plan, Phase 6).

MuJoCo has no "instance" — the same <body> cannot appear twice with the same
names. So this takes the arm model XML (the file whose worldbody holds the
SO-101, e.g. mujoco_menagerie/robotstudio_so101/scene.xml) and emits a scene
with the arm twice:

  - the first arm unchanged (its bodies/joints/geoms keep their names);
  - a deep copy of it with every id-prefixed `arm2_`, rooted at (0, 0.30, 0) —
    the same 30 cm as the base -> arm2_base static transform (README §3.6 R3).

All ids that MuJoCo disallows duplicates for (body, joint, geom, site, and any
child worldbody) are prefixed; the shared <asset>s are left shared, which is
legal — two geoms can use the same mesh.

Usage (run where the model lives, on the Windows operator PC / in the MuJoCo env):
    python tools\\make_two_arm_scene.py C:\\path\\to\\robotstudio_so101\\scene.xml
    # -> scene_two_arms.xml next to it (--out to change, --offset-y to change the gap)

The generated file is what `windows/mujoco_twin.py` loads once the twin knows
it holds two arms (pass --both-arms).
"""

import argparse
import copy
import sys
import xml.etree.ElementTree as ET

PREFIX = "arm2_"
# MuJoCo requires these name ids to be unique across the compiled model.
RENAME = {"body", "joint", "geom", "site", "tendon", "equality", "exclude", "fixed", "motor",
          "general", "position", "velocity", "cylinder", "slider", "hinge", "ball", "slide",
          "free", "touch"}


def _q(tag):
    """Local tag without namespace."""
    return tag.rsplit("}", 1)[-1]


def rename_ids(elem, ns):
    """Prefix every id-carrying attribute that MuJoCo must keep unique."""
    tag = _q(elem.tag)
    for attr in ("name", "id"):
        v = elem.get(attr)
        if v and tag in RENAME:
            elem.set(attr, f"{PREFIX}{v}")
    if tag == "class":
        cls = elem.get("class")
        if cls:
            elem.set("class", f"{PREFIX}{cls}")
    # referential attrs inside an element point at names that were also renamed
    if tag == "default":
        for child in elem:
            if child.get("class"):
                child.set("class", f"{PREFIX}{child.get('class')}")
    for child in elem:
        rename_ids(child, ns)


def find_arm_body(worldbody):
    """The arm is the first top-level body that actually carries a joint."""
    for body in worldbody.findall("body"):
        if body.find(".//joint") is not None:
            return body
    return worldbody.find("body")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("model", help="path to the single-arm model XML (e.g. .../robotstudio_so101/scene.xml)")
    ap.add_argument("--out", default=None, help="output path (default: <model dir>/scene_two_arms.xml)")
    ap.add_argument("--offset-y", type=float, default=0.30, help="gap in metres (default 0.30, matches base->arm2_base)")
    ap.add_argument("--keep-first-at", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                    help="x y z of the first arm's root body (default 0 0 0)")
    args = ap.parse_args()

    tree = ET.parse(args.model)
    root = tree.getroot()
    ns = ""
    if "}" in root.tag:
        ns = root.tag.split("}", 1)[0] + "}"

    worldbody = root.find(f"{ns}worldbody")
    if worldbody is None:
        print("error: no <worldbody> in model", file=sys.stderr)
        return 1
    arm1 = find_arm_body(worldbody)
    if arm1 is None:
        print("error: no body with a <joint> under <worldbody>", file=sys.stderr)
        return 1

    arm2 = copy.deepcopy(arm1)
    rename_ids(arm2, ns)
    # root the second arm at the same height/axis, shifted along -y (matches the
    # base -> arm2_base static transform at y = +0.30 in the cell frame; the
    # twin and the bench agree on the number, one literal each).
    pos = arm2.get("pos")
    arm2.set("pos", f"{args.keep_first_at[0]} {args.keep_first_at[1] + args.offset_y} {args.keep_first_at[2]}")

    # arm1's root body may itself sit at a nonzero pos in the model; keep it as-is
    # and make arm2 an explicit copy at the model root's + offset. To keep both at
    # the same elevation regardless of the input, normalise arm1 to keep_first_at too.
    arm1x, arm1y, arm1z = (arm1.get("pos") or "0 0 0").split()
    arm1.set("pos", f"{args.keep_first_at[0]} {args.keep_first_at[1]} {args.keep_first_at[2]}")

    # worldbody children that aren't bodies (lights, cameras) stay in the scene map
    # we keep; then the two arms.
    other = [c for c in worldbody if _q(c.tag) != "body"]
    worldbody[:] = other + [arm1, arm2]

    out = args.out or (args.model.rsplit("/", 1)[0] + "/scene_two_arms.xml")
    tree.write(out, encoding="utf-8", xml_declaration=True)
    print(f"wrote {out}")
    print("  arm 1: unchanged, rooted as in the source")
    print(f"  arm 2: {PREFIX}-prefixed, rooted at (0, {args.keep_first_at[1] + args.offset_y}, 0)")
    print("load it in mujoco_twin with --both-arms and check model.nq has risen and")
    print("`arm2_` frames show the second arm 0.30 m from the first (Plan Phase 6, T6.1/T6.2).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
