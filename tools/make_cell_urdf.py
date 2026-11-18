#!/usr/bin/env python3
"""Write the two-arm cell URDF: arm 1 as it is, arm 2 as a prefix-renamed copy 30 cm to its left.

RViz on this board renders only the FIRST display of a given class — a second RobotModel loads and
reports OK but draws nothing (same bug as the second Image display). So both arms have to be one
model: a single robot_state_publisher drives the whole tree, and this file is what it loads.

Arm 1 keeps its plain link names; arm 2's links and joints are prefixed `arm2_` and hang off a fixed
joint `base -> arm2_base` at (0, 0.30, 0), which is the 30 cm of README §15 R3. The MuJoCo twin
reads the same number. **ADR-0023:** arm 1 is prefixed `arm1_` as well, the mount joint is
`arm1_base -> arm2_base`, and the script refuses to write a file where any link or joint is bare.

    python3 tools/make_cell_urdf.py        # writes docker/ws/arm/urdf/so101_cell.urdf
"""

import argparse
import json
import os
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEOMETRY = os.path.join(ROOT, "docker/ws/cell_geometry.json")


def cell_offset(path=GEOMETRY):
    """Where arm 2 stands, from the ONE file that says it (README §15 R3)."""
    with open(path, encoding="utf-8") as fh:
        off = json.load(fh)["arm2_offset_m"]
    return float(off["x"]), float(off["y"]), float(off["z"])


def rename_tree(root, prefix, skip_joints=()):
    """Prefix every link and joint name, and every parent/child reference (ADR-0023)."""
    for tag in ("link", "joint"):
        for node in root.findall(tag):
            name = node.get("name")
            if name and not name.startswith(prefix):
                node.set("name", prefix + name)
    for joint in root.findall("joint"):
        for tag in ("parent", "child"):
            el = joint.find(tag)
            link = el.get("link") if el is not None else None
            if link and not link.startswith(prefix):
                el.set("link", prefix + link)


def main():
    ap = argparse.ArgumentParser(description="single URDF holding both arms")
    ap.add_argument("--src", default=os.path.join(ROOT, "docker/ws/arm/urdf/so101.urdf"))
    ap.add_argument("--out", default=os.path.join(ROOT, "docker/ws/arm/urdf/so101_cell.urdf"))
    ap.add_argument("--prefix1", default="arm1_", help="frame prefix of arm 1 (ADR-0023)")
    ap.add_argument("--prefix2", default="arm2_", help="frame prefix of arm 2")
    ap.add_argument("--geometry", default=GEOMETRY, help="cell geometry file (the single source)")
    ap.add_argument("--offset-y", type=float, default=None,
                    help="override the y of the cell geometry file (normally leave it alone)")
    args = ap.parse_args()
    off_x, off_y, off_z = cell_offset(args.geometry)
    if args.offset_y is not None:
        print(f"warning: --offset-y {args.offset_y} overrides {args.geometry}")
        off_y = args.offset_y

    tree = ET.parse(args.src)
    arm1 = tree.getroot()

    arm2 = ET.parse(args.src).getroot()
    rename_tree(arm2, args.prefix2)

    # ADR-0023: arm 1 is prefixed too, so no bare frame exists anywhere to collide with arm 2.
    rename_tree(arm1, args.prefix1)

    # The join: a fixed joint from arm 1's base to arm 2's base, offset in +y.
    mount = ET.SubElement(arm1, "joint", {"name": args.prefix1 + "mount", "type": "fixed"})
    ET.SubElement(mount, "parent", {"link": args.prefix1 + "base"})
    ET.SubElement(mount, "child", {"link": args.prefix2 + "base"})
    ET.SubElement(mount, "origin", {"xyz": "%g %g %g" % (off_x, off_y, off_z), "rpy": "0 0 0"})

    for el in list(arm2):
        if el.tag in ("link", "joint"):
            arm1.append(el)

    arm1.set("name", "so101_cell")
    tree.write(args.out, encoding="utf-8", xml_declaration=True)
    links = [l.get("name") for l in arm1.findall("link")]
    joints = [j.get("name") for j in arm1.findall("joint")]
    print("wrote %s: %d links, %d joints (arm 2 at %+g %+g %+g, from %s)"
          % (args.out, len(links), len(joints), off_x, off_y, off_z, args.geometry))

    # A single bare name anywhere is the bug R2 describes, and it has no symptom until RViz draws
    # one arm's shoulder on the other's base — so refuse to write a file that has one.
    bare = [n for n in links + joints
            if not (n.startswith(args.prefix1) or n.startswith(args.prefix2))]
    if bare:
        raise SystemExit("error: %d unprefixed link(s)/joint(s) survived: %s" % (len(bare), bare[:5]))
    roots = set(l.get("name") for l in arm1.findall("link"))
    for j in arm1.findall("joint"):
        roots.discard(j.find("child").get("link"))       # a root is a parent and never a child
    print("root: %s | every link and joint is %s* or %s*"
          % (", ".join(sorted(roots)), args.prefix1, args.prefix2))


if __name__ == "__main__":
    main()
