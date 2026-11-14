#!/usr/bin/env python3
"""Write the two-arm cell URDF: arm 1 as it is, arm 2 as a prefix-renamed copy 30 cm to its left.

RViz on this board renders only the FIRST display of a given class — a second RobotModel loads and
reports OK but draws nothing (same bug as the second Image display). So both arms have to be one
model: a single robot_state_publisher drives the whole tree, and this file is what it loads.

Arm 1 keeps its plain link names; arm 2's links and joints are prefixed `arm2_` and hang off a fixed
joint `base -> arm2_base` at (0, 0.30, 0), which is the 30 cm of README §3.6 R3. The MuJoCo twin
reads the same number.

    python3 tools/make_cell_urdf.py        # writes docker/ws/arm/urdf/so101_cell.urdf
"""

import argparse
import os
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARM_OFFSET_Y = 0.30


def rename(el, tag, prefix):
    for node in el.findall(tag):
        name = node.get("name")
        if name and not name.startswith(prefix):
            node.set("name", prefix + name)


def main():
    ap = argparse.ArgumentParser(description="single URDF holding both arms")
    ap.add_argument("--src", default=os.path.join(ROOT, "docker/ws/arm/urdf/so101.urdf"))
    ap.add_argument("--out", default=os.path.join(ROOT, "docker/ws/arm/urdf/so101_cell.urdf"))
    ap.add_argument("--prefix", default="arm2_")
    ap.add_argument("--offset-y", type=float, default=ARM_OFFSET_Y)
    args = ap.parse_args()

    tree = ET.parse(args.src)
    arm1 = tree.getroot()

    arm2 = ET.parse(args.src).getroot()
    rename(arm2, "link", args.prefix)
    rename(arm2, "joint", args.prefix)
    for joint in arm2.findall("joint"):
        for tag in ("parent", "child"):
            el = joint.find(tag)
            link = el.get("link") if el is not None else None
            if link and not link.startswith(args.prefix):
                el.set("link", args.prefix + link)

    # The join: a fixed joint from arm 1's base to arm 2's base, offset in +y.
    mount = ET.SubElement(arm1, "joint", {"name": args.prefix + "mount", "type": "fixed"})
    ET.SubElement(mount, "parent", {"link": "base"})
    ET.SubElement(mount, "child", {"link": args.prefix + "base"})
    ET.SubElement(mount, "origin", {"xyz": "0 %g 0" % args.offset_y, "rpy": "0 0 0"})

    for el in list(arm2):
        if el.tag in ("link", "joint"):
            arm1.append(el)

    arm1.set("name", "so101_cell")
    tree.write(args.out, encoding="utf-8", xml_declaration=True)
    links = [l.get("name") for l in arm1.findall("link")]
    joints = [j.get("name") for j in arm1.findall("joint")]
    print("wrote %s: %d links, %d joints (arm 2 at y=%+g)" % (args.out, len(links), len(joints), args.offset_y))


if __name__ == "__main__":
    main()
