#!/usr/bin/env python3
"""Put a floor, a table top and a block into a two-arm scene — without editing the generator.

`make_two_arm_scene.py` builds two arms and nothing else: no floor, no table, no block. That is why the viewer
opens and shows arms floating over nothing, and why nothing falls. Rather than making the generator know about
props, this writes a new scene that contains the original plus the things a bench has.

    python3 sim/add_block.py --scene scene_two_arms.xml --out scene_bench.xml \\
        --floor-z 0 --table-z 0.02 --block 0.14 0.02 0.10 --size 0.015

Idempotent in the useful direction: it refuses to add a second block if one with the same name is already
there, because a silent duplicate is a scene that looks right and behaves twice.
"""
from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET

BLOCK = "block"


def build(scene, floor_z=0.0, table_z=None, block=None, size=0.015, mass=0.05):
    tree = ET.parse(scene)
    root = tree.getroot()
    world = root.find("worldbody")
    if world is None:
        raise SystemExit("add_block: %s has no <worldbody>" % scene)
    names = {b.get("name") for b in world.findall("body")}
    if block is not None and BLOCK in names:
        raise SystemExit("add_block: the scene already has a body named %r" % BLOCK)

    if not any(g.get("type") == "plane" for g in world.findall("geom")):
        ET.SubElement(world, "geom", {"name": "floor", "type": "plane", "size": "1 1 0.05",
                                      "pos": "0 0 %.4f" % float(floor_z),
                                      "rgba": "0.32 0.32 0.38 1"})
    if table_z is not None:
        ET.SubElement(world, "geom", {"name": "table", "type": "box", "size": "0.30 0.30 0.01",
                                      "pos": "0.10 0.15 %.4f" % (float(floor_z) + float(table_z)),
                                      "rgba": "0.55 0.45 0.35 1", "contype": "1", "conaffinity": "1"})
    if block is not None:
        x, y, z = (float(v) for v in block)
        body = ET.SubElement(world, "body", {"name": BLOCK, "pos": "%.4f %.4f %.4f" % (x, y, z)})
        ET.SubElement(body, "freejoint")
        ET.SubElement(body, "geom", {"name": "block_geom", "type": "box",
                                     "size": "%.4f %.4f %.4f" % (size, size, size),
                                     "rgba": "0.90 0.70 0.20 1", "mass": "%g" % mass,
                                     "friction": "1.5 0.02 0.001"})
    return tree


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--floor-z", type=float, default=0.0)
    ap.add_argument("--table-z", type=float, default=None,
                    help="height of a table top above the floor; omit for none")
    ap.add_argument("--block", nargs=3, metavar=("X", "Y", "Z"), default=None,
                    help="centre of the block in metres")
    ap.add_argument("--size", type=float, default=0.015, help="half-extent of the block, metres")
    ap.add_argument("--mass", type=float, default=0.05)
    args = ap.parse_args(argv)

    tree = build(args.scene, args.floor_z, args.table_z, args.block, args.size, args.mass)
    tree.write(args.out, encoding="utf-8", xml_declaration=True)
    print("add_block: wrote %s (floor%s%s)" % (
        args.out, "" if args.table_z is None else ", table",
        "" if args.block is None else ", block at %.3f %.3f %.3f" % tuple(float(v) for v in args.block)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
