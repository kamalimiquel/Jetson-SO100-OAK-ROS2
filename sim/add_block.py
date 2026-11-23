#!/usr/bin/env python3
"""Put a floor, a table top and a block into a two-arm scene — without editing the generator.

`make_two_arm_scene.py` builds two arms and nothing else: no floor, no table, no block. That is why the viewer
opens and shows arms floating over nothing, and why nothing falls. Rather than making the generator know about
props, this writes a new scene that contains the original plus the things a bench has.

    python3 sim/add_block.py --scene scene_two_arms.xml --out scene_bench.xml \\
        --floor-z 0 --table-z 0.02 --block 0.14 0.02 0.10 --size 0.015

It edits the file as **text**, not as XML: MuJoCo parses its own MJCF leniently and the generated two-arm scene is
not well-formed XML (a stray character in a comment is enough), so an ElementTree round trip fails on a file that
MuJoCo itself opens happily. Idempotent in the useful direction: it refuses to add a second block with the same
name, because a silent duplicate is a scene that looks right and behaves twice.
"""
from __future__ import annotations

import argparse
import re
import sys

BLOCK = "block"


def build(scene_text, floor_z=0.0, table_z=None, block=None, size=0.015, mass=0.05, gravity=True):
    """Return the new scene text. Pure: no files, so it can be tested."""
    text = scene_text
    lower = text.lower()

    # Comments matter here for a real reason: the generated scene mentions <option ...> inside a comment, and a
    # naive search patched the comment instead of the element, which MuJoCo then refused to parse.
    def spans(t):
        return [m.span() for m in re.finditer(r"<!--.*?-->", t, re.S)]

    def find_outside(needle, start=0):
        skip = spans(text)
        i = text.lower().find(needle, start)
        while i != -1 and any(a <= i < b for a, b in skip):
            i = text.lower().find(needle, i + 1)
        return i

    # gravity: make it explicit, because a scene that relies on the default cannot be read
    opt_start = find_outside("<option")
    has_gravity = False
    if opt_start != -1:
        opt_end = text.find(">", opt_start)
        head_opt = text[opt_start:opt_end]
        has_gravity = "gravity=" in head_opt
        zero = 'gravity="0 0 0"' in head_opt.replace("'", '"')
        if gravity and not has_gravity:
            head_txt, slash = text[:opt_end], ""
            if head_txt.endswith("/"):            # <option .../> : the attribute goes before the slash
                head_txt, slash = head_txt[:-1], "/"
            text = head_txt + ' gravity="0 0 -9.81"' + slash + text[opt_end:]
            print("add_block: gravity was unset (MuJoCo's default, which is also -9.81) — written explicitly")
        elif gravity and zero:
            fixed = head_opt.replace('gravity="0 0 0"', 'gravity="0 0 -9.81"')
            text = text[:opt_start] + fixed + text[opt_end:]
            print("add_block: gravity was zero — set to 0 0 -9.81 so the block falls")
    elif gravity:
        i = find_outside("<worldbody")
        text = text[:i] + '<option gravity="0 0 -9.81"/>\n  ' + text[i:]
        print("add_block: no <option> — added gravity 0 0 -9.81")

    if 'flag gravity="disable"' in lower.replace("'", '"'):
        print('add_block: WARNING the scene disables gravity with <flag gravity="disable"/> — the block will '
              'not fall either until that is removed')
    held = text.count('gravcomp="1"')
    if held:
        print("add_block: %d body(ies) carry gravcomp (held up without actuators)" % held)
    if find_outside('name="block"') != -1:
        raise SystemExit("add_block: the scene already has a body named 'block'")

    wb_end = text.lower().rfind("</worldbody>")
    if wb_end == -1:
        raise SystemExit("add_block: the scene has no </worldbody> to insert before")

    parts = []
    if 'type="plane"' not in lower:
        parts.append('    <geom name="floor" type="plane" size="1 1 0.05" pos="0 0 %.4f" '
                     'rgba="0.32 0.32 0.38 1"/>' % float(floor_z))
    if table_z is not None:
        parts.append('    <geom name="table" type="box" size="0.30 0.30 0.01" pos="0.10 0.15 %.4f" '
                     'rgba="0.55 0.45 0.35 1"/>' % (float(floor_z) + float(table_z)))
    if block is not None:
        x, y, z = (float(v) for v in block)
        parts.append('    <body name="block" pos="%.4f %.4f %.4f">\n'
                     '      <freejoint/>\n'
                     '      <geom name="block_geom" type="box" size="%.4f %.4f %.4f" '
                     'rgba="0.90 0.70 0.20 1" mass="%g" friction="1.5 0.02 0.001"/>\n'
                     '    </body>' % (x, y, z, size, size, size, mass))
    if not parts:
        return text
    return text[:wb_end] + "\n".join(parts) + "\n  " + text[wb_end:]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--floor-z", type=float, default=0.0)
    ap.add_argument("--table-z", type=float, default=None,
                    help="height of a table top above the floor; omit for none")
    ap.add_argument("--block", nargs="+", metavar="X Y [Z]", default=None,
                    help="centre of the block: X Y and Z, or X Y alone with --drop-cm to compute Z above the "
                         "table so it falls when the model starts")
    ap.add_argument("--drop-cm", type=float, default=20.0)
    ap.add_argument("--size", type=float, default=0.015, help="half-extent of the block, metres")
    ap.add_argument("--mass", type=float, default=0.05)
    ap.add_argument("--no-gravity", action="store_true")
    args = ap.parse_args(argv)

    block = args.block
    if block is not None:
        if len(block) == 2:
            if args.table_z is None:
                raise SystemExit("add_block: --drop-cm needs --table-z to know where the table top is")
            top = float(args.floor_z) + float(args.table_z) + float(args.size)
            block = [float(block[0]), float(block[1]), top + float(args.drop_cm) / 100.0]
            print("add_block: block starts %.0f cm above the table top (z=%.3f), so it falls"
                  % (args.drop_cm, block[2]))
        elif len(block) != 3:
            raise SystemExit("add_block: --block takes X Y (with --drop-cm) or X Y Z")

    with open(args.scene, encoding="utf-8", errors="replace") as fh:
        text = fh.read()
    out = build(text, args.floor_z, args.table_z, block, args.size, args.mass, gravity=not args.no_gravity)
    with open(args.out, "w", encoding="utf-8") as fh:
        fh.write(out)
    print("add_block: wrote %s (floor%s%s)" % (
        args.out, "" if args.table_z is None else ", table",
        "" if block is None else ", block at %.3f %.3f %.3f" % tuple(float(v) for v in block)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
