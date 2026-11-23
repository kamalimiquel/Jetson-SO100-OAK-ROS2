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


def diagnose(scene_text):
    """Say what could hold an arm up, before anything is written. Runs on every path, including --separate."""
    low = scene_text.lower().replace("'", '"')
    if 'flag gravity="disable"' in low:
        print('add_block: WARNING the scene disables gravity with <flag gravity="disable"/> — nothing will fall')
    held = low.count('gravcomp="1"')
    if held:
        print("add_block: %d body(ies) carry gravcomp (held up without actuators)" % held)
    fl = low.count("frictionloss=")
    if fl:
        print("add_block: %d joint(s) declare frictionloss — a light arm can be held by static friction alone, "
              "which is physics, not compensation" % fl)
    if not held and 'flag gravity="disable"' not in low and not fl:
        print("add_block: no gravcomp, no gravity flag, no frictionloss — if the arms do not fall, they are "
              "already at rest or something else holds them; check /objects twice")


def build(scene_text, floor_z=0.0, table_z=None, block=None, size=0.015, mass=0.05, gravity=False):
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

    # Gravity is NOT rewritten by default: MuJoCo's default is already 0 0 -9.81, and patching an attribute in
    # someone else's file is how two runs in a row produced an unparseable scene. It is reported, and only
    # rewritten with --write-gravity.
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
    ap.add_argument("--separate", action="store_true",
                    help="write the props into <out>_props.xml and insert a single <include> line: the scene "
                         "itself is then untouched apart from that one line")
    ap.add_argument("--write-gravity", action="store_true",
                    help="rewrite <option gravity> to 0 0 -9.81 if it is zero or unset. Off by default: the "
                         "default is already -9.81 and editing that tag has broken scenes before")
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
    diagnose(text)
    if args.separate:
        # An <include> inside <worldbody> is replaced by the fragment's elements, so the fragment must hold
        # body/geom directly — wrapping it in <mujoco><worldbody> would nest a worldbody inside a worldbody.
        props = []
        if 'type="plane"' not in text.lower():
            props.append('    <geom name="floor" type="plane" size="1 1 0.05" pos="0 0 %.4f" '
                         'rgba="0.32 0.32 0.38 1"/>' % float(args.floor_z))
        if args.table_z is not None:
            props.append('    <geom name="table" type="box" size="0.30 0.30 0.01" pos="0.10 0.15 %.4f" '
                         'rgba="0.55 0.45 0.35 1"/>' % (float(args.floor_z) + float(args.table_z)))
        if block is not None:
            props.append('    <body name="block" pos="%.4f %.4f %.4f">\n      <freejoint/>\n'
                         '      <geom name="block_geom" type="box" size="%.4f %.4f %.4f" '
                         'rgba="0.90 0.70 0.20 1" mass="%g" friction="1.5 0.02 0.001"/>\n    </body>'
                         % (block[0], block[1], block[2], args.size, args.size, args.size, args.mass))
        props += [""]
        props_path = args.out.replace(".xml", "") + "_props.xml"
        with open(props_path, "w", encoding="utf-8") as fh:
            fh.write("\n".join(props))
        inc = '    <include file="%s"/>\n  ' % props_path.split("\\")[-1].split("/")[-1]
        i = text.lower().rfind("</worldbody>")
        if i == -1:
            raise SystemExit("add_block: the scene has no </worldbody> to insert the include before")
        out = text[:i] + inc + text[i:]
        print("add_block: props in %s, one <include> line added to the scene" % props_path)
    else:
        out = build(text, args.floor_z, args.table_z, block, args.size, args.mass, gravity=args.write_gravity)
    with open(args.out, "w", encoding="utf-8") as fh:
        fh.write(out)
    print("add_block: wrote %s (floor%s%s)" % (
        args.out, "" if args.table_z is None else ", table",
        "" if block is None else ", block at %.3f %.3f %.3f" % tuple(float(v) for v in block)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
