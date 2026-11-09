#!/usr/bin/env python3
"""Convert the Menagerie SO-101 MJCF into a URDF for RViz.

Why this exists: the physical arm is an SO-101 (that is the model the MuJoCo twin
loads and the one that matches reality). The URDF that RViz was showing came from
the SO-100 model: different link frames and different joint axes, so RViz drew a
different robot. This generates the URDF straight from `so101.xml`, body by body:

    joint origin  = body `pos` + `quat` (MJCF quats are w x y z)
    joint axis    = the joint's `axis`
    joint limits  = the joint's `range` (already radians)
    visuals       = every `class="visual"` mesh geom, with its own pos/quat

Mesh files are converted to OBJ separately: RViz's STL loader silently fails on
these binary files, OBJ loads fine.

    python3 tools/mjcf_so101_to_urdf.py --mjcf /tmp/so101.xml --meshes docker/web/so101/meshes --out docker/ws/arm/urdf/so101.urdf
"""

import argparse
import math
import os
import xml.etree.ElementTree as ET


def quat_to_rpy(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    # rotation matrix
    R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    # URDF rpy is R = Rz*Ry*Rx
    sy = -R[2][0]
    if abs(sy) < 0.999999:
        pitch = math.asin(sy)
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        pitch = math.copysign(math.pi / 2, sy)
        roll = math.atan2(-R[1][2], R[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def fmt(v):
    return " ".join("%.8g" % x for x in v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", required=True)
    ap.add_argument("--meshes", required=True, help="directory holding the .obj files")
    ap.add_argument("--out", required=True)
    ap.add_argument("--uri-prefix", default="file:///srv/web/so101/meshes")
    args = ap.parse_args()

    root = ET.parse(args.mjcf).getroot()
    lines = ["<?xml version='1.0' encoding='utf-8'?>",
             "<!-- generated from menagerie robotstudio_so101/so101.xml by tools/mjcf_so101_to_urdf.py -->",
             '<robot name="so101">']

    links = []
    joints = []

    def rpy_of(el):
        if el is None:
            return (0.0, 0.0, 0.0)
        if el.get("quat"):
            q = [float(v) for v in el.get("quat").split()]
            return quat_to_rpy(*q)
        if el.get("euler"):
            e = [float(v) for v in el.get("euler").split()]
            return tuple(e)
        return (0.0, 0.0, 0.0)

    def xyz_of(el):
        if el is None or not el.get("pos"):
            return (0.0, 0.0, 0.0)
        return tuple(float(v) for v in el.get("pos").split())

    def walk(body, parent_name):
        name = body.get("name")
        visuals = []
        for g in body.findall("geom"):
            if g.get("type") != "mesh" or g.get("class") != "visual":
                continue
            mesh = g.get("mesh")
            obj = mesh + ".obj"
            if not os.path.exists(os.path.join(args.meshes, obj)):
                print("  !! missing mesh", obj)
                continue
            p = xyz_of(g)
            rpy = rpy_of(g)
            visuals.append((mesh, p, rpy))
        links.append((name, visuals))

        joint = body.find("joint")
        if joint is None and parent_name is not None:
            # a body with no joint (e.g. a camera mount) is rigidly attached: emit a
            # fixed joint, otherwise the link floats and urdfdom rejects the model
            joints.append(("fixed_%s" % name, parent_name, name, xyz_of(body),
                           rpy_of(body), (0.0, 0.0, 1.0), 0.0, 0.0))
        if joint is not None and joint.get("name") and parent_name is not None:
            p = xyz_of(body)
            rpy = rpy_of(body)
            axes = joint.get("axis", "0 0 1")
            rng = joint.get("range")
            lo, hi = (float(rng.split()[0]), float(rng.split()[1])) if rng else (-3.14, 3.14)
            joints.append((joint.get("name"), parent_name, name, p, rpy,
                           tuple(float(v) for v in axes.split()), lo, hi))

        for ch in body.findall("body"):
            walk(ch, name)

    for body in root.find("worldbody").findall("body"):
        walk(body, None)

    for name, visuals in links:
        lines.append('  <link name="%s">' % name)
        for mesh, p, rpy in visuals:
            lines.append('    <visual>')
            lines.append('      <origin xyz="%s" rpy="%s"/>' % (fmt(p), fmt(rpy)))
            lines.append('      <geometry>')
            lines.append('        <mesh filename="%s/%s.obj"/>' % (args.uri_prefix, mesh))
            lines.append('      </geometry>')
            lines.append('    </visual>')
        lines.append('  </link>')

    for name, parent, child, p, rpy, axis, lo, hi in joints:
        fixed = name.startswith("fixed_")
        lines.append('  <joint name="%s" type="%s">' % (name, "fixed" if fixed else "revolute"))
        lines.append('    <parent link="%s"/>' % parent)
        lines.append('    <child link="%s"/>' % child)
        lines.append('    <origin xyz="%s" rpy="%s"/>' % (fmt(p), fmt(rpy)))
        if not fixed:
            lines.append('    <axis xyz="%s"/>' % fmt(axis))
            lines.append('    <limit lower="%.6f" upper="%.6f" effort="35" velocity="10"/>' % (lo, hi))
        lines.append('  </joint>')

    lines.append('</robot>')
    open(args.out, "w").write("\n".join(lines) + "\n")
    print("wrote %s: %d links, %d joints" % (args.out, len(links), len(joints)))
    for j in joints:
        print("  %-14s %s -> %s  axis=%s range=%.2f..%.2f" % (j[0], j[1], j[2], fmt(j[5]), j[6], j[7]))


if __name__ == "__main__":
    main()
