"""Generate the SO-101 joint-naming reference diagram (SVG -> PNG).

One picture that answers "what is this joint called in each layer we use":
LeRobot bus -> joint server (:8002) -> arm_bridge -> /joint_states ->
deg2rad -> RViz/TF, plus the MuJoCo twin and the F710 teleop.
"""

import math

W, H = 1820, 1160
BG = "#070c16"
PANEL = "#0d1421"
PANEL2 = "#111a2b"
EDGE = "#1f2b40"
INK = "#e6edf7"
DIM = "#8494ad"
ACCENT = "#22d3ee"
GREEN = "#34d399"
AMBER = "#fbbf24"
VIOLET = "#a78bfa"
ROSE = "#fb7185"

JOINTS = [
    # n, name,        idx, axis-note,            lo_rad,     hi_rad,     lo_deg,  hi_deg,  driver
    (1, "shoulder_pan",  0, "vertical axis",   -1.91986,  1.91986,  -110.0,  110.0,  "±170°"),
    (2, "shoulder_lift", 1, "horizontal",      -1.74533,  1.74533,  -100.0,  100.0,  "±170°"),
    (3, "elbow_flex",    2, "horizontal",      -1.69,     1.69,      -96.8,   96.8,  "±170°"),
    (4, "wrist_flex",    3, "horizontal",      -1.65806,  1.65806,   -95.0,   95.0,  "±170°"),
    (5, "wrist_roll",    4, "along forearm",   -2.74385,  2.84121,  -157.2,  162.8,  "±170°"),
    (6, "gripper",       5, "jaw open/close",  -0.17453,  1.74533,  -10.0,   100.0,  "0–100 %"),
]

F710 = {
    1: "idx 0 · left stick X",
    2: "idx 1 · left stick Y",
    3: "idx 2 · right stick X",
    4: "idx 3 · D-pad ↑↓",
    5: "idx 4 · right stick Y",
    6: "idx 5 · LB/RB · LT/RT",
}


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;")


def text(x, y, s, size=13, fill=INK, weight="normal", anchor="start", family="'JetBrains Mono',monospace", op=1.0):
    return (f'<text x="{x}" y="{y}" font-family="{family}" font-size="{size}" fill="{fill}" '
            f'font-weight="{weight}" text-anchor="{anchor}" opacity="{op}">{esc(s)}</text>')


def arm_geometry():
    """Base at the bottom left, arm arching up and to the right (the bench pose)."""
    base = (208.0, 806.0)                      # J1, vertical axis
    shoulder = (208.0, 712.0)                  # J2, above the base
    a1 = math.radians(-56)
    elbow = (shoulder[0] + 226 * math.cos(a1), shoulder[1] + 226 * math.sin(a1))
    a2 = math.radians(-9)
    wrist = (elbow[0] + 196 * math.cos(a2), elbow[1] + 196 * math.sin(a2))
    a3 = math.radians(-8)
    grip_base = (wrist[0] + 88 * math.cos(a3), wrist[1] + 88 * math.sin(a3))
    return base, shoulder, elbow, wrist, grip_base, a1, a2, a3


def build():
    base, shoulder, elbow, wrist, grip_base, a1, a2, a3 = arm_geometry()
    s = []
    s.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    s.append('<defs>')
    s.append('<pattern id="grid" width="40" height="40" patternUnits="userSpaceOnUse">'
             '<path d="M 40 0 L 0 0 0 40" fill="none" stroke="#141d2d" stroke-width="0.6"/></pattern>')
    s.append('<marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">'
             f'<path d="M 0 0 L 10 5 L 0 10 z" fill="{ACCENT}"/></marker>')
    s.append('</defs>')
    s.append(f'<rect width="{W}" height="{H}" fill="{BG}"/>')
    s.append(f'<rect width="{W}" height="{H}" fill="url(#grid)"/>')

    # ---------- header
    s.append(text(56, 66, "SO-101 · joint names across the stack", 30, INK, "700"))
    s.append(text(56, 96, "one robot, one set of names — what changes between layers is units, ranges and index order",
                  14, DIM))

    # ---------- left panel: the arm
    px, py, pw, ph = 40, 130, 660, 940
    s.append(f'<rect x="{px}" y="{py}" width="{pw}" height="{ph}" rx="12" fill="{PANEL}" stroke="{EDGE}"/>')
    s.append(text(px + 24, py + 36, "the joints, physical", 13, ACCENT, "700"))

    # the useful headline inside the panel
    s.append(f'<rect x="{px+22}" y="{py+54}" width="{pw-44}" height="86" rx="8" fill="{PANEL2}" stroke="{EDGE}"/>')
    s.append(text(px + 36, py + 78, "the same string everywhere", 11.5, GREEN, "700"))
    s.append(text(px + 36, py + 100, "LeRobot · joint server · arm_bridge · /joint_states ·", 9.8, DIM))
    s.append(text(px + 36, py + 114, "URDF · MJCF · RViz · MuJoCo twin · F710", 9.8, DIM))
    s.append(text(px + 36, py + 132, "motor N in the bus  →  index N-1 in /joint_states", 9.8, AMBER))

    # plinth + riser
    s.append(f'<rect x="{base[0]-64}" y="{base[1]+14}" width="128" height="24" rx="6" fill="#16233a" stroke="{EDGE}"/>')
    s.append(f'<rect x="{base[0]-26}" y="{shoulder[1]+2}" width="52" height="{base[1]-shoulder[1]-2}" rx="5" fill="#1b2b45" stroke="{EDGE}"/>')
    s.append(f'<line x1="{base[0]}" y1="{base[1]-52}" x2="{base[0]}" y2="{base[1]+38}" stroke="{ACCENT}" '
             f'stroke-width="1.1" stroke-dasharray="5,5" opacity="0.7"/>')
    s.append(f'<ellipse cx="{base[0]}" cy="{base[1]}" rx="30" ry="10" fill="none" stroke="{ACCENT}" stroke-width="1.5" opacity="0.85"/>')

    def link(p1, p2, w=26):
        return (f'<line x1="{p1[0]:.1f}" y1="{p1[1]:.1f}" x2="{p2[0]:.1f}" y2="{p2[1]:.1f}" '
                f'stroke="#20304c" stroke-width="{w}" stroke-linecap="round"/>'
                f'<line x1="{p1[0]:.1f}" y1="{p1[1]:.1f}" x2="{p2[0]:.1f}" y2="{p2[1]:.1f}" '
                f'stroke="#2c4266" stroke-width="{w-10}" stroke-linecap="round"/>')

    s.append(link(shoulder, elbow, 34))
    s.append(link(elbow, wrist, 29))
    s.append(link(wrist, grip_base, 22))

    def badge(p, n, color, r=13):
        return (f'<circle cx="{p[0]:.1f}" cy="{p[1]:.1f}" r="{r}" fill="{BG}" stroke="{color}" stroke-width="2.2"/>'
                f'<text x="{p[0]:.1f}" y="{p[1]+4.5:.1f}" font-family="\'JetBrains Mono\',monospace" font-size="11.5" '
                f'fill="{color}" font-weight="700" text-anchor="middle">J{n}</text>')

    s.append(badge(base, 1, ACCENT))
    s.append(badge(shoulder, 2, "#60a5fa", 11.5))
    s.append(badge(elbow, 3, GREEN))
    s.append(badge(wrist, 4, AMBER))

    roll_p = (wrist[0] + 0.40 * (grip_base[0] - wrist[0]), wrist[1] + 0.52 * (grip_base[1] - wrist[1]))
    s.append(f'<circle cx="{roll_p[0]:.1f}" cy="{roll_p[1]:.1f}" r="17" fill="none" stroke="{VIOLET}" '
             f'stroke-width="1.8" stroke-dasharray="4,3"/>')
    s.append(badge(roll_p, 5, VIOLET, 11.5))

    gx, gy = grip_base
    s.append(f'<rect x="{gx-6:.1f}" y="{gy-18:.1f}" width="24" height="9" rx="3" fill="#22314c" stroke="{EDGE}"/>')
    s.append(f'<rect x="{gx-6:.1f}" y="{gy+9:.1f}" width="24" height="9" rx="3" fill="#22314c" stroke="{EDGE}"/>')
    s.append(f'<path d="M {gx+18:.1f} {gy-13:.1f} L {gx+38:.1f} {gy-24:.1f}" stroke="{ROSE}" stroke-width="3" stroke-linecap="round"/>')
    s.append(f'<path d="M {gx+18:.1f} {gy+13:.1f} L {gx+38:.1f} {gy+24:.1f}" stroke="{ROSE}" stroke-width="3" stroke-linecap="round"/>')
    s.append(badge((gx + 26, gy), 6, ROSE, 12))

    # names written in place, no leader lines: the table on the right maps number -> details
    def tag(x, y, n, name, color, anchor="start"):
        return (text(x, y - 2, f"J{n}", 11, color, "700", anchor) +
                text(x, y + 13, name, 11, INK, "700", anchor))

    s.append(text(base[0], base[1] + 74, "J1  shoulder_pan", 12.5, ACCENT, "700", "middle"))
    s.append(text(base[0], base[1] + 90, "base rotation · vertical axis", 9.5, DIM, "normal", "middle"))
    s.append(text(base[0] - 48, shoulder[1] + 2, "J2", 11.5, "#60a5fa", "700", "end"))
    s.append(text(base[0] - 48, shoulder[1] + 17, "shoulder_lift", 11.5, INK, "700", "end"))
    s.append(text(base[0] - 48, shoulder[1] + 31, "shoulder height", 9.5, DIM, "normal", "end"))
    s.append(text(elbow[0] - 4, elbow[1] - 58, "J3", 11.5, GREEN, "700", "middle"))
    s.append(text(elbow[0] - 4, elbow[1] - 43, "elbow_flex", 11.5, INK, "700", "middle"))
    s.append(text(elbow[0] - 4, elbow[1] - 29, "the elbow", 9.5, DIM, "normal", "middle"))
    s.append(text(wrist[0] - 8, wrist[1] - 62, "J4", 11.5, AMBER, "700", "middle"))
    s.append(text(wrist[0] - 8, wrist[1] - 47, "wrist_flex", 11.5, INK, "700", "middle"))
    s.append(text(wrist[0] - 8, wrist[1] - 33, "wrist pitch", 9.5, DIM, "normal", "middle"))
    s.append(text(roll_p[0] - 54, roll_p[1] + 62, "J5", 11.5, VIOLET, "700", "middle"))
    s.append(text(roll_p[0] - 54, roll_p[1] + 77, "wrist_roll", 11.5, INK, "700", "middle"))
    s.append(text(roll_p[0] - 54, roll_p[1] + 91, "forearm twist", 9.5, DIM, "normal", "middle"))
    s.append(text(gx + 22, gy + 58, "J6  gripper", 11.5, ROSE, "700", "middle"))
    s.append(text(gx + 22, gy + 73, "jaw · 0–100 %", 9.5, DIM, "normal", "middle"))

    # bottom note: the numbers that catch people out
    s.append(f'<rect x="{px+22}" y="{py+772}" width="{pw-44}" height="140" rx="8" fill="{PANEL2}" stroke="{EDGE}"/>')
    s.append(text(px + 36, py + 798, "watch these three", 11, AMBER, "700"))
    s.append(text(px + 36, py + 820, "1 · the gripper is not a joint in the driver: it is a", 9.8, DIM))
    s.append(text(px + 36, py + 834, "     percentage, 5 closed and 95 open on this arm", 9.8, DIM))
    s.append(text(px + 36, py + 854, "2 · elbow_flex and wrist_flex are pitch joints; only", 9.8, DIM))
    s.append(text(px + 36, py + 868, "     wrist_roll twists along the forearm", 9.8, DIM))
    s.append(text(px + 36, py + 888, "3 · LeRobot degrees start at the calibration pose,", 9.8, DIM))
    s.append(text(px + 36, py + 902, "     the model starts at zero — they are not the same", 9.8, DIM))

    # ---------- right panel: the table
    tx, ty, tw = 730, 130, 1050
    s.append(f'<rect x="{tx}" y="{ty}" width="{tw}" height="440" rx="12" fill="{PANEL}" stroke="{EDGE}"/>')
    s.append(text(tx + 24, ty + 36, "the same names in every layer", 13, ACCENT, "700"))
    s.append(text(tx + 24, ty + 58, "LeRobot bus · joint server :8002 · arm_bridge · /joint_states · URDF · MJCF · RViz · MuJoCo twin · F710",
                  10.5, DIM))

    cols = [("motor", tx + 24, 60), ("joint name", tx + 92, 190), ("/joint_states", tx + 292, 130),
            ("model range (rad)", tx + 430, 210), ("model range (deg)", tx + 648, 190), ("driver limit", tx + 848, 150)]
    hy = ty + 96
    for label, x, _ in cols:
        s.append(text(x, hy, label, 10, DIM, "700"))
    s.append(f'<line x1="{tx+16}" y1="{hy+12}" x2="{tx+tw-16}" y2="{hy+12}" stroke="{EDGE}"/>')

    row_h = 50
    for i, (n, name, idx, note, lo, hi, lod, hid, drv) in enumerate(JOINTS):
        y = hy + 34 + i * row_h
        if i % 2 == 0:
            s.append(f'<rect x="{tx+12}" y="{y-24}" width="{tw-24}" height="{row_h-4}" rx="6" fill="#0f1828"/>')
        color = [ACCENT, "#60a5fa", GREEN, AMBER, VIOLET, ROSE][i]
        s.append(f'<circle cx="{tx+38}" cy="{y-6}" r="11" fill="none" stroke="{color}" stroke-width="1.6"/>')
        s.append(text(tx + 38, y - 1, str(n), 12, color, "700", "middle"))
        s.append(text(tx + 92, y, name, 14, INK, "700"))
        s.append(text(tx + 92, y + 16, note, 9.5, DIM))
        s.append(text(tx + 292, y, f"idx {idx}", 12.5, ACCENT))
        s.append(text(tx + 292, y + 16, "degrees in, degrees out", 9.5, DIM))
        s.append(text(tx + 430, y, f"{lo:+.5f} … {hi:+.5f}", 11.5, INK))
        s.append(text(tx + 430, y + 16, "MuJoCo / URDF / RViz", 9.5, DIM))
        s.append(text(tx + 648, y, f"{lod:+.1f}° … {hid:+.1f}°", 11.5, INK))
        s.append(text(tx + 848, y, drv, 12, AMBER, "700"))

    # ---------- pipe of layers
    ly = 610
    s.append(f'<rect x="{tx}" y="{ly}" width="{tw}" height="240" rx="12" fill="{PANEL}" stroke="{EDGE}"/>')
    s.append(text(tx + 24, ly + 34, "where the numbers travel", 13, GREEN, "700"))
    s.append(text(tx + 24, ly + 56, "every hop keeps the joint order — only the unit changes", 10.5, DIM))

    hops = [
        ("LeRobot SDK", "SO100Follower · Present_Position", "degrees", GREEN),
        ("joint server", "HTTP :8002 · /read · /write", "degrees", GREEN),
        ("arm_bridge", "/joint_states · 50 Hz", "degrees", ACCENT),
        ("deg2rad node", "converts, nothing else", "radians", VIOLET),
        ("RViz / TF", "so101.urdf · robot_state_publisher", "radians", VIOLET),
        ("MuJoCo twin", "robotstudio_so101 · Windows", "radians", AMBER),
    ]
    bw, gap = 148, 22
    bx0 = tx + 24
    for i, (name, sub, unit, color) in enumerate(hops):
        bx = bx0 + i * (bw + gap)
        s.append(f'<rect x="{bx}" y="{ly+76}" width="{bw}" height="86" rx="8" fill="{PANEL2}" stroke="{color}" stroke-width="1.2"/>')
        s.append(text(bx + 10, ly + 98, name, 11, color, "700"))
        # wrap the sublabel into two short lines
        words = sub.split(" · ")
        for k, wd in enumerate(words[:3]):
            s.append(text(bx + 10, ly + 115 + k * 13, wd[:24], 9, DIM))
        s.append(text(bx + 10, ly + 154, unit, 10, INK, "700"))
        if i < len(hops) - 1:
            s.append(f'<line x1="{bx+bw+3}" y1="{ly+119}" x2="{bx+bw+gap-4}" y2="{ly+119}" '
                     f'stroke="{ACCENT}" stroke-width="1.4" marker-end="url(#arr)"/>')

    s.append(text(tx + 24, ly + 190, "gripper is the odd one: the driver talks 0–100 %, the model -0.17 … +1.75 rad, and 5 = closed, 95 = open on the bench.",
                  10.5, AMBER))
    s.append(text(tx + 24, ly + 212, "/joint_targets carries degrees and tolerates 5 values: with five the server leaves motor 6 untouched — hand-eye sweeps rely on it.",
                  10.5, DIM))

    # ---------- F710 strip
    fy = 880
    s.append(f'<rect x="{tx}" y="{fy}" width="{tw}" height="190" rx="12" fill="{PANEL}" stroke="{EDGE}"/>')
    s.append(text(tx + 24, fy + 34, "F710 teleop · which control moves which joint", 13, ROSE, "700"))
    s.append(text(tx + 24, fy + 56, "joint-step is the default mode; START toggles it to cartesian-feel", 10.5, DIM))
    cw = (tw - 48) / 2
    for i, (n, name, idx, note, lo, hi, lod, hid, drv) in enumerate(JOINTS):
        col = i % 2
        row = i // 2
        x = tx + 24 + col * (cw + 8)
        y = fy + 92 + row * 30
        color = [ACCENT, "#60a5fa", GREEN, AMBER, VIOLET, ROSE][i]
        s.append(text(x, y, f"J{n}", 11, color, "700"))
        s.append(text(x + 26, y, name, 11, INK))
        s.append(text(x + 150, y, F710[n], 10.5, DIM))

    # ---------- legend / footer
    s.append(text(56, 1104, "SO-101 · ADR-0021 · generated by tools/make_joint_names_figure.py", 10, DIM))
    s.append(text(W - 40, 1104, "limits: MJCF so101.xml · driver: lerobot_joint_server.py + f710_teleop_ws.py", 10, DIM, "normal", "end"))
    s.append('</svg>')
    return "".join(s)


if __name__ == "__main__":
    out = "/home/miquel/Projects/Jetson-SO100-OAK-ROS2/docs/so101-joint-names.svg"
    with open(out, "w") as fh:
        fh.write(build())
    print("wrote", out)
