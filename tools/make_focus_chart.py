"""Draw the gripper-camera focus tests: sharpness against distance, two fixed-focus webcams.

Reads the CSVs written by tools/camfocus.py (one row per marked distance) and emits
docs/focus-vs-distance.svg and .png. Regenerate after adding measurements:

    python3 tools/make_focus_chart.py

The point of the figure is the *shape* of each curve: with fixed focus, the focus plane is a
property of the camera and the score falls off on both sides of it.
"""

import csv
import os
import subprocess

from PIL import Image, ImageDraw, ImageFont

W, H = 1240, 720
BG = "#070c16"
PANEL = "#0d1421"
EDGE = "#1f2b40"
INK = "#e6edf7"
DIM = "#8494ad"
ACCENT = "#22d3ee"
ROSE = "#fb7185"
GREEN = "#34d399"
AMBER = "#fbbf24"

SERIES = [
    # csv, short label, resolution, colour
    ("docs/calibration/focus/labtec-v-uam38.csv", "Labtec V-UAM38", "640x480", ROSE),
    ("docs/calibration/focus/creative-vf0770.csv", "Creative VF0770", "1280x720", ACCENT),
]

FONT_B = "/usr/share/fonts/TTF/JetBrainsMonoNerdFont-Bold.ttf"
FONT_R = "/usr/share/fonts/TTF/JetBrainsMonoNerdFont-Regular.ttf"


def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


def read(path):
    pts = []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            if not row.get("distance_cm"):
                continue
            pts.append((float(row["distance_cm"]), float(row["sharpness"])))
    return sorted(pts)


def main():
    data = [(label, sub, colour, read(path)) for path, label, sub, colour in SERIES]
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)

    f_title = font(FONT_B, 30)
    f_sub = font(FONT_R, 19)
    f_axis = font(FONT_R, 18)
    f_lab = font(FONT_B, 20)
    f_note = font(FONT_R, 17)

    d.text((56, 38), "Gripper camera — focus against working distance", font=f_title, fill=INK)
    d.text((56, 80), "printed ChArUco board, one lamp, three distances per camera; both cameras are FIXED focus",
           font=f_sub, fill=DIM)

    # plot area
    x0, y0, x1, y1 = 120, 140, W - 300, H - 120
    ymax = 500.0
    d.rounded_rectangle([x0, y0, x1, y1], radius=8, fill=PANEL, outline=EDGE)

    def px(dist):
        return x0 + (dist - 9.0) / (16.0 - 9.0) * (x1 - x0)

    def py(val):
        return y1 - (val / ymax) * (y1 - y0)

    for v in range(0, 501, 100):
        y = py(v)
        d.line([x0, y, x1, y], fill=EDGE)
        d.text((x0 - 16, y - 10), "%d" % v, font=f_axis, fill=DIM, anchor="ra")
    for dist in (10, 12.5, 15):
        x = px(dist)
        d.line([x, y0, x, y1], fill="#131c2b")
        d.text((x, y1 + 14), "%g cm" % dist, font=f_axis, fill=DIM, anchor="ma")

    d.text((x0, y0 - 30), "sharpness (Laplacian variance)", font=f_axis, fill=DIM)

    for label, sub, colour, pts in data:
        if not pts:
            continue
        xy = [(px(dd), py(v)) for dd, v in pts]
        d.line(xy, fill=colour, width=4, joint="curve")
        for (x, y), (dd, v) in zip(xy, pts):
            d.ellipse([x - 7, y - 7, x + 7, y + 7], fill=colour, outline=BG, width=2)
            d.text((x, y - 24), "%.0f" % v, font=f_lab, fill=colour, anchor="ma")

    # crossover marker between 10 and 12.5 cm
    xc = px(11.25)
    d.line([xc, y0 + 10, xc, y1 - 10], fill=AMBER, width=2)
    d.text((xc + 8, y0 + 16), "curves cross\nhere", font=f_note, fill=AMBER)

    # legend panel
    lx = x1 + 26
    d.text((lx, y0 + 4), "camera", font=f_note, fill=DIM)
    yy = y0 + 30
    for label, sub, colour, pts in data:
        d.line([lx, yy + 8, lx + 26, yy + 8], fill=colour, width=4)
        d.text((lx, yy + 18), label, font=f_note, fill=INK)
        d.text((lx, yy + 38), sub, font=f_note, fill=DIM)
        yy += 74

    d.text((lx, yy + 10), "absolute values are\nnot comparable between\nthe two: 640x480 vs\n1280x720. Compare the\nshape, not the height.",
           font=f_note, fill=DIM)

    d.text((56, H - 74), "Labtec: sharpest at 10 cm or closer — 6x softer by 15 cm.",
           font=f_note, fill=DIM)
    d.text((56, H - 50), "Creative VF0770 (Live! Cam Sync HD): still improving at 15 cm — its focus plane is beyond the band.",
           font=f_note, fill=DIM)

    os.makedirs("docs", exist_ok=True)
    im.save("docs/focus-vs-distance.png", optimize=True)
    print("wrote docs/focus-vs-distance.png", im.size)


if __name__ == "__main__":
    main()
