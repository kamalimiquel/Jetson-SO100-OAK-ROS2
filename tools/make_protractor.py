#!/usr/bin/env python3
"""Printable angle scales for the two joints an inclinometer cannot reach.

`shoulder_pan` turns about the vertical axis, so gravity gives no reference — the scale goes flat on
the table, centred on the pan axis, with a thin pointer taped to the rotating link. `wrist_roll` turns
about the forearm's own axis, where a protractor card is read against a fixed mark, or through the
wrist camera if the card is taped to the gripper.

Everything is drawn at a known size and the PDF is A4 at 1:1, so a print without "fit to page" is
already in millimetres:

    python3 tools/make_protractor.py --out docs/        # writes table + wrist sheets (png + pdf)
    python3 tools/make_protractor.py --check docs/      # re-measures the drawn ticks from the PNG

Sizes: the table sheet is a 190 mm scale with 1 deg ticks (a 2 deg print error is 4 mm at the rim, so
read to the nearest half degree); the wrist disk is 80 mm with 5 deg ticks and a bold index line.
"""

import argparse
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

PX_PER_MM = 12          # 305 dpi: a 0.5 mm tick is 6 px, still crisp after printing
A4_MM = (210, 297)


def font(size_mm):
    px = int(size_mm * PX_PER_MM)
    for path in ("/usr/share/fonts/TTF/DejaVuSans-Bold.ttf",
                 "/usr/share/fonts/TTF/DejaVuSans.ttf"):
        if os.path.exists(path):
            return ImageFont.truetype(path, px)
    return ImageFont.load_default()


def mm(v):
    return int(round(v * PX_PER_MM))


def sheet(width_mm, height_mm, bg="white"):
    img = Image.new("RGB", (mm(width_mm), mm(height_mm)), bg)
    return img, ImageDraw.Draw(img)


def angle_scale(d, cx, cy, radius_mm, tick_minor_deg, tick_major_deg, label_every_deg,
                start_deg=0, span_deg=360, colour="black"):
    """Draw a protractor: ticks inward from the rim, labels outside it."""
    r_out = mm(radius_mm)
    r_minor = mm(radius_mm - 2)
    r_mid = mm(radius_mm - 5)
    r_major = mm(radius_mm - 9)
    f = font(3.2)
    for a in range(start_deg, start_deg + span_deg + 1):
        rad = math.radians(a - 90)          # 0 deg at 12 o'clock, clockwise
        x, y = math.cos(rad), math.sin(rad)
        if a % tick_major_deg == 0:
            d.line([(cx + r_major * x, cy + r_major * y), (cx + r_out * x, cy + r_out * y)],
                   fill=colour, width=max(2, mm(0.35)))
        elif a % tick_minor_deg == 0:
            d.line([(cx + r_mid * x, cy + r_mid * y), (cx + r_out * x, cy + r_out * y)],
                   fill=colour, width=max(1, mm(0.15)))
        if a % label_every_deg == 0:
            lx, ly = cx + mm(radius_mm + 5) * x, cy + mm(radius_mm + 5) * y
            txt = str(a)
            w = d.textlength(txt, font=f)
            d.text((lx - w / 2, ly - mm(1.8)), txt, font=f, fill=colour)


def make_table_sheet(path_png, path_pdf):
    img, d = sheet(*A4_MM)
    cx, cy = mm(A4_MM[0] / 2), mm(120)
    # the labels sit radius+5 mm out, so the scale has to leave room for them on the widest axis
    radius = min(95, A4_MM[0] / 2 - 10, 120 - 10)
    d.ellipse([cx - mm(radius), cy - mm(radius), cx + mm(radius), cy + mm(radius)],
              outline="black", width=mm(0.4))
    angle_scale(d, cx, cy, radius, 1, 10, 10, span_deg=359)  # 360 would sit on top of 0
    # crosshair: the axis of shoulder_pan goes here
    d.line([(cx - mm(12), cy), (cx + mm(12), cy)], fill="red", width=mm(0.3))
    d.line([(cx, cy - mm(12)), (cx, cy + mm(12))], fill="red", width=mm(0.3))
    d.ellipse([cx - mm(2), cy - mm(2), cx + mm(2), cy + mm(2)], outline="red", width=mm(0.3))
    f = font(4.2)
    d.text((mm(12), mm(12)), "SO-100 · escala de shoulder_pan", font=f, fill="black")
    f2 = font(3.2)
    d.text((mm(12), mm(20), ),
           "Centra la cruz roja en el eje del hombro, con 0° mirando al frente del banco.", font=f2)
    d.text((mm(12), mm(26)),
           "Cinta un puntero fino al eslabón que gira (no a la base) y apunta al borde interior.",
           font=f2)
    d.text((mm(12), mm(A4_MM[1] - 16)),
           "Imprimir al 100 % (sin 'ajustar a la página'). Marca de escala: 100 mm entre las dos muescas de abajo.",
           font=f2)
    # 100 mm calibration bar
    y = mm(A4_MM[1] - 12)
    d.line([(mm(20), y), (mm(120), y)], fill="black", width=mm(0.5))
    for x in (20, 120):
        d.line([(mm(x), y - mm(3)), (mm(x), y + mm(3))], fill="black", width=mm(0.5))
    img.save(path_png, dpi=(int(PX_PER_MM * 25.4),) * 2)
    img.save(path_pdf, "PDF", resolution=float(PX_PER_MM * 25.4))


def make_wrist_sheet(path_png, path_pdf):
    img, d = sheet(120, 120)
    cx, cy, radius = mm(60), mm(60), 40
    d.ellipse([cx - mm(radius), cy - mm(radius), cx + mm(radius), cy + mm(radius)],
              outline="black", width=mm(0.4))
    angle_scale(d, cx, cy, radius, 5, 15, 15)
    # the index line: this is what you read against the fixed mark
    d.line([(cx, cy - mm(radius)), (cx, cy + mm(6))], fill="red", width=mm(0.9))
    d.ellipse([cx - mm(3), cy - mm(3), cx + mm(3), cy + mm(3)], outline="red", width=mm(0.4))
    f = font(3.6)
    d.text((mm(8), mm(8)), "SO-100 · disco de wrist_roll", font=f, fill="black")
    f2 = font(2.8)
    d.text((mm(8), mm(14)), "Pega este disco al eslabón que gira con wrist_roll,", font=f2)
    d.text((mm(8), mm(18.5)), "centrado en su eje, y lee la línea roja contra una", font=f2)
    d.text((mm(8), mm(23)), "marca fija en el antebrazo. 0° = la línea roja.", font=f2)
    d.text((mm(8), mm(112)), "Imprimir al 100 %. Disco de 80 mm de diámetro.", font=f2)
    img.save(path_png, dpi=(int(PX_PER_MM * 25.4),) * 2)
    img.save(path_pdf, "PDF", resolution=float(PX_PER_MM * 25.4))


def check(directory):
    """Re-measure the drawn geometry from the PNG: the print is only as good as the file."""
    ok = True
    for name, dia_mm in (("protractor-table.png", 220), ("protractor-wrist.png", 80)):
        path = os.path.join(directory, name)
        if not os.path.exists(path):
            print("missing %s" % path)
            ok = False
            continue
        im = Image.open(path)
        print("%-24s %4dx%-4d px  %5.1f mm at %.0f dpi"
              % (name, im.width, im.height,
                 im.width / PX_PER_MM, PX_PER_MM * 25.4))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=".")
    ap.add_argument("--check", nargs="?", const=".")
    args = ap.parse_args()
    if args.check:
        return check(args.check)
    os.makedirs(args.out, exist_ok=True)
    make_table_sheet(os.path.join(args.out, "protractor-table.png"),
                     os.path.join(args.out, "protractor-table.pdf"))
    make_wrist_sheet(os.path.join(args.out, "protractor-wrist.png"),
                     os.path.join(args.out, "protractor-wrist.pdf"))
    print("wrote protractor-table.{png,pdf} (220 mm, 1 deg ticks) and "
          "protractor-wrist.{png,pdf} (80 mm disk, 5 deg ticks) in %s" % args.out)
    print("print at 100 %%; the table sheet carries a 100 mm bar to prove the scale")
    return 0


if __name__ == "__main__":
    sys.exit(main())
