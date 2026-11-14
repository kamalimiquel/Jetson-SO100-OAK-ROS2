#!/usr/bin/env python3
"""Both wrist cameras in one image, for the RViz panel (:not a port — a node).

RViz on this board draws at most **two** compressed-image panels (llvmpipe, no GPU path), and the
third one sits on "No Image" for ever. With two arms there are three camera views to show, so the two
jaw cameras are composed into a single 2-up frame here and RViz keeps two panels: the scene colour
camera and this pair.

Publishes `/gripper/pair/compressed` (sensor_msgs/CompressedImage, jpeg) at up to 8 Hz. It is started
and stopped together with RViz by the shutdown server on :8099 — nothing decodes anything while the
RViz tab is closed.

    python3 wrist_pair.py [--out /gripper/pair/compressed] [--hz 8]
"""

import argparse
import io
import threading
import time

import numpy as np
import rclpy
from PIL import Image, ImageDraw
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

LEFT = "/gripper/image/compressed"          # arm 1 (Labtec)
RIGHT = "/arm2/gripper/image/compressed"    # arm 2 (Creative VF0770)
TILE_H = 480                                # both tiles are scaled to this height
LABEL_H = 26


class Pair(Node):
    def __init__(self, out_topic, hz):
        super().__init__("wrist_pair")
        self.pub = self.create_publisher(CompressedImage, out_topic, 5)
        self.pub_rate = hz
        self.frames = {LEFT: None, RIGHT: None}
        self.lock = threading.Lock()
        self.dirty = threading.Event()
        self.create_subscription(CompressedImage, LEFT, lambda m: self.on(m, LEFT), 5)
        self.create_subscription(CompressedImage, RIGHT, lambda m: self.on(m, RIGHT), 5)
        self.create_timer(1.0 / hz, self.publish)
        self.get_logger().info("wrist pair: %s + %s -> %s" % (LEFT, RIGHT, out_topic))

    def on(self, msg, key):
        with self.lock:
            self.frames[key] = bytes(msg.data)
        self.dirty.set()

    def publish(self):
        if not self.dirty.is_set():
            return
        self.dirty.clear()
        with self.lock:
            left, right = self.frames[LEFT], self.frames[RIGHT]
        pair = compose(left, right)
        if pair is None:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        # `base` is a frame that exists in TF: the RViz image panel only draws a message whose
        # frame it can resolve, and a made-up frame name leaves the panel on "No Image".
        msg.header.frame_id = "base"
        msg.format = "jpeg"
        msg.data = pair
        self.pub.publish(msg)


def tile(jpeg, label):
    """One tile scaled to TILE_H, with a caption bar. A missing frame becomes a placeholder."""
    if jpeg:
        try:
            img = Image.open(io.BytesIO(jpeg)).convert("RGB")
            w = max(1, int(img.width * TILE_H / img.height))
            img = img.resize((w, TILE_H), Image.BILINEAR)
        except Exception:                                     # noqa: BLE001
            img = None
    else:
        img = None
    if img is None:
        img = Image.new("RGB", (int(TILE_H * 4 / 3), TILE_H), (18, 22, 30))
        d = ImageDraw.Draw(img)
        d.text((16, TILE_H // 2 - 8), "waiting for %s" % label, fill=(140, 150, 165))
    out = Image.new("RGB", (img.width, TILE_H + LABEL_H), (11, 18, 32))
    out.paste(img, (0, LABEL_H))
    ImageDraw.Draw(out).text((10, 7), label, fill=(232, 237, 245))
    return out


def compose(left, right):
    a = tile(left, "arm 1  %s" % LEFT)
    b = tile(right, "arm 2  %s" % RIGHT)
    # even (multiple of 4) dimensions: Ogre builds the texture from the JPEG and an odd width makes
    # the upload fail silently, which is exactly what the RViz panel shows as "No Image"
    w = ((a.width + b.width) + 3) // 4 * 4
    h = (a.height + 3) // 4 * 4
    canvas = Image.new("RGB", (w, h), (11, 18, 32))
    canvas.paste(a, (0, 0))
    canvas.paste(b, (a.width, 0))
    ImageDraw.Draw(canvas).line([(a.width, 0), (a.width, a.height)], fill=(38, 48, 66), width=2)
    buf = io.BytesIO()
    canvas.save(buf, format="JPEG", quality=80)
    return buf.getvalue()


def main():
    ap = argparse.ArgumentParser(description="compose both wrist cameras into one image")
    ap.add_argument("--out", default="/gripper/pair/compressed")
    ap.add_argument("--hz", type=float, default=8.0)
    args = ap.parse_args()
    rclpy.init()
    node = Pair(args.out, args.hz)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:                                     # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
