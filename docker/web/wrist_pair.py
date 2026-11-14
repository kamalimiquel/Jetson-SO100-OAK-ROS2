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
TILE_W = 640                                # fixed slot per camera, see tile()
TILE_H = 480
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
    """One tile fitted into a fixed TILE_W x TILE_H slot, with a caption bar.

    The slot is a constant size on purpose: RViz rebuilds the Ogre texture whenever the message's
    dimensions change, and a canvas that shifts by a few pixels between frames leaves the panel on
    "No Image". Same reason for the multiple-of-4 padding.
    """
    out = Image.new("RGB", (TILE_W, TILE_H + LABEL_H), (11, 18, 32))
    img = None
    if jpeg:
        try:
            img = Image.open(io.BytesIO(jpeg)).convert("RGB")
            scale = min(TILE_W / img.width, TILE_H / img.height)
            img = img.resize((max(1, int(img.width * scale)), max(1, int(img.height * scale))),
                             Image.BILINEAR)
        except Exception:                                     # noqa: BLE001
            img = None
    if img is None:
        d = ImageDraw.Draw(out)
        d.text((16, LABEL_H + TILE_H // 2 - 8), "waiting for %s" % label, fill=(140, 150, 165))
    else:
        out.paste(img, ((TILE_W - img.width) // 2, LABEL_H + (TILE_H - img.height) // 2))
    ImageDraw.Draw(out).text((10, 7), label, fill=(232, 237, 245))
    return out


def compose(left, right):
    a = tile(left, "arm 1  %s" % LEFT)
    b = tile(right, "arm 2  %s" % RIGHT)
    canvas = Image.new("RGB", (TILE_W * 2, TILE_H + LABEL_H), (11, 18, 32))
    canvas.paste(a, (0, 0))
    canvas.paste(b, (TILE_W, 0))
    ImageDraw.Draw(canvas).line([(TILE_W, 0), (TILE_W, canvas.height)],
                                fill=(38, 48, 66), width=2)
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
