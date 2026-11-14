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
SCENE = "/oak/scene/color/compressed"       # the OAK, for context
OUT = "/gripper/cameras/compressed"
TILE_W = 426                                # slot per wrist camera, see tile()
TILE_H = 320
SCENE_W = 852                               # scene spans both wrist slots
LABEL_H = 26


class Pair(Node):
    def __init__(self, out_topic, hz):
        super().__init__("wrist_pair")
        self.pub = self.create_publisher(CompressedImage, out_topic, 5)
        self.pub_rate = hz
        self.frames = {LEFT: None, RIGHT: None, SCENE: None}
        self.lock = threading.Lock()
        self.dirty = threading.Event()
        for topic in (LEFT, RIGHT, SCENE):
            self.create_subscription(CompressedImage, topic, lambda m, t=topic: self.on(m, t), 5)
        self.create_timer(1.0 / hz, self.publish)
        self.get_logger().info("cameras: %s + %s + %s -> %s"
                               % (SCENE, LEFT, RIGHT, out_topic))

    def on(self, msg, key):
        with self.lock:
            self.frames[key] = bytes(msg.data)
        self.dirty.set()

    def publish(self):
        if not self.dirty.is_set():
            return
        self.dirty.clear()
        if not rclpy.ok():                       # a SIGTERM raced us: leave quietly, no traceback
            raise SystemExit(0)
        if self.pub.get_subscription_count() == 0:
            return                                # nobody is watching (RViz closed): cost zero
        with self.lock:
            scene, left, right = self.frames[SCENE], self.frames[LEFT], self.frames[RIGHT]
        pair = compose(scene, left, right)
        if pair is None:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        # `base` is a frame that exists in TF: the RViz image panel only draws a message whose
        # frame it can resolve, and a made-up frame name leaves the panel on "No Image".
        msg.header.frame_id = "base"
        msg.format = "jpeg"
        msg.data = pair
        try:
            self.pub.publish(msg)
        except Exception as exc:                 # noqa: BLE001 — context torn down mid-publish
            self.get_logger().warn("publish skipped: %s" % exc)
            raise SystemExit(0)


def tile(jpeg, label, w=None, h=None):
    """One tile fitted into a fixed TILE_W x TILE_H slot, with a caption bar.

    The slot is a constant size on purpose: RViz rebuilds the Ogre texture whenever the message's
    dimensions change, and a canvas that shifts by a few pixels between frames leaves the panel on
    "No Image". Same reason for the multiple-of-4 padding.
    """
    tw, th = w or TILE_W, h or TILE_H
    out = Image.new("RGB", (tw, th + LABEL_H), (11, 18, 32))
    img = None
    if jpeg:
        try:
            img = Image.open(io.BytesIO(jpeg)).convert("RGB")
            scale = min(tw / img.width, th / img.height)
            img = img.resize((max(1, int(img.width * scale)), max(1, int(img.height * scale))),
                             Image.BILINEAR)
        except Exception:                                     # noqa: BLE001
            img = None
    if img is None:
        d = ImageDraw.Draw(out)
        d.text((16, LABEL_H + th // 2 - 8), "waiting for %s" % label, fill=(140, 150, 165))
    else:
        out.paste(img, ((tw - img.width) // 2, LABEL_H + (th - img.height) // 2))
    ImageDraw.Draw(out).text((10, 7), label, fill=(232, 237, 245))
    return out


def compose(scene, left, right):
    """Scene colour camera on top, the two jaws below it — all three in ONE image.

    RViz on this board renders only the first image display it is given (a second one sits on
    "No Image" whatever its size or rate, and the 3-D view starves while it tries), so the three
    camera views are composed here instead of being three panels.
    """
    top = tile(scene, "scene  %s" % SCENE, SCENE_W, int(SCENE_W * 9 / 16))
    a = tile(left, "arm 1  %s" % LEFT)
    b = tile(right, "arm 2  %s" % RIGHT)
    canvas = Image.new("RGB", (SCENE_W, top.height + a.height), (11, 18, 32))
    canvas.paste(top, (0, 0))
    canvas.paste(a, (0, top.height))
    canvas.paste(b, (TILE_W, top.height))
    d = ImageDraw.Draw(canvas)
    d.line([(TILE_W, top.height), (TILE_W, canvas.height)], fill=(38, 48, 66), width=2)
    d.line([(0, top.height), (canvas.width, top.height)], fill=(38, 48, 66), width=2)
    buf = io.BytesIO()
    canvas.save(buf, format="JPEG", quality=80)
    return buf.getvalue()


def main():
    ap = argparse.ArgumentParser(description="compose both wrist cameras into one image")
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--hz", type=float, default=4.0)
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
