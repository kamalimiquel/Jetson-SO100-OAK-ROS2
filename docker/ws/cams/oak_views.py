"""Make the OAK streams viewable in RViz: colourised depth + normalised confidence.

RViz's Image display cannot colormap a 16UC1 depth image, and a raw confidence map
is mostly flat grey. This node republishes both as JPEG CompressedImage, so the RViz
Image displays (Transport Hint: compressed) just work — same pattern as the colour
stream, and cheap enough for the llvmpipe desktop.

    /oak/scene/depth        (16UC1)  ->  /oak/scene/depth/color/compressed   TURBO
    /oak/scene/confidence             ->  /oak/scene/confidence/compressed   grey
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage

FPS = 5.0
CLIP_M = 2.0          # depth range shown: 0 .. 2 m


class StreamsToJpeg(Node):
    def __init__(self):
        super().__init__("oak_stream_views")
        self.pub_depth = self.create_publisher(CompressedImage, "/oak/scene/depth/color/compressed", 5)
        self.pub_conf = self.create_publisher(CompressedImage, "/oak/scene/confidence/compressed", 5)
        self.create_subscription(Image, "/oak/scene/depth", self.on_depth, 5)
        self.create_subscription(Image, "/oak/scene/confidence", self.on_conf, 5)
        self.last = {"depth": 0.0, "conf": 0.0}
        self.get_logger().info("depth/confidence JPEG views at %.1f Hz" % FPS)

    # ---------------------------------------------------------------- helpers
    @staticmethod
    def _arr(msg):
        if msg.encoding in ("16UC1", "mono16"):
            a = np.frombuffer(bytes(msg.data), dtype=np.uint16)
        else:
            a = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        return a.reshape(msg.height, msg.width)

    def _send(self, pub, img, stamp):
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        pub.publish(msg)

    def _due(self, key):
        now = time.time()
        if now - self.last[key] < 1.0 / FPS:
            return False
        self.last[key] = now
        return True

    # ------------------------------------------------------------- callbacks
    def on_depth(self, msg: Image):
        if not self._due("depth"):
            return
        try:
            d = self._arr(msg).astype(np.float32)
        except Exception:  # noqa: BLE001
            return
        if msg.encoding not in ("16UC1", "mono16"):
            d = d * 65535.0                       # proportional depth without factory calib
        # millimetres -> metres, unknown (0) shown as black
        metres = np.where(d > 0, d / 1000.0, np.nan)
        norm = np.clip(metres / CLIP_M, 0.0, 1.0)
        grey = np.nan_to_num(norm * 255.0, nan=0.0).astype(np.uint8)
        colour = cv2.applyColorMap(grey, cv2.COLORMAP_TURBO)
        colour[np.isnan(metres)] = (0, 0, 0)
        self._send(self.pub_depth, colour, msg.header.stamp)

    def on_conf(self, msg: Image):
        if not self._due("conf"):
            return
        try:
            c = self._arr(msg).astype(np.float32)
        except Exception:  # noqa: BLE001
            return
        hi = float(c.max()) or 1.0
        grey = (c / hi * 255.0).astype(np.uint8)
        self._send(self.pub_conf, cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR), msg.header.stamp)


def main():
    rclpy.init()
    node = StreamsToJpeg()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
