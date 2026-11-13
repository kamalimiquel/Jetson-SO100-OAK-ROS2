"""Wrist camera: a plain USB RGB webcam on the gripper (ADR-0022), as ROS2 topics.

ADR-0022 replaced the OAK-D Lite on the wrist with a UVC webcam, because the jaws work at
10-15 cm and that is a *focus* problem, not a stereo one. This node is the whole driver: open
the device with V4L2, read MJPG frames, publish them compressed.

    /gripper/image/compressed   CompressedImage (jpeg), 10 Hz by default
    /gripper/camera_info        CameraInfo — only when intrinsics are known (see below)

Env:
    WRIST_DEVICE   default /dev/so100_wrist (a udev symlink, so re-enumeration does not matter)
    WRIST_W/H      default 1280x720 (the VF0770's MJPG maximum; the 5M WebCam offers 1920x1080)
    WRIST_FPS      camera-side frame rate, default 15
    WRIST_HZ       publish rate, default 10
    WRIST_CALIB    optional YAML with fx, fy, cx, cy — until it exists no camera_info is
                   published, because a made-up K is worse than none

The frame is `gripper_color_optical_frame` (x right, y down, z forward, REP-103). There is no
static transform to the arm yet: that needs the mount geometry, and it is the piece that turns
this feed into an eye-in-hand measurement. Until then it is a view of the jaws, which is what
ADR-0022 asked for.
"""

import array
import os
import time

import cv2
import rclpy
from rclpy.node import Node

try:
    import yaml
except ImportError:            # the image may not ship PyYAML; the intrinsics are optional
    yaml = None
from sensor_msgs.msg import CameraInfo, CompressedImage

DEVICE = os.environ.get("WRIST_DEVICE", "/dev/so100_wrist")
W = int(os.environ.get("WRIST_W", "1280"))
H = int(os.environ.get("WRIST_H", "720"))
FPS = int(os.environ.get("WRIST_FPS", "15"))
HZ = float(os.environ.get("WRIST_HZ", "10"))
CALIB = os.environ.get("WRIST_CALIB", "")
# Two-arm (Phase 2): WRIST_TOPIC_NS renames the topics, WRIST_FRAME the frame id,
# WRIST_NODE_NAME the node, so a second instance can run in the same container
# (e.g. /dev/arm2_wrist -> /arm2/gripper/image/compressed, arm2_gripper_color_optical_frame).
NS = os.environ.get("WRIST_TOPIC_NS", "/gripper").rstrip("/")
FRAME = os.environ.get("WRIST_FRAME", "gripper_color_optical_frame")
NODE_NAME = os.environ.get("WRIST_NODE_NAME", "wrist_cam")
JPEG_QUALITY = 80


class WristCam(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.pub_img = self.create_publisher(CompressedImage, "%s/image/compressed" % NS, 5)
        self.pub_info = self.create_publisher(CameraInfo, "%s/camera_info" % NS, 5)
        self.k = self._load_calib()
        self.cap = None
        self.warned = False
        self.frames = 0
        self.t_open = 0.0
        self.get_logger().info(
            "wrist camera: %s %dx%d@%d fps -> %s/image/compressed at %.0f Hz%s"
            % (DEVICE, W, H, FPS, NS, HZ, "" if self.k else " (no intrinsics yet: no camera_info)"))
        self.timer = self.create_timer(1.0 / max(HZ, 1.0), self.tick)

    def _load_calib(self):
        if yaml is None:
            if CALIB:
                self.get_logger().warn("wrist camera: PyYAML missing, ignoring %s" % CALIB)
            return None
        if not CALIB or not os.path.exists(CALIB):
            return None
        try:
            with open(CALIB) as fh:
                doc = yaml.safe_load(fh) or {}
            k = (float(doc["fx"]), float(doc["fy"]), float(doc["cx"]), float(doc["cy"]))
            self.get_logger().info("wrist camera: intrinsics from %s" % CALIB)
            return k
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("wrist camera: cannot read %s (%s)" % (CALIB, exc))
            return None

    def open_camera(self):
        """(Re)open the device. V4L2 has to be asked for MJPG explicitly, otherwise the
        driver falls back to raw YUYV and the USB link becomes the bottleneck."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
        if not cap.isOpened():
            if not self.warned:
                self.warned = True
                self.get_logger().warn(
                    "wrist camera: cannot open %s — is the webcam plugged in and the udev "
                    "symlink present? (scripts/provision-jetson.sh adds the rule)" % DEVICE)
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        cap.set(cv2.CAP_PROP_FPS, FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.warned = False
        self.t_open = time.time()
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        name = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4))
        self.get_logger().info(
            "wrist camera: %s open at %dx%d %.4s (asked for %dx%d MJPG)"
            % (DEVICE, int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
               int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), name, W, H))
        return cap

    def tick(self):
        if self.cap is None:
            self.cap = self.open_camera()
            if self.cap is None:
                return
        ok, frame = self.cap.read()
        if not ok or frame is None:
            # a pulled cable or a UVC hiccup: reopen, do not die
            self.get_logger().warn("wrist camera: read failed, reopening")
            self.cap = self.open_camera()
            return
        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if not ok:
            return
        stamp = self.get_clock().now().to_msg()
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME
        msg.format = "jpeg"
        msg.data = array.array("B", jpg.tobytes())      # rclpy's fast path for uint8[]
        self.pub_img.publish(msg)
        if self.k is not None:
            info = CameraInfo()
            info.header.stamp = stamp
            info.header.frame_id = FRAME
            info.width, info.height = frame.shape[1], frame.shape[0]
            fx, fy, cx, cy = self.k
            info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            info.distortion_model = "plumb_bob"
            self.pub_info.publish(info)
        self.frames += 1
        if self.frames % 300 == 0:
            self.get_logger().info("wrist camera: %d frames, %.1f fps over the last window"
                                   % (self.frames, 300.0 / max(1e-3, time.time() - self.t_open)))


def main():
    rclpy.init()
    node = WristCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.cap is not None:
        node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
