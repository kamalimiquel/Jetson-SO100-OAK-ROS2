#!/usr/bin/env python3
"""OAK camera ROS2 node (ADR-0007/0008) — image python3.8 + depthai 2.33 cp38 wheel.

One node, one pipeline per device, namespaced:
    /oak/scene/color/compressed   sensor_msgs/CompressedImage (JPEG)
    /oak/scene/color/camera_info  CameraInfo
    /oak/scene/depth              Image 16UC1 (mm with factory calib; right-camera frame)
    /oak/scene/depth/camera_info  CameraInfo
(same set under /oak/gripper when a second device is present)

Env: OAK_SCENE_MXID (default 184430101111F90F00), OAK_GRIPPER_MXID (default:
     any other device). A device without factory stereo calibration still
     streams — depth is then proportional, not millimetres (logged).
"""

import os

import cv2
import depthai as dai
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

SCENE_MXID = os.environ.get("OAK_SCENE_MXID", "184430101111F90F00")
GRIPPER_MXID = os.environ.get("OAK_GRIPPER_MXID", "")
PUBLISH_HZ = 15.0


def build_pipeline() -> dai.Pipeline:
    p = dai.Pipeline()
    color = p.createColorCamera()
    color.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    color.setInterleaved(False)
    color.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    xcolor = p.createXLinkOut()
    xcolor.setStreamName("color")
    color.video.link(xcolor.input)

    left = p.createMonoCamera()
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    right = p.createMonoCamera()
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    stereo = p.createStereoDepth()
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    xdepth = p.createXLinkOut()
    xdepth.setStreamName("depth")
    stereo.depth.link(xdepth.input)
    return p


def camera_info(fx, fy, cx, cy, w, h, frame_id):
    info = CameraInfo()
    info.width = w
    info.height = h
    info.header.frame_id = frame_id
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    info.distortion_model = "plumb_bob"
    return info


class OakNode(Node):
    def __init__(self):
        super().__init__("oak_cams")
        self.bridge = CvBridge()
        self.tfb = StaticTransformBroadcaster(self)
        self.devices = self._discover()
        if not self.devices:
            self.get_logger().error("no OAK devices — check cabling/udev")
            raise SystemExit(1)

        self.streams = []  # (ns, queue_color, queue_depth, info dicts, t_last)
        for ns, info in self.devices:
            pipe = build_pipeline()
            device = dai.Device(pipe, info)
            qc = device.getOutputQueue("color", 2, False)
            qd = device.getOutputQueue("depth", 2, False)
            calib = device.readCalibration()  # CalibrationHandler (empty if no factory calib)
            color_i = calib.getDefaultIntrinsics(dai.CameraBoardSocket.RGB) if calib else None
            right_i = calib.getDefaultIntrinsics(dai.CameraBoardSocket.RIGHT) if calib else None
            color_ok = color_i is not None and color_i[0] > 0
            right_ok = right_i is not None and right_i[0] > 0
            self.get_logger().info(
                f"{ns}: device {info.getMxId()} — factory calib: "
                f"color={'ok' if color_ok else 'MISSING'} stereo={'ok' if right_ok else 'MISSING'}"
            )
            self._publish_static_tfs(ns)
            self.streams.append({
                "ns": ns, "qc": qc, "qd": qd, "device": device,
                "color_info": camera_info(
                    color_i[0], color_i[1], color_i[2], color_i[3],
                    1920, 1080, f"{ns}_color_optical_frame") if color_ok else None,
                "depth_info": camera_info(
                    right_i[0], right_i[1], right_i[2], right_i[3],
                    1280, 720, f"{ns}_right_optical_frame") if right_ok else None,
            })
        self.create_timer(1.0 / PUBLISH_HZ, self.tick)

    def _discover(self):
        found = []
        for info in dai.Device.getAllAvailableDevices():
            mxid = info.getMxId()
            if mxid == SCENE_MXID:
                found.append(("scene", info))
            elif mxid == GRIPPER_MXID or not GRIPPER_MXID:
                found.append(("gripper", info))
        found.sort(key=lambda t: 0 if t[0] == "scene" else 1)
        # De-duplicate namespaces if the same device matched twice.
        seen, out = set(), []
        for ns, info in found:
            if ns in seen:
                continue
            seen.add(ns)
            out.append((ns, info))
        return out

    def _publish_static_tfs(self, ns):
        for child in ("color_optical_frame", "right_optical_frame"):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f"{ns}_camera_link"
            t.child_frame_id = f"{ns}_{child}"
            t.transform.rotation.w = 1.0
            self.tfb.sendTransform(t)

    def tick(self):
        for s in self.streams:
            ns = s["ns"]
            color_frames = s["qc"].tryGetAll()
            depth_frames = s["qd"].tryGetAll()
            if color_frames:
                frame = color_frames[-1].getCvFrame()
                ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                if ok:
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = f"{ns}_color_optical_frame"
                    msg.format = "jpeg"
                    msg.data = jpg.tobytes()
                    self._pub(f"/oak/{ns}/color/compressed", CompressedImage).publish(msg)
                    if s["color_info"]:
                        s["color_info"].header.stamp = msg.header.stamp
                        self._pub(f"/oak/{ns}/color/camera_info", CameraInfo).publish(s["color_info"])
            if depth_frames:
                depth = depth_frames[-1].getFrame()
                msg = self.bridge.cv2_to_imgmsg(depth, encoding="mono16")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f"{ns}_right_optical_frame"
                self._pub(f"/oak/{ns}/depth", Image).publish(msg)
                if s["depth_info"]:
                    s["depth_info"].header.stamp = msg.header.stamp
                    self._pub(f"/oak/{ns}/depth/camera_info", CameraInfo).publish(s["depth_info"])

    def _pub(self, topic, cls):
        pub = getattr(self, "_pubcache", None)
        if pub is None:
            pub = self._pubcache = {}
        if topic not in pub:
            pub[topic] = self.create_publisher(cls, topic, 10)
        return pub[topic]


def main():
    rclpy.init()
    try:
        node = OakNode()
    except SystemExit:
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
