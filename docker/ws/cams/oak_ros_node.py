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

import array
import os

import cv2
import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

SCENE_MXID = os.environ.get("OAK_SCENE_MXID", "184430101111F90F00")
GRIPPER_MXID = os.environ.get("OAK_GRIPPER_MXID", "")
PUBLISH_HZ = 15.0
CAMERA_FPS = 10.0   # ask the sensor for what we can publish: pulling 1080p at 30 fps
                    # and throwing 2/3 of it away was burning two cores in getCvFrame()
PCL_HZ = 3.0  # point cloud throttle — 720p clouds are heavy over the LAN
DEPTH_HZ = 5.0  # raw depth/confidence: 2.7 MB per frame, and nothing consumes them faster


def build_pipeline() -> dai.Pipeline:
    p = dai.Pipeline()
    color = p.createColorCamera()
    color.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    color.setInterleaved(False)
    color.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    color.setFps(CAMERA_FPS)
    xcolor = p.createXLinkOut()
    xcolor.setStreamName("color")
    color.video.link(xcolor.input)

    left = p.createMonoCamera()
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    right = p.createMonoCamera()
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    sd = p.createStereoDepth()
    # Default stereo stops at ~0.65 m (baseline 7.5 cm, disparity range 96): anything
    # nearer — the arm, the gripper, the board on it — comes back as invalid or as
    # background. Extended disparity doubles the range to ~0.33 m, which is where this
    # cell actually works, and the left/right check kills the speckle that filled the
    # cloud with phantom points.
    sd.setExtendedDisparity(True)
    sd.setLeftRightCheck(True)
    left.out.link(sd.left)
    right.out.link(sd.right)
    xd = p.createXLinkOut()
    xd.setStreamName("depth")
    sd.depth.link(xd.input)
    xconf = p.createXLinkOut()
    xconf.setStreamName("confidence")
    sd.confidenceMap.link(xconf.input)
    # the two rectified views the depth is actually computed from — what the "Stereo Cam"
    # tab shows. setOutputRectified is off by default in 2.33.
    sd.setOutputRectified(True)
    xl = p.createXLinkOut()
    xl.setStreamName("left")
    sd.rectifiedLeft.link(xl.input)
    xr = p.createXLinkOut()
    xr.setStreamName("right")
    sd.rectifiedRight.link(xr.input)
    return p


def _k(raw):
    """depthai getDefaultIntrinsics nests the [fx, fy, cx, cy] vector — unwrap."""
    if not raw:
        return None
    while isinstance(raw, (list, tuple)) and len(raw) == 1:
        raw = raw[0]
    try:
        return [float(x) for x in raw]
    except TypeError:
        return None


def create_cloud_xyz32(header, pts_m):
    """Minimal PointCloud2(xyz, float32) builder — the dusty image lacks
    sensor_msgs_py, and this is all the helper would do anyway."""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(pts_m)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    # rclpy validates and converts a bytes/list into uint8[] one element at a time in this
    # build (2 MB took 650 ms), and rejects numpy outright. array.array('B') is the fast
    # path: it is stored as-is and serialised straight from the buffer (60 ms for 2 MB).
    arr = np.ascontiguousarray(pts_m, dtype="<f4")
    msg.data = array.array("B", arr.tobytes())
    return msg


def create_cloud_xyzrgb(header, pts_m, rgb_u8):
    """PointCloud2 with the PCL layout x,y,z,rgb (rgb packed into a float32).

    Consumers (rviz, Open3D, open3d-style readers, GraspGen's converters) expect exactly this
    offset order, so it is built by hand rather than depending on sensor_msgs_py.
    """
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(pts_m)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    packed = np.zeros((len(pts_m), 4), dtype=np.float32)
    packed[:, :3] = np.ascontiguousarray(pts_m, dtype=np.float32)
    c = np.ascontiguousarray(rgb_u8, dtype=np.uint32)
    packed[:, 3] = ((c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]).view(np.float32)
    msg.data = array.array("B", packed.astype("<f4").tobytes())
    return msg


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
            qconf = device.getOutputQueue("confidence", 2, False)
            ql = device.getOutputQueue("left", 2, False)
            qr = device.getOutputQueue("right", 2, False)
            calib = device.readCalibration()  # CalibrationHandler (empty if no factory calib)
            color_i = _k(calib.getDefaultIntrinsics(dai.CameraBoardSocket.RGB)) if calib else None
            right_i = _k(calib.getDefaultIntrinsics(dai.CameraBoardSocket.RIGHT)) if calib else None
            color_ok = color_i is not None and color_i[0] > 0
            right_ok = right_i is not None and right_i[0] > 0
            # getDefaultIntrinsics can come back empty on units whose calibration is
            # stored per-resolution; ask point-blank for the depth geometry instead
            probe_k = None
            try:
                pr = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, 1280, 720)
                probe_k = (pr[0][0], pr[1][1], pr[0][2], pr[1][2])
            except Exception:  # noqa: BLE001
                pass
            self.get_logger().info(
                f"{ns}: device {info.getMxId()} — calib: default color="
                f"{'ok' if color_ok else 'MISSING'} stereo={'ok' if right_ok else 'MISSING'}"
                + (f", depth 1280x720 fx={probe_k[0]:.2f} cx={probe_k[2]:.2f}" if probe_k else
                   ", depth intrinsics unavailable")
            )
            self._publish_static_tfs(ns)
            stream = {
                "ns": ns, "qc": qc, "qd": qd, "qconf": qconf, "ql": ql, "qr": qr,
                "device": device,
                "calib": calib,
                "k_cache": {},
                "color_bgr": None,
                "colour_cache": {},
                "pcl_last": 0.0,
                "depth_last": 0.0,
                "conf_last": 0.0,
                "cloud_warned": False,
                "color_info": camera_info(
                    color_i[0], color_i[1], color_i[2], color_i[3],
                    1920, 1080, f"{ns}_color_optical_frame") if color_ok else None,
                "depth_info": camera_info(
                    right_i[0], right_i[1], right_i[2], right_i[3],
                    1280, 720, f"{ns}_right_optical_frame") if right_ok else None,
                "q_imu": None, "imu": {"acc": None, "gyr": None, "rot": None},
            }
            # IMU (POE only — the Lite has none). Raw accel is m/s^2, gyro rad/s.
            try:
                imu = device.getIMU()
                imu.enableIMUSensor([
                    dai.IMUSensor.ACCELEROMETER_RAW,
                    dai.IMUSensor.GYROSCOPE_RAW,
                    dai.IMUSensor.ROTATION_VECTOR,
                ], 100)
                imu.setBatchReportThreshold(1)
                imu.setMaxBatchReports(5)
                stream["q_imu"] = device.getOutputQueue("imu", 8, False)
                self.get_logger().info(f"{ns}: IMU enabled (accel+gyro+rotvec @100Hz)")
            except Exception:  # noqa: BLE001
                self.get_logger().info(f"{ns}: no IMU on this device")
            self.streams.append(stream)
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
        for child in ("color_optical_frame", "right_optical_frame", "imu_link"):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f"{ns}_camera_link"
            t.child_frame_id = f"{ns}_{child}"
            t.transform.rotation.w = 1.0
            self.tfb.sendTransform(t)

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        for s in self.streams:
            ns = s["ns"]
            color_frames = s["qc"].tryGetAll()
            depth_frames = s["qd"].tryGetAll()
            conf_frames = s["qconf"].tryGetAll()
            if color_frames:
                frame = color_frames[-1].getCvFrame()
                s["color_bgr"] = frame
                ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                if ok:
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = f"{ns}_color_optical_frame"
                    msg.format = "jpeg"
                    msg.data = array.array("B", jpg.tobytes())
                    self._pub(f"/oak/{ns}/color/compressed", CompressedImage).publish(msg)
                    if s["color_info"]:
                        s["color_info"].header.stamp = msg.header.stamp
                        self._pub(f"/oak/{ns}/color/camera_info", CameraInfo).publish(s["color_info"])
            if depth_frames and now - s.get("depth_last", 0.0) >= 1.0 / DEPTH_HZ:
                s["depth_last"] = now
                depth = depth_frames[-1].getFrame()
                msg = self.bridge.cv2_to_imgmsg(depth, encoding="mono16")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f"{ns}_right_optical_frame"
                self._pub(f"/oak/{ns}/depth", Image).publish(msg)
                if s["depth_info"]:
                    s["depth_info"].header.stamp = msg.header.stamp
                    self._pub(f"/oak/{ns}/depth/camera_info", CameraInfo).publish(s["depth_info"])
            for side, q in (("left", s["ql"]), ("right", s["qr"])):
                frames = q.tryGetAll()
                if not frames:
                    continue
                ok, jpg = cv2.imencode(".jpg", frames[-1].getCvFrame(),
                                       [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not ok:
                    continue
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f"{ns}_right_optical_frame"
                msg.format = "jpeg"
                msg.data = array.array("B", jpg.tobytes())
                self._pub(f"/oak/{ns}/{side}/compressed", CompressedImage).publish(msg)
            if conf_frames and now - s.get("conf_last", 0.0) >= 1.0 / DEPTH_HZ:
                s["conf_last"] = now
                conf = conf_frames[-1].getCvFrame()
                msg = self.bridge.cv2_to_imgmsg(conf, encoding="mono8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f"{ns}_right_optical_frame"
                self._pub(f"/oak/{ns}/confidence", Image).publish(msg)
            if depth_frames and now - s["pcl_last"] >= 1.0 / PCL_HZ:
                self._publish_cloud(s, depth_frames[-1].getFrame(), now)
            if s["q_imu"] is not None:
                for packet in s["q_imu"].tryGetAll():
                    for report in packet.packets:
                        if isinstance(report, dai.IMUReportAccelerometer):
                            s["imu"]["acc"] = (report.x, report.y, report.z)
                        elif isinstance(report, dai.IMUReportGyroscope):
                            s["imu"]["gyr"] = (report.x, report.y, report.z)
                        elif isinstance(report, dai.IMUReportRotationVectorWXYZ):
                            s["imu"]["rot"] = (report.i, report.j, report.k, report.real)
                if any(v is not None for v in s["imu"].values()):
                    imu = Imu()
                    imu.header.stamp = self.get_clock().now().to_msg()
                    imu.header.frame_id = f"{ns}_imu_link"
                    imu.orientation_covariance[0] = -1.0
                    imu.angular_velocity_covariance[0] = -1.0
                    imu.linear_acceleration_covariance[0] = -1.0
                    if s["imu"]["rot"] is not None:
                        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = s["imu"]["rot"]
                    if s["imu"]["gyr"] is not None:
                        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = s["imu"]["gyr"]
                    if s["imu"]["acc"] is not None:
                        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = s["imu"]["acc"]
                    self._pub(f"/oak/{ns}/imu", Imu).publish(imu)

    def _cloud_intrinsics(self, s, w, h):
        """Intrinsics that go with the depth map, in order of trust.

        The map is produced by the stereo pair and delivered in the RIGHT camera's
        geometry at the stereo resolution, so the right camera's K at that exact size
        is the only correct choice — not the colour camera's, and definitely not a
        hard-coded placeholder. Falls back to env vars, then gives up.
        """
        key = (w, h)
        cache = s["k_cache"]
        if key in cache:
            return cache[key]
        k = None
        calib = s.get("calib")
        if calib is not None:
            for sock in ("CAM_C", "RIGHT"):
                try:
                    raw = calib.getCameraIntrinsics(getattr(dai.CameraBoardSocket, sock), w, h)
                    if raw and float(raw[0][0]) > 0:
                        k = (float(raw[0][0]), float(raw[1][1]), float(raw[0][2]), float(raw[1][2]))
                        break
                except Exception:  # noqa: BLE001
                    continue
        source = "device"
        if k is None:
            env = f"OAK_{s['ns'].upper()}"
            try:
                k = (float(os.environ[f"{env}_FX"]), float(os.environ[f"{env}_FY"]),
                     float(os.environ[f"{env}_CX"]), float(os.environ[f"{env}_CY"]))
                source = "env"
            except KeyError:
                if not s["cloud_warned"]:
                    s["cloud_warned"] = True
                    self.get_logger().warn(
                        f"{s['ns']}: point cloud disabled — no calibration for {w}x{h}")
                return None
        cache[key] = k
        self.get_logger().info(
            f"{s['ns']}: cloud intrinsics ({source}, {w}x{h}) fx={k[0]:.2f} fy={k[1]:.2f} "
            f"cx={k[2]:.2f} cy={k[3]:.2f}")
        return k

    def _colour_model(self, s, cw, ch):
        """Colour intrinsics at the frame size, plus the depth→colour extrinsic, cached.

        The cloud is built on the depth camera's geometry, but colour has to come from a pixel
        of the *colour* camera: the device's own extrinsic is the exact relation between the two
        sensors, so the board homography of §10.4 (a plane approximation) is not needed here.
        Returns None when the unit has no usable calibration, in which case the cloud stays
        uncoloured in the depth frame instead of being coloured wrongly.
        """
        key = (cw, ch)
        cache = s["colour_cache"]
        if key in cache:
            return cache[key]
        model = None
        calib = s.get("calib")
        if calib is not None:
            try:
                raw = calib.getCameraIntrinsics(dai.CameraBoardSocket.RGB, cw, ch)
                k = (float(raw[0][0]), float(raw[1][1]), float(raw[0][2]), float(raw[1][2]))
            except Exception:  # noqa: BLE001
                k = None
            if k and k[0] > 0:
                try:
                    m = np.array(calib.getCameraExtrinsics(dai.CameraBoardSocket.RIGHT,
                                                           dai.CameraBoardSocket.RGB),
                                 dtype=float)
                except Exception:  # noqa: BLE001
                    m = None
                if m is not None:
                    if m.shape == (4, 4):
                        R, t = m[:3, :3], m[:3, 3]
                    elif m.shape == (3, 4):
                        R, t = m[:, :3], m[:, 3]
                    else:
                        R = t = None
                    if R is not None:
                        # depthai 2.x returns this translation in CENTIMETRES (the legacy
                        # convention); the cloud is in metres, so 3.719 becomes 37.19 mm.
                        t = np.asarray(t, dtype=float) * 0.01
                        model = (k, R, t)
                        self.get_logger().info(
                            f"{s['ns']}: cloud colour model {cw}x{ch} fx={k[0]:.2f} "
                            f"cx={k[2]:.2f} | depth→colour t=({t[0] * 1000:.1f}, "
                            f"{t[1] * 1000:.1f}, {t[2] * 1000:.1f}) mm")
        cache[key] = model
        return model

    def _publish_cloud(self, s, depth, now):
        """Unproject the depth map, then colour every point from the colour camera.

        Geometry is built on the depth camera's own K (the map arrives in the right camera's
        geometry), moved into the colour camera's frame with the device extrinsic, and
        projected with the colour K to fetch the pixel. The result is published in the *colour*
        optical frame with the PCL `x, y, z, rgb` layout, so a cloud point and its pixel are the
        same thing — the step that turns "the camera sees" into "the object is there".

        Points that fall outside the colour image keep their geometry but are coloured dim grey,
        because dropping them would leave holes in the shape a grasp is aimed at.
        """
        ns = s["ns"]
        env = f"OAK_{ns.upper()}"
        h, w = depth.shape
        K = self._cloud_intrinsics(s, w, h)
        if K is None:
            return
        fx, fy, cx, cy = K
        scale = float(os.environ.get(f"{env}_DEPTH_SCALE", "1.0"))
        uu, vv = np.meshgrid(np.arange(0, w, 2), np.arange(0, h, 2))
        z = depth[::2, ::2].astype(np.float32) * scale / 1000.0
        valid = (z > 0.1) & (z < 5.0)
        x = (uu - cx) * z / fx
        y = (vv - cy) * z / fy
        pts = np.stack([x[valid], y[valid], z[valid]], axis=1)
        if not len(pts):
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        colour = s.get("color_bgr")
        model = self._colour_model(s, colour.shape[1], colour.shape[0]) if colour is not None else None
        if model is None:
            header.frame_id = f"{ns}_right_optical_frame"
            self._pub(f"/oak/{ns}/points", PointCloud2).publish(create_cloud_xyz32(header, pts))
            s["pcl_last"] = now
            return

        (kfx, kfy, kcx, kcy), R, t = model
        p = pts @ R.T + t
        zc = p[:, 2]
        rgb = np.full((len(p), 3), 80, dtype=np.uint8)
        front = zc > 0.05
        if front.any():
            u = kfx * p[front, 0] / zc[front] + kcx
            v = kfy * p[front, 1] / zc[front] + kcy
            ch, cw = colour.shape[:2]
            inside = (u >= 0) & (u < cw - 1) & (v >= 0) & (v < ch - 1)
            idx = np.flatnonzero(front)[inside]
            if len(idx):
                bgr = colour[v[inside].astype(np.int32), u[inside].astype(np.int32)]
                rgb[idx] = bgr[:, ::-1]          # BGR -> RGB
        header.frame_id = f"{ns}_color_optical_frame"
        self._pub(f"/oak/{ns}/points", PointCloud2).publish(create_cloud_xyzrgb(header, p, rgb))
        s["pcl_last"] = now

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
