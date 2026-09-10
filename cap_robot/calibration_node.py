"""역할: 고정 카메라 ArUco로 workspace/base TF 산출. 인터페이스: RGB/CameraInfo -> /tf.

# [변경] 원본 마커 배치/offset을 보존하되 CameraInfo 대기, 시각·재투영 오차 검증.
"""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .ros_support import parameter, load_config, run_node
from .protocol import finite_vector


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibration")
        self.cfg = load_config(parameter(self, "config_file", ""))
        self.bridge, self.info = CvBridge(), None
        self.tf, self.static = TransformBroadcaster(self), StaticTransformBroadcaster(self)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = (
            cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())
            if hasattr(cv2.aruco, "ArucoDetector")
            else None
        )
        self.create_subscription(
            CameraInfo, self.cfg["camera_info_topic"], self.on_info, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, self.cfg["image_topic"], self.on_image, qos_profile_sensor_data
        )
        transforms = []
        for item in self.cfg["robot_offsets"]:
            v = finite_vector(item["pose"], 7, "marker-base pose")
            q = np.array(v[3:])
            norm = np.linalg.norm(q)
            if norm < 1e-8:
                raise ValueError("invalid calibration quaternion")
            transforms.append(
                self.msg(
                    item["marker"], item["base"], v[:3], q / norm, self.get_clock().now().to_msg()
                )
            )
        self.static.sendTransform(transforms)

    def on_info(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.info = msg

    @staticmethod
    def msg(parent, child, xyz, q, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(
            float, xyz
        )
        (
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ) = map(float, q)
        return t

    def solve(self, object_pts, image_pts, k, dist):
        ok, rvec, tvec = cv2.solvePnP(
            np.asarray(object_pts, dtype=np.float32),
            np.asarray(image_pts, dtype=np.float32),
            k,
            dist,
        )
        if not ok or not np.isfinite(tvec).all() or tvec[2, 0] <= 0:
            raise ValueError("PnP invalid")
        projected, _ = cv2.projectPoints(
            np.asarray(object_pts, dtype=np.float32), rvec, tvec, k, dist
        )
        error = np.linalg.norm(
            projected.reshape(-1, 2) - np.asarray(image_pts).reshape(-1, 2), axis=1
        ).mean()
        if error > self.cfg.get("max_reprojection_error_px", 2.0):
            raise ValueError("PnP reprojection error")
        return tvec.flatten(), Rotation.from_rotvec(rvec.flatten()).as_quat()

    def on_image(self, msg):
        if self.info is None:
            return
        if msg.header.frame_id != self.info.header.frame_id:
            return
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        if not 0 <= age <= self.cfg.get("max_image_age", 0.5):
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "mono8")
            if self.detector:
                corners, ids, _ = self.detector.detectMarkers(img)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(img, self.dictionary)
            if ids is None:
                return
            k = np.array(self.info.k).reshape(3, 3)
            dist = np.array(self.info.d)
            size = float(self.cfg["marker_size_m"])
            h = size / 2
            single = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)
            objects, images, transforms = [], [], []
            for i, raw_id in enumerate(ids.flatten()):
                mid = int(raw_id)
                xyz, q = self.solve(single, corners[i][0], k, dist)
                transforms.append(
                    self.msg(msg.header.frame_id, f"marker_{mid}", xyz, q, msg.header.stamp)
                )
                if mid in self.cfg["workspace_markers"]:
                    offset = np.array(self.cfg["workspace_markers"][mid])
                    objects.extend(single + offset)
                    images.extend(corners[i][0])
            if len(objects) >= 4:
                xyz, q = self.solve(objects, images, k, dist)
                transforms.append(
                    self.msg(
                        msg.header.frame_id, self.cfg["workspace_frame"], xyz, q, msg.header.stamp
                    )
                )
            self.tf.sendTransform(transforms)
        except (ValueError, cv2.error):
            return  # No plausible-looking TF on failed calibration.


def main(args=None):
    run_node(CalibrationNode, args)
