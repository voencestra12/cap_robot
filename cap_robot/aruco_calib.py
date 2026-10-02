from collections import deque
import time

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation as R
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class StaticPoseEstimator:
    def __init__(self, min_samples=30, min_duration_sec=1.0,
                 translation_tolerance_m=0.01, rotation_tolerance_deg=3.0,
                 max_age_sec=5.0):
        values = [min_duration_sec, translation_tolerance_m,
                  rotation_tolerance_deg, max_age_sec]
        if (min_samples < 3 or not np.all(np.isfinite(values))
                or min(values) <= 0 or max_age_sec < min_duration_sec
                or rotation_tolerance_deg >= 180):
            raise ValueError('Invalid static calibration sample limits')
        self.min_samples = int(min_samples)
        self.min_duration_ns = int(min_duration_sec * 1e9)
        self.translation_tolerance_m = translation_tolerance_m
        self.rotation_tolerance_rad = np.deg2rad(rotation_tolerance_deg)
        self.max_age_ns = int(max_age_sec * 1e9)
        self.samples = deque(maxlen=3 * self.min_samples)
        self.last_stamp_ns = None

    def add(self, stamp_ns, translation, rotvec):
        """Return (translation, xyzw quaternion) only with sufficient consensus.

        Duplicate/out-of-order images cannot inflate the sample count. A bounded
        time window lets a bad early observation expire instead of poisoning a
        calibration forever. Rotation distances and means are on SO(3).
        """
        translation = np.asarray(translation, dtype=float).reshape(3)
        rotvec = np.asarray(rotvec, dtype=float).reshape(3)
        if not np.all(np.isfinite(np.r_[translation, rotvec])):
            return None
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return None
        self.last_stamp_ns = stamp_ns
        self.samples.append((stamp_ns, translation.copy(), R.from_rotvec(rotvec)))
        while self.samples and stamp_ns - self.samples[0][0] > self.max_age_ns:
            self.samples.popleft()
        if len(self.samples) < self.min_samples:
            return None

        stamps, translations, rotations = zip(*self.samples)
        translations = np.asarray(translations)
        quaternions = np.asarray([r.as_quat() for r in rotations])
        # Quaternion sign is immaterial: q and -q describe the same rotation.
        angles = 2 * np.arccos(np.clip(np.abs(quaternions @ quaternions.T), 0, 1))
        distances = np.linalg.norm(translations[:, None] - translations[None, :], axis=2)
        neighbors = ((distances <= self.translation_tolerance_m)
                     & (angles <= self.rotation_tolerance_rad))
        inliers = neighbors[np.argmax(neighbors.sum(axis=1))]
        required = max(self.min_samples, int(np.ceil(0.8 * len(self.samples))))
        if np.count_nonzero(inliers) < required:
            return None

        center = translations[inliers].mean(axis=0)
        rotation = R.from_quat(quaternions[inliers]).mean()
        # Check support around the final mean, not only around the chosen sample.
        inliers &= (np.linalg.norm(translations - center, axis=1)
                    <= self.translation_tolerance_m)
        inliers &= ((rotation.inv() * R.from_quat(quaternions)).magnitude()
                    <= self.rotation_tolerance_rad)
        if np.count_nonzero(inliers) < required:
            return None
        accepted_stamps = np.asarray(stamps)[inliers]
        if accepted_stamps[-1] - accepted_stamps[0] < self.min_duration_ns:
            return None
        return (translations[inliers].mean(axis=0),
                R.from_quat(quaternions[inliers]).mean().as_quat())


class ArucoCalibNode(Node):
    STATIC_CAMERA_CHILDREN = ('workspace_0', 'marker_1', 'marker_12')

    def __init__(self):
        super().__init__('aruco_calib')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_size_m', 0.05)
        self.declare_parameter('target_debug_log_period_sec', 2.0)
        self.declare_parameter('show_window', False)
        self.declare_parameter('calibration_min_samples', 30)
        self.declare_parameter('calibration_min_duration_sec', 1.0)
        self.declare_parameter('calibration_max_age_sec', 5.0)
        self.declare_parameter('calibration_translation_tolerance_m', 0.01)
        self.declare_parameter('calibration_rotation_tolerance_deg', 3.0)
        self.declare_parameter('calibration_max_reprojection_error_px', 2.0)
        self.declare_parameter(
            'offset_1',
            [-0.195949, 0.004359, 0.026819, 0.020803, 0.005481, 0.696245, 0.717482],
        )
        self.declare_parameter(
            'offset_2',
            [-0.198899, -0.001916, 0.024214, 0.014971, 0.013757, 0.706619, 0.707302],
        )
        self.declare_parameter('workspace_marker_ids', [0, 2, 4, 11, 13, 15])
        self.declare_parameter(
            'workspace_marker_offsets_mm',
            [
                0.0, 0.0, 0.0,
                300.0, 0.0, 0.0,
                600.0, 0.0, 0.0,
                600.0, 350.0, 0.0,
                300.0, 350.0, 0.0,
                0.0, 350.0, 0.0,
            ],
        )

        self.image_topic = str(self.get_parameter('image_topic').value).strip()
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value).strip()
        self.camera_frame = str(self.get_parameter('camera_frame').value).strip()
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
        self.target_debug_log_period_sec = float(
            self.get_parameter('target_debug_log_period_sec').value
        )
        self.show_window = bool(self.get_parameter('show_window').value)
        self._last_warn_time = {}
        self._last_detected_ids_log_time = 0.0
        self._calibration_frame = None
        self._static_camera_poses = {}
        self._static_camera_published = False
        self._pose_estimators = {
            child: StaticPoseEstimator(
                min_samples=int(self.get_parameter('calibration_min_samples').value),
                min_duration_sec=float(self.get_parameter('calibration_min_duration_sec').value),
                max_age_sec=float(self.get_parameter('calibration_max_age_sec').value),
                translation_tolerance_m=float(
                    self.get_parameter('calibration_translation_tolerance_m').value),
                rotation_tolerance_deg=float(
                    self.get_parameter('calibration_rotation_tolerance_deg').value),
            ) for child in self.STATIC_CAMERA_CHILDREN
        }
        self.max_reprojection_error_px = float(
            self.get_parameter('calibration_max_reprojection_error_px').value)
        if not np.isfinite(self.max_reprojection_error_px) or self.max_reprojection_error_px <= 0:
            raise ValueError('calibration_max_reprojection_error_px must be positive and finite')

        self.dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.mtx = np.array([
            [606.001831, 0.0, 325.263245],
            [0.0, 605.723694, 243.518890],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # 정상 동작이 확인된 캘리브레이션 값. YAML/ROS 파라미터로 덮어쓸 수 있습니다.
        self.OFFSET_1 = self.normalize_offset_quaternion(
            self.offset_array_to_dict(self.get_parameter('offset_1').value, 'offset_1'),
            'OFFSET_1',
        )
        self.OFFSET_2 = self.normalize_offset_quaternion(
            self.offset_array_to_dict(self.get_parameter('offset_2').value, 'offset_2'),
            'OFFSET_2',
        )

        self.MARKER_OFFSETS_MM = self.build_marker_offsets(
            self.get_parameter('workspace_marker_ids').value,
            self.get_parameter('workspace_marker_offsets_mm').value,
        )

        self.bridge = CvBridge()
        self.camera_info_received = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # marker -> robot_base는 시작 시 송신하고, 카메라 TF 확정 시 함께 재송신한다.
        # camera -> marker도 관측 누적 후 static으로 연결한다. 부모 관계는 유지한다.
        self.publish_static_robot_base_offsets()

        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
            aruco.DetectorParameters(),
        )

        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            # [MERGED] RealSense의 SensorDataQoS(BEST_EFFORT)와 호환합니다.
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            camera_qos,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            camera_qos,
        )

        self.get_logger().info(
            # [MERGED] ArUco 노드는 캘리브레이션/TF 발행만 담당합니다.
            '🔥 멀티 에이전트(1&2) ArUco TF 노드 가동 완료 '
            f'(image={self.image_topic}, camera_info={self.camera_info_topic}, '
            f'camera_frame={self.camera_frame}, marker_size_m={self.marker_size_m}) 🔥'
        )

    @staticmethod
    def offset_array_to_dict(raw_values, label):
        values = [float(value) for value in raw_values]
        if len(values) != 7:
            raise ValueError(
                f'{label}은 [x, y, z, qx, qy, qz, qw] 7개 값이어야 합니다: {values}'
            )
        return {
            'x': values[0],
            'y': values[1],
            'z': values[2],
            'rx': values[3],
            'ry': values[4],
            'rz': values[5],
            'rw': values[6],
        }

    @staticmethod
    def build_marker_offsets(raw_ids, raw_offsets):
        marker_ids = [int(value) for value in raw_ids]
        offsets = [float(value) for value in raw_offsets]
        expected = len(marker_ids) * 3
        if len(offsets) != expected:
            raise ValueError(
                'workspace_marker_offsets_mm 길이가 올바르지 않습니다: '
                f'ids={len(marker_ids)}, offsets={len(offsets)}, expected={expected}'
            )
        return {
            marker_id: offsets[index * 3:(index + 1) * 3]
            for index, marker_id in enumerate(marker_ids)
        }

    def warn_throttled(self, key, message):
        now = time.time()
        last = self._last_warn_time.get(key, 0.0)
        if now - last >= self.target_debug_log_period_sec:
            self._last_warn_time[key] = now
            self.get_logger().warn(message)

    @staticmethod
    def normalize_quaternion_xyzw(q, label='quaternion'):
        q = np.asarray(q, dtype=float)
        norm = float(np.linalg.norm(q))
        if norm < 1e-9:
            raise ValueError(f'{label} norm이 0에 가깝습니다: {q}')
        return q / norm, norm

    def normalize_offset_quaternion(self, offset, label):
        q = np.array([offset['rx'], offset['ry'], offset['rz'], offset['rw']], dtype=float)
        q_norm, norm = self.normalize_quaternion_xyzw(q, label)
        fixed = dict(offset)
        fixed['rx'], fixed['ry'], fixed['rz'], fixed['rw'] = map(float, q_norm)
        if abs(norm - 1.0) > 1e-3:
            self.get_logger().warn(
                f'⚠️ {label} quaternion 정규화: norm={norm:.6f} -> 1.000000, '
                f'xyzw=({fixed["rx"]:.6f}, {fixed["ry"]:.6f}, '
                f'{fixed["rz"]:.6f}, {fixed["rw"]:.6f})'
            )
        return fixed

    def make_robot_base_transform(self, marker_frame, robot_frame, offset, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = marker_frame
        t.child_frame_id = robot_frame
        t.transform.translation.x = float(offset['x'])
        t.transform.translation.y = float(offset['y'])
        t.transform.translation.z = float(offset['z'])
        q, _ = self.normalize_quaternion_xyzw(
            [offset['rx'], offset['ry'], offset['rz'], offset['rw']], robot_frame
        )
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        return t

    def publish_static_robot_base_offsets(self):
        stamp = self.get_clock().now().to_msg()
        transforms = [
            self.make_robot_base_transform('marker_1', 'robot_1_base', self.OFFSET_1, stamp),
            self.make_robot_base_transform('marker_12', 'robot_2_base', self.OFFSET_2, stamp),
        ]
        # Humble의 broadcaster는 마지막 메시지만 유지한다. 매번 전체 static
        # 묶음을 보내 늦게 접속한 listener도 marker -> base를 받을 수 있게 한다.
        for child, (translation, quaternion) in self._static_camera_poses.items():
            transforms.append(self.make_transform(
                self._calibration_frame, child, translation, None, stamp, quaternion))
        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info(
            '📌 static TF 송신 완료: marker_1 -> robot_1_base, marker_12 -> robot_2_base'
        )

    def camera_info_callback(self, msg):
        mtx = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(msg.d, dtype=np.float64)
        if (not np.all(np.isfinite(mtx)) or not np.all(np.isfinite(dist))
                or mtx[0, 0] <= 0 or mtx[1, 1] <= 0):
            self.camera_info_received = False
            self.warn_throttled('camera_info', '유효한 CameraInfo 대기 중')
            return
        if self._calibration_frame is not None and (
            (msg.header.frame_id and msg.header.frame_id != self._calibration_frame)
            or not np.array_equal(mtx, self.mtx)
            or not np.array_equal(dist if dist.size else np.zeros(5), self.dist)
        ):
            self.camera_info_received = False
            self.warn_throttled('camera_changed', 'CameraInfo 변경: aruco_calib 재시작 필요')
            return
        self.mtx = mtx
        self.dist = dist
        if self.dist.size == 0:
            self.dist = np.zeros(5, dtype=np.float64)
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id
        if not self.camera_info_received:
            self.get_logger().info(
                f'📷 CameraInfo 수신 완료: frame={self.camera_frame}, '
                f'fx={self.mtx[0, 0]:.3f}, fy={self.mtx[1, 1]:.3f}, '
                f'cx={self.mtx[0, 2]:.3f}, cy={self.mtx[1, 2]:.3f}'
            )
        self.camera_info_received = True

    def image_callback(self, msg):
        if not self.camera_info_received:
            self.warn_throttled('camera_info', '캘리브레이션용 CameraInfo 수신 대기 중')
            return
        parent_frame = msg.header.frame_id or self.camera_frame
        if parent_frame != self.camera_frame:
            self.warn_throttled('image_frame', 'Image와 CameraInfo의 frame_id 불일치')
            return
        if self._calibration_frame is None:
            self._calibration_frame = parent_frame
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'이미지 변환 실패: {error}')
            return

        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY))

        if ids is not None:
            detected_ids = [int(x) for x in ids.flatten()]
            now = time.time()
            if now - self._last_detected_ids_log_time >= 5.0:
                self._last_detected_ids_log_time = now
                self.get_logger().info(f'🔎 detected marker ids={detected_ids}')

            aruco.drawDetectedMarkers(color_image, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size_m, self.mtx, self.dist
            )
            half = self.marker_size_m / 2
            marker_points = np.array([
                [-half, half, 0], [half, half, 0],
                [half, -half, 0], [-half, -half, 0],
            ], dtype=np.float32)
            for i, m_id in enumerate(ids.flatten()):
                child = f'marker_{int(m_id)}'
                if child in self.STATIC_CAMERA_CHILDREN:
                    self.observe_static_pose(
                        child, marker_points, corners[i][0],
                        tvecs[i].reshape(3), rvecs[i].reshape(3), msg.header.stamp)
                    continue
                self.broadcast_tf(
                    self.camera_frame,
                    f'marker_{int(m_id)}',
                    tvecs[i].reshape(3),
                    rvecs[i].reshape(3),
                    msg.header.stamp,
                )

            matched_pts = [
                (
                    np.array([
                        [-self.marker_size_m / 2 + val[0] / 1000,  self.marker_size_m / 2 + val[1] / 1000, val[2] / 1000],
                        [ self.marker_size_m / 2 + val[0] / 1000,  self.marker_size_m / 2 + val[1] / 1000, val[2] / 1000],
                        [ self.marker_size_m / 2 + val[0] / 1000, -self.marker_size_m / 2 + val[1] / 1000, val[2] / 1000],
                        [-self.marker_size_m / 2 + val[0] / 1000, -self.marker_size_m / 2 + val[1] / 1000, val[2] / 1000],
                    ], dtype=np.float32),
                    corners[i][0],
                )
                for i, m_id in enumerate(ids.flatten())
                if int(m_id) in self.MARKER_OFFSETS_MM
                for val in [self.MARKER_OFFSETS_MM[int(m_id)]]
            ]

            if matched_pts:
                object_points = np.vstack([p[0] for p in matched_pts]).astype(np.float32)
                image_points = np.vstack([p[1] for p in matched_pts]).astype(np.float32)
            else:
                object_points = np.empty((0, 3), dtype=np.float32)
                image_points = np.empty((0, 2), dtype=np.float32)

            if (
                len(object_points) >= 4
                and 'workspace_0' not in self._static_camera_poses
                and len(set(detected_ids).intersection(self.MARKER_OFFSETS_MM)) >= 4
            ):
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.mtx,
                    self.dist,
                )
                if success:
                    self.observe_static_pose(
                        'workspace_0', object_points, image_points,
                        tvec.flatten(), rvec.flatten(), msg.header.stamp)
            elif len(matched_pts) < 4 and 'workspace_0' not in self._static_camera_poses:
                self.warn_throttled(
                    'workspace_markers',
                    f'workspace_0 계산용 대응점 부족: matched_marker_count={len(matched_pts)}, '
                    f'corner_count={len(object_points)} (필요: 작업대 마커 4개 이상, 현재 ids={detected_ids})'
                )

        if not self._static_camera_published:
            pending = [child for child in self.STATIC_CAMERA_CHILDREN
                       if child not in self._static_camera_poses]
            self.warn_throttled('calibration_pending', f'static TF 관측 수집 중: {pending}')

        if self.show_window:
            cv2.imshow('ArUco calibration', color_image)
            cv2.waitKey(1)

    def observe_static_pose(self, child, object_points, image_points, tvec, rvec, stamp):
        if child in self._static_camera_poses:
            return
        if not np.all(np.isfinite(np.r_[tvec, rvec])):
            return
        # Reject poses behind the camera and poor fits before temporal consensus.
        camera_points = R.from_rotvec(rvec).apply(object_points) + tvec
        if np.any(camera_points[:, 2] <= 0):
            return
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, self.mtx, self.dist)
        residuals = projected.reshape(-1, 2) - np.asarray(image_points).reshape(-1, 2)
        error_px = float(np.sqrt(np.mean(np.sum(residuals ** 2, axis=1))))
        if not np.isfinite(error_px) or error_px > self.max_reprojection_error_px:
            return
        pose = self._pose_estimators[child].add(
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec), tvec, rvec)
        if pose is None:
            return
        self._static_camera_poses[child] = pose
        self.get_logger().info(f'{child}: 안정적인 다중 프레임 추정 완료')
        if len(self._static_camera_poses) == len(self.STATIC_CAMERA_CHILDREN):
            self.publish_static_robot_base_offsets()
            self._static_camera_published = True
            self.get_logger().info(
                'static TF 확정: camera -> workspace_0, marker_1, marker_12 '
                '(재캘리브레이션: aruco_calib 노드 재시작)')

    def make_transform(self, parent, child, tvec, rvec, stamp, quat=None):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        q = R.from_rotvec(rvec).as_quat() if quat is None else np.asarray(quat, dtype=float)
        q, norm = self.normalize_quaternion_xyzw(q, child)
        if abs(norm - 1.0) > 1e-3:
            self.warn_throttled(
                f'quat_{child}',
                f'{child} quaternion 자동 정규화: norm={norm:.6f}'
            )
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        return t

    def broadcast_tf(self, parent, child, tvec, rvec, stamp, quat=None):
        if child in self.STATIC_CAMERA_CHILDREN:
            raise ValueError(f'{child} must only be published on /tf_static')
        self.tf_broadcaster.sendTransform(
            self.make_transform(parent, child, tvec, rvec, stamp, quat))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
