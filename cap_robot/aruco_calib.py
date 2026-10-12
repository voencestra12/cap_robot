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


class ArucoCalibNode(Node):
    def __init__(self):
        super().__init__('aruco_calib')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_size_m', 0.05)
        self.declare_parameter('target_debug_log_period_sec', 2.0)
        self.declare_parameter('show_window', False)
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
        # 노드 실행 중에는 카메라가 고정이라는 전제로, 아래 프레임들은 시작 시 N프레임을 모아
        # robust 평균을 낸 뒤 lock하고 /tf_static으로만 송신한다. (위치를 바꾸면 노드 재시작)
        self.declare_parameter('lock_marker_ids', [1, 12])
        self.declare_parameter('lock_sample_count', 30)
        self.declare_parameter('lock_outlier_translation_m', 0.01)
        self.declare_parameter('lock_outlier_rotation_deg', 5.0)
        # lock 이후 실시간 추정이 lock 값과 계속 다르면 카메라 이동을 의심해 경고만 한다.
        self.declare_parameter('drift_warn_translation_m', 0.01)
        self.declare_parameter('drift_warn_rotation_deg', 2.0)
        self.declare_parameter('drift_warn_frames', 15)
        # 작은/먼 단일 마커는 IPPE 두 해(planar ambiguity) 사이에서 뒤집힌다.
        # lock 마커는 작업대 평면에 놓여 있으므로 법선이 workspace_0 z축과 가까운 해를 고르고,
        # 그래도 이 각도보다 기울어 있으면 샘플을 버린다.
        self.declare_parameter('lock_marker_max_tilt_deg', 20.0)

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

        self.lock_marker_ids = {int(value) for value in self.get_parameter('lock_marker_ids').value}
        self.lock_sample_count = max(5, int(self.get_parameter('lock_sample_count').value))
        self.lock_outlier_translation_m = float(
            self.get_parameter('lock_outlier_translation_m').value
        )
        self.lock_outlier_rotation_rad = np.deg2rad(
            float(self.get_parameter('lock_outlier_rotation_deg').value)
        )
        self.drift_warn_translation_m = float(self.get_parameter('drift_warn_translation_m').value)
        self.drift_warn_rotation_rad = np.deg2rad(
            float(self.get_parameter('drift_warn_rotation_deg').value)
        )
        self.drift_warn_frames = max(1, int(self.get_parameter('drift_warn_frames').value))
        self.lock_marker_min_normal_dot = float(np.cos(np.deg2rad(
            float(self.get_parameter('lock_marker_max_tilt_deg').value)
        )))
        self.lock_frames = {f'marker_{marker_id}' for marker_id in self.lock_marker_ids}
        self.lock_frames.add('workspace_0')
        self._lock_samples = {frame: [] for frame in self.lock_frames}
        # frame -> (tvec, quat_xyzw)
        self._locked_poses = {}
        self._drift_counts = {frame: 0 for frame in self.lock_frames}
        # child_frame -> TransformStamped. /tf_static은 depth=1 latched라서
        # 마지막 메시지만 늦게 뜬 노드에 전달되므로, 항상 전체 목록을 한 번에 보낸다.
        self._static_transforms = {}

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

        # marker -> robot_base는 고정 캘리브레이션이므로 /tf_static으로 송신한다.
        # camera -> marker_1/marker_12/workspace_0도 lock 후 /tf_static으로만 송신해
        # camera -> marker_1 -> robot_1_base 체인 전체가 static이 된다.
        # 주의: lock 대상 프레임을 /tf(동적)로 한 번이라도 보내면 수신 측 tf2 버퍼가 그 프레임을
        # 동적 캐시로 잡아버려, 이후 static 값이 cache_time(10s) 뒤 만료되며 체인이 끊긴다.
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
        for transform in (
            self.make_robot_base_transform('marker_1', 'robot_1_base', self.OFFSET_1, stamp),
            self.make_robot_base_transform('marker_12', 'robot_2_base', self.OFFSET_2, stamp),
        ):
            self._static_transforms[transform.child_frame_id] = transform
        self.send_static_transforms()
        self.get_logger().info(
            '📌 static TF 송신 완료: marker_1 -> robot_1_base, marker_12 -> robot_2_base'
        )

    def send_static_transforms(self):
        # 이전에 보낸 static TF까지 항상 함께 보내야 late subscriber가 전체 트리를 받는다.
        self.static_tf_broadcaster.sendTransform(list(self._static_transforms.values()))

    def camera_info_callback(self, msg):
        self.mtx = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64)
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
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'이미지 변환 실패: {error}')
            return

        parent_frame = msg.header.frame_id or self.camera_frame
        if parent_frame:
            self.camera_frame = parent_frame

        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY))

        if ids is not None:
            detected_ids = [int(x) for x in ids.flatten()]
            now = time.time()
            if now - self._last_detected_ids_log_time >= 5.0:
                self._last_detected_ids_log_time = now
                self.get_logger().info(f'🔎 detected marker ids={detected_ids}')

            aruco.drawDetectedMarkers(color_image, corners, ids)

            # workspace_0를 먼저 풀어, 같은 프레임의 작업대 법선으로 lock 마커의 pose flip을 판별한다.
            workspace_rotation = None
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
                and len(set(detected_ids).intersection(self.MARKER_OFFSETS_MM)) >= 4
            ):
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.mtx,
                    self.dist,
                )
                if success:
                    workspace_rotation = R.from_rotvec(rvec.reshape(3)).as_matrix()
                    self.handle_lock_sample('workspace_0', tvec.reshape(3), rvec.reshape(3))
            elif len(object_points) < 4 and 'workspace_0' not in self._locked_poses:
                self.warn_throttled(
                    'workspace_markers',
                    f'workspace_0 계산용 대응점 부족: matched_marker_count={len(matched_pts)}, '
                    f'corner_count={len(object_points)} (필요: 4개 이상 corner, 현재 ids={detected_ids})'
                )
            if workspace_rotation is None and 'workspace_0' in self._locked_poses:
                workspace_rotation = R.from_quat(self._locked_poses['workspace_0'][1]).as_matrix()

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size_m, self.mtx, self.dist
            )
            for i, m_id in enumerate(ids.flatten()):
                marker_frame = f'marker_{int(m_id)}'
                if marker_frame in self.lock_frames:
                    # lock 대상은 절대 /tf로 보내지 않는다 (static과 섞이면 체인이 끊김).
                    pose = self.solve_planar_marker_pose(
                        marker_frame, corners[i][0], workspace_rotation
                    )
                    if pose is not None:
                        self.handle_lock_sample(marker_frame, pose[0], pose[1])
                    continue
                self.broadcast_tf(
                    self.camera_frame,
                    marker_frame,
                    tvecs[i].reshape(3),
                    rvecs[i].reshape(3),
                    msg.header.stamp,
                )

        if self.show_window:
            cv2.imshow('ArUco calibration', color_image)
            cv2.waitKey(1)

    def solve_planar_marker_pose(self, marker_frame, marker_corners, workspace_rotation):
        """IPPE 두 해 중 마커 법선(z)이 workspace_0 z축과 가장 가까운 해를 (tvec, rvec)로 반환한다."""
        if workspace_rotation is None:
            # 기준 평면 없이는 flip을 판별할 수 없으므로 샘플을 쓰지 않는다.
            return None
        half = self.marker_size_m / 2.0
        object_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)
        count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            object_points,
            np.asarray(marker_corners, dtype=np.float32),
            self.mtx,
            self.dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not count:
            return None
        workspace_normal = workspace_rotation[:, 2]
        candidates = [
            (
                float(np.dot(R.from_rotvec(rvec.reshape(3)).as_matrix()[:, 2], workspace_normal)),
                tvec.reshape(3),
                rvec.reshape(3),
            )
            for rvec, tvec in zip(rvecs, tvecs)
        ]
        normal_dot, tvec, rvec = max(candidates, key=lambda candidate: candidate[0])
        if normal_dot < self.lock_marker_min_normal_dot:
            self.warn_throttled(
                f'tilt_{marker_frame}',
                f'⚠️ {marker_frame} 법선이 작업대 평면과 맞지 않아 샘플을 버립니다 '
                f'(기울기={np.rad2deg(np.arccos(np.clip(normal_dot, -1.0, 1.0))):.1f}°). '
                '마커가 작업대에 평평하게 붙어 있는지 확인하세요.'
            )
            return None
        return tvec, rvec

    @staticmethod
    def quaternion_angle(q1, q2):
        # q와 -q는 같은 회전이므로 |dot|을 사용한다.
        dot = min(1.0, abs(float(np.dot(q1, q2))))
        return 2.0 * np.arccos(dot)

    @staticmethod
    def quaternion_mean(quats):
        # Markley 평균: sum(q q^T)의 최대 고유벡터. 부호(q/-q)에 무관하다.
        quats = np.asarray(quats, dtype=float)
        _, vectors = np.linalg.eigh(quats.T @ quats)
        q = vectors[:, -1]
        return q / np.linalg.norm(q)

    def robust_average_pose(self, samples):
        """(tvec, quat) 샘플에서 일부 가림/pose flip으로 튄 값을 버리고 평균을 낸다."""
        tvecs = np.array([sample[0] for sample in samples], dtype=float)
        quats = np.array([sample[1] for sample in samples], dtype=float)
        t_ref = np.median(tvecs, axis=0)
        # 회전 기준은 다른 샘플들과의 각도 합이 최소인 샘플(medoid)로 잡아 flip 영향을 줄인다.
        angle_matrix = np.array([[self.quaternion_angle(a, b) for b in quats] for a in quats])
        q_ref = quats[int(np.argmin(angle_matrix.sum(axis=1)))]
        inliers = [
            index for index in range(len(samples))
            if np.linalg.norm(tvecs[index] - t_ref) <= self.lock_outlier_translation_m
            and self.quaternion_angle(quats[index], q_ref) <= self.lock_outlier_rotation_rad
        ]
        if len(inliers) < max(3, len(samples) // 2):
            return None, len(inliers)
        return (
            (tvecs[inliers].mean(axis=0), self.quaternion_mean(quats[inliers])),
            len(inliers),
        )

    def handle_lock_sample(self, frame, tvec, rvec):
        quat = R.from_rotvec(np.asarray(rvec, dtype=float)).as_quat()
        tvec = np.asarray(tvec, dtype=float)

        if frame in self._locked_poses:
            self.check_drift(frame, tvec, quat)
            return

        samples = self._lock_samples[frame]
        samples.append((tvec, quat))
        if len(samples) < self.lock_sample_count:
            return

        pose, inlier_count = self.robust_average_pose(samples)
        samples.clear()
        if pose is None:
            self.get_logger().warn(
                f'⚠️ {frame} lock 실패: inlier {inlier_count}/{self.lock_sample_count}. '
                '마커가 가려지거나 흔들리는지 확인하세요. 다시 수집합니다.'
            )
            return

        self._locked_poses[frame] = pose
        self._static_transforms[frame] = self.make_transform(
            self.camera_frame, frame, pose[0], pose[1], self.get_clock().now().to_msg()
        )
        self.send_static_transforms()
        self.get_logger().info(
            f'🔒 {frame} lock 완료 (inlier {inlier_count}/{self.lock_sample_count}): '
            f't=({pose[0][0]:.4f}, {pose[0][1]:.4f}, {pose[0][2]:.4f}) '
            f'q=({pose[1][0]:.4f}, {pose[1][1]:.4f}, {pose[1][2]:.4f}, {pose[1][3]:.4f})'
        )
        pending = sorted(self.lock_frames - set(self._locked_poses))
        if pending:
            self.get_logger().info(f'⏳ lock 대기 중: {pending}')
        else:
            self.get_logger().info('✅ 모든 기준 프레임 lock 완료 (/tf_static)')

    def check_drift(self, frame, tvec, quat):
        locked_t, locked_q = self._locked_poses[frame]
        translation_error = float(np.linalg.norm(tvec - locked_t))
        rotation_error = self.quaternion_angle(quat, locked_q)
        if (
            translation_error <= self.drift_warn_translation_m
            and rotation_error <= self.drift_warn_rotation_rad
        ):
            self._drift_counts[frame] = 0
            return
        self._drift_counts[frame] += 1
        if self._drift_counts[frame] >= self.drift_warn_frames:
            self.warn_throttled(
                f'drift_{frame}',
                f'⚠️ {frame}가 lock 값과 계속 다릅니다 '
                f'(Δt={translation_error * 1000:.1f}mm, Δθ={np.rad2deg(rotation_error):.1f}°). '
                '카메라/마커가 움직였다면 aruco_calib을 재시작하세요.'
            )

    def make_transform(self, parent, child, tvec, quat, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        q, _ = self.normalize_quaternion_xyzw(quat, child)
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        return t

    def broadcast_tf(self, parent, child, tvec, rvec, stamp, quat=None):
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
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
