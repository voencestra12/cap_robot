import time
from collections import deque

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener


class ArucoCalibNode(Node):
    def __init__(self):
        super().__init__('aruco_calib')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_size_m', 0.05)
        self.declare_parameter('target_debug_log_period_sec', 2.0)
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
        self._last_warn_time = {}
        self._last_detected_ids_log_time = 0.0

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

        self.x1_buf, self.y1_buf, self.z1_buf = deque(maxlen=10), deque(maxlen=10), deque(maxlen=10)
        self.x2_buf, self.y2_buf, self.z2_buf = deque(maxlen=10), deque(maxlen=10), deque(maxlen=10)

        self.bridge = CvBridge()
        self.camera_info_received = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # marker -> robot_base는 고정 캘리브레이션이므로 /tf_static으로 1회 송신한다.
        # camera -> marker는 매 프레임 동적으로 송신하므로, 이 static transform과 연결되어
        # camera -> marker_1 -> robot_1_base 체인이 만들어진다.
        self.publish_static_robot_base_offsets()

        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
            aruco.DetectorParameters(),
        )

        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
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

        self.target_pub_1 = self.create_publisher(Point, '/robot_1_target', 10)
        self.target_pub_2 = self.create_publisher(Point, '/robot_2_target', 10)

        self.get_logger().info(
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
        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info(
            '📌 static TF 송신 완료: marker_1 -> robot_1_base, marker_12 -> robot_2_base'
        )

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
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size_m, self.mtx, self.dist
            )
            for i, m_id in enumerate(ids.flatten()):
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

            if len(object_points) >= 4:
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.mtx,
                    self.dist,
                )
                if success:
                    self.broadcast_tf(
                        self.camera_frame,
                        'workspace_0',
                        tvec.flatten(),
                        rvec.flatten(),
                        msg.header.stamp,
                    )
            else:
                self.warn_throttled(
                    'workspace_markers',
                    f'workspace_0 계산용 대응점 부족: matched_marker_count={len(matched_pts)}, '
                    f'corner_count={len(object_points)} (필요: 4개 이상 corner, 현재 ids={detected_ids})'
                )

            self.publish_targets()

        cv2.imshow('Final System', color_image)
        cv2.waitKey(1)

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

    def publish_targets(self):
        try:
            trans1 = self.tf_buffer.lookup_transform('robot_1_base', 'workspace_0', rclpy.time.Time())
            robot1_x = trans1.transform.translation.x * 1000
            robot1_y = trans1.transform.translation.y * 1000
            robot1_z = trans1.transform.translation.z * 1000

            self.x1_buf.append(robot1_x)
            self.y1_buf.append(robot1_y)
            self.z1_buf.append(robot1_z)
            msg1 = Point()
            msg1.x = float(np.mean(self.x1_buf))
            msg1.y = float(np.mean(self.y1_buf))
            msg1.z = float(np.mean(self.z1_buf))
            self.target_pub_1.publish(msg1)
        except Exception as error:
            self.warn_throttled('robot_1_target', f'robot_1_base <- workspace_0 조회 실패: {error}')

        try:
            trans2 = self.tf_buffer.lookup_transform('robot_2_base', 'workspace_0', rclpy.time.Time())
            robot2_x = trans2.transform.translation.x * 1000
            robot2_y = trans2.transform.translation.y * 1000
            robot2_z = trans2.transform.translation.z * 1000

            self.x2_buf.append(robot2_x)
            self.y2_buf.append(robot2_y)
            self.z2_buf.append(robot2_z)
            msg2 = Point()
            msg2.x = float(np.mean(self.x2_buf))
            msg2.y = float(np.mean(self.y2_buf))
            msg2.z = float(np.mean(self.z2_buf))
            self.target_pub_2.publish(msg2)
        except Exception as error:
            self.warn_throttled('robot_2_target', f'robot_2_base <- workspace_0 조회 실패: {error}')


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
