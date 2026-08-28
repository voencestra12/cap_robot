"""바구니 손잡이용 추가 인식 노드.

[MERGED] 세트 1의 빨간 스티커 기반 손잡이 인식을 ROS 이미지 토픽 기반으로
분리했습니다. ArUco 캘리브레이션과 RealSense 장치 소유권을 공유하지 않으며,
모든 외부 좌표는 ``workspace_0`` 기준으로 발행합니다.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from cap_robot.utils import quaternion_xyzw_to_matrix


class YoloExtraPerception(Node):
    """두 빨간 손잡이를 검출하고 workspace 좌표 및 접근성 정보를 발행합니다."""

    def __init__(self):
        super().__init__('yolo_extra_perception')

        self.declare_parameter('color_topic', '/fixed_camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic',
            '/fixed_camera/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'camera_info_topic',
            '/fixed_camera/camera/color/camera_info',
        )
        self.declare_parameter('workspace_frame', 'workspace_0')
        self.declare_parameter('state_topic', '/perception/yolo_extra')
        self.declare_parameter('handle_0_topic', '/basket/handle_0_pose')
        self.declare_parameter('handle_1_topic', '/basket/handle_1_pose')
        self.declare_parameter(
            'agent_base_frames',
            ['agent1:robot_1_base', 'agent2:robot_2_base'],
        )
        self.declare_parameter('minimum_area_px', 50.0)
        self.declare_parameter('depth_window_radius_px', 2)
        self.declare_parameter('smoothing_window', 5)
        self.declare_parameter('maximum_frame_age_sec', 0.5)
        self.declare_parameter('tf_lookup_timeout_sec', 0.3)
        self.declare_parameter('show_window', False)
        self.declare_parameter('sdk_x_min', 50.0)
        self.declare_parameter('sdk_x_max', 750.0)
        self.declare_parameter('sdk_y_min', -600.0)
        self.declare_parameter('sdk_y_max', 600.0)
        self.declare_parameter('sdk_z_min', 20.0)
        self.declare_parameter('sdk_z_max', 700.0)

        self.color_topic = str(self.get_parameter('color_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.workspace_frame = str(self.get_parameter('workspace_frame').value).strip()
        self.state_topic = str(self.get_parameter('state_topic').value).strip()
        self.minimum_area_px = float(self.get_parameter('minimum_area_px').value)
        self.depth_radius = max(0, int(self.get_parameter('depth_window_radius_px').value))
        self.maximum_frame_age_sec = float(
            self.get_parameter('maximum_frame_age_sec').value
        )
        self.tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_lookup_timeout_sec').value)
        )
        self.show_window = bool(self.get_parameter('show_window').value)
        self.base_frames = self._parse_agent_base_frames(
            self.get_parameter('agent_base_frames').value
        )
        self.safety = {
            'x': (
                float(self.get_parameter('sdk_x_min').value),
                float(self.get_parameter('sdk_x_max').value),
            ),
            'y': (
                float(self.get_parameter('sdk_y_min').value),
                float(self.get_parameter('sdk_y_max').value),
            ),
            'z': (
                float(self.get_parameter('sdk_z_min').value),
                float(self.get_parameter('sdk_z_max').value),
            ),
        }

        smoothing_window = max(1, int(self.get_parameter('smoothing_window').value))
        self._position_history = [
            deque(maxlen=smoothing_window),
            deque(maxlen=smoothing_window),
        ]
        self._last_positions = None
        self._last_depth = None
        self._last_depth_stamp_sec = 0.0
        self._intrinsics = None
        self._bridge = CvBridge()
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_warning = {}

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Image, self.color_topic, self._color_callback, sensor_qos)
        self.create_subscription(Image, self.depth_topic, self._depth_callback, sensor_qos)
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            sensor_qos,
        )
        self._state_pub = self.create_publisher(String, self.state_topic, 10)
        self._pose_pubs = [
            self.create_publisher(
                PoseStamped,
                str(self.get_parameter('handle_0_topic').value),
                10,
            ),
            self.create_publisher(
                PoseStamped,
                str(self.get_parameter('handle_1_topic').value),
                10,
            ),
        ]

        self.get_logger().info(
            '[MERGED] yolo_extra_perception 시작: '
            f'color={self.color_topic}, depth={self.depth_topic}, '
            f'output={self.state_topic}, frame={self.workspace_frame}'
        )

    @staticmethod
    def _parse_agent_base_frames(values):
        result = {}
        for raw in values:
            parts = str(raw).split(':', 1)
            if len(parts) == 2 and parts[0].strip() and parts[1].strip():
                result[parts[0].strip()] = parts[1].strip()
        if not result:
            raise ValueError('agent_base_frames에 agent_id:frame 항목이 필요합니다.')
        return result

    @staticmethod
    def _stamp_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _warn_throttled(self, key, message, period_sec=2.0):
        now = time.monotonic()
        if now - self._last_warning.get(key, 0.0) >= period_sec:
            self._last_warning[key] = now
            self.get_logger().warn(message)

    def _camera_info_callback(self, msg):
        if len(msg.k) != 9:
            return
        self._intrinsics = {
            'fx': float(msg.k[0]),
            'fy': float(msg.k[4]),
            'cx': float(msg.k[2]),
            'cy': float(msg.k[5]),
            'frame_id': msg.header.frame_id,
        }

    def _depth_callback(self, msg):
        try:
            self._last_depth = self._bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough',
            ).copy()
            self._last_depth_stamp_sec = self._stamp_sec(msg.header.stamp)
        except Exception as error:
            self._warn_throttled('depth_decode', f'Depth 이미지 변환 실패: {error}')

    def _median_depth_m(self, u, v):
        if self._last_depth is None:
            return None
        height, width = self._last_depth.shape[:2]
        u0, u1 = max(0, u - self.depth_radius), min(width, u + self.depth_radius + 1)
        v0, v1 = max(0, v - self.depth_radius), min(height, v + self.depth_radius + 1)
        values = np.asarray(self._last_depth[v0:v1, u0:u1], dtype=float).reshape(-1)
        values = values[np.isfinite(values) & (values > 0.0)]
        if not values.size:
            return None
        value = float(np.median(values))
        # RealSense z16는 mm, 32FC1은 m입니다.
        return value * 0.001 if self._last_depth.dtype == np.uint16 else value

    def _lookup_transform(self, target_frame, source_frame, stamp):
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=self.tf_timeout,
            )
        except Exception:
            # 카메라/TF 타임스탬프가 조금 어긋난 경우 최신 TF를 한 번 사용합니다.
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=self.tf_timeout,
            )

    @staticmethod
    def _transform_point_mm(transform, xyz_m):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = quaternion_xyzw_to_matrix(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )
        transformed = matrix @ np.asarray(xyz_m, dtype=float) + np.array(
            [translation.x, translation.y, translation.z],
            dtype=float,
        )
        return transformed * 1000.0

    def _deproject(self, u, v, depth_m):
        i = self._intrinsics
        return np.array([
            (float(u) - i['cx']) * depth_m / i['fx'],
            (float(v) - i['cy']) * depth_m / i['fy'],
            depth_m,
        ])

    def _detect_candidates(self, color_image, camera_frame, stamp):
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower_1, upper_1 = np.array([0, 120, 70]), np.array([10, 255, 255])
        lower_2, upper_2 = np.array([170, 120, 70]), np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_1, upper_1) | cv2.inRange(hsv, lower_2, upper_2)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        try:
            workspace_from_camera = self._lookup_transform(
                self.workspace_frame,
                camera_frame,
                stamp,
            )
        except Exception as error:
            self._warn_throttled(
                'workspace_tf',
                f'{self.workspace_frame} <- {camera_frame} TF 조회 실패: {error}',
            )
            return [], mask

        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.minimum_area_px:
                continue
            moments = cv2.moments(contour)
            if abs(moments['m00']) < 1e-9:
                continue
            u = int(round(moments['m10'] / moments['m00']))
            v = int(round(moments['m01'] / moments['m00']))
            depth_m = self._median_depth_m(u, v)
            if depth_m is None or not 0.05 <= depth_m <= 5.0:
                continue
            camera_xyz = self._deproject(u, v, depth_m)
            workspace_xyz = self._transform_point_mm(workspace_from_camera, camera_xyz)
            candidates.append({
                'position': workspace_xyz,
                'pixel': (u, v),
                'area_px': area,
                'depth_m': depth_m,
            })
            if len(candidates) == 2:
                break
        return candidates, mask

    def _stabilize(self, candidates):
        if len(candidates) != 2:
            return None
        if self._last_positions is None:
            ordered = sorted(candidates, key=lambda item: float(item['position'][0]))
        else:
            orders = [candidates, list(reversed(candidates))]
            ordered = min(
                orders,
                key=lambda pair: sum(
                    np.linalg.norm(pair[index]['position'] - self._last_positions[index])
                    for index in range(2)
                ),
            )
        stabilized = []
        for index, candidate in enumerate(ordered):
            self._position_history[index].append(candidate['position'])
            item = dict(candidate)
            item['position'] = np.mean(self._position_history[index], axis=0)
            stabilized.append(item)
        self._last_positions = [item['position'].copy() for item in stabilized]
        return stabilized

    def _inside_safety_box(self, xyz_mm):
        x, y, z = map(float, xyz_mm)
        return (
            self.safety['x'][0] <= x <= self.safety['x'][1]
            and self.safety['y'][0] <= y <= self.safety['y'][1]
            and self.safety['z'][0] <= z <= self.safety['z'][1]
        )

    def _reachability(self, handles, stamp):
        result = {}
        for agent_id, base_frame in self.base_frames.items():
            try:
                base_from_workspace = self._lookup_transform(
                    base_frame,
                    self.workspace_frame,
                    stamp,
                )
            except Exception as error:
                result[agent_id] = {'valid': False, 'reason': f'TF 조회 실패: {error}'}
                continue
            per_handle = {}
            for index, handle in enumerate(handles):
                base_xyz = self._transform_point_mm(
                    base_from_workspace,
                    np.asarray(handle['position'], dtype=float) * 0.001,
                )
                grasp_workspace = np.asarray(handle['position'], dtype=float).copy()
                grasp_workspace[2] += 10.0
                approach_workspace = np.asarray(handle['position'], dtype=float).copy()
                approach_workspace[2] += 200.0
                grasp = self._transform_point_mm(
                    base_from_workspace, grasp_workspace * 0.001
                )
                approach = self._transform_point_mm(
                    base_from_workspace, approach_workspace * 0.001
                )
                per_handle[f'basket_handle_{index}'] = {
                    'base_frame': base_frame,
                    'base_xyz_mm': [round(float(value), 2) for value in base_xyz],
                    'within_safety_box': bool(
                        self._inside_safety_box(grasp)
                        and self._inside_safety_box(approach)
                    ),
                    'radial_distance_mm': round(float(np.linalg.norm(base_xyz[:2])), 2),
                }
            result[agent_id] = {'valid': True, 'handles': per_handle}
        return result

    def _publish_invalid(self, stamp, reason):
        payload = {
            'schema_version': 1,
            'source': 'red_handle_detector',
            'frame_id': self.workspace_frame,
            'stamp_sec': self._stamp_sec(stamp),
            'valid': False,
            'reason': reason,
            'objects': {},
            'reachability': {},
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._state_pub.publish(msg)

    def _publish_valid(self, handles, stamp):
        first, second = (item['position'] for item in handles)
        line_yaw = math.atan2(second[1] - first[1], second[0] - first[0])
        grasp_yaw = line_yaw + math.pi / 2.0
        qz, qw = math.sin(grasp_yaw / 2.0), math.cos(grasp_yaw / 2.0)
        objects = {}
        for index, item in enumerate(handles):
            name = f'basket_handle_{index}'
            x, y, z = map(float, item['position'])
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.workspace_frame
            pose.pose.position.x = x * 0.001
            pose.pose.position.y = y * 0.001
            pose.pose.position.z = z * 0.001
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self._pose_pubs[index].publish(pose)
            objects[name] = {
                'x_mm': round(x, 2),
                'y_mm': round(y, 2),
                'z_mm': round(z, 2),
                'yaw_deg': round(math.degrees(grasp_yaw), 2),
                'area_px': round(float(item['area_px']), 1),
                'depth_m': round(float(item['depth_m']), 4),
            }

        payload = {
            'schema_version': 1,
            'source': 'red_handle_detector',
            'frame_id': self.workspace_frame,
            'stamp_sec': self._stamp_sec(stamp),
            'valid': True,
            'objects': objects,
            'reachability': self._reachability(handles, stamp),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._state_pub.publish(msg)

    def _color_callback(self, msg):
        if self._last_depth is None or self._intrinsics is None:
            self._publish_invalid(msg.header.stamp, 'depth 또는 CameraInfo 대기 중')
            return
        color_stamp = self._stamp_sec(msg.header.stamp)
        if abs(color_stamp - self._last_depth_stamp_sec) > self.maximum_frame_age_sec:
            self._publish_invalid(msg.header.stamp, 'color/depth 시간 차 초과')
            return
        try:
            color = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            camera_frame = msg.header.frame_id or self._intrinsics['frame_id']
            candidates, mask = self._detect_candidates(color, camera_frame, msg.header.stamp)
            handles = self._stabilize(candidates)
            if handles is None:
                self._publish_invalid(
                    msg.header.stamp,
                    f'유효한 빨간 손잡이 2개 필요: detected={len(candidates)}',
                )
            else:
                self._publish_valid(handles, msg.header.stamp)
                for index, item in enumerate(handles):
                    cv2.circle(color, item['pixel'], 6, (0, 255, 0), -1)
                    cv2.putText(
                        color,
                        f'handle_{index}',
                        (item['pixel'][0] + 7, item['pixel'][1] - 7),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                    )
            if self.show_window:
                cv2.imshow('yolo_extra_perception', color)
                cv2.imshow('yolo_extra_mask', mask)
                cv2.waitKey(1)
        except Exception as error:
            self._warn_throttled('color_callback', f'손잡이 인식 실패: {error}')
            self._publish_invalid(msg.header.stamp, str(error))


def main(args=None):
    rclpy.init(args=args)
    node = YoloExtraPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
