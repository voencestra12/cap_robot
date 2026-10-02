"""Synthetic PnP + real ROS TF transport; never starts camera/robot drivers.

Run with ROS sourced, ROS_LOCALHOST_ONLY=1 and an isolated ROS_DOMAIN_ID.
Only marker detection is substituted with known projected corners.
"""
import ast
import os
from pathlib import Path
import threading
import time
from unittest.mock import Mock

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
cv2 = pytest.importorskip('cv2')
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener

from cap_robot.aruco_calib import ArucoCalibNode

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_LOCALHOST_ONLY') != '1'
    or os.environ.get('ROS_DOMAIN_ID', '0') == '0',
    reason='Set ROS_LOCALHOST_ONLY=1 and a nonzero isolated ROS_DOMAIN_ID for TF tests',
)


def make_agent_cache(buffer, clock):
    # Exercise the actual existing cache timer without importing/starting xArm.
    path = Path(__file__).resolve().parents[1] / 'cap_robot/robot_agent.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'RobotAgentNode')
    names = {'update_workspace_tf_cache', 'tf_lookup_time', 'now_sec'}
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    module = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
    namespace = {'Duration': Duration, 'RclpyTime': Time}
    exec(compile(module, str(path), 'exec'), namespace)
    agent = namespace['RobotAgentNode']()
    agent.tf_buffer = buffer
    agent.get_clock = lambda: clock
    agent.get_logger = Mock(return_value=Mock())
    agent.tf_cache_lock = threading.Lock()
    agent.workspace_tf_cache = {}
    agent.tf_ready_agents = set()
    agent.tf_lookup_delay_sec = 0.1
    agent.workspace_frame = 'workspace_0'
    agent.agent_base_frames = {'agent1': 'robot_1_base', 'agent2': 'robot_2_base'}
    return agent


def configure_camera(node):
    info = CameraInfo()
    info.header.frame_id = node.camera_frame
    info.k = node.mtx.flatten().tolist()
    node.camera_info_callback(info)


def feed_images(node, count=35, workspace_shift=0.0, ids=(0, 2, 4, 11, 1, 12)):
    half = node.marker_size_m / 2
    square = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
    rvec = np.array([np.pi, 0, 0])
    corners = []
    for marker_id in ids:
        if marker_id in node.MARKER_OFFSETS_MM:
            points = square + np.array(node.MARKER_OFFSETS_MM[marker_id]) / 1000
            tvec = np.array([workspace_shift, 0., 1.5])
        else:
            points = square
            tvec = np.array([0.1 if marker_id == 1 else 0.5, 0.1, 1.2])
        projected, _ = cv2.projectPoints(points, rvec, tvec, node.mtx, node.dist)
        corners.append(projected.reshape(1, 4, 2).astype(np.float32))
    node.detector = Mock()
    node.detector.detectMarkers.return_value = (corners, np.array(ids).reshape(-1, 1), [])
    image = node.bridge.cv2_to_imgmsg(np.zeros((480, 640, 3), dtype=np.uint8), encoding='bgr8')
    image.header.frame_id = node.camera_frame
    for i in range(count):
        image.header.stamp = Time(nanoseconds=1_000_000_000 + i * 50_000_000).to_msg()
        node.image_callback(image)


def spin_until(nodes, condition, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.01)
        if condition():
            return
    assert condition(), 'Timed out waiting for isolated ROS TF delivery'


def test_static_tf_late_listener_restart_and_agent_cache():
    rclpy.init()
    node = None
    sink = None
    listener = None
    try:
        node = ArucoCalibNode()
        sent_dynamic = []
        node.tf_broadcaster.sendTransform = sent_dynamic.append
        # No CameraInfo / insufficient workspace markers / insufficient frames.
        feed_images(node)
        assert not node._static_camera_poses
        configure_camera(node)
        feed_images(node, count=29, ids=(0, 2, 4, 1, 12))
        assert not node._static_camera_published
        assert 'workspace_0' not in node._static_camera_poses
        feed_images(node)
        assert node._static_camera_published
        assert len(node._static_camera_poses) == 3
        locked_translation = node._static_camera_poses['workspace_0'][0].copy()
        feed_images(node, workspace_shift=0.5)
        np.testing.assert_array_equal(
            node._static_camera_poses['workspace_0'][0], locked_translation)
        assert sent_dynamic
        assert not {t.child_frame_id for t in sent_dynamic} & set(node.STATIC_CAMERA_CHILDREN)

        # Start listener AFTER publication: transient-local must contain all 5 edges.
        sink = Node('static_tf_test_listener')
        buffer = Buffer()
        listener = TransformListener(buffer, sink)
        ready = lambda: all(buffer.can_transform(base, 'workspace_0', Time())
                            for base in ('robot_1_base', 'robot_2_base'))
        spin_until([node, sink], ready)
        for child in node.STATIC_CAMERA_CHILDREN:
            transform = buffer.lookup_transform(node.camera_frame, child, Time())
            assert transform.header.frame_id == node.camera_frame
        for marker, base in [('marker_1', 'robot_1_base'), ('marker_12', 'robot_2_base')]:
            assert buffer.lookup_transform(marker, base, Time()).header.frame_id == marker
        agent = make_agent_cache(buffer, sink.get_clock())
        agent.update_workspace_tf_cache()
        old = {key: entry[0].transform.translation for key, entry in agent.workspace_tf_cache.items()}
        assert set(old) == {'agent1', 'agent2'}
        for base in ('robot_1_base', 'robot_2_base'):
            # No further images or dynamic updates are needed, even far in future.
            buffer.lookup_transform(base, 'workspace_0', Time(seconds=100_000))

        node.destroy_node()
        node = ArucoCalibNode()
        configure_camera(node)
        assert not node._static_camera_published
        feed_images(node, workspace_shift=0.05)
        assert node._static_camera_published
        spin_until([node, sink], lambda: abs(buffer.lookup_transform(
            node.camera_frame, 'workspace_0', Time()).transform.translation.x - 0.05) < 1e-4)
        agent.update_workspace_tf_cache()
        for key, entry in agent.workspace_tf_cache.items():
            a, b = old[key], entry[0].transform.translation
            assert np.linalg.norm([a.x - b.x, a.y - b.y, a.z - b.z]) == pytest.approx(0.05, abs=1e-4)
    finally:
        if listener is not None:
            listener.unregister()
        if sink is not None:
            sink.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_pose_quality_rejects_bad_projection_depth_and_frame():
    rclpy.init()
    node = None
    try:
        node = ArucoCalibNode()
        configure_camera(node)
        points = np.array([[-.025, .025, 0], [.025, .025, 0],
                           [.025, -.025, 0], [-.025, -.025, 0]])
        rotvec = np.array([np.pi, 0, 0])
        translation = np.array([0., 0., 1.])
        projected, _ = cv2.projectPoints(points, rotvec, translation, node.mtx, node.dist)
        for i in range(40):
            stamp = Time(nanoseconds=i * 100_000_000).to_msg()
            node.observe_static_pose('marker_1', points, projected + 10, translation, rotvec, stamp)
            node.observe_static_pose('marker_12', points, projected, -translation, rotvec, stamp)
        assert not node._static_camera_poses
        assert all(not estimator.samples for estimator in node._pose_estimators.values())
        image = node.bridge.cv2_to_imgmsg(np.zeros((10, 10, 3), dtype=np.uint8), encoding='bgr8')
        image.header.frame_id = 'different_camera'
        node.image_callback(image)
        assert node._calibration_frame is None
        feed_images(node, count=1)
        info = CameraInfo()
        info.header.frame_id = node.camera_frame
        info.k = (node.mtx * 2).flatten().tolist()
        node.camera_info_callback(info)
        assert not node.camera_info_received
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
