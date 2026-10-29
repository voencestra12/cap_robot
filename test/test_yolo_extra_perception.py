"""실제 ATS에 지연/누락 프레임을 넣어 pairing 및 기존 출력 형식을 검증한다."""

from contextlib import ExitStack
import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
from geometry_msgs.msg import TransformStamped
import numpy as np
from rclpy.node import Node

from cap_robot.yolo_extra_perception import YoloExtraPerception
from cap_robot.material_handling import basket_center_from_perception


class SynchronizedPerceptionTest(unittest.TestCase):
    def setUp(self):
        # ROS graph/카메라 없이 실제 Subscriber, ATS, CvBridge 및 검출 로직을 사용한다.
        stack = ExitStack()
        self.addCleanup(stack.close)
        parameters = self.parameters = {}
        stack.enter_context(patch.object(Node, '__init__', return_value=None))
        stack.enter_context(patch.object(
            Node, 'declare_parameter', side_effect=lambda key, value: parameters.setdefault(key, value),
        ))
        stack.enter_context(patch.object(
            Node, 'get_parameter', side_effect=lambda key: SimpleNamespace(value=parameters[key]),
        ))
        self.subscribe = stack.enter_context(patch.object(Node, 'create_subscription'))
        stack.enter_context(patch.object(Node, 'create_publisher', side_effect=lambda *a: Mock()))
        stack.enter_context(patch.object(Node, 'get_logger', return_value=Mock()))
        stack.enter_context(patch('cap_robot.yolo_extra_perception.Buffer'))
        stack.enter_context(patch('cap_robot.yolo_extra_perception.TransformListener'))
        self.node = YoloExtraPerception()
        self.node._intrinsics = dict(fx=100.0, fy=100.0, cx=50.0, cy=50.0,
                                     frame_id='camera_color_optical_frame')
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        self.node._lookup_transform = Mock(return_value=transform)

    def image(self, stamp_sec, *, depth=False, depth_mm=1000, sticker_bgr=(255, 0, 0)):
        if depth:
            data = np.full((100, 100), depth_mm, dtype=np.uint16)
            encoding = '16UC1'
        else:
            data = np.zeros((100, 100, 3), dtype=np.uint8)
            for center in ((25, 50), (75, 50)):
                cv2.circle(data, center, 8, sticker_bgr, -1)
            encoding = 'bgr8'
        msg = self.node._bridge.cv2_to_imgmsg(data, encoding=encoding)
        stamp_ns = round(stamp_sec * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.header.frame_id = 'camera_color_optical_frame'
        return msg

    def payload(self):
        return json.loads(self.node._state_pub.publish.call_args.args[0].data)

    def test_subscribers_use_approximate_sync_and_keep_camera_info_subscription(self):
        self.assertEqual(self.node.sync_queue_size, 10)
        self.assertEqual(self.node._image_sync.slop.nanoseconds, 50_000_000)
        self.assertEqual(self.subscribe.call_count, 3)
        self.assertEqual(self.subscribe.call_args.args[2], self.node._camera_info_callback)
        self.assertEqual(self.node.maximum_frame_age_sec, 0.5)

    def test_delayed_depth_matches_queued_color_instead_of_latest_depth(self):
        self.node._depth_sub.signalMessage(self.image(101, depth=True, depth_mm=2000))
        self.node._color_sub.signalMessage(self.image(100))
        self.node._state_pub.publish.assert_not_called()
        self.node._depth_sub.signalMessage(self.image(100.02, depth=True))
        payload = self.payload()
        self.assertTrue(payload['valid'])
        self.assertEqual(set(payload), {
            'schema_version', 'source', 'frame_id', 'stamp_sec',
            'valid', 'objects', 'reachability',
        })
        self.assertEqual(payload['source'], 'blue_handle_detector')
        self.assertEqual(payload['frame_id'], 'workspace_0')
        self.assertEqual(payload['stamp_sec'], 100)
        self.assertEqual(set(payload['objects']), {'basket_handle_0', 'basket_handle_1'})
        for index, x_mm in enumerate((-250, 250)):
            handle = payload['objects'][f'basket_handle_{index}']
            self.assertEqual(handle['depth_m'], 1.0)
            self.assertEqual([handle[k] for k in ('x_mm', 'y_mm', 'z_mm')], [x_mm, 0, 1000])
            self.assertEqual(set(handle), {
                'x_mm', 'y_mm', 'z_mm', 'yaw_deg', 'area_px', 'depth_m',
            })
            pose = self.node._pose_pubs[index].publish.call_args.args[0]
            self.assertEqual(pose.header.frame_id, 'workspace_0')
            self.assertEqual(pose.header.stamp.sec, 100)
            self.assertEqual(pose.pose.position.x, x_mm * 0.001)
            self.assertEqual(pose.pose.position.z, 1.0)

    def test_missing_pairs_do_not_reuse_depth_and_queues_remain_bounded(self):
        self.node._depth_sub.signalMessage(self.image(100, depth=True))
        for stamp in range(101, 121):
            self.node._color_sub.signalMessage(self.image(stamp))
        self.node._state_pub.publish.assert_not_called()
        self.assertTrue(all(len(q) <= 10 for q in self.node._image_sync.queues))
        self.node._depth_sub.signalMessage(self.image(120.02, depth=True))
        self.assertTrue(self.payload()['valid'])
        self.assertEqual(self.payload()['stamp_sec'], 120)

    def test_callback_rejects_pair_outside_slop_before_detection(self):
        with patch.object(self.node, '_detect_candidates') as detect:
            self.node._synchronized_callback(self.image(100), self.image(100.1, depth=True))
            detect.assert_not_called()
        self.assertFalse(self.payload()['valid'])
        self.assertEqual(self.payload()['reason'], 'color/depth 시간 차 초과')

    def test_decode_failure_does_not_stall_sync_or_reuse_previous_depth(self):
        for bad_depth in (True, False):
            with self.subTest(depth=bad_depth):
                self.node._state_pub.publish.reset_mock()
                color, depth = self.image(100), self.image(100.02, depth=True)
                (depth if bad_depth else color).encoding = 'invalid_encoding'
                self.node._color_sub.signalMessage(color)
                self.node._depth_sub.signalMessage(depth)
                self.assertFalse(self.payload()['valid'])
                if bad_depth:
                    self.assertIsNone(self.node._last_depth)
                self.node._color_sub.signalMessage(self.image(101))
                self.node._depth_sub.signalMessage(self.image(101.02, depth=True))
                self.assertTrue(self.payload()['valid'])

    def test_camera_info_wait_keeps_invalid_schema(self):
        self.node._intrinsics = None
        self.node._color_sub.signalMessage(self.image(100))
        self.node._depth_sub.signalMessage(self.image(100.02, depth=True))
        payload = self.payload()
        self.assertFalse(payload['valid'])
        self.assertEqual(payload['objects'], {})
        self.assertEqual(payload['reachability'], {})
        self.assertEqual(set(payload), {
            'schema_version', 'source', 'frame_id', 'stamp_sec',
            'valid', 'reason', 'objects', 'reachability',
        })

    def test_blue_stickers_set_basket_center_and_ignore_larger_red_objects(self):
        color = self.image(100)
        data = self.node._bridge.imgmsg_to_cv2(color, desired_encoding='bgr8').copy()
        cv2.rectangle(data, (5, 5), (40, 30), (0, 0, 255), -1)
        cv2.rectangle(data, (60, 65), (95, 95), (0, 0, 255), -1)
        color.data = self.node._bridge.cv2_to_imgmsg(data, encoding='bgr8').data
        self.node._synchronized_callback(color, self.image(100.02, depth=True))
        state = dict(self.payload(), received_age_sec=0.0)
        center = basket_center_from_perception(state, 'workspace_0', 2.0, 100.1)
        self.assertEqual([center[axis] for axis in ('x', 'y', 'z')], [0, 0, 1000])

    def test_red_stickers_do_not_produce_basket_targets(self):
        self.node._synchronized_callback(
            self.image(100, sticker_bgr=(0, 0, 255)), self.image(100.02, depth=True),
        )
        state = self.payload()
        self.assertFalse(state['valid'])
        self.assertEqual(state['source'], 'blue_handle_detector')
        self.assertEqual(state['objects'], {})
        for publisher in self.node._pose_pubs:
            publisher.publish.assert_not_called()

    def test_hsv_thresholds_include_endpoints_and_exclude_outside_values(self):
        self.node._last_depth = np.full((100, 100), 1000, dtype=np.uint16)
        for hsv_value, expected in (
            ((80, 255, 255), True), ((150, 255, 255), True),
            ((120, 80, 255), True), ((120, 255, 40), True),
            ((79, 255, 255), False), ((151, 255, 255), False),
            ((120, 79, 255), False), ((120, 255, 39), False),
        ):
            hsv = np.full((100, 100, 3), hsv_value, dtype=np.uint8)
            with self.subTest(hsv=hsv_value), patch.object(cv2, 'cvtColor', return_value=hsv):
                candidates, mask = self.node._detect_candidates(
                    hsv, 'camera_color_optical_frame', self.image(100).header.stamp,
                )
                self.assertEqual(bool(np.all(mask == 255)), expected)
                self.assertEqual(len(candidates), int(expected))

    def test_hsv_parameter_overrides_are_used(self):
        self.parameters.update(sticker_hsv_lower=[100, 100, 60],
                               sticker_hsv_upper=[130, 240, 250])
        node = YoloExtraPerception()
        np.testing.assert_array_equal(node.sticker_hsv_lower, [100, 100, 60])
        np.testing.assert_array_equal(node.sticker_hsv_upper, [130, 240, 250])

    def test_invalid_hsv_parameter_ranges_are_rejected(self):
        for lower, upper in (
            ([80, 80], [150, 255, 255]),
            ([-1, 80, 40], [150, 255, 255]),
            ([80, 80, 40], [180, 255, 255]),
            ([80, 80, 40], [150, 256, 255]),
            ([151, 80, 40], [150, 255, 255]),
        ):
            self.parameters.update(sticker_hsv_lower=lower, sticker_hsv_upper=upper)
            with self.subTest(lower=lower, upper=upper), self.assertRaises(ValueError):
                YoloExtraPerception()


if __name__ == '__main__':
    unittest.main()
