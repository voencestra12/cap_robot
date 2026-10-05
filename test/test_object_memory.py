"""동종 다중 개체, 순서 변경, 가림 및 모호한 target의 회귀 검증."""

import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import cv2
import numpy as np

from cap_robot.object_memory import ObjectMemory, object_class, resolve_object_target


def detection(x, name='빵', y=0, z=30):
    return name, (x, y, z, 0)


class ObjectMemoryTest(unittest.TestCase):
    def test_all_instances_are_retained_even_when_close(self):
        memory = ObjectMemory()
        poses = memory.update([detection(100), detection(110), detection(200, '햄')], now=0)
        self.assertEqual(list(poses), ['빵_1', '빵_2', '햄_1'])
        self.assertEqual(poses['빵_1'][0], 100)
        self.assertEqual(poses['빵_2'][0], 110)

    def test_reordered_noisy_detections_keep_ids(self):
        memory = ObjectMemory()
        memory.update([detection(100), detection(200)], now=0)
        poses = memory.update([detection(202), detection(98)], now=1)
        self.assertEqual(list(poses), ['빵_2', '빵_1'])
        self.assertEqual(poses['빵_1'][0], 98)
        self.assertEqual(poses['빵_2'][0], 202)

    def test_global_assignment_avoids_losing_a_valid_match(self):
        memory = ObjectMemory(match_distance_mm=60)
        memory.update([detection(0), detection(70)], now=0)
        # x=30을 가장 가까운 0에 먼저 연결하면 x=-40은 매칭되지 못합니다.
        poses = memory.update([detection(30), detection(-40)], now=1)
        self.assertEqual(poses['빵_1'][0], -40)
        self.assertEqual(poses['빵_2'][0], 30)

    def test_short_occlusion_retains_id_but_hides_old_pose(self):
        memory = ObjectMemory(retention_sec=3)
        memory.update([detection(100), detection(200)], now=0)
        poses = memory.update([detection(201)], now=1)
        self.assertEqual(set(poses), {'빵_2'})
        poses = memory.update([detection(102), detection(202)], now=2)
        self.assertEqual(set(poses), {'빵_1', '빵_2'})

    def test_expired_ids_are_not_reused(self):
        memory = ObjectMemory(retention_sec=3)
        memory.update([detection(100)], now=0)
        self.assertEqual(memory.update([], now=1), {})
        poses = memory.update([detection(100)], now=4)
        self.assertEqual(set(poses), {'빵_2'})

    def test_distance_and_class_gate_matching(self):
        memory = ObjectMemory(match_distance_mm=60)
        memory.update([detection(100)], now=0)
        poses = memory.update([detection(161), detection(100, '햄')], now=1)
        self.assertEqual(set(poses), {'빵_2', '햄_1'})

    def test_distance_is_three_dimensional(self):
        memory = ObjectMemory(match_distance_mm=60)
        memory.update([detection(100)], now=0)
        self.assertEqual(set(memory.update([detection(100, z=100)], now=1)), {'빵_2'})

    def test_invalid_coordinates_do_not_modify_memory(self):
        memory = ObjectMemory()
        memory.update([detection(100)], now=0)
        for value in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                memory.update([detection(value)], now=1)
        self.assertEqual(set(memory.update([detection(101)], now=2)), {'빵_1'})

    def test_invalid_configuration(self):
        for value in (-1, 0, float('inf'), float('nan')):
            with self.assertRaises(ValueError):
                ObjectMemory(match_distance_mm=value)
            with self.assertRaises(ValueError):
                ObjectMemory(retention_sec=value)


class TargetResolutionTest(unittest.TestCase):
    def test_exact_id_or_unique_class_only(self):
        poses = {'빵_1': (), '빵_2': (), '햄_1': ()}
        self.assertEqual(resolve_object_target('빵_2', poses), '빵_2')
        self.assertIsNone(resolve_object_target('빵', poses))
        self.assertEqual(resolve_object_target('햄', poses), '햄_1')
        self.assertIsNone(resolve_object_target('없는물체', poses))

    def test_missing_id_does_not_match_prefix_or_old_class_key(self):
        self.assertIsNone(resolve_object_target('빵_1', {'빵_10': ()}))
        self.assertIsNone(resolve_object_target('빵_1', {'빵': ()}))
        self.assertIsNone(resolve_object_target('', {'빵_1': ()}))
        self.assertEqual(resolve_object_target('빵', {'빵': ()}), '빵')

    def test_class_extraction(self):
        self.assertEqual(object_class('빵_12'), '빵')
        self.assertEqual(object_class('빵'), '빵')
        self.assertEqual(object_class('some_class_2'), 'some_class')
        self.assertEqual(object_class('basket_handle_0'), 'basket_handle_0')


class PerceptionIntegrationTest(unittest.TestCase):
    def test_yolo_loop_exposes_every_instance_to_prompt(self):
        # 실제 perception 메서드를 실행하되 카메라/YOLO/GUI/ROS만 대체합니다.
        path = Path(__file__).resolve().parents[1] / 'cap_robot/robot_agent.py'
        tree = ast.parse(path.read_text())
        node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                          and n.name == 'RobotAgentNode')
        node_class.bases = []
        node_class.body = [n for n in node_class.body if isinstance(n, ast.FunctionDef)
                           and n.name in ('run_perception', 'poses_for_prompt')]
        drawing = Mock(wraps=cv2)
        drawing.FONT_HERSHEY_SIMPLEX = cv2.FONT_HERSHEY_SIMPLEX
        drawing.imshow = Mock()
        drawing.waitKey = Mock(side_effect=[-1, ord('q')])
        drawing.destroyAllWindows = Mock()
        namespace = dict(np=np, math=math, cv2=drawing, rclpy=Mock(ok=lambda: True),
                         object_class=object_class)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[node_class], type_ignores=[])),
                     str(path), 'exec'), namespace)
        node = namespace['RobotAgentNode']()
        node.agent_id = 'test'
        node.is_moving = False
        node.perception_lock = threading.Lock()
        node.object_memory = ObjectMemory()
        node.get_latest_perception_frames = lambda: (
            np.zeros((100, 200, 3), dtype=np.uint8), None, {},
        )
        node.median_depth_m = lambda depth, x, y: None if x > 150 else 1.0
        node.deproject_pixel_to_point = lambda x, y, depth, intrinsics: (x, y, depth)
        node.camera_point_to_workspace = lambda x, y, z: (x, y, z)

        def result(centers):
            masks = [np.array([[x - 5, 10], [x + 5, 10], [x + 5, 30], [x - 5, 30]],
                              dtype=np.float32) for x in centers]
            return [SimpleNamespace(boxes=[SimpleNamespace(cls=[0]) for _ in centers],
                                    masks=SimpleNamespace(xy=masks))]

        # 마지막 물체는 depth가 없으므로 후보에서 제외되어야 합니다.
        model = SimpleNamespace(names={0: 'bread'}, predict=Mock(side_effect=[
            result([20, 80, 180]), result([81, 21, 180]),
        ]))
        node.models = [('test.pt', model)]
        node.run_perception()
        self.assertEqual(node.current_detected_items, ['빵_2', '빵_1'])
        self.assertEqual(node.latest_poses['빵_1'][0], 21)
        self.assertEqual(node.latest_poses['빵_2'][0], 81)
        prompt = node.poses_for_prompt(node.latest_poses)
        self.assertEqual(prompt['빵_2']['class_name'], '빵')
        self.assertEqual(prompt['빵_2']['x_mm'], 81)
        labels = [call.args[1] for call in drawing.putText.call_args_list]
        self.assertTrue(any('[Bread_1]' in label for label in labels))
        self.assertTrue(any('[Bread_2]' in label for label in labels))


if __name__ == '__main__':
    unittest.main()
