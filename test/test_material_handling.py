"""재료 설정/적층 계산 및 실제 Agent 메서드의 ROS 없는 실행 검증."""
import ast
import copy
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest

import numpy as np
from unittest.mock import Mock, patch

from cap_robot import object_profiles as settings
from cap_robot.llm_api import DEFAULT_PNP_ACTIONS, validate_actions
from cap_robot.material_handling import (
    assembly_move_pose, assembly_placement, bind_gripper_actions,
    material_profile, validate_assembly_tasks, basket_center_from_perception, placement_tilt,
)

from cap_robot.utils import rpy_to_matrix, matrix_to_rpy, quaternion_xyzw_to_matrix


def perception():
    return {
        'valid': True, 'source': 'red_handle_detector', 'frame_id': 'workspace_0',
        'received_age_sec': 0.1, 'stamp_sec': 100.0,
        'objects': {
            'basket_handle_0': {'x_mm': 250, 'y_mm': 200, 'z_mm': 60},
            'basket_handle_1': {'x_mm': 350, 'y_mm': 200, 'z_mm': 80},
        },
    }


def place(task, tasks, agent_id=None):
    return assembly_placement(
        task, tasks, agent_id,
        basket_center=basket_center_from_perception(perception(), 'workspace_0', 2.0, 100.1),
    )


def recipe():
    return [
        {
            'task_id': f'layer_{i}',
            'depends_on': [] if i == 0 else [f'layer_{i - 1}'],
            'execution_mode': 'single_agent',
            'assembly': {
                'id': 'sandwich_1', 'container': 'basket',
                'layer_index': i, 'material': material,
            },
        }
        for i, material in enumerate(('빵', '양상추', '바나나'))
    ]


def mode_actions():
    actions = copy.deepcopy(DEFAULT_PNP_ACTIONS)
    for i, mode in ((0, 'open'), (3, 'grasp'), (7, 'release')):
        actions[i] = {'api': 'control_gripper', 'mode': mode}
    return actions


class ConfiguredTest(unittest.TestCase):
    def setUp(self):
        # 가상의 테스트 값이며 실제 로봇 설정으로 저장하지 않는다.
        profiles = {
            '빵': {'grip_position': 620, 'thickness_mm': 20, 'place_tcp_offset_mm': 35,
                  'place_pitch_offset_deg': 30, 'place_roll_offset_deg': 0},
            '양상추': {'grip_position': 120, 'thickness_mm': 0, 'place_tcp_offset_mm': 30},
            '바나나': {'grip_position': 400, 'thickness_mm': 35, 'place_tcp_offset_mm': 45},
        }
        config = patch.multiple(
            settings, OBJECT_PROFILES=profiles, BASKET_FLOOR_Z_MM=10,
            BASKET_YAW_DEG=0,
            AGENT_PROFILE_OVERRIDES={},
        )
        config.start()
        self.addCleanup(config.stop)


class MaterialHandlingTest(ConfiguredTest):
    def test_legacy_grasp_is_overridden_but_open_release_stay_open(self):
        actions = bind_gripper_actions(validate_actions(DEFAULT_PNP_ACTIONS, 40), '빵')
        self.assertEqual([actions[i]['position'] for i in (0, 3, 7)], [850, 620, 850])
        self.assertEqual(validate_actions(actions, 40), actions)

    def test_modes_select_each_material(self):
        for material, expected in [('빵', 620), ('양상추', 120), ('바나나', 400)]:
            with self.subTest(material=material):
                actions = bind_gripper_actions(validate_actions(mode_actions(), 40), material)
                self.assertEqual(actions[3]['position'], expected)

    def test_unregistered_legacy_object_keeps_its_position(self):
        actions = bind_gripper_actions(validate_actions(DEFAULT_PNP_ACTIONS, 40), '마우스')
        self.assertEqual(actions[3]['position'], 300)

    def test_missing_grip_never_falls_back_to_llm(self):
        settings.OBJECT_PROFILES['빵']['grip_position'] = None
        with self.assertRaisesRegex(ValueError, 'grip_position'):
            bind_gripper_actions(validate_actions(DEFAULT_PNP_ACTIONS, 40), '빵')

    def test_invalid_grip_and_thickness_are_rejected(self):
        for invalid in (None, -1, 851, float('nan'), float('inf'), True):
            with self.subTest(grip=invalid):
                settings.OBJECT_PROFILES['빵']['grip_position'] = invalid
                with self.assertRaises(ValueError):
                    material_profile('빵')
        settings.OBJECT_PROFILES['빵']['grip_position'] = 620
        for invalid in (None, -1, float('nan'), float('inf')):
            with self.subTest(thickness=invalid):
                settings.OBJECT_PROFILES['빵']['thickness_mm'] = invalid
                with self.assertRaises(ValueError):
                    place(recipe()[0], recipe())

    def test_offsets_use_prior_layers_and_current_tcp_offset(self):
        tasks = recipe()
        placements = [place(task, tasks) for task in tasks]
        self.assertEqual([p['stack_offset_mm'] for p in placements], [0, 20, 20])
        self.assertEqual([p['reference_place_pose']['z'] for p in placements], [45, 60, 75])
        self.assertEqual(place(tasks[2], tasks), placements[2])

    def test_no_duplicate_40_mm_and_approach_uses_same_release_reference(self):
        pose = place(recipe()[2], recipe())['reference_place_pose']
        self.assertEqual(assembly_move_pose(pose, 40, 40)['z'], 75)
        self.assertEqual(assembly_move_pose(pose, 200, 40)['z'], 235)
        self.assertEqual(pose['z'], 75)

    def test_agent_override_does_not_change_common_stack_height(self):
        settings.AGENT_PROFILE_OVERRIDES['agent2'] = {
            '바나나': {'grip_position': 450, 'place_tcp_offset_mm': 50},
        }
        first = place(recipe()[2], recipe(), 'agent1')
        second = place(recipe()[2], recipe(), 'agent2')
        self.assertEqual(first['stack_offset_mm'], second['stack_offset_mm'])
        self.assertEqual(second['reference_place_pose']['z'], 80)
        self.assertEqual(material_profile('바나나', 'agent2')['grip_position'], 450)

    def test_taught_attitudes_require_two_finite_rpy_triplets(self):
        pick = [179.4, 0.0, -0.1]
        place_rpy = [141.5, -0.1, -0.1]
        for key in ('pick_rpy_robot_deg', 'place_rpy_robot_deg'):
            for invalid in (None, [], [1, 2], [1, 2, 3, 4], '141.5,0,0',
                            [True, 0, 0], [float('nan'), 0, 0], [0, float('inf'), 0]):
                with self.subTest(key=key, invalid=invalid):
                    profile = dict(pick_rpy_robot_deg=pick, place_rpy_robot_deg=place_rpy)
                    profile[key] = invalid
                    settings.AGENT_PROFILE_OVERRIDES['agent2'] = {'빵': profile}
                    with self.assertRaises(ValueError):
                        material_profile('빵', 'agent2')
            settings.AGENT_PROFILE_OVERRIDES['agent2'] = {'빵': {key: pick}}
            with self.assertRaises(ValueError):
                material_profile('빵', 'agent2')

    def test_taught_attitudes_are_agent_specific_with_zero_offsets(self):
        settings.OBJECT_PROFILES['빵'].update(place_roll_offset_deg=0, place_pitch_offset_deg=0)
        settings.AGENT_PROFILE_OVERRIDES['agent2'] = {'빵': {
            'pick_rpy_robot_deg': [179.4, 0.0, -0.1],
            'place_rpy_robot_deg': [141.5, -0.1, -0.1],
        }}
        self.assertIsNone(placement_tilt('빵', 'agent1'))
        self.assertIsNotNone(placement_tilt('빵', 'agent2'))
        self.assertIsNone(placement_tilt('바나나', 'agent2'))

    def test_bad_layer_order_dependency_or_material_is_rejected(self):
        for field, value in [('layer_index', 0), ('layer_index', 3),
                             ('layer_index', True), ('material', '사과'),
                             ('container', 'plate')]:
            tasks = recipe()
            tasks[1]['assembly'][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                validate_assembly_tasks(tasks)
        tasks = recipe()
        tasks[1]['depends_on'] = []
        with self.assertRaises(ValueError):
            validate_assembly_tasks(tasks)

    def test_missing_basket_or_tcp_calibration_is_rejected(self):
        with patch.object(settings, 'BASKET_FLOOR_Z_MM', None):
            with self.assertRaisesRegex(ValueError, 'BASKET_FLOOR'):
                place(recipe()[0], recipe())
        settings.OBJECT_PROFILES['빵']['place_tcp_offset_mm'] = None
        with self.assertRaisesRegex(ValueError, 'place_tcp_offset'):
            place(recipe()[0], recipe())

    def test_wrong_mode_order_and_out_of_range_positions_are_rejected(self):
        actions = mode_actions()
        actions[3]['mode'] = 'release'
        with self.assertRaises(ValueError):
            validate_actions(actions, 40)
        actions = mode_actions()
        actions[3]['position'] = 851
        with self.assertRaises(ValueError):
            validate_actions(actions, 40)


    def test_center_is_average_in_workspace_not_basket_floor(self):
        center = basket_center_from_perception(perception(), 'workspace_0', 2, 100.1)
        self.assertEqual([center[a] for a in ('x', 'y', 'z')], [300, 200, 70])
        placement = place(recipe()[0], recipe())
        self.assertEqual(placement['reference_place_pose']['z'], 45)

    def test_missing_stale_nonfinite_or_wrong_frame_red_points_are_rejected(self):
        for case in ('missing', 'invalid', 'frame', 'stale_receive', 'stale_source', 'nan', 'same'):
            state = perception()
            if case == 'missing':
                del state['objects']['basket_handle_1']
            elif case == 'invalid':
                state['valid'] = False
            elif case == 'frame':
                state['frame_id'] = 'camera'
            elif case == 'stale_receive':
                state['received_age_sec'] = 3
            elif case == 'stale_source':
                state['stamp_sec'] = 90
            elif case == 'nan':
                state['objects']['basket_handle_0']['x_mm'] = float('nan')
            else:
                state['objects']['basket_handle_1'] = dict(state['objects']['basket_handle_0'])
            with self.subTest(case=case), self.assertRaises(ValueError):
                basket_center_from_perception(state, 'workspace_0', 2, 100.1)

    def test_rpy_round_trip_including_vertical_and_singular_attitudes(self):
        for rpy in ((180, 30, 0), (180, -30, 90), (210, 0, -40),
                    (10, 90, 30), (10, -90, 30)):
            matrix = rpy_to_matrix(*map(math.radians, rpy))
            np.testing.assert_allclose(rpy_to_matrix(*matrix_to_rpy(matrix)), matrix, atol=1e-10)


def load_agent_methods():
    # Node 생성/import 없이 실제 메서드 본문을 로드한다. SDK/ROS 초기화는 하지 않는다.
    path = Path(__file__).resolve().parents[1] / 'cap_robot/robot_agent.py'
    tree = ast.parse(path.read_text())
    original = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                    and n.name == 'RobotAgentNode')
    names = {
        'validate_actions', 'resolve_task_assembly', 'refresh_execution_assembly',
        'validate_guidebook_policy_candidate', 'build_tasks', 'validate_received_task',
        'execute_task', 'execute_place_action', 'get_basket_center_workspace',
        'workspace_rpy_to_robot', 'move_to_sdk', 'move_to_robot_tf',
        'normalize_angle_rad', 'tf_pose_to_sdk_pose',
        'refresh_guidebook_task_states_locked', 'apply_task_status',
        'find_detected_target', 'normalize_zone', 'resolve_target_destination',
    }
    original.bases = []
    original.body = [n for n in original.body if isinstance(n, ast.FunctionDef) and n.name in names]
    module = ast.fix_missing_locations(ast.Module(body=[original], type_ignores=[]))
    namespace = {
        'math': math, 'time': time,
        'np': np, 'rpy_to_matrix': rpy_to_matrix, 'matrix_to_rpy': matrix_to_rpy,
        'basket_center_from_perception': basket_center_from_perception,
        'placement_tilt': placement_tilt,
        'validate_llm_actions': validate_actions,
        'assembly_move_pose': assembly_move_pose,
        'assembly_placement': assembly_placement,
        'bind_gripper_actions': bind_gripper_actions,
    }
    exec(compile(module, str(path), 'exec'), namespace)
    return namespace['RobotAgentNode']


class AgentAssemblyTest(ConfiguredTest):
    def setUp(self):
        super().setUp()
        self.node = load_agent_methods()()
        node = self.node
        node.agent_id = 'agent1'
        node.workspace_frame = 'workspace_0'
        node.extra_perception_max_age_sec = 2.0
        node.get_extra_perception_snapshot = Mock(side_effect=lambda: perception())
        node.now_sec = Mock(return_value=100.1)
        node.agent_specs = {'agent1': ['A'], 'agent2': ['B']}
        node.PICK_PLACE_Z_OFFSET_MM = 40
        node.DEFAULT_PNP_ACTIONS = DEFAULT_PNP_ACTIONS
        node.guidebook_lock = threading.Lock()
        node.placed_points_lock = threading.Lock()
        node.placed_points = {'A': [], 'B': []}
        node.current_mission_id = 'test_mission'
        node.current_plan_revision = 1
        node.guidebook_tasks = {t['task_id']: t for t in recipe()}
        node.guidebook_task_status = {'layer_0': 'SUCCEEDED', 'layer_1': 'SUCCEEDED', 'layer_2': 'READY'}
        node.get_logger = Mock(return_value=Mock())
        node.update_workspace_tf_cache = Mock()
        node.get_agent_base_frame = Mock(return_value='robot_base')
        node.convert_object_pose = lambda pose, agent: dict(pose)
        # 기울어진 좌표계를 모사: workspace Z를 robot X로 옮겨 변환 순서도 검증한다.
        node.convert_place_pose = lambda pose, agent: {
            'x': pose['z'], 'y': pose['y'], 'z': -pose['x'], 'yaw': pose['yaw'],
        }
        node.workspace_rpy_to_robot = lambda roll, pitch, yaw: dict(roll=roll, pitch=pitch, yaw=yaw)
        node.arm = Mock()
        node.control_gripper = Mock(return_value=True)
        node.move_to_robot_tf = Mock(return_value=True)
        node.return_to_home_joint_pose = Mock(return_value=True)
        node.task_shared_zone_plan = Mock(return_value={
            'needed': False, 'reason': 'test', 'acquire_before_api': None,
        })
        self.poses = {'바나나': (10, 20, 30, 0)}

    def build(self):
        node = self.node
        task = node.guidebook_tasks['layer_2']
        result = {
            'can_execute': True, 'target': '바나나',
            'destination': {'type': 'basket_stack'}, 'actions': mode_actions(),
        }
        policy, _ = node.validate_guidebook_policy_candidate(result, task, self.poses)
        policy['generated_policy'][0].update({
            'mission_id': node.current_mission_id,
            'plan_revision': node.current_plan_revision,
            'guidebook_task_id': 'layer_2',
        })
        return node.build_tasks(policy, self.poses)[0]

    def test_candidate_build_wire_validation_and_execution(self):
        task = json.loads(json.dumps(self.build(), ensure_ascii=False))
        self.assertTrue(self.node.validate_received_task(task))
        self.assertEqual(task['stack_offset_mm'], 20)
        self.assertEqual(task['reference_place_pose']['z'], 75)
        self.assertTrue(self.node.execute_task(task))
        self.assertEqual([c.args[0] for c in self.node.control_gripper.call_args_list], [850, 400, 850])
        moves = self.node.move_to_robot_tf.call_args_list
        # place approach/release/retreat: TF 변환 전에 160/0/160 mm를 더한다.
        self.assertEqual([moves[i].args[0] for i in (3, 4, 5)], [235, 75, 235])
        self.assertEqual([moves[i].args[2] for i in (3, 4, 5)], [-300, -300, -300])

    def test_executing_predecessor_does_not_make_stack_ready(self):
        self.node.guidebook_task_status.update(layer_1='EXECUTING', layer_2='BLOCKED')
        self.node.refresh_guidebook_task_states_locked()
        self.assertEqual(self.node.guidebook_task_status['layer_2'], 'BLOCKED')
        with self.assertRaisesRegex(ValueError, '앞 층'):
            self.build()
        self.node.guidebook_task_status['layer_1'] = 'SUCCEEDED'
        self.node.refresh_guidebook_task_states_locked()
        self.assertEqual(self.node.guidebook_task_status['layer_2'], 'READY')

    def test_stale_or_altered_assembly_is_rejected_before_sdk(self):
        for alteration in ('revision', 'assembly', 'target', 'predecessor'):
            task = self.build()
            if alteration == 'revision':
                task['plan_revision'] = 0
            elif alteration == 'assembly':
                del task['assembly']
            elif alteration == 'target':
                task['target'] = '빵'
            else:
                self.node.guidebook_task_status['layer_1'] = 'FAILED'
            with self.subTest(alteration=alteration):
                self.assertFalse(self.node.execute_task(task))
                self.node.arm.assert_not_called()
                self.assertEqual(self.node.arm.mock_calls, [])
                self.node.move_to_robot_tf.assert_not_called()
            self.node.guidebook_task_status['layer_1'] = 'SUCCEEDED'

    def test_missing_calibration_is_rechecked_before_motion(self):
        task = self.build()
        settings.OBJECT_PROFILES['바나나']['grip_position'] = None
        self.assertFalse(self.node.execute_task(task))
        self.assertEqual(self.node.arm.mock_calls, [])

    def test_stale_success_does_not_unlock_next_layer(self):
        self.node.guidebook_task_status.update(layer_1='EXECUTING', layer_2='BLOCKED')
        payload = {'mission_id': 'test_mission', 'plan_revision': 0,
                   'task_id': 'layer_1', 'status': 'SUCCEEDED', 'agent_id': 'agent2'}
        self.node.apply_task_status(payload)
        self.assertEqual(self.node.guidebook_task_status['layer_2'], 'BLOCKED')
        payload['plan_revision'] = 1
        self.node.apply_task_status(payload)
        self.assertEqual(self.node.guidebook_task_status['layer_2'], 'READY')

    def bread_task(self):
        self.node.guidebook_task_status['layer_0'] = 'READY'
        task = {
            'task_id': 'bread_execution', 'assignee_id': 'agent1', 'target': '빵',
            'mission_id': 'test_mission', 'plan_revision': 1, 'guidebook_task_id': 'layer_0',
            'assembly': dict(recipe()[0]['assembly']), 'destination': '바구니 적층',
            'destination_type': 'workspace', 'reference_object_pose': dict(x=10, y=20, z=30, yaw=0),
            'object_pose': dict(x=10, y=20, z=30, yaw=0),
            'reference_place_pose': dict(x=300, y=200, z=45, yaw=0),
            'place_pose': dict(x=45, y=200, z=-300, yaw=0), 'actions': mode_actions(),
        }
        return task

    def test_bread_tilts_only_at_height_and_restores_after_retreat(self):
        task = self.bread_task()
        events = []
        self.node.move_to_robot_tf.side_effect = lambda *a, **kw: events.append(('move', a, kw)) or True
        self.node.control_gripper.side_effect = lambda pos: events.append(('grip', pos)) or True
        self.assertTrue(self.node.execute_task(task))
        moves = [e for e in events if e[0] == 'move']
        # pick approach/pick/lift are unchanged, followed by 5 placement moves.
        places = moves[3:]
        self.assertEqual([round(e[2]['pitch_deg']) for e in places], [0, 30, 30, 30, 0])
        self.assertEqual([e[1][0] for e in places], [205, 205, 45, 205, 205])
        self.assertTrue(all(e[2]['speed'] == 20 for e in places))
        release = max(i for i, e in enumerate(events) if e == ('grip', 850))
        self.assertTrue(events[release - 1][2]['label'].endswith('lower'))
        self.assertTrue(events[release + 1][2]['label'].endswith('retreat'))
        self.assertTrue(events[release + 2][2]['label'].endswith('restore'))

    def taught_bread_task(self):
        settings.AGENT_PROFILE_OVERRIDES['agent2'] = {'빵': {
            'pick_rpy_robot_deg': [179.4, 0.0, -0.1],
            'place_rpy_robot_deg': [141.5, -0.1, -0.1],
            'place_roll_offset_deg': 0, 'place_pitch_offset_deg': 0,
            'place_tilt_speed': 75,
        }}
        self.node.agent_id = 'agent2'
        self.node.workspace_rpy_to_robot = Mock(
            side_effect=AssertionError('Base 자세에는 workspace TF를 적용하면 안 된다.'))
        task = self.bread_task()
        task['assignee_id'] = 'agent2'
        return task

    def test_taught_bread_attitudes_reach_sdk_at_changing_positions(self):
        node = self.node
        node.dry_run = False
        node.check_sdk_pose_or_raise = Mock()
        # 실제 move_to_robot_tf/move_to_sdk를 사용하고 하드웨어만 mock한다.
        node.move_to_robot_tf = lambda *a, **kw: type(node).move_to_robot_tf(node, *a, **kw)
        expected_rpy = [[179.4, 0, -0.1]] * 4 + [[141.5, -0.1, -0.1]] * 3 + [[179.4, 0, -0.1]]
        for assembly in (True, False):
            for pick_x in (10, 120):
                with self.subTest(assembly=assembly, pick_x=pick_x):
                    task = self.taught_bread_task()
                    task['object_pose'].update(x=pick_x, yaw=1.2)
                    if not assembly:
                        for key in ('assembly', 'guidebook_task_id', 'mission_id', 'plan_revision'):
                            task.pop(key)
                        task['reference_place_pose']['yaw'] = -0.9
                        task['place_pose'] = dict(x=401, y=202, z=150, yaw=-0.9)
                    events = []
                    def sdk_move(**kwargs):
                        events.append(('move', kwargs))
                        return 0
                    node.arm.set_position.side_effect = sdk_move
                    node.control_gripper.side_effect = lambda pos: events.append(('grip', pos)) or True
                    with patch.object(time, 'sleep'):
                        self.assertTrue(node.execute_task(task))
                    moves = [value for kind, value in events if kind == 'move']
                    np.testing.assert_allclose(
                        [[m['roll'], m['pitch'], m['yaw']] for m in moves], expected_rpy,
                        atol=1e-10)
                    self.assertTrue(all(m['is_radian'] is False for m in moves))
                    self.assertEqual([m['x'] for m in moves[:3]], [pick_x] * 3)
                    self.assertEqual([m['speed'] for m in moves[3:]], [75] * 5)
                    # 접근 후 제자리 회전, 해제 후 후퇴하고 높은 위치에서만 자세 복원.
                    xyz = [[m[k] for k in ('x', 'y', 'z')] for m in moves]
                    self.assertEqual(xyz[3], xyz[4])
                    self.assertEqual(xyz[6], xyz[7])
                    self.assertNotEqual(xyz[5], xyz[6])
                    if not assembly:
                        self.assertEqual(xyz[5], [401, 202, 140])  # 150 + 40 - agent2 보정 50
                    release = max(i for i, event in enumerate(events) if event == ('grip', 850))
                    self.assertEqual(events[release - 1][1], moves[5])
                    self.assertEqual(events[release + 1][1], moves[6])
                    node.workspace_rpy_to_robot.assert_not_called()

    def test_failed_taught_rotation_stops_before_lowering_and_release(self):
        task = self.taught_bread_task()
        self.node.move_to_robot_tf.side_effect = [True, True, True, True, False]
        self.assertFalse(self.node.execute_task(task))
        self.assertEqual(self.node.move_to_robot_tf.call_count, 5)
        self.assertEqual([c.args[0] for c in self.node.control_gripper.call_args_list], [850, 620])
        self.node.return_to_home_joint_pose.assert_not_called()

    def test_incomplete_taught_pose_is_rejected_before_sdk(self):
        task = self.taught_bread_task()
        settings.AGENT_PROFILE_OVERRIDES['agent2']['빵'].pop('place_rpy_robot_deg')
        self.assertFalse(self.node.execute_task(task))
        self.assertEqual(self.node.arm.mock_calls, [])
        self.node.move_to_robot_tf.assert_not_called()

    def test_failed_tilt_never_descends_or_releases(self):
        task = self.bread_task()
        self.node.move_to_robot_tf.side_effect = [True, True, True, True, False]
        self.assertFalse(self.node.execute_task(task))
        self.assertEqual(self.node.move_to_robot_tf.call_count, 5)
        self.assertEqual([c.args[0] for c in self.node.control_gripper.call_args_list], [850, 620])
        self.node.return_to_home_joint_pose.assert_not_called()

    def test_center_refreshes_at_execution_then_freezes_during_motion(self):
        task = self.build()
        state = perception()
        for point in state['objects'].values():
            point['x_mm'] += 50
        self.node.get_extra_perception_snapshot.side_effect = lambda: state
        def move(*args, **kwargs):
            state['valid'] = False  # Robot occludes the red dots after execution starts.
            return True
        self.node.move_to_robot_tf.side_effect = move
        self.assertTrue(self.node.execute_task(task))
        self.assertEqual(task['reference_place_pose']['x'], 350)
        self.assertEqual(task['basket_center_workspace']['x'], 350)

    def test_stale_basket_at_execution_is_rejected_before_sdk(self):
        task = self.build()
        self.node.get_extra_perception_snapshot.side_effect = lambda: {'valid': False}
        self.assertFalse(self.node.execute_task(task))
        self.assertEqual(self.node.arm.mock_calls, [])

    def test_workspace_attitude_is_transformed_for_rotated_robot(self):
        node = self.node
        node.lookup_workspace_transform = Mock(return_value=SimpleNamespace(
            transform=SimpleNamespace(rotation=SimpleNamespace(x=0, y=0, z=2**-0.5, w=2**-0.5)),
        ))
        node.quaternion_to_rotation_matrix = lambda x, y, z, w: quaternion_xyzw_to_matrix((x, y, z, w))
        attitude = type(node).workspace_rpy_to_robot(node, math.pi, math.pi / 6, 0)
        self.assertAlmostEqual(math.degrees(attitude['roll']), 180)
        self.assertAlmostEqual(math.degrees(attitude['pitch']), 30)
        self.assertAlmostEqual(math.degrees(attitude['yaw']), 90)

    def test_sdk_receives_roll_pitch_in_degrees(self):
        node = self.node
        node.dry_run = False
        node.check_sdk_pose_or_raise = Mock()
        node.arm.set_position.return_value = 0
        with patch.object(time, 'sleep'):
            self.assertTrue(type(node).move_to_robot_tf(
                node, 100, 200, 300, yaw=math.pi/2, roll_deg=180, pitch_deg=30,
            ))
        sent = node.arm.set_position.call_args.kwargs
        self.assertEqual((sent['roll'], sent['pitch'], sent['yaw']), (180, 30, 90))
        self.assertFalse(sent['is_radian'])



if __name__ == '__main__':
    unittest.main()
