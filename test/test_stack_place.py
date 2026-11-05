"""No ROS node, SDK connection, live camera, or robot motion is created."""
import copy
import json
import math
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import cv2
import pytest

from cap_robot import object_profiles, stack_place as vision
from cap_robot.material_handling import assembly_placement
import test_material_handling as handling
from test_material_handling import load_agent_methods, recipe
import test_object_memory as memory_tests


def config(**overrides):
    cfg = dict(copy.deepcopy(vision.DEFAULTS),
               basket_half_extent_mm=[60., 60.], target_z_range_mm=[5., 80.],
               max_correction_mm=20., tcp_frame='agent1_tcp')
    return dict(cfg, **overrides)


def metadata(stamp=100.1, received=10.1):
    return dict(color_stamp=stamp, depth_stamp=stamp,
                color_received=received, depth_received=received,
                color_frame='camera_optical', depth_frame='camera_optical',
                camera_frame='camera_optical')


def observation(x=300, y=170, **extra):
    return dict(frame_id='workspace_0', error=None,
                detections=[('빵_99', (x, y, 30, 0), 0.9)], **extra)


def test_real_profile_and_offset_are_preserved():
    profile = object_profiles.OBJECT_PROFILES['양상추']
    assert {k: profile[k] for k in ('grip_position', 'thickness_mm',
                                  'place_tcp_offset_mm', 'place_xy_offset_mm')} == {
        'grip_position': 100.0, 'thickness_mm': 0.0,
        'place_tcp_offset_mm': 30.0, 'place_xy_offset_mm': [0.0, -30.0]}
    for agent in ('agent1', 'agent2'):
        task = recipe()[1]
        nominal = assembly_placement(task, recipe(), agent,
                                     basket_center=dict(x=300, y=200))['reference_place_pose']
        assert (nominal['x'], nominal['y']) == (300, 170)
        cfg = config(max_correction_mm=50.)
        for xy in ((300, 200), (310, 210), (290, 195)):
            result = vision.corrected_place(nominal, [(*xy, 30)] * 3, [0., 0., 0.], np.eye(3), cfg)
            assert result == dict(nominal, x=xy[0], y=xy[1])


@pytest.mark.parametrize('change', [
    dict(color_stamp=99.0), dict(depth_stamp=99.0),
    dict(color_received=9.0), dict(depth_received=9.0),
    dict(color_stamp=101.0), dict(depth_stamp=100.3),
    dict(depth_frame='unaligned_depth'), dict(camera_frame='other'),
    dict(color_stamp=math.nan),
])
def test_old_future_unsynchronized_or_wrong_frame_is_rejected(change):
    meta = dict(metadata(), **change)
    with pytest.raises(ValueError):
        vision.validate_frames(meta, dict(since_ros=100.0, since_mono=10.0), config(), 100.5, 10.5)


@pytest.mark.parametrize('detections', [[],
    [('햄_1', (300, 170, 30, 0), 0.99)],
    [('빵_1', (100, 100, 30, 0), 0.99)],
    [('빵_1', (300, 170, 200, 0), 0.99)],
    [('빵_1', (300, 170, 30, 0), 0.5)],
    [('빵_1', (300, 170, 30, 0), 0.9), ('빵_2', (310, 170, 30, 0), 0.9)],
])
def test_missing_outside_low_confidence_or_multiple_bread_is_rejected(detections):
    obs = observation()
    obs['detections'] = detections
    with pytest.raises(ValueError):
        vision.previous_layer_center(obs, dict(x=300, y=200), '빵', config())


def test_class_and_position_choose_candidate_without_id_dependency():
    obs = observation()
    obs['detections'] += [('빵_1', (500, 170, 30, 0), 0.9),
                          ('양상추_1', (300, 170, 30, 0), 0.9)]
    assert vision.previous_layer_center(obs, dict(x=300, y=200), '빵', config()) == (300, 170, 30)


@pytest.fixture
def agent(monkeypatch, tmp_path):
    case = handling.AgentAssemblyTest()
    case.setUp()
    node = case.node
    save_debug = vision.save_observation_debug
    monkeypatch.setattr(vision, 'save_observation_debug',
                        lambda *args: save_debug(*args, directory=tmp_path))
    node.stack_vision_config = config(
        enabled=True, observation_validated=True)
    node.zone_token_enabled = node.enable_sdk_safety_box = True
    node._shared_zone_box = lambda: dict(x_min=150., x_max=450., y_min=-50., y_max=500.)
    node.lookup_workspace_transform = Mock(return_value=SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0., z=0.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
    node.arm.version_number = (2, 4, 0)
    node.arm.get_state.return_value = (0, 2)
    node.arm.get_is_moving.return_value = False
    node.arm.error_code = 0
    node.arm.set_position.return_value = 0
    node.arm.only_check_result = 0
    node.quaternion_to_rotation_matrix = lambda *a: np.eye(3)
    node.convert_place_pose = lambda pose, agent: dict(pose)
    # Blocking moves report their final measured TCP without connecting to hardware.
    node.arm.get_position.side_effect = lambda **kw: (
        0, [*node.move_to_robot_tf.call_args.args[:3], 180., 0., 0.])
    def acquire(_):
        node._zone_token_held = True
        return True
    node.acquire_zone_token = Mock(side_effect=acquire)
    node.enable_perception = True
    node.perception_settle_sec = 0.0
    node.is_moving = True
    node.perception_lock = threading.Lock()
    node.place_verification_request = None
    node.place_observations = []
    node.check_sdk_pose_or_raise = Mock()
    node._zone_token_held = False
    node.release_zone_token = Mock()
    start = time.monotonic()
    node.now_sec = lambda: 100.1 + time.monotonic() - start
    # Preserve the production lettuce profile values in the otherwise synthetic recipe fixture.
    object_profiles.OBJECT_PROFILES['양상추'].update(
        grip_position=100., place_xy_offset_mm=[0., -30.], stack_tcp_to_material_center_mm=[0., 0., 0.])
    node.guidebook_task_status.update(layer_0='SUCCEEDED', layer_1='READY')
    task = case.bread_task()
    node.guidebook_task_status['layer_0'] = 'SUCCEEDED'
    task.update(target='양상추', guidebook_task_id='layer_1',
                assembly=copy.deepcopy(recipe()[1]['assembly']))
    yield node, task
    case.doCleanups()


def feed(monkeypatch, node, observations, clock=None):
    iterator = iter(observations)
    previous_meta = None
    def deliver(seconds):
        nonlocal previous_meta
        if clock is not None:
            clock.now += seconds
        try:
            obs = copy.deepcopy(next(iterator))
        except StopIteration:
            return
        obs.setdefault('meta', metadata(node.now_sec(), time.monotonic()))
        if obs.pop('_repeat_previous', False):
            obs['meta'] = previous_meta
        previous_meta = obs['meta']
        with node.perception_lock:
            node.place_observations.append(obs)
    monkeypatch.setattr(time, 'sleep', deliver)


def use_observation_clock(monkeypatch, node):
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(time, 'monotonic', lambda: clock.now)
    node.now_sec = lambda: 100.1 + clock.now - 10.0
    return clock


def test_first_empty_frame_then_three_stable_frames_allows_correction(agent, monkeypatch):
    node, task = agent
    clock = use_observation_clock(monkeypatch, node)
    empty = observation()
    empty['detections'] = []
    feed(monkeypatch, node, [empty, observation(309, 180), observation(310, 180),
                             observation(311, 180)], clock)
    original_move = node.move_to_robot_tf.side_effect

    def move(*args, **kwargs):
        if kwargs.get('label') == 'corrected approach':
            # Four distinct post-stop frames have been consumed before correction.
            assert clock.now == pytest.approx(10.08)
        return original_move(*args, **kwargs) if callable(original_move) else True

    node.move_to_robot_tf.side_effect = move
    assert node.execute_task(task)
    assert task['reference_place_pose'] == dict(x=310, y=180, z=60, yaw=0)
    assert [c.kwargs['label'] for c in node.move_to_robot_tf.call_args_list].count('corrected approach') == 2
    node.return_to_home_joint_pose.assert_called_once()
    node.release_zone_token.assert_called_once()


@pytest.mark.parametrize('points', [
    [observation(300, 170), observation(301, 170), observation(310, 180),
     observation(311, 180), observation(309, 180)],
    # Adjacent points are close, but the full XYZ cluster exceeds stability_mm.
    [observation(300, 170), observation(302, 170), observation(304, 170),
     observation(305, 170), observation(306, 170)],
])
def test_outlier_restarts_cluster_without_mixing_old_samples(agent, monkeypatch, points):
    node, task = agent
    assert node.validate_received_task(task)
    node._zone_token_held = True
    clock = use_observation_clock(monkeypatch, node)
    feed(monkeypatch, node, points, clock)
    _, accepted, centers = node.observe_previous_layer(task)
    expected = [p['detections'][0][1][:3] for p in points[-3:]]
    assert centers == expected
    assert [p['frame_number'] for p in accepted] == [3, 4, 5]
    assert len({p['meta']['color_stamp'] for p in accepted}) == 3
    assert len({p['meta']['depth_stamp'] for p in accepted}) == 3
    nominal = task['reference_place_pose']
    corrected = vision.corrected_place(nominal, centers, [0., 0., 0.], np.eye(3), config())
    assert corrected['x'] == pytest.approx(sum(p[0] for p in expected) / 3)
    assert clock.now == pytest.approx(10.1)


@pytest.mark.parametrize('mode', ['empty', 'two_samples', 'unstable'])
def test_retry_deadline_is_five_seconds_without_place_or_recovery(agent, monkeypatch, mode):
    node, task = agent
    clock = use_observation_clock(monkeypatch, node)
    empty = observation()
    empty['detections'] = []
    points = ([empty] * 300 if mode == 'empty' else
              [observation()] * 2 if mode == 'two_samples' else
              [observation(300 + (i % 2) * 10, 170) for i in range(300)])
    feed(monkeypatch, node, points, clock)
    assert node.stack_vision_config['timeout_sec'] == 5.0
    assert not node.execute_task(task)
    assert 5.0 <= clock.now - 10.0 <= 5.021
    assert node.move_to_robot_tf.call_count == 6
    assert node.control_gripper.call_count == 2
    node.return_to_home_joint_pose.assert_not_called()
    node.release_zone_token.assert_not_called()


@pytest.mark.parametrize('kind', ['color', 'depth'])
def test_empty_frame_cannot_be_reused_as_a_new_sample(agent, monkeypatch, kind):
    node, task = agent
    clock = use_observation_clock(monkeypatch, node)
    empty = observation(meta=metadata(100.12, 10.02))
    empty['detections'] = []
    meta = metadata(100.14, 10.04)
    meta[f'{kind}_stamp'] = 100.12
    feed(monkeypatch, node, [empty, observation(meta=meta)] + [observation()] * 3, clock)
    assert not node.execute_task(task)
    assert clock.now < 10.1
    assert node.move_to_robot_tf.call_count == 6
    assert node.control_gripper.call_count == 2


def test_settle_barrier_precedes_observation_timeout(agent, monkeypatch):
    node, task = agent
    assert node.validate_received_task(task)
    node._zone_token_held = True
    node.perception_settle_sec = 0.5
    clock = use_observation_clock(monkeypatch, node)
    stamps = iter([10.51, 10.53, 10.55])

    def deliver(_):
        clock.now = next(stamps)
        node.place_observations.append(observation(meta=metadata(node.now_sec(), clock.now)))

    monkeypatch.setattr(time, 'sleep', deliver)
    request, accepted, _ = node.observe_previous_layer(task)
    assert request['since_mono'] == 10.5
    assert request['since_ros'] == pytest.approx(100.6)
    assert all(p['meta']['color_received'] > 10.5 and p['meta']['depth_received'] > 10.5
               for p in accepted)


def test_checks_report_each_rejection_without_relaxing_thresholds():
    obs = observation()
    obs['detections'] = [('빵_1', (500, 200, 90, 0), 0.5)]
    checks = []
    with pytest.raises(vision.NoPreviousLayerCandidate):
        vision.previous_layer_center(obs, dict(x=300, y=200), '빵', config(), checks)
    assert len(checks) == 1
    assert checks[0]['workspace_xyz_mm'] == [500, 200, 90]
    assert checks[0]['basket_xy']['passed'] is False
    assert checks[0]['z_range']['passed'] is False
    assert checks[0]['confidence_check'] == dict(passed=False, minimum=0.6)


@pytest.mark.parametrize('error', ['invalid depth', 'invalid TF'])
def test_empty_error_frame_is_fatal_instead_of_retryable(error):
    obs = observation()
    obs.update(error=error, detections=[])
    with pytest.raises(ValueError) as caught:
        vision.previous_layer_center(obs, dict(x=300, y=200), '빵', config())
    assert not isinstance(caught.value, vision.NoPreviousLayerCandidate)


def test_observation_failure_saves_last_matching_rgb_and_raw_results(agent, monkeypatch, tmp_path):
    node, task = agent
    clock = use_observation_clock(monkeypatch, node)
    first = observation(rgb=np.zeros((20, 30, 3), dtype=np.uint8))
    first['detections'] = []
    last = observation(rgb=np.full((20, 30, 3), 42, dtype=np.uint8),
                       annotated_rgb=np.full((20, 30, 3), 84, dtype=np.uint8),
                       raw_detections=[dict(class_name='Bread', confidence=0.9,
                                            bbox_xyxy=[1, 2, 10, 12], depth_valid=False)])
    last.update(error='invalid depth', detections=[])
    feed(monkeypatch, node, [first, last], clock)
    assert not node.execute_task(task)
    files = list(tmp_path.glob('*.json'))
    assert len(files) == 1
    payload = json.loads(files[0].read_text())
    assert payload['frame_number'] == 2
    assert 'invalid depth' in payload['failure']
    assert payload['raw_detections'] == last['raw_detections']
    assert payload['meta']['color_received'] == pytest.approx(10.04)
    np.testing.assert_array_equal(cv2.imread(str(tmp_path / payload['rgb_file'])), last['rgb'])
    np.testing.assert_array_equal(cv2.imread(str(tmp_path / payload['annotated_rgb_file'])), last['annotated_rgb'])
    assert node.move_to_robot_tf.call_count == 6
    assert node.control_gripper.call_count == 2


def test_debug_write_error_preserves_original_observation_failure(agent, monkeypatch):
    node, task = agent
    clock = use_observation_clock(monkeypatch, node)
    monkeypatch.setattr(vision, 'save_observation_debug', Mock(side_effect=OSError('disk full')))
    obs = observation()
    obs['error'] = 'TF unavailable'
    feed(monkeypatch, node, [obs], clock)
    assert node.validate_received_task(task)
    node._zone_token_held = True
    with pytest.raises(ValueError, match='TF unavailable'):
        node.observe_previous_layer(task)
    assert node.is_moving and node.place_verification_request is None


def test_late_error_in_same_batch_cannot_be_hidden_by_three_good_samples(agent, monkeypatch):
    node, task = agent
    assert node.validate_received_task(task)
    node._zone_token_held = True
    clock = use_observation_clock(monkeypatch, node)

    def deliver(_):
        for i in range(4):
            clock.now += 0.02
            obs = observation(meta=metadata(node.now_sec(), clock.now))
            if i == 3:
                obs['error'] = 'TF unavailable'
            node.place_observations.append(obs)

    monkeypatch.setattr(time, 'sleep', deliver)
    with pytest.raises(ValueError, match='TF unavailable'):
        node.observe_previous_layer(task)


@pytest.mark.parametrize('agent_id', ['agent1'])
@pytest.mark.parametrize('delta', [(0, 30), (10, 35)])
def test_execution_corrects_once_after_lift_and_uses_existing_transform(agent, monkeypatch, agent_id, delta):
    node, task = agent
    node.agent_id = task['assignee_id'] = agent_id
    node.stack_vision_config['max_correction_mm'] = 50.
    # Exercise the real convert_place_pose/workspace_place_to_robot_place methods.
    node.workspace_point_to_robot = Mock(side_effect=lambda x, y, z, **kw: (x, y, z))
    node.workspace_yaw_to_robot = Mock(side_effect=lambda yaw, **kw: yaw + 0.2)
    node.convert_place_pose = lambda pose, agent: type(node).convert_place_pose(node, pose, agent)
    feed(monkeypatch, node, [observation(300 + delta[0], 170 + delta[1])] * 3)
    original = node.observe_previous_layer
    def verify(t):
        assert node.move_to_robot_tf.call_count == 6  # pick trio + high approach
        assert node.move_to_robot_tf.call_args.kwargs['label'] == 'observe'
        assert node._zone_token_held
        assert node.control_gripper.call_count == 2
        return original(t)
    node.observe_previous_layer = verify
    assert node.execute_task(task)
    assert task['reference_place_pose'] == dict(x=300 + delta[0], y=170 + delta[1], z=60, yaw=0)
    # A correction straight below the view needs no separate zero-length level move.
    moves = node.move_to_robot_tf.call_args_list[-3:]
    assert node.move_to_robot_tf.call_count == (10 if delta == (0, 30) else 11)
    assert [c.args[:3] for c in moves] == [
        (300 + delta[0], 170 + delta[1], 220),
        (300 + delta[0], 170 + delta[1], 60),
        (300 + delta[0], 170 + delta[1], 220)]
    assert all(c.kwargs['yaw'] == 0.2 for c in moves)
    assert all(c.kwargs['agent_id'] == agent_id for c in node.workspace_point_to_robot.call_args_list)
    assert node.check_sdk_pose_or_raise.call_count == (9 if delta == (0, 30) else 10)
    assert node.check_sdk_pose_or_raise.call_args_list[-2].args[2] == 230.
    assert node.is_moving and node.place_verification_request is None


@pytest.mark.parametrize('failure', ['missing', 'multiple', 'stale', 'duplicate', 'unstable',
                                      'excessive', 'tf', 'depth', 'safety'])
def test_failure_stops_before_place_release_and_home(agent, monkeypatch, failure):
    node, task = agent
    clock = use_observation_clock(monkeypatch, node)
    observations = [observation() for _ in range(3)]
    if failure == 'missing':
        observations[0]['detections'] = []
    elif failure == 'multiple':
        observations[0]['detections'] *= 2
    elif failure == 'stale':
        observations[0]['meta'] = metadata(99., 0.)
    elif failure == 'duplicate':
        observations[1]['_repeat_previous'] = True
    elif failure == 'unstable':
        observations[1] = observation(305, 170)
    elif failure == 'excessive':
        observations = [observation(325, 170)] * 3
    elif failure in ('tf', 'depth'):
        observations[0]['error'] = failure
    elif failure == 'safety':
        node.check_sdk_pose_or_raise.side_effect = ValueError('outside safety box')
    feed(monkeypatch, node, observations, clock)
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == (3 if failure == 'safety' else 6)
    assert node.control_gripper.call_count == 2
    node.release_zone_token.assert_not_called()
    assert node._place_observation_fault
    node.return_to_home_joint_pose.assert_not_called()
    assert task['reference_place_pose']['y'] == 170
    assert node.is_moving and node.place_verification_request is None


def test_timeout_never_reuses_latest_poses(agent, monkeypatch):
    node, task = agent
    node.stack_vision_config['timeout_sec'] = 0.01
    node.latest_poses = {'빵_1': (300, 170, 30, 0)}
    node.latest_poses_stamp = math.inf
    monkeypatch.setattr(time, 'sleep', lambda _: None)
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 6


def test_bread_predecessor_must_have_succeeded(agent, monkeypatch):
    node, task = agent
    node.guidebook_task_status['layer_0'] = 'EXECUTING'
    # Isolate the additional bread guard; production retains its existing predecessor wait.
    node.wait_for_assembly_predecessors = Mock()
    feed(monkeypatch, node, [observation()] * 3)
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 3


def test_corrected_path_rechecks_token_and_releases_it(agent, monkeypatch):
    node, task = agent
    feed(monkeypatch, node, [observation(310, 180)] * 3)
    checked = []
    def classify(t):
        checked.append(t['reference_place_pose']['x'])
        return dict(needed=len(checked) == 2, acquire_before_api='move_to_place', reason='test')
    node.task_shared_zone_plan.side_effect = classify
    def acquire(_):
        assert node.move_to_robot_tf.call_count == 3
        node._zone_token_held = True
        return True
    node.acquire_zone_token = Mock(side_effect=acquire)
    assert node.execute_task(task)
    assert checked == [300, 310]
    node.acquire_zone_token.assert_called_once()
    node.release_zone_token.assert_called_once()


def test_lost_token_before_observation_stops(agent, monkeypatch):
    node, task = agent
    node.acquire_zone_token = Mock(return_value=True)
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 3
    node.release_zone_token.assert_called_once()


def test_long_token_wait_precedes_fresh_observation(agent, monkeypatch):
    node, task = agent
    feed(monkeypatch, node, [observation()] * 3)
    def acquire(_):
        assert node.move_to_robot_tf.call_count == 3
        node._zone_token_held = True
        start = time.monotonic()
        node.now_sec = lambda: 200.0 + time.monotonic() - start
        node.get_extra_perception_snapshot = lambda: dict(handling.perception(), stamp_sec=node.now_sec())
        return True
    node.acquire_zone_token = Mock(side_effect=acquire)
    assert node.execute_task(task)
    node.release_zone_token.assert_called_once()


def test_disabled_and_other_materials_bypass_verification(agent):
    node, task = agent
    node.observe_previous_layer = Mock(side_effect=AssertionError('must not verify'))
    node.stack_vision_config['enabled'] = False
    assert node.execute_task(task)
    assert task['reference_place_pose']['y'] == 170
    node.stack_vision_config['enabled'] = True
    for material in ('빵', '햄'):
        assert not node.needs_stack_place(dict(assembly=dict(container='basket', material=material)))
    assert not node.needs_stack_place(dict(target='양상추'))


def test_unset_geometry_is_rejected():
    with pytest.raises(ValueError):
        vision.validate_config(vision.DEFAULTS)


@pytest.mark.parametrize('material, model_class', [('빵', 'bread'), ('양상추', 'lettuce'), ('햄', 'ham')])
@pytest.mark.parametrize('failure', [None, 'depth', 'tf'])
def test_actual_yolo_loop_emits_only_current_stamped_observation(failure, material, model_class):
    case = memory_tests.PerceptionIntegrationTest()
    case.setUp()
    node = case.node
    cfg = config()
    node.run_perception.__globals__['stack_place'] = vision
    node.stack_vision_config = cfg
    node.workspace_frame = 'workspace_0'
    node.now_sec = lambda: 100.5
    node.place_verification_request = dict(since_ros=100., since_mono=41., last_stamps=(0., 0.), target_material=material)
    node.place_observations = []
    meta = metadata(100.1, 42.)
    node.get_latest_perception_frames = lambda: (np.zeros((100, 200, 3)), None, {}, meta)
    transform = object()
    node.place_camera_transform = Mock(return_value=transform)
    if failure == 'tf':
        node.place_camera_transform.side_effect = ValueError('TF unavailable')
    node.camera_point_to_workspace = Mock(return_value=(300, 170, 30))
    if failure == 'depth':
        node.median_depth_m = lambda *a: None
    model = SimpleNamespace(names={0: model_class}, predict=Mock(return_value=case.result([20])))
    node.models = [('existing_model.pt', model)]
    case.drawing.waitKey.side_effect = [ord('q')]
    node.run_perception()
    assert len(node.place_observations) == 1
    obs = node.place_observations[0]
    assert obs['meta'] is meta
    assert obs['visibility']['image_size'] == [100, 200]
    assert obs['visibility']['objects'][0]['material'] == material
    raw = obs['raw_detections'][0]
    assert raw['class_name'] == model_class
    assert raw['confidence'] == 0.9
    assert raw['bbox_xyxy'] == [15, 10, 25, 30]
    assert raw['depth_valid'] is (failure != 'depth')
    case.drawing.rectangle.assert_called_once()
    assert np.any(obs['annotated_rgb'])
    assert not np.any(obs['rgb'])
    assert not np.any(model.predict.call_args.kwargs['source'])
    if failure == 'depth':
        assert obs['visibility']['objects'][0]['depth_m'] is None
    if failure:
        assert obs['error']
        assert not obs['detections']
        assert node.latest_poses == {}
        assert raw['status'] == ('invalid depth' if failure == 'depth' else 'TF rejected')
    else:
        assert obs['detections'] == [(material, (300, 170, 30, 0.), 0.9)]
        assert all(c.kwargs['transform'] is transform for c in node.camera_point_to_workspace.call_args_list)
        assert raw['workspace_xyz_mm'] == [300, 170, 30]


def test_yolo_observation_waits_for_post_settle_and_both_new_acquisition_stamps():
    case = memory_tests.PerceptionIntegrationTest()
    case.setUp()
    node = case.node
    node.run_perception.__globals__['stack_place'] = vision
    node.stack_vision_config = config()
    node.workspace_frame = 'workspace_0'
    node.now_sec = lambda: 100.5
    node.place_verification_request = dict(
        since_ros=100., since_mono=41., last_stamps=(0., 0.), target_material='빵')
    node.place_observations = []
    metas = [metadata(99.99, 40.99), metadata(100.1, 42.), metadata(100.1, 42.),
             dict(metadata(100.12, 42.), depth_stamp=100.1),
             dict(metadata(100.1, 42.), depth_stamp=100.12), metadata(100.14, 42.)]
    frames = iter((np.zeros((100, 200, 3), dtype=np.uint8), None, {}, m) for m in metas)
    node.get_latest_perception_frames = lambda: next(frames)
    node.place_camera_transform = Mock(return_value=object())
    node.camera_point_to_workspace = Mock(return_value=(300, 170, 30))
    model = SimpleNamespace(names={0: 'bread'}, predict=Mock(return_value=case.result([20])))
    node.models = [('bread.pt', model)]
    node.run_perception()
    assert model.predict.call_count == 2
    assert [o['meta']['color_stamp'] for o in node.place_observations] == [100.1, 100.14]
    assert [o['frame_number'] for o in node.place_observations] == [1, 2]
    assert all(o['error'] is None for o in node.place_observations)


def test_callbacks_preserve_acquisition_and_receipt_stamps():
    node = load_agent_methods()()
    node.perception_lock = threading.Lock()
    node.bridge = SimpleNamespace(imgmsg_to_cv2=lambda *a, **kw: np.ones((2, 2)))
    node.camera_intrinsics = dict(fx=1., fy=1., cx=0., cy=0.)
    node.camera_frame = 'camera_optical'
    msg = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=100, nanosec=500000000), frame_id='camera_optical'))
    before = time.monotonic()
    node.color_callback(msg)
    node.depth_callback(msg)
    _, _, _, meta = node.get_latest_perception_frames()
    assert meta['color_stamp'] == meta['depth_stamp'] == 100.5
    assert before <= meta['color_received'] <= meta['depth_received'] <= time.monotonic()


def rigid_transform(xyz=(0., 0., 0.), quat=(0., 0., 0., 1.)):
    return SimpleNamespace(translation=SimpleNamespace(**dict(zip(('x', 'y', 'z'), xyz))),
                           rotation=SimpleNamespace(**dict(zip(('x', 'y', 'z', 'w'), quat))))


def test_tf_queries_capture_time_and_rejects_invalid_rotation():
    node = load_agent_methods()()
    node.stack_vision_config = config()
    node.tf_lookup_timeout_sec = 0.1
    mount = rigid_transform((.01, .02, .03))
    tcp = rigid_transform((.3, .2, .4), (0., 0., math.sqrt(.5), math.sqrt(.5)))
    node.tf_buffer = Mock()
    def lookup(parent, child, stamp, **kw):
        return SimpleNamespace(header=SimpleNamespace(frame_id=parent), child_frame_id=child,
                               transform=mount if parent == 'agent1_tcp' else tcp)
    node.tf_buffer.lookup_transform.side_effect = lookup
    args = (metadata(), dict(fx=100., fy=100., cx=1., cy=1.), np.zeros((2, 2, 3)), np.ones((2, 2)))
    result = node.place_camera_transform(*args)
    assert (result.translation.x, result.translation.y, result.translation.z) == pytest.approx((.28, .21, .43))
    calls = node.tf_buffer.lookup_transform.call_args_list
    assert [c.args[:2] for c in calls] == [('agent1_tcp', 'camera_optical'), ('workspace_0', 'agent1_tcp')]
    assert all(c.args[2].seconds == 100.1 for c in calls)
    mount.rotation.w = 0.
    with pytest.raises(ValueError):
        node.place_camera_transform(*args)


def test_observation_uses_blue_mean_xy_and_existing_high_place_z(agent):
    node, task = agent
    assert node.validate_received_task(task)
    start = dict(x=10., y=20., z=180., yaw=0.)
    node.arm.get_position.side_effect = None
    node.arm.get_position.return_value = (0, [10., 20., 180., 180., 0., 0.])
    view, _, path = node.plan_stack_observation(task, start, None, dict(z_offset=200., speed=100.))
    assert view == dict(x=300., y=200., z=220., yaw=0.)
    task['basket_center_workspace']['z'] = 900.
    assert node.plan_stack_observation(task, start, None, dict(z_offset=200., speed=100.))[0] == view
    assert path == [dict(x=10., y=20., z=180., yaw=0.),
                    dict(x=10., y=20., z=220., yaw=0.), view, view]


@pytest.mark.parametrize('pick_z,base_z', [(314.2, 0.), (180., 0.), (314.2, 200.),
                                         (180., -200.)])
def test_observation_compares_workspace_heights_and_executes_up_then_level(agent, pick_z, base_z):
    node, task = agent
    assert node.validate_received_task(task)
    # Base and workspace have different Z origins, plus a 90 degree XY rotation.
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    translation = np.array([100., -50., base_z])
    node.lookup_workspace_transform.return_value.transform = rigid_transform(
        translation / 1000., (0., 0., math.sqrt(.5), math.sqrt(.5)))
    node.quaternion_to_rotation_matrix = lambda *a: rotation
    def convert(point, agent):
        xyz = rotation @ np.array([point[a] for a in ('x', 'y', 'z')]) + translation
        return dict(zip(('x', 'y', 'z'), xyz), yaw=point['yaw'])
    node.convert_place_pose = convert
    start = dict(x=10., y=20., z=pick_z, yaw=0.)  # Base, not workspace
    node.move_to_robot_tf(start['x'], start['y'], start['z'])
    view, attitude, path = node.plan_stack_observation(
        task, start, None, dict(z_offset=200., speed=100.))
    height = max(pick_z - base_z, 220.)
    assert view['z'] == pytest.approx(height)  # workspace view
    assert path[0]['z'] == pytest.approx(pick_z)  # Base/SDK path
    assert all(p['z'] >= pick_z for p in path)
    node._zone_token_held = True
    node.move_stack_observation_path(task, path, attitude, 'observe', 100.)
    moves = [c.args[:3] for c in node.move_to_robot_tf.call_args_list]
    # First rise stays at the actual Pick XY. All later motion keeps SDK Z.
    assert moves[1] == pytest.approx((10., 20., height + base_z))
    assert all(p[2] == pytest.approx(height + base_z) for p in moves[1:])
    assert min(p[2] for p in moves) >= pick_z
    assert max(p[2] for p in moves) <= max(pick_z, 220. + base_z)
    assert moves[-1][:2] == pytest.approx((-100., 250.))


def tilt_workspace(node, tilt_deg):
    angle = math.radians(tilt_deg)
    rotation = handling.rpy_to_matrix(0., angle, 0.)
    node.lookup_workspace_transform.return_value.transform = rigid_transform(
        quat=(0., math.sin(angle / 2), 0., math.cos(angle / 2)))
    node.quaternion_to_rotation_matrix = lambda *a: rotation
    def convert(point, agent):
        xyz = rotation @ np.array([point[a] for a in ('x', 'y', 'z')])
        return dict(zip(('x', 'y', 'z'), xyz), yaw=point['yaw'])
    node.convert_place_pose = convert
    return rotation


# Field runs on 2026-11-05 measured roughly 0.5-2.7 deg between workspace_0 and Base.
@pytest.mark.parametrize('tilt_deg', [-2.7, -0.5, 0.5, 2.7])
def test_field_workspace_tilt_keeps_sdk_level_and_descends_only_after_correction(
        agent, monkeypatch, tilt_deg):
    node, task = agent
    rotation = tilt_workspace(node, tilt_deg)
    feed(monkeypatch, node, [observation(310, 180)] * 3)
    assert node.execute_task(task)
    assert task['reference_place_pose'] == dict(x=310, y=180, z=60, yaw=0)
    moves = [c.args[:3] for c in node.move_to_robot_tf.call_args_list]
    # Pick lift, observation and corrected approach, then the unchanged Place trio.
    lift, approach = moves[2], moves[2:-3]
    # Level moves may absorb a sub-tolerance (< 1 mm) Z step; larger steps are vertical.
    for before, after in zip(approach, approach[1:]):
        assert math.dist(before, after) >= vision.POSITION_TOLERANCE_MM or before == after
        if not math.isclose(before[2], after[2], abs_tol=vision.POSITION_TOLERANCE_MM):
            assert before[:2] == pytest.approx(after[:2], abs=1e-6)
    assert all(p[2] >= lift[2] - 1e-6 for p in approach[:-1])
    final = approach[-2], approach[-1]
    assert (final[0][:2] == pytest.approx(final[1][:2], abs=1e-6)
            or abs(final[0][2] - final[1][2]) < vision.POSITION_TOLERANCE_MM)
    assert approach[-1] == pytest.approx(moves[-3])  # original high approach
    view = rotation.T @ np.array(moves[5])
    assert view[:2] == pytest.approx((300., 200.))  # blue-handle mean
    assert view[2] >= 220. - 1e-6  # not below the existing high Place Z
    corrected = rotation.T @ np.array(approach[-1])
    assert corrected == pytest.approx((310., 180., 220.))
    node.release_zone_token.assert_called_once()


@pytest.mark.parametrize('tilt_deg', [-90., -15., 15., 180.])
def test_tilted_or_inverted_workspace_cannot_lower_tcp_or_invent_extra_rise(agent, tilt_deg):
    node, task = agent
    tilt_workspace(node, tilt_deg)
    node.observe_previous_layer = Mock(side_effect=AssertionError('unsafe observation'))
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 3  # unchanged Pick only
    node.observe_previous_layer.assert_not_called()
    node.arm.set_position.assert_not_called()  # no path or motion after Pick
    assert node.control_gripper.call_count == 2
    node.return_to_home_joint_pose.assert_not_called()
    node.release_zone_token.assert_not_called()
    assert node._place_observation_fault


def test_inconsistent_tf_to_sdk_mapping_is_rejected_before_observation(agent):
    node, task = agent
    node.convert_place_pose = lambda p, agent: dict(p, z=p['z'] + 10.)
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 3
    node.arm.set_position.assert_not_called()
    assert node.control_gripper.call_count == 2


@pytest.mark.parametrize('stage', ['pick', 'entry', 'waypoint'])
@pytest.mark.parametrize('failure', ['sdk_error', 'nan', 'below_lift'])
def test_actual_tcp_feedback_failure_blocks_observation_and_place(agent, stage, failure):
    node, task = agent
    original = node.arm.get_position.side_effect
    def position(**kwargs):
        code, pose = original(**kwargs)
        fail_at = {'pick': 1, 'entry': 2, 'waypoint': 3}[stage]
        if node.arm.get_position.call_count == fail_at:
            if failure == 'sdk_error':
                code = -1
            else:
                # Beyond POSITION_TOLERANCE_MM (1.0 mm).
                pose[2] = math.nan if failure == 'nan' else pose[2] - 2.
        return code, pose
    node.arm.get_position.side_effect = position
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == (4 if stage == 'waypoint' else 3)
    assert node.control_gripper.call_count == 2
    node.return_to_home_joint_pose.assert_not_called()
    node.release_zone_token.assert_not_called()
    assert node._place_observation_fault


def test_sub_millimetre_tcp_feedback_noise_does_not_block_observation(agent, monkeypatch):
    node, task = agent
    original = node.arm.get_position.side_effect
    def noisy(**kwargs):
        code, pose = original(**kwargs)
        return code, [pose[0] + .3, pose[1] - .3, pose[2] - .4, *pose[3:]]
    node.arm.get_position.side_effect = noisy
    feed(monkeypatch, node, [observation(310, 180)] * 3)
    assert node.execute_task(task)
    assert task['reference_place_pose'] == dict(x=310, y=180, z=60, yaw=0)
    moves = [c.args[:3] for c in node.move_to_robot_tf.call_args_list]
    # The path starts at the measured TCP; no sub-tolerance rise back to the command.
    assert moves[3] == pytest.approx((moves[2][0] + .3, moves[2][1] - .3, moves[2][2] - .4))
    node.release_zone_token.assert_called_once()


def test_field_c24_preflight_never_checks_near_zero_segments(agent, monkeypatch):
    """2026-11-05: measured Z 316.874725 vs command 316.8748 produced a 0.00008 mm
    rise and a zero-length final check; the controller rejected the path (C24)."""
    node, task = agent
    original = node.arm.get_position.side_effect
    def measured(**kwargs):
        code, pose = original(**kwargs)
        return code, [pose[0], pose[1], pose[2] - 7.8e-5, *pose[3:]]
    node.arm.get_position.side_effect = measured
    feed(monkeypatch, node, [observation(310, 180)] * 3)
    sequences = []
    preflight = node.preflight_stack_commands
    def record(commands):
        sequences.append([node.arm.get_position()[1][:6]] + [
            [c[a] for a in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')] for c in commands])
        return preflight(commands)
    node.preflight_stack_commands = record
    assert node.execute_task(task)
    assert len(sequences) == 2
    for sequence in sequences:  # each starts at the arm's measured pose
        for before, after in zip(sequence, sequence[1:]):
            moved = math.dist(before[:3], after[:3])
            rotated = before[3:] != pytest.approx(after[3:])
            assert moved >= vision.POSITION_TOLERANCE_MM or (moved == 0 and rotated)


def test_tf_shift_after_observation_cannot_turn_xy_correction_into_descent(agent, monkeypatch):
    node, task = agent
    feed(monkeypatch, node, [observation()] * 3)
    observe = node.observe_previous_layer
    def shifted(task):
        result = observe(task)
        node.convert_place_pose = lambda p, agent: dict(p, z=p['z'] - 1.)
        return result
    node.observe_previous_layer = shifted
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 6  # observation completed, no correction
    assert node.control_gripper.call_count == 2
    node.return_to_home_joint_pose.assert_not_called()
    node.release_zone_token.assert_not_called()
    assert node._place_observation_fault


@pytest.mark.parametrize('overrides', [
    dict(observation_validated=False),
    dict(tcp_frame='agent2_tcp'),
    dict(tcp_frame=''),
])
def test_unmeasured_or_invalid_observation_blocks_before_sdk(agent, overrides):
    node, task = agent
    node.stack_vision_config.update(overrides)
    assert not node.execute_task(task)
    node.arm.clean_error.assert_not_called()
    node.move_to_robot_tf.assert_not_called()
    node.control_gripper.assert_not_called()


@pytest.mark.parametrize('disabled', ['zone_token_enabled', 'enable_sdk_safety_box'])
def test_observation_requires_safety_gates(agent, disabled):
    node, task = agent
    setattr(node, disabled, False)
    assert not node.execute_task(task)
    node.arm.clean_error.assert_not_called()


def test_corrected_transfer_and_descent_never_mix_horizontal_and_vertical_motion(agent, monkeypatch):
    node, task = agent
    feed(monkeypatch, node, [observation(310, 180)] * 3)
    events = []
    original_wait = node.wait_for_assembly_predecessors
    def wait(t):
        events.append('predecessors')
        assert node.move_to_robot_tf.call_count == 3
        original_wait(t)
    node.wait_for_assembly_predecessors = wait
    original_acquire = node.acquire_zone_token.side_effect
    def acquire(t):
        assert events == ['predecessors']
        events.append('token')
        return original_acquire(t)
    node.acquire_zone_token.side_effect = acquire
    original_verify = node.observe_previous_layer
    def verify(t):
        assert events == ['predecessors', 'token']
        assert node.move_to_robot_tf.call_count == 6
        assert node.place_verification_request is None
        return original_verify(t)
    node.observe_previous_layer = verify
    assert node.execute_task(task)
    points = [c.args[:3] for c in node.move_to_robot_tf.call_args_list[2:]]
    for before, after in zip(points, points[1:]):
        if not math.isclose(before[2], after[2]):
            assert before[:2] == pytest.approx(after[:2])
    # Observe at the basket mean; correct horizontally at the same high Z.
    assert points[0][2] == 230.  # existing Pick lift
    assert all(p[2] >= points[0][2] for p in points[:5])
    assert points[3] == (300., 200., 230.)
    assert points[4] == (310., 180., 230.)
    assert points[5] == (310., 180., 220.)  # descent only after XY correction
    assert all(p[:2] == (310., 180.) for p in points[4:])
    assert points[-3][2] == 220.  # original high approach
    assert points[-2][2] == 60.   # original release Z
    assert points[-1][2] == 220.  # original retreat
    node.release_zone_token.assert_called_once()


@pytest.mark.parametrize('stage', ['ingress_end', 'corrected', 'transfer'])
def test_entire_path_is_preflighted_before_its_first_move(agent, monkeypatch, stage):
    node, task = agent
    feed(monkeypatch, node, [observation()] * 3)
    calls = 0
    def safety(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == {'ingress_end': 4, 'corrected': 6, 'transfer': 9}[stage]:
            raise ValueError('outside safety box')
    node.check_sdk_pose_or_raise.side_effect = safety
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == (3 if stage == 'ingress_end' else 6)
    assert node.control_gripper.call_count == 2
    node.return_to_home_joint_pose.assert_not_called()


@pytest.mark.parametrize('after_move', [4, 6, 7, 9])
def test_token_loss_stops_each_observation_segment_and_release(agent, monkeypatch, after_move):
    node, task = agent
    feed(monkeypatch, node, [observation()] * 3)
    def move(*args, **kwargs):
        if node.move_to_robot_tf.call_count == after_move:
            node._zone_token_held = False
        return True
    node.move_to_robot_tf.side_effect = move
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == after_move
    assert node.control_gripper.call_count == 2
    node.return_to_home_joint_pose.assert_not_called()
    node.release_zone_token.assert_not_called()


def test_observation_tf_failure_prevents_observation_motion(agent):
    node, task = agent
    node.lookup_workspace_transform.side_effect = RuntimeError('TF unavailable')
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 3
    assert node.control_gripper.call_count == 2


def test_fault_blocks_next_task_without_releasing_or_moving(agent, monkeypatch):
    node, task = agent
    clock = use_observation_clock(monkeypatch, node)
    obs = observation()
    obs['detections'] = []
    feed(monkeypatch, node, [obs], clock)
    assert not node.execute_task(task)
    node.arm.clean_error.reset_mock()
    node.move_to_robot_tf.reset_mock()
    node.control_gripper.reset_mock()
    assert not node.execute_task(task)
    assert not node.execute_cooperative_task({})
    node.arm.clean_error.assert_not_called()
    node.move_to_robot_tf.assert_not_called()
    node.control_gripper.assert_not_called()
    node.release_zone_token.assert_not_called()


def test_observation_keeps_measured_pick_attitude(agent, monkeypatch):
    node, task = agent
    object_profiles.OBJECT_PROFILES['양상추'].update(
        pick_rpy_robot_deg=[178., 1., 2.], place_rpy_robot_deg=[170., 2., 3.])
    feed(monkeypatch, node, [observation()] * 3)
    assert node.execute_task(task)
    for move in node.move_to_robot_tf.call_args_list[3:7]:
        assert move.kwargs['roll_deg'] == 178.
        assert move.kwargs['pitch_deg'] == 1.
        assert move.kwargs['yaw'] == math.radians(2.)


def test_stale_frames_after_transfer_preflight_abort_before_motion(agent, monkeypatch):
    node, task = agent
    feed(monkeypatch, node, [observation()] * 3)
    def safety(*args, **kwargs):
        if node.check_sdk_pose_or_raise.call_count == 9:
            node.now_sec = lambda: 999.
    node.check_sdk_pose_or_raise.side_effect = safety
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 6
    assert node.control_gripper.call_count == 2
    node.return_to_home_joint_pose.assert_not_called()


def sandwich():
    tasks = []
    for i, material in enumerate(('빵', '양상추', '햄', '빵')):
        tasks.append(dict(task_id=f'layer_{i}', depends_on=[] if i == 0 else [f'layer_{i-1}'],
                          assembly=dict(id='sandwich_1', container='basket', layer_index=i,
                                        material=material)))
    return tasks


@pytest.mark.parametrize('layer, expected', [(1, '빵'), (2, '양상추'), (3, '햄')])
def test_target_is_immediate_layer_not_any_prior_bread(layer, expected):
    tasks = sandwich()
    statuses = {t['task_id']: 'SUCCEEDED' for t in tasks}
    assert vision.get_previous_layer_target(tasks[layer], tasks, statuses)['material'] == expected
    statuses[f'layer_{layer-1}'] = 'FAILED'
    with pytest.raises(ValueError):
        vision.get_previous_layer_target(tasks[layer], tasks, statuses)


def test_repeated_previous_material_layers_fail_closed():
    tasks = sandwich()
    tasks[2]['assembly']['material'] = '빵'
    with pytest.raises(ValueError, match='높이 구분'):
        vision.get_previous_layer_target(tasks[3], tasks, {t['task_id']: 'SUCCEEDED' for t in tasks})


@pytest.mark.parametrize('failure', ['missing', 'duplicate', 'other_assembly'])
def test_previous_layer_must_be_unique_in_same_assembly(failure):
    tasks = sandwich()
    if failure == 'missing':
        del tasks[0]
    elif failure == 'duplicate':
        tasks.append(copy.deepcopy(tasks[0]))
    else:
        tasks[0]['assembly']['id'] = 'other'
    with pytest.raises(ValueError):
        vision.get_previous_layer_target(sandwich()[1], tasks, {t['task_id']: 'SUCCEEDED' for t in tasks})


@pytest.mark.parametrize('material', ['햄', '빵'])
def test_real_uncalibrated_profiles_reject_offsets(material):
    assert object_profiles.OBJECT_PROFILES[material]['stack_tcp_to_material_center_mm'] is None
    with pytest.raises(ValueError):
        vision.material_center_offset(material, 'agent1')


@pytest.mark.parametrize('agent_id, expected', [('agent1', [1., 2., 3.]), ('agent2', [-1., 4., 5.])])
def test_agent_specific_center_offset(monkeypatch, agent_id, expected):
    profile = dict(object_profiles.OBJECT_PROFILES['햄'], stack_tcp_to_material_center_mm={
        'agent1': [1., 2., 3.], 'agent2': [-1., 4., 5.]})
    monkeypatch.setitem(object_profiles.OBJECT_PROFILES, '햄', profile)
    assert vision.material_center_offset('햄', agent_id) == expected
    with pytest.raises(ValueError):
        vision.material_center_offset('햄', 'agent3')


def test_local_offset_rotates_all_xyz_components_and_preserves_pose():
    nominal = dict(x=999., y=-999., z=90., yaw=0.4, roll=1.2, pitch=0.3)
    rotation = handling.rpy_to_matrix(0., math.pi / 2, math.pi / 2)
    pose = vision.calculate_stack_tcp_pose([300., 200., 30.], [10., 20., 30.], rotation, nominal)
    assert pose == pytest.approx(dict(nominal, x=320., y=170.))


def set_layer(node, task, layer, calibrated=True):
    tasks = sandwich()
    node.guidebook_tasks = {t['task_id']: t for t in tasks}
    node.guidebook_task_status = {t['task_id']: ('SUCCEEDED' if i < layer else 'READY')
                                for i, t in enumerate(tasks)}
    object_profiles.OBJECT_PROFILES['햄'] = dict(
        grip_position=30., thickness_mm=5., place_tcp_offset_mm=100.,
        place_roll_offset_deg={'agent1': 30., 'agent2': -30.}, place_tilt_speed=75.)
    material = tasks[layer]['assembly']['material']
    object_profiles.OBJECT_PROFILES[material]['stack_tcp_to_material_center_mm'] = (
        [10., 20., 5.] if calibrated else None)
    task.update(target=material, guidebook_task_id=f'layer_{layer}',
                assembly=copy.deepcopy(tasks[layer]['assembly']))
    node.stack_vision_config.update(materials=[material], max_correction_mm=100.)
    return material


@pytest.mark.parametrize('layer', [0, 1, 2, 3])
@pytest.mark.parametrize('agent_id', ['agent1', 'agent2'])
def test_only_agent1_lettuce_layer_one_can_use_vision(agent, layer, agent_id):
    node, task = agent
    node.agent_id = task['assignee_id'] = agent_id
    set_layer(node, task, layer)
    if layer == 1 and agent_id == 'agent1':
        assert node.needs_stack_place(task)
        return
    node.prepare_stack_place = Mock(side_effect=AssertionError('out-of-scope vision'))
    assert not node.needs_stack_place(task)
    assert node.execute_task(task)
    node.prepare_stack_place.assert_not_called()
    node.arm.set_position.assert_not_called()  # no controller preflight for other Agents/materials


@pytest.mark.parametrize('after_move', [7, 8, 9])
def test_token_loss_before_place_or_release_stops_immediately(agent, monkeypatch, after_move):
    node, task = agent
    feed(monkeypatch, node, [observation()] * 3)
    def move(*args, **kwargs):
        if node.move_to_robot_tf.call_count == after_move:
            node._zone_token_held = False
        return True
    node.move_to_robot_tf.side_effect = move
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == after_move
    node.release_zone_token.assert_not_called()
    node.return_to_home_joint_pose.assert_not_called()
    assert node.control_gripper.call_count == 2


def test_first_bread_bypasses_vision_even_with_unset_offset(agent):
    node, task = agent
    set_layer(node, task, 0, calibrated=False)
    node.prepare_stack_place = Mock(side_effect=AssertionError('first layer cannot observe'))
    assert node.execute_task(task)
    assert task['reference_place_pose']['x'] == 300.
    assert task['reference_place_pose']['y'] == 200.


def test_stale_blue_handles_after_long_wait_fail_without_observation_motion(agent):
    node, task = agent
    def acquire(_):
        node._zone_token_held = True
        node.now_sec = lambda: 200.
        return True
    node.acquire_zone_token.side_effect = acquire
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 3
    assert node.control_gripper.call_count == 2
    node.release_zone_token.assert_not_called()


def test_observation_uses_only_real_tcp_camera_tf():
    node = load_agent_methods()()
    node.stack_vision_config = config()
    node.tf_lookup_timeout_sec = 1.
    transform = rigid_transform((.01, .02, .03))
    node.tf_buffer = Mock()
    node.tf_buffer.lookup_transform.return_value = SimpleNamespace(
        header=SimpleNamespace(frame_id='agent1_tcp'), child_frame_id='camera_optical', transform=transform)
    assert node.stack_camera_mount(metadata()) is transform
    call = node.tf_buffer.lookup_transform.call_args
    assert call.args[:2] == ('agent1_tcp', 'camera_optical')
    assert call.args[2].seconds == 100.1
    node.tf_buffer.lookup_transform.side_effect = RuntimeError('missing active TCP')
    with pytest.raises(RuntimeError):
        node.stack_camera_mount(metadata())


@pytest.mark.parametrize('start_z,end_z', [(180., 220.), (300., 220.)])
def test_transit_uses_higher_existing_height_without_low_horizontal_motion(start_z, end_z):
    start, end = dict(x=10., y=20., z=start_z), dict(x=300., y=200., z=end_z)
    path = [start] + vision.transit_waypoints(start, end)
    for a, b in zip(path, path[1:]):
        if (a['x'], a['y']) != (b['x'], b['y']):
            assert a['z'] == b['z'] == max(start_z, end_z)


def test_predecessor_failure_during_observation_segment_stops_next_move(agent):
    node, task = agent
    def move(*args, **kwargs):
        if node.move_to_robot_tf.call_count == 4:
            node.guidebook_task_status['layer_0'] = 'FAILED'
        return True
    node.move_to_robot_tf.side_effect = move
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 4
    assert node.control_gripper.call_count == 2
    node.release_zone_token.assert_not_called()


@pytest.mark.parametrize('attitude_mode', ['default', 'workspace_tilt', 'base_taught'])
def test_offset_rotation_matches_actual_release_command(agent, attitude_mode):
    node, task = agent
    node.stack_vision_config['enabled'] = False
    profile = object_profiles.OBJECT_PROFILES['양상추']
    if attitude_mode == 'workspace_tilt':
        profile.update(place_roll_offset_deg=30., place_pitch_offset_deg=15.)
    elif attitude_mode == 'base_taught':
        profile.update(pick_rpy_robot_deg=[178., 1., 2.], place_rpy_robot_deg=[170., 20., 35.])
    assert node.validate_received_task(task)
    node.workspace_rpy_to_robot = lambda *args: type(node).workspace_rpy_to_robot(node, *args)
    state = dict(started=False, released=False)
    assert node.execute_place_action(task, dict(z_offset=200., speed=100.), state)
    assert node.execute_place_action(task, dict(z_offset=40., speed=100.), state)
    actual = node.move_to_robot_tf.call_args.kwargs
    base_from_tcp = handling.rpy_to_matrix(math.radians(actual.get('roll_deg', 180.)),
                                          math.radians(actual.get('pitch_deg', 0.)), actual['yaw'])
    base_from_workspace = node.quaternion_to_rotation_matrix(0., 0., 0., 1.)
    np.testing.assert_allclose(node.stack_release_rotation(task), base_from_workspace.T @ base_from_tcp,
                               atol=1e-10)


def test_controller_preflight_covers_whole_corrected_path_before_motion(agent, monkeypatch):
    node, task = agent
    feed(monkeypatch, node, [observation(310, 180)] * 3)
    assert node.execute_task(task)
    checked = [c.kwargs for c in node.arm.set_position.call_args_list]
    # Observation: one level move from the standing Pick lift. Correction: level XY,
    # descent, then release and retreat; the repeated high approach is not re-checked.
    assert [p['only_check_type'] for p in checked] == [1, 1, 3, 3, 2]
    assert all(p['wait'] is False and p['is_radian'] is False for p in checked)
    assert [(p['x'], p['y'], p['z']) for p in checked] == [
        (300., 200., 230.), (310., 180., 230.), (310., 180., 220.),
        (310., 180., 60.), (310., 180., 220.)]
    node.arm.set_only_check_type.assert_not_called()  # no global SDK mode changes


@pytest.mark.parametrize('check_index', [1, 2, 3, 4, 5])
@pytest.mark.parametrize('failure', ['return_code', 'collision_result', 'exception'])
def test_controller_path_failure_never_executes_unchecked_path(agent, monkeypatch, check_index, failure):
    node, task = agent
    feed(monkeypatch, node, [observation()] * 3)
    def check(**kwargs):
        if node.arm.set_position.call_count == check_index:
            if failure == 'exception':
                raise RuntimeError('controller offline')
            if failure == 'collision_result':
                node.arm.only_check_result = 1
                return 0
            return -7
        return 0
    node.arm.set_position.side_effect = check
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == (3 if check_index == 1 else 6)
    assert node.control_gripper.call_count == 2
    assert task['reference_place_pose']['y'] == 170.
    assert node._place_observation_fault
    node.release_zone_token.assert_not_called()
    node.return_to_home_joint_pose.assert_not_called()


@pytest.mark.parametrize('version', [(1, 11, 99), (1, 10, 0), None, (2,), ('2', 4, 0)])
def test_unsupported_firmware_cannot_skip_path_validation(agent, version):
    node, task = agent
    node.arm.version_number = version
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 3
    node.arm.set_position.assert_not_called()
    node.release_zone_token.assert_not_called()


@pytest.mark.parametrize('stage', ['ingress', 'observation'])
@pytest.mark.parametrize('failure', ['moving', 'paused', 'error', 'state_query', 'disconnected'])
def test_capture_requires_successful_normal_stop(agent, stage, failure):
    node, task = agent
    def state():
        if stage == 'observation' and node.move_to_robot_tf.call_count < 6:
            return 0, 2
        if failure == 'moving':
            node.arm.get_is_moving.return_value = True
        elif failure == 'error':
            node.arm.error_code = 1
        elif failure == 'disconnected':
            raise RuntimeError('disconnected')
        return (-1, 2) if failure == 'state_query' else (0, 3 if failure == 'paused' else 2)
    node.arm.get_state.side_effect = state
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == (3 if stage == 'ingress' else 6)
    assert node.place_verification_request is None
    assert node.control_gripper.call_count == 2
    node.release_zone_token.assert_not_called()
    node.return_to_home_joint_pose.assert_not_called()


@pytest.mark.parametrize('after_move', [4, 5, 6, 7])
def test_failed_motion_never_continues_to_capture_or_release(agent, monkeypatch, after_move):
    node, task = agent
    feed(monkeypatch, node, [observation()] * 3)
    node.move_to_robot_tf.side_effect = lambda *a, **kw: node.move_to_robot_tf.call_count != after_move
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == after_move
    assert node.control_gripper.call_count == 2
    node.return_to_home_joint_pose.assert_not_called()
    node.release_zone_token.assert_not_called()


def test_first_layer_must_be_bread_for_agent1_lettuce(agent):
    node, task = agent
    node.guidebook_tasks['layer_0']['assembly']['material'] = '바나나'
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 3
    assert node.control_gripper.call_count == 2


@pytest.mark.parametrize('link', ['mount', 'tcp'])
@pytest.mark.parametrize('failure', ['missing', 'wrong_parent', 'wrong_child', 'nan', 'zero_rotation'])
def test_each_capture_tf_link_fails_closed(link, failure):
    node = load_agent_methods()()
    node.stack_vision_config = config()
    node.tf_lookup_timeout_sec = .1
    node.tf_buffer = Mock()
    def lookup(parent, child, stamp, **kw):
        transform = rigid_transform()
        selected = (parent == 'agent1_tcp') == (link == 'mount')
        if selected:
            if failure == 'missing':
                raise RuntimeError('no TF at capture time')
            if failure == 'nan':
                transform.translation.x = math.nan
            if failure == 'zero_rotation':
                transform.rotation.w = 0.
        return SimpleNamespace(
            header=SimpleNamespace(frame_id='wrong' if selected and failure == 'wrong_parent' else parent),
            child_frame_id='wrong' if selected and failure == 'wrong_child' else child, transform=transform)
    node.tf_buffer.lookup_transform.side_effect = lookup
    with pytest.raises((ValueError, RuntimeError)):
        node.place_camera_transform(metadata(), dict(fx=1., fy=1., cx=0., cy=0.),
                                    np.zeros((2, 2, 3)), np.ones((2, 2)))


def test_acquisition_tf_chain_produces_real_workspace_xyz_with_rotated_mount():
    node = load_agent_methods()()
    node.camera_frame = 'camera_optical'
    # Both links rotate; TCP<-camera translation must rotate with workspace<-TCP.
    a = math.sqrt(.5)
    parent = rigid_transform((.3, .2, .4), (0., 0., a, a))
    child = rigid_transform((.01, .02, .03), (a, 0., 0., a))
    transform = vision.compose_transforms(parent, child)
    point = node.camera_point_to_workspace(.01, .02, .5, transform=transform)
    # camera (.01,.02,.5) -> TCP (.02,-.48,.05) -> workspace (.78,.22,.45)
    assert point == pytest.approx((780., 220., 450.))
    assert (parent.translation.x, child.translation.x) == (.3, .01)  # inputs are unchanged


@pytest.mark.parametrize('attitude_mode', ['default', 'workspace_tilt', 'base_taught'])
def test_preflight_release_attitude_exactly_matches_existing_place_moves(agent, monkeypatch, attitude_mode):
    node, task = agent
    profile = object_profiles.OBJECT_PROFILES['양상추']
    if attitude_mode == 'workspace_tilt':
        profile.update(place_roll_offset_deg=30., place_pitch_offset_deg=15.)
    elif attitude_mode == 'base_taught':
        profile.update(pick_rpy_robot_deg=[178., 1., 2.], place_rpy_robot_deg=[170., 20., 35.])
    feed(monkeypatch, node, [observation()] * 3)
    assert node.execute_task(task)
    def pose(move):
        return (*move.args[:3], math.degrees(move.kwargs['yaw']),
                move.kwargs.get('roll_deg', 180.), move.kwargs.get('pitch_deg', 0.))
    # After the single observation check, preflight mirrors every correction/Place
    # move except exact repeats of the pose the arm already holds.
    plan = [c.kwargs for c in node.arm.set_position.call_args_list][1:]
    moves = node.move_to_robot_tf.call_args_list[5:]
    actual = [m for prev, m in zip(moves, moves[1:]) if pose(m) != pytest.approx(pose(prev))]
    assert len(plan) == len(actual)
    for checked, move in zip(plan, actual):
        assert (checked['x'], checked['y'], checked['z']) == pytest.approx(move.args[:3])
        assert checked['yaw'] == pytest.approx(math.degrees(move.kwargs['yaw']))
        assert checked['roll'] == pytest.approx(move.kwargs.get('roll_deg', 180.))
        assert checked['pitch'] == pytest.approx(move.kwargs.get('pitch_deg', 0.))
        assert checked['speed'] == move.kwargs['speed']


def test_config_cleanup_separates_default_off_from_field_settings_and_has_no_manual_geometry():
    from pathlib import Path
    import yaml
    removed = {'observation_camera_xy_offset_mm', 'observation_camera_z_mm',
               'observation_transit_z_mm', 'observation_speed_mm_s', 'observation_z_offset'}
    assert vision.DEFAULTS['enabled'] is False
    assert vision.DEFAULTS['observation_validated'] is False
    assert not removed.intersection(vision.DEFAULTS)
    for agent_id in ('agent1', 'agent2'):
        path = Path(__file__).resolve().parents[1] / 'config' / f'{agent_id}.yaml'
        config_values = yaml.safe_load(path.read_text())[f'/{agent_id}/robot_agent_node']['ros__parameters']
        # Agent1 is the existing field experiment configuration; it is not the
        # unconfigured runtime default. Keep Agent2 disabled.
        assert config_values['stack_place_vision.enabled'] is (agent_id == 'agent1')
        assert config_values['stack_place_vision.observation_validated'] is (agent_id == 'agent1')
        assert not {f'stack_place_vision.{key}' for key in removed}.intersection(config_values)
