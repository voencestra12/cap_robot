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
        'place_tcp_offset_mm': 60.0, 'place_xy_offset_mm': [0.0, -30.0]}
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
    # agent1_tcp <- camera optical: ~100 mm beside and 87 mm behind the TCP, optical
    # axis along the tool axis (handeye_agent1.yaml + RealSense optical convention).
    node.camera_frame = 'camera_optical'
    node.tcp_camera_mount = rigid_transform((.1, 0., -.087))
    node.tf_buffer = Mock()
    node.tf_buffer.lookup_transform.side_effect = lambda parent, child, *a, **kw: SimpleNamespace(
        header=SimpleNamespace(frame_id=parent), child_frame_id=child,
        transform=node.tcp_camera_mount)
    node.tf_lookup_timeout_sec = 0.1
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
        if kwargs.get('label', '').endswith('place'):
            # Four distinct post-stop frames have been consumed before the corrected Place.
            assert clock.now == pytest.approx(10.08)
        return original_move(*args, **kwargs) if callable(original_move) else True

    node.move_to_robot_tf.side_effect = move
    assert node.execute_task(task)
    assert task['reference_place_pose'] == dict(x=310, y=180, z=60, yaw=0)
    labels = [c.kwargs['label'] for c in node.move_to_robot_tf.call_args_list]
    assert labels.count('look at previous layer') == 1
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
    assert node.move_to_robot_tf.call_count == 4
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
    assert node.move_to_robot_tf.call_count == 4
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
    assert node.move_to_robot_tf.call_count == 4
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
        assert node.move_to_robot_tf.call_count == 4  # pick trio + in-place look
        assert node.move_to_robot_tf.call_args.kwargs['label'] == 'look at previous layer'
        assert node._zone_token_held
        assert node.control_gripper.call_count == 2
        return original(t)
    node.observe_previous_layer = verify
    assert node.execute_task(task)
    assert task['reference_place_pose'] == dict(x=300 + delta[0], y=170 + delta[1], z=60, yaw=0)
    # Pick trio, in-place look, then the unchanged Place approach/release/retreat.
    moves = node.move_to_robot_tf.call_args_list[-3:]
    assert node.move_to_robot_tf.call_count == 7
    assert [c.args[:3] for c in moves] == [
        (300 + delta[0], 170 + delta[1], 220),
        (300 + delta[0], 170 + delta[1], 60),
        (300 + delta[0], 170 + delta[1], 220)]
    assert all(c.kwargs['yaw'] == 0.2 for c in moves)
    assert all(c.kwargs['agent_id'] == agent_id for c in node.workspace_point_to_robot.call_args_list)
    assert node.is_moving and node.place_verification_request is None


@pytest.mark.parametrize('failure', ['missing', 'multiple', 'stale', 'duplicate', 'unstable',
                                      'excessive', 'tf', 'depth'])
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
    feed(monkeypatch, node, observations, clock)
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 4  # Pick trio + look; no Place
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
    assert node.move_to_robot_tf.call_count == 4


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


# 4: in-place look, 5: Place approach, 6: Place descent (before release), 7: retreat (before home)
@pytest.mark.parametrize('after_move', [4, 5, 6, 7])
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
    # open + grasp; the release precedes the retreat (move 7)
    assert node.control_gripper.call_count == (3 if after_move == 7 else 2)
    node.return_to_home_joint_pose.assert_not_called()
    node.release_zone_token.assert_not_called()


def test_observation_tf_failure_prevents_observation_motion(agent):
    node, task = agent
    node.tf_buffer.lookup_transform.side_effect = RuntimeError('TCP->camera TF unavailable')
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
    node.arm.set_position.assert_not_called()  # no extra stack-vision SDK commands


@pytest.mark.parametrize('after_move', [5, 6, 7])
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
    # open + grasp; the release precedes the retreat (move 7)
    assert node.control_gripper.call_count == (3 if after_move == 7 else 2)


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


@pytest.mark.parametrize('stage', ['ingress', 'observation'])
@pytest.mark.parametrize('failure', ['moving', 'paused', 'error', 'state_query', 'disconnected'])
def test_capture_requires_successful_normal_stop(agent, stage, failure):
    node, task = agent
    def state():
        if stage == 'observation' and node.move_to_robot_tf.call_count < 4:
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
    assert node.move_to_robot_tf.call_count == (3 if stage == 'ingress' else 4)
    assert node.place_verification_request is None
    assert node.control_gripper.call_count == 2
    node.release_zone_token.assert_not_called()
    node.return_to_home_joint_pose.assert_not_called()


# 4: in-place look, 5: Place approach, 6: Place descent; all before release
@pytest.mark.parametrize('after_move', [4, 5, 6])
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


def optical_ray_miss_mm(rotation, tcp, mount_rotation, mount_offset, target):
    camera = np.array(tcp) + rotation @ np.array(mount_offset)
    axis = rotation @ np.array(mount_rotation)[:, 2]
    to_target = np.array(target) - camera
    return float(np.linalg.norm(np.cross(axis, to_target))), float(np.dot(axis, to_target))


VERTICAL = handling.rpy_to_matrix(math.pi, 0., math.radians(-93.1))  # Pick lift attitude
MOUNT_R, MOUNT_T = np.eye(3), np.array([100., 0., -87.])            # TCP <- camera, mm


@pytest.mark.parametrize('target', [(450., -180., 90.), (300., 150., 60.), (200., 40., 120.)])
def test_look_at_points_camera_optical_axis_at_target_without_moving_tcp(target):
    tcp = (260.4, 77.7, 316.7)
    rotation, tilt, clamped = vision.look_at_attitude(VERTICAL, tcp, MOUNT_R, MOUNT_T, target, 89.)
    assert not clamped
    miss, ahead = optical_ray_miss_mm(rotation, tcp, MOUNT_R, MOUNT_T, target)
    assert miss < 1e-6 and ahead > 0  # the target lies on the optical axis, in front
    assert np.allclose(rotation.T @ rotation, np.eye(3)) and np.linalg.det(rotation) == pytest.approx(1.)
    assert tilt == pytest.approx(math.degrees(math.acos(-rotation[2, 2])))


def test_look_at_target_already_on_axis_keeps_pick_attitude():
    tcp = np.array([260., 80., 316.])
    below = tcp + VERTICAL @ MOUNT_T + VERTICAL @ np.array([0., 0., 250.])
    rotation, tilt, clamped = vision.look_at_attitude(VERTICAL, tcp, MOUNT_R, MOUNT_T, below)
    assert np.allclose(rotation, VERTICAL, atol=1e-9) and tilt == pytest.approx(0., abs=1e-6)
    assert not clamped


def test_look_at_clamps_tilt_but_keeps_direction_toward_target():
    tcp, far = (260., 80., 316.), (700., -400., 0.)
    rotation, tilt, clamped = vision.look_at_attitude(VERTICAL, tcp, MOUNT_R, MOUNT_T, far, 30.)
    assert clamped and tilt == pytest.approx(30.)
    assert math.degrees(math.acos(-rotation[2, 2])) == pytest.approx(30.)
    lean = rotation[:2, 2] / np.linalg.norm(rotation[:2, 2])
    wanted = (np.array(far[:2]) - tcp[:2]) / np.linalg.norm(np.array(far[:2]) - tcp[:2])
    assert float(np.dot(lean, wanted)) > 0.95  # leans toward the basket, only less


@pytest.mark.parametrize('action_speed,expected', [(100., 20.), (10., 10.)])
def test_look_rotation_uses_existing_material_tilt_speed_cap(action_speed, expected):
    assert vision.look_speed('양상추', 'agent1', action_speed) == expected


@pytest.mark.parametrize('max_tilt', [89., 45.])
def test_lettuce_looks_in_place_then_runs_unchanged_place_actions(agent, monkeypatch, max_tilt):
    node, task = agent
    monkeypatch.setattr(vision, 'MAX_LOOK_TILT_DEG', max_tilt)  # fixture needs ~60 deg
    feed(monkeypatch, node, [observation(310, 180)] * 3)
    order = []
    observe = node.observe_previous_layer
    def verify(t):
        order.append('observe')
        return observe(t)
    node.observe_previous_layer = verify
    original = node.move_to_robot_tf.side_effect
    def move(*args, **kwargs):
        order.append(kwargs.get('label'))
        return original(*args, **kwargs) if callable(original) else True
    node.move_to_robot_tf.side_effect = move
    assert node.execute_task(task)
    moves = node.move_to_robot_tf.call_args_list
    lift, look = moves[2], moves[3]
    assert look.kwargs['label'] == 'look at previous layer'
    assert look.args[:3] == pytest.approx(lift.args[:3])  # rotation only, TCP stays
    assert look.kwargs['speed'] == 20.  # lettuce place_tilt_speed cap
    # Camera optical axis through the basket center at the previous-layer Z gate.
    rotation = handling.rpy_to_matrix(math.radians(look.kwargs['roll_deg']),
                                      math.radians(look.kwargs['pitch_deg']), look.kwargs['yaw'])
    target = (300., 200., float(np.mean(node.stack_vision_config['target_z_range_mm'])))
    miss, _ = optical_ray_miss_mm(rotation, lift.args[:3], MOUNT_R, MOUNT_T * 1., target)
    tilt = math.degrees(math.acos(-rotation[2, 2]))
    if max_tilt == 89.:
        assert miss < 1e-6 and tilt < 89.
    else:
        assert tilt == pytest.approx(45.)  # limited lean toward the basket
    assert order.index('look at previous layer') < order.index('observe')
    # Then the existing Place actions at the corrected XY, back in the Place attitude.
    assert [m.args[:3] for m in moves[4:]] == [(310., 180., 220.), (310., 180., 60.), (310., 180., 220.)]
    assert all('roll_deg' not in m.kwargs for m in moves[4:])
    node.release_zone_token.assert_called_once()


@pytest.mark.parametrize('failure', ['sdk_error', 'nan'])
def test_unreadable_tcp_blocks_look_and_place(agent, failure):
    node, task = agent
    original = node.arm.get_position.side_effect
    def position(**kwargs):
        code, pose = original(**kwargs)
        return (-1, pose) if failure == 'sdk_error' else (0, [*pose[:2], math.nan, *pose[3:]])
    node.arm.get_position.side_effect = position
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 3  # Pick only
    assert node.control_gripper.call_count == 2
    node.release_zone_token.assert_not_called()
    assert node._place_observation_fault


@pytest.mark.parametrize('tops,expected', [
    ([(0, 0, 37.3)] * 3, 97.3),              # 2026-11-06: measured top 12 mm above nominal
    ([(0, 0, 36.), (0, 0, 37.), (0, 0, 38.)], 97.),  # mean of the stable samples
    ([(0, 0, 20.)] * 3, 85.),                # lower than nominal: never descend further
])
def test_release_z_follows_measured_top_but_never_below_nominal(tops, expected):
    # Lettuce place_tcp_offset_mm 60; nominal = floor 5 + bread 20 + 60 = 85.
    assert vision.stack_release_z(85., tops, '양상추', 'agent1') == pytest.approx(expected)


def test_measured_higher_bread_raises_the_whole_unchanged_place_sequence(agent, monkeypatch):
    node, task = agent
    high = [dict(observation(310, 180), detections=[('빵_99', (310, 180, 42., 0), 0.9)])] * 3
    feed(monkeypatch, node, high)
    assert node.execute_task(task)
    # Fixture nominal release Z 60 (top 30 + 30); measured top 42 -> release 72.
    assert task['reference_place_pose'] == dict(x=310, y=180, z=72., yaw=0)
    place = [m.args[:3] for m in node.move_to_robot_tf.call_args_list[4:]]
    assert place == [(310., 180., 232.), (310., 180., 72.), (310., 180., 232.)]
    node.release_zone_token.assert_called_once()


# 2026-11-06: the same bread in workspace Z under three ArUco locks, and each lock's
# workspace -> Base Z offset (from the logged bread release ws 95 -> Base Z).
FIELD_LOCKS = [(-58.2, 220.5 - 95.), (37.3, 127.9 - 95.), (15.0, 148.2 - 95.)]


@pytest.mark.parametrize('ws_z,offset', FIELD_LOCKS)
def test_base_z_gate_accepts_same_bread_under_every_aruco_lock(ws_z, offset):
    obs = observation()
    obs['detections'] = [('빵_3', (300, 170, ws_z, 0), 0.9)]
    cfg = config(target_z_range_mm=[55., 85.])
    checks = []
    center = vision.previous_layer_center(obs, dict(x=300, y=200), '빵', cfg, diagnostics=checks,
                                          base_z=lambda x, y, z: z + offset)
    assert center == (300, 170, ws_z)  # workspace XYZ is still returned for the Place math
    assert checks[0]['z_range'] == dict(passed=True, value=pytest.approx(ws_z + offset),
                                        frame='robot_base', limits=[55., 85.])
    # The old workspace gate rejected the -58 mm lock (and 14.2-15 mm frames at the edge).
    if not 15.0 <= ws_z <= 45.0:
        with pytest.raises(vision.NoPreviousLayerCandidate):
            vision.previous_layer_center(obs, dict(x=300, y=200), '빵',
                                         config(target_z_range_mm=[15., 45.]))


def test_observation_gate_uses_base_z_from_the_workspace_transform(agent, monkeypatch):
    node, task = agent
    node.stack_vision_config['target_z_range_mm'] = [55., 85.]
    identity = node.convert_place_pose
    node.convert_place_pose = lambda pose, agent: dict(identity(pose, agent), z=pose.get('z', 0.) + 40.)
    feed(monkeypatch, node, [observation(310, 180)] * 3)  # ws z 30 -> Base z 70
    assert node.execute_task(task)
    assert (task['reference_place_pose']['x'], task['reference_place_pose']['y']) == (310, 180)


def test_timeout_reports_rejected_fresh_frames(agent, monkeypatch):
    node, task = agent
    assert node.validate_received_task(task)
    clock = use_observation_clock(monkeypatch, node)
    node._zone_token_held = True
    def sleep(seconds):
        clock.now += seconds
        request = node.place_verification_request
        skipped = request.setdefault('skipped', {})
        skipped['정지 이후의 최신 RGB/Depth 프레임이 아닙니다.'] = skipped.get(
            '정지 이후의 최신 RGB/Depth 프레임이 아닙니다.', 0) + 1
    monkeypatch.setattr(time, 'sleep', sleep)
    with pytest.raises(ValueError, match='거부된 새 프레임=.*최신 RGB/Depth'):
        node.observe_previous_layer(task)


def test_stack_vision_place_moves_use_the_material_speed_cap(agent, monkeypatch):
    node, task = agent
    feed(monkeypatch, node, [observation(310, 180)] * 3)
    assert node.execute_task(task)
    moves = node.move_to_robot_tf.call_args_list
    # Look rotation and the following approach/descent/retreat: lettuce place_tilt_speed 20.
    assert [m.kwargs['speed'] for m in moves[3:]] == [20.] * 4
    assert all(m.kwargs['speed'] > 20. for m in moves[:3])  # Pick moves unchanged


def test_regular_place_keeps_action_speed(agent):
    node, task = agent
    node.stack_vision_config['enabled'] = False
    state = dict(started=False, released=False)
    assert node.validate_received_task(task)
    assert node.execute_place_action(task, dict(z_offset=200., speed=100.), state)
    assert node.move_to_robot_tf.call_args.kwargs['speed'] == 100.


@pytest.mark.parametrize('fault_after_move', [4, 5, 6])  # look, Place approach, Place descent
def test_controller_error_after_a_move_stops_before_release(agent, monkeypatch, fault_after_move):
    """2026-11-06: C31 was reported after the descent returned; release and retreat followed."""
    node, task = agent
    feed(monkeypatch, node, [observation(310, 180)] * 3)
    original = node.move_to_robot_tf.side_effect
    def move(*args, **kwargs):
        result = original(*args, **kwargs) if callable(original) else True
        if node.move_to_robot_tf.call_count == fault_after_move:
            node.arm.error_code = 31
        return result
    node.move_to_robot_tf.side_effect = move
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == fault_after_move  # nothing after the fault
    assert node.control_gripper.call_count == 2                  # no release attempt
    node.return_to_home_joint_pose.assert_not_called()
    node.release_zone_token.assert_not_called()
    assert node._place_observation_fault
