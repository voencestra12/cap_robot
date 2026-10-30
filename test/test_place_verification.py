"""No ROS node, SDK connection, live camera, or robot motion is created."""
import copy
import math
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from cap_robot import object_profiles, place_verification as vision
from cap_robot.material_handling import assembly_placement
import test_material_handling as handling
from test_material_handling import load_agent_methods, recipe
import test_object_memory as memory_tests


def config(**overrides):
    return dict(copy.deepcopy(vision.DEFAULTS), baseline_calibrated=True,
                **overrides)


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
        # Calibration need not equal the lettuce offset; do not infer either from the other.
        cfg = config(expected_bread_xy_offset_mm=[2.0, -25.0])
        for delta in ((0, 0), (10, 10), (-10, -5)):
            centers = [(302 + delta[0], 175 + delta[1], 30)] * 3
            result = vision.corrected_place(nominal, dict(x=300, y=200), centers, cfg)
            assert result == dict(nominal, x=300 + delta[0], y=170 + delta[1])


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
    [('빵_1', (100, 100, 30, 0), 0.99)],
    [('빵_1', (300, 170, 200, 0), 0.99)],
    [('빵_1', (300, 170, 30, 0), 0.5)],
    [('빵_1', (300, 170, 30, 0), 0.9), ('빵_2', (310, 170, 30, 0), 0.9)],
])
def test_missing_outside_low_confidence_or_multiple_bread_is_rejected(detections):
    obs = observation()
    obs['detections'] = detections
    with pytest.raises(ValueError):
        vision.bread_center(obs, dict(x=300, y=200), config())


def test_class_and_position_choose_candidate_without_id_dependency():
    obs = observation()
    obs['detections'] += [('빵_1', (500, 170, 30, 0), 0.9),
                          ('양상추_1', (300, 170, 30, 0), 0.9)]
    assert vision.bread_center(obs, dict(x=300, y=200), config()) == (300, 170, 30)


@pytest.fixture
def agent():
    case = handling.AgentAssemblyTest()
    case.setUp()
    node = case.node
    node.place_vision_config = config(
        enabled=True, expected_bread_xy_offset_mm=[0., -30.], observation_validated=True,
        observation_camera_offset_mm=[0., 0., 260.], observation_transit_clearance_mm=30.)
    node.zone_token_enabled = node.enable_sdk_safety_box = True
    node._shared_zone_box = lambda: dict(x_min=150., x_max=450., y_min=-50., y_max=500.)
    node.lookup_workspace_transform = Mock(return_value=SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0., z=0.),
        rotation=SimpleNamespace(x=0., y=0.70710678, z=0., w=0.70710678))))
    node.quaternion_to_rotation_matrix = lambda *a: np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
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
        grip_position=100., place_xy_offset_mm=[0., -30.])
    node.guidebook_task_status.update(layer_0='SUCCEEDED', layer_1='READY')
    task = case.bread_task()
    node.guidebook_task_status['layer_0'] = 'SUCCEEDED'
    task.update(target='양상추', guidebook_task_id='layer_1',
                assembly=copy.deepcopy(recipe()[1]['assembly']))
    yield node, task
    case.doCleanups()


def feed(monkeypatch, node, observations):
    iterator = iter(observations)
    previous_meta = None
    def deliver(_):
        nonlocal previous_meta
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


@pytest.mark.parametrize('agent_id', ['agent1', 'agent2'])
@pytest.mark.parametrize('delta', [(0, 0), (10, 10)])
def test_execution_corrects_once_after_lift_and_uses_existing_transform(agent, monkeypatch, agent_id, delta):
    node, task = agent
    node.agent_id = task['assignee_id'] = agent_id
    # Exercise the real convert_place_pose/workspace_place_to_robot_place methods.
    node.workspace_point_to_robot = Mock(side_effect=lambda x, y, z, **kw: (z, y, -x))
    node.workspace_yaw_to_robot = Mock(side_effect=lambda yaw, **kw: yaw + 0.2)
    node.convert_place_pose = lambda pose, agent: type(node).convert_place_pose(node, pose, agent)
    feed(monkeypatch, node, [observation(300 + delta[0], 170 + delta[1])] * 3)
    original = node.verify_place_target
    def verify(t):
        assert node.move_to_robot_tf.call_count == 6  # pick trio + observation lift/traverse/arrival
        assert node.move_to_robot_tf.call_args.kwargs['label'] == 'observe'
        assert node._zone_token_held
        assert node.control_gripper.call_count == 2
        return original(t)
    node.verify_place_target = verify
    assert node.execute_task(task)
    assert task['reference_place_pose'] == dict(x=300 + delta[0], y=170 + delta[1], z=60, yaw=0)
    moves = node.move_to_robot_tf.call_args_list[8:]
    assert [c.args[:3] for c in moves] == [
        (220, 170 + delta[1], -300 - delta[0]),
        (60, 170 + delta[1], -300 - delta[0]),
        (220, 170 + delta[1], -300 - delta[0])]
    assert all(c.kwargs['yaw'] == 0.2 for c in moves)
    assert all(c.kwargs['agent_id'] == agent_id for c in node.workspace_point_to_robot.call_args_list)
    assert node.check_sdk_pose_or_raise.call_count == 10
    assert node.check_sdk_pose_or_raise.call_args_list[4].args[2] == -300 - delta[0] - (10 if agent_id == 'agent2' else 0)
    assert node.is_moving and node.place_verification_request is None


@pytest.mark.parametrize('failure', ['missing', 'multiple', 'stale', 'duplicate', 'unstable',
                                      'excessive', 'tf', 'depth', 'safety'])
def test_failure_stops_before_place_release_and_home(agent, monkeypatch, failure):
    node, task = agent
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
    feed(monkeypatch, node, observations)
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
    node.place_vision_config['timeout_sec'] = 0.01
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
        return True
    node.acquire_zone_token = Mock(side_effect=acquire)
    assert node.execute_task(task)
    node.release_zone_token.assert_called_once()


def test_disabled_and_other_materials_bypass_verification(agent):
    node, task = agent
    node.verify_place_target = Mock(side_effect=AssertionError('must not verify'))
    node.place_vision_config['enabled'] = False
    assert node.execute_task(task)
    assert task['reference_place_pose']['y'] == 170
    node.place_vision_config['enabled'] = True
    for material in ('빵', '햄'):
        assert not node.needs_place_verification(dict(assembly=dict(container='basket', material=material)))
    assert not node.needs_place_verification(dict(target='양상추'))


def test_unset_baseline_is_rejected():
    with pytest.raises(ValueError):
        vision.validate_config(vision.DEFAULTS)


@pytest.mark.parametrize('failure', [None, 'depth', 'tf'])
def test_actual_yolo_loop_emits_only_current_stamped_observation(failure):
    case = memory_tests.PerceptionIntegrationTest()
    case.setUp()
    node = case.node
    cfg = config()
    node.run_perception.__globals__['place_verification'] = vision
    node.place_vision_config = cfg
    node.workspace_frame = 'workspace_0'
    node.now_sec = lambda: 100.5
    node.place_verification_request = dict(since_ros=100., since_mono=41., last_stamps=(0., 0.))
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
    model = SimpleNamespace(names={0: 'bread'}, predict=Mock(return_value=case.result([20])))
    node.models = [('existing_model.pt', model)]
    case.drawing.waitKey.side_effect = [ord('q')]
    node.run_perception()
    assert len(node.place_observations) == 1
    obs = node.place_observations[0]
    assert obs['meta'] is meta
    assert obs['visibility']['image_size'] == [100, 200]
    assert obs['visibility']['objects'][0]['material'] == '빵'
    if failure == 'depth':
        assert obs['visibility']['objects'][0]['depth_m'] is None
    if failure:
        assert obs['error']
        assert not obs['detections']
    else:
        assert obs['detections'] == [('빵', (300, 170, 30, 0.), 0.9)]
        assert all(c.kwargs['transform'] is transform for c in node.camera_point_to_workspace.call_args_list)


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


def test_tf_queries_capture_time_and_rejects_invalid_rotation():
    node = load_agent_methods()()
    node.tf_lookup_timeout_sec = 0.1
    transform = SimpleNamespace(translation=SimpleNamespace(x=0., y=0., z=0.),
                                rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    node.tf_buffer = Mock()
    node.tf_buffer.lookup_transform.return_value = SimpleNamespace(
        header=SimpleNamespace(frame_id='workspace_0'), child_frame_id='camera_optical',
        transform=transform)
    args = (metadata(), dict(fx=100., fy=100., cx=1., cy=1.), np.zeros((2, 2, 3)), np.ones((2, 2)))
    assert node.place_camera_transform(*args) is transform
    call = node.tf_buffer.lookup_transform.call_args
    assert call.args[:2] == ('workspace_0', 'camera_optical')
    assert call.args[2].seconds == 100.1
    transform.rotation.w = 0.
    with pytest.raises(ValueError):
        node.place_camera_transform(*args)


def test_observation_uses_basket_and_rotated_camera_mount():
    cfg = config(observation_validated=True,
                 observation_camera_offset_mm=[20., -40., 250.],
                 observation_tcp_to_camera_mm=[10., 20., 30.],
                 observation_transit_clearance_mm=25.)
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    basket = dict(x=300., y=200., z=70.)
    pose = vision.observation_pose(basket, cfg, rotation)
    assert pose == dict(x=340., y=150., z=290.)
    shifted = vision.observation_pose(dict(basket, x=400., z=90.), cfg, rotation)
    assert shifted == dict(pose, x=440., z=310.)


@pytest.mark.parametrize('overrides', [
    dict(observation_validated=False),
    dict(observation_camera_offset_mm=[1., 2.]),
    dict(observation_tcp_to_camera_mm=[0., math.nan, 0.]),
    dict(observation_transit_clearance_mm=0.),
    dict(observation_speed_mm_s=-1.),
])
def test_unmeasured_or_invalid_observation_blocks_before_sdk(agent, overrides):
    node, task = agent
    node.place_vision_config.update(overrides)
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
    original_verify = node.verify_place_target
    def verify(t):
        assert events == ['predecessors', 'token']
        assert node.move_to_robot_tf.call_count == 6
        assert node.place_verification_request is None
        return original_verify(t)
    node.verify_place_target = verify
    assert node.execute_task(task)
    # Invert the fixture's robot <- workspace transform.
    points = [(-c.args[2], c.args[1], c.args[0])
              for c in node.move_to_robot_tf.call_args_list[2:]]
    for before, after in zip(points, points[1:]):
        if not math.isclose(before[2], after[2]):
            assert before[:2] == pytest.approx(after[:2])
    # Observation is camera-derived, separate from nominal or corrected Place XY.
    assert points[3][:2] == (300., 200.)
    assert all(p[:2] == (310., 180.) for p in points[5:])
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
        if calls == {'ingress_end': 4, 'corrected': 6, 'transfer': 10}[stage]:
            raise ValueError('outside safety box')
    node.check_sdk_pose_or_raise.side_effect = safety
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == (3 if stage == 'ingress_end' else 6)
    assert node.control_gripper.call_count == 2
    node.return_to_home_joint_pose.assert_not_called()


@pytest.mark.parametrize('after_move', [4, 6, 7, 10])
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
    obs = observation()
    obs['detections'] = []
    feed(monkeypatch, node, [obs])
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
    for move in node.move_to_robot_tf.call_args_list[3:8]:
        assert move.kwargs['roll_deg'] == 178.
        assert move.kwargs['pitch_deg'] == 1.
        assert move.kwargs['yaw'] == math.radians(2.)


def test_stale_frames_after_transfer_preflight_abort_before_motion(agent, monkeypatch):
    node, task = agent
    feed(monkeypatch, node, [observation()] * 3)
    def safety(*args, **kwargs):
        if node.check_sdk_pose_or_raise.call_count == 10:
            node.now_sec = lambda: 999.
    node.check_sdk_pose_or_raise.side_effect = safety
    assert not node.execute_task(task)
    assert node.move_to_robot_tf.call_count == 6
    assert node.control_gripper.call_count == 2
    node.return_to_home_joint_pose.assert_not_called()
