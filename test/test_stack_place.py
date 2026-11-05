"""No ROS node, SDK connection, live camera, or robot motion is created."""
import copy
import math
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
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
def agent():
    case = handling.AgentAssemblyTest()
    case.setUp()
    node = case.node
    node.stack_vision_config = config(
        enabled=True, observation_validated=True)
    node.zone_token_enabled = node.enable_sdk_safety_box = True
    node._shared_zone_box = lambda: dict(x_min=150., x_max=450., y_min=-50., y_max=500.)
    node.lookup_workspace_transform = Mock(return_value=SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0., z=0.),
        rotation=SimpleNamespace(x=0., y=0.70710678, z=0., w=0.70710678))))
    node.arm.version_number = (2, 4, 0)
    node.arm.get_state.return_value = (0, 2)
    node.arm.get_is_moving.return_value = False
    node.arm.error_code = 0
    node.arm.set_position.return_value = 0
    node.arm.only_check_result = 0
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
        grip_position=100., place_xy_offset_mm=[0., -30.], stack_tcp_to_material_center_mm=[0., 0., 0.])
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


@pytest.mark.parametrize('agent_id', ['agent1'])
@pytest.mark.parametrize('delta', [(0, 30), (10, 35)])
def test_execution_corrects_once_after_lift_and_uses_existing_transform(agent, monkeypatch, agent_id, delta):
    node, task = agent
    node.agent_id = task['assignee_id'] = agent_id
    node.stack_vision_config['max_correction_mm'] = 50.
    # Exercise the real convert_place_pose/workspace_place_to_robot_place methods.
    node.workspace_point_to_robot = Mock(side_effect=lambda x, y, z, **kw: (z, y, -x))
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
    moves = node.move_to_robot_tf.call_args_list[7:]
    assert [c.args[:3] for c in moves] == [
        (220, 170 + delta[1], -300 - delta[0]),
        (60, 170 + delta[1], -300 - delta[0]),
        (220, 170 + delta[1], -300 - delta[0])]
    assert all(c.kwargs['yaw'] == 0.2 for c in moves)
    assert all(c.kwargs['agent_id'] == agent_id for c in node.workspace_point_to_robot.call_args_list)
    assert node.check_sdk_pose_or_raise.call_count == 9
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
    if failure == 'depth':
        assert obs['visibility']['objects'][0]['depth_m'] is None
    if failure:
        assert obs['error']
        assert not obs['detections']
    else:
        assert obs['detections'] == [(material, (300, 170, 30, 0.), 0.9)]
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
    # SDK xyz corresponding to workspace (10, 20, 180), under the fixture TF.
    start = dict(x=180., y=20., z=-10., yaw=0.)
    view, _, path = node.plan_stack_observation(task, start, None, dict(z_offset=200., speed=100.))
    assert view == dict(x=300., y=200., z=220., yaw=0.)
    task['basket_center_workspace']['z'] = 900.
    assert node.plan_stack_observation(task, start, None, dict(z_offset=200., speed=100.))[0] == view
    assert path == [dict(x=10., y=20., z=180., yaw=0.),
                    dict(x=10., y=20., z=220., yaw=0.), view, view]


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
    # Invert the fixture's robot <- workspace transform.
    points = [(-c.args[2], c.args[1], c.args[0])
              for c in node.move_to_robot_tf.call_args_list[2:]]
    for before, after in zip(points, points[1:]):
        if not math.isclose(before[2], after[2]):
            assert before[:2] == pytest.approx(after[:2])
    # Observe at the basket mean; correct horizontally at the same high Z.
    assert points[3] == (300., 200., 220.)
    assert points[4] == (310., 180., 220.)
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
    assert [p['only_check_type'] for p in checked] == [1, 3, 3, 2, 1, 3, 3, 3, 2]
    assert all(p['wait'] is False and p['is_radian'] is False for p in checked)
    # Controller planning visits view -> corrected high -> approach -> release -> retreat.
    assert [(p['x'], p['y'], p['z']) for p in checked[4:]] == [
        (220., 200., -300.), (220., 180., -310.), (220., 180., -310.),
        (60., 180., -310.), (220., 180., -310.)]
    node.arm.set_only_check_type.assert_not_called()  # no global SDK mode changes


@pytest.mark.parametrize('check_index', [1, 3, 4, 5, 6, 8, 9])
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
    assert node.move_to_robot_tf.call_count == (3 if check_index <= 4 else 6)
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
        if stage == 'observation' and node.arm.get_state.call_count == 1:
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
    plan = [c.kwargs for c in node.arm.set_position.call_args_list][6:]
    actual = node.move_to_robot_tf.call_args_list[7:]
    assert len(plan) == len(actual)
    for checked, move in zip(plan, actual):
        assert (checked['x'], checked['y'], checked['z']) == pytest.approx(move.args[:3])
        assert checked['yaw'] == pytest.approx(math.degrees(move.kwargs['yaw']))
        assert checked['roll'] == pytest.approx(move.kwargs.get('roll_deg', 180.))
        assert checked['pitch'] == pytest.approx(move.kwargs.get('pitch_deg', 0.))
        assert checked['speed'] == move.kwargs['speed']


def test_config_cleanup_keeps_feature_disabled_and_has_no_manual_camera_geometry():
    from pathlib import Path
    import yaml
    removed = {'observation_camera_xy_offset_mm', 'observation_camera_z_mm',
               'observation_transit_z_mm', 'observation_speed_mm_s'}
    assert not removed.intersection(vision.DEFAULTS)
    for agent_id in ('agent1', 'agent2'):
        path = Path(__file__).resolve().parents[1] / 'config' / f'{agent_id}.yaml'
        config_values = yaml.safe_load(path.read_text())[f'/{agent_id}/robot_agent_node']['ros__parameters']
        assert config_values['stack_place_vision.enabled'] is False
        assert config_values['stack_place_vision.observation_validated'] is False
        assert not {f'stack_place_vision.{key}' for key in removed}.intersection(config_values)
