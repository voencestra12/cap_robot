"""ROS-free, fail-closed lettuce placement verification (workspace_0, mm)."""

import math

try:
    from .material_handling import number
    from .object_memory import object_class
except ImportError:
    from material_handling import number
    from object_memory import object_class


DEFAULTS = {
    'enabled': False,
    # No inferred relationship between the grasp TCP and the YOLO centroid.
    'baseline_calibrated': False,
    'expected_bread_xy_offset_mm': [0.0, 0.0],
    'basket_half_extent_mm': [60.0, 60.0],
    'bread_z_range_mm': [5.0, 80.0],
    'max_correction_mm': 20.0,
    'stability_mm': 3.0,
    'timeout_sec': 5.0,
    'max_age_sec': 1.0,
    'sync_tolerance_sec': 0.08,
    'min_confidence': 0.6,
    'samples': 3,
    # Unmeasured placeholders: enabling vision must not authorize a new motion.
    'observation_validated': False,
    'observation_camera_offset_mm': [0.0, 0.0, 0.0],
    'observation_tcp_to_camera_mm': [0.0, 0.0, 0.0],
    'observation_transit_clearance_mm': 0.0,
    'observation_speed_mm_s': 30.0,
}


def validate_config(config):
    if not config['baseline_calibrated']:
        raise ValueError('정상 빵 YOLO 중심의 바구니 대비 실측 기준값이 필요합니다.')
    for key in ('expected_bread_xy_offset_mm', 'basket_half_extent_mm', 'bread_z_range_mm'):
        values = config[key]
        if len(values) != 2:
            raise ValueError(f'{key}: 두 값이 필요합니다.')
        for value in values:
            number(value, key)
    if min(config['basket_half_extent_mm']) <= 0:
        raise ValueError('바구니 내부 반폭은 양수여야 합니다.')
    if config['bread_z_range_mm'][0] >= config['bread_z_range_mm'][1]:
        raise ValueError('빵 Z 범위가 올바르지 않습니다.')
    if any(abs(v) > h for v, h in zip(
            config['expected_bread_xy_offset_mm'], config['basket_half_extent_mm'])):
        raise ValueError('정상 빵 중심은 바구니 내부여야 합니다.')
    for key in ('max_correction_mm', 'stability_mm', 'timeout_sec', 'max_age_sec',
                'sync_tolerance_sec'):
        if number(config[key], key) <= 0:
            raise ValueError(f'{key}: 양수가 필요합니다.')
    number(config['min_confidence'], 'min_confidence', 0.3, 1.0)
    if type(config['samples']) is not int or not 2 <= config['samples'] <= 8:
        raise ValueError('samples는 2~8 정수여야 합니다.')


def validate_observation_config(config):
    if config.get('observation_validated') is not True:
        raise ValueError('관측 XYZ/카메라 자세/이동 경로의 실측 검증이 필요합니다: observation_validated=false')
    for key in ('observation_camera_offset_mm', 'observation_tcp_to_camera_mm'):
        if len(config[key]) != 3:
            raise ValueError(f'{key}: XYZ 세 값이 필요합니다.')
        for value in config[key]:
            number(value, key)
    if number(config['observation_transit_clearance_mm'], 'transit clearance') <= 0:
        raise ValueError('관측 이동 여유 높이는 실측한 양수여야 합니다.')
    if number(config['observation_speed_mm_s'], 'observation speed') <= 0:
        raise ValueError('관측 속도는 양수여야 합니다.')


def observation_pose(basket, config, workspace_from_tcp):
    """Nominal TF TCP = desired camera origin - rotated TCP-to-optical translation."""
    validate_observation_config(config)
    camera = [number(basket[a], f'basket.{a}') + d for a, d in
              zip(('x', 'y', 'z'), config['observation_camera_offset_mm'])]
    mount = config['observation_tcp_to_camera_mm']
    tcp = [camera[i] - sum(workspace_from_tcp[i][j] * mount[j] for j in range(3))
           for i in range(3)]
    return dict(zip(('x', 'y', 'z'), (number(v, 'observation TCP') for v in tcp)))


def transit_waypoints(start, end, clearance, minimum_z):
    """Workspace vertical lift, horizontal traverse, vertical arrival; no diagonal descent."""
    height = max(start['z'], end['z'], minimum_z) + clearance
    return [dict(start, z=height), dict(end, z=height), dict(end)]


def validate_frames(meta, request, config, now_ros, now_mono):
    """Both acquisition stamps AND callback receipts must follow the stop/settle barrier."""
    for kind in ('color', 'depth'):
        stamp = number(meta[f'{kind}_stamp'], f'{kind} stamp', 0)
        received = number(meta[f'{kind}_received'], f'{kind} received', 0)
        if (stamp <= request['since_ros'] or received <= request['since_mono']
                or not 0 <= now_ros - stamp <= config['max_age_sec']
                or not 0 <= now_mono - received <= config['max_age_sec']):
            raise ValueError('정지 이후의 최신 RGB/Depth 프레임이 아닙니다.')
    if abs(meta['color_stamp'] - meta['depth_stamp']) > config['sync_tolerance_sec']:
        raise ValueError('RGB/Depth 시각 차이가 너무 큽니다.')
    if not meta['camera_frame'] or meta['color_frame'] != meta['camera_frame']:
        raise ValueError('RGB와 CameraInfo 좌표계가 다릅니다.')
    if meta['depth_frame'] != meta['camera_frame']:
        raise ValueError('Depth는 color optical frame에 정렬되어야 합니다.')


def bread_center(observation, basket, config):
    if observation.get('frame_id') != 'workspace_0' or observation.get('error'):
        raise ValueError(f"빵 관측 실패: {observation.get('error', '좌표계 오류')}")
    candidates = []
    hx, hy = config['basket_half_extent_mm']
    zmin, zmax = config['bread_z_range_mm']
    for name, pose, confidence in observation['detections']:
        if object_class(name) != '빵':
            continue
        x, y, z = (number(v, 'bread xyz') for v in pose[:3])
        if abs(x - basket['x']) <= hx and abs(y - basket['y']) <= hy and zmin <= z <= zmax:
            candidates.append((x, y, z, confidence))
    if len(candidates) != 1:
        raise ValueError(f'바구니 내부 빵 후보가 정확히 하나여야 합니다: {len(candidates)}')
    x, y, z, confidence = candidates[0]
    if number(confidence, 'confidence', 0, 1) < config['min_confidence']:
        raise ValueError('빵 검출 신뢰도가 부족합니다.')
    return x, y, z


def corrected_place(nominal, basket, centers, config):
    if len(centers) < config['samples']:
        raise ValueError('서로 다른 새 프레임 관측이 부족합니다.')
    if any(math.dist(a, b) > config['stability_mm'] for a in centers for b in centers):
        raise ValueError('빵 중심 검출이 불안정합니다.')
    expected = [basket[axis] + offset for axis, offset in
                zip(('x', 'y'), config['expected_bread_xy_offset_mm'])]
    deltas = [(p[0] - expected[0], p[1] - expected[1]) for p in centers]
    if any(math.hypot(*delta) > config['max_correction_mm'] for delta in deltas):
        raise ValueError('빵 위치 보정량이 허용 범위를 초과합니다.')
    dx, dy = (sum(d[i] for d in deltas) / len(deltas) for i in (0, 1))
    # nominal already contains the lettuce TCP XY offset. Never apply it again.
    return dict(nominal, x=nominal['x'] + dx, y=nominal['y'] + dy)
