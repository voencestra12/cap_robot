"""ROS-free stack placement geometry and fail-closed observation checks (mm)."""

import math
import numpy as np

try:
    from .material_handling import number
    from .object_memory import object_class
    from . import object_profiles as profiles
except ImportError:
    from material_handling import number
    from object_memory import object_class
    import object_profiles as profiles


# NaN is an explicitly unset ROS double, never an executable geometry default.
DEFAULTS = {
    'enabled': False,
    'materials': ['양상추'],
    'basket_half_extent_mm': [math.nan, math.nan],
    'target_z_range_mm': [math.nan, math.nan],
    'max_correction_mm': math.nan,
    'stability_mm': 3.0,
    'timeout_sec': 5.0,
    'max_age_sec': 1.0,
    'sync_tolerance_sec': 0.08,
    'min_confidence': 0.6,
    'samples': 3,
    'observation_validated': False,
    'tcp_frame': '',
    'observation_camera_xy_offset_mm': [math.nan, math.nan],
    'observation_camera_z_mm': math.nan,
    'observation_transit_z_mm': math.nan,
    'observation_speed_mm_s': math.nan,
}


def vector(value, size, label):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'{label}: 실측한 {size}개 값이 필요합니다.')
    return [number(v, label) for v in value]


def material_center_offset(material, agent_id):
    value = profiles.OBJECT_PROFILES[material].get('stack_tcp_to_material_center_mm')
    if isinstance(value, dict):
        value = value.get(agent_id)
    return vector(value, 3, f'{material}/{agent_id}.stack_tcp_to_material_center_mm')


def validate_config(config):
    materials = config['materials']
    if not isinstance(materials, (list, tuple)) or not materials or any(
            material not in profiles.OBJECT_PROFILES for material in materials):
        raise ValueError('materials에는 등록된 재료 목록이 필요합니다.')
    if min(vector(config['basket_half_extent_mm'], 2, 'basket_half_extent_mm')) <= 0:
        raise ValueError('바구니 내부 반폭은 실측한 양수여야 합니다.')
    low, high = vector(config['target_z_range_mm'], 2, 'target_z_range_mm')
    if low >= high:
        raise ValueError('이전 층 관측 Z 범위가 올바르지 않습니다.')
    for key in ('max_correction_mm', 'stability_mm', 'timeout_sec', 'max_age_sec',
                'sync_tolerance_sec'):
        if number(config[key], key) <= 0:
            raise ValueError(f'{key}: 양수가 필요합니다.')
    number(config['min_confidence'], 'min_confidence', 0.3, 1.0)
    if type(config['samples']) is not int or not 2 <= config['samples'] <= 8:
        raise ValueError('samples는 2~8 정수여야 합니다.')


def validate_observation_config(config):
    if config.get('observation_validated') is not True:
        raise ValueError('관측 자세/높이/경로 실측 검증 필요: observation_validated=false')
    if not isinstance(config['tcp_frame'], str) or not config['tcp_frame'].strip():
        raise ValueError('실제 활성 TCP에 대응하는 TF tcp_frame이 필요합니다.')
    vector(config['observation_camera_xy_offset_mm'], 2, 'observation_camera_xy_offset_mm')
    for key in ('observation_camera_z_mm', 'observation_transit_z_mm'):
        number(config[key], key)
    if number(config['observation_speed_mm_s'], 'observation_speed_mm_s') <= 0:
        raise ValueError('관측 속도는 실측 검증한 양수여야 합니다.')


def get_previous_layer_target(task, tasks, statuses):
    assembly = task['assembly']
    index = assembly['layer_index']
    previous = [t for t in tasks if t.get('assembly', {}).get('id') == assembly['id']
                and t['assembly']['layer_index'] == index - 1]
    if index < 1 or len(previous) != 1:
        raise ValueError('동일 assembly의 직전 층이 정확히 하나여야 합니다.')
    target = previous[0]
    if statuses.get(target['task_id']) != 'SUCCEEDED':
        raise ValueError('직전 층 Task의 SUCCEEDED가 확인되지 않았습니다.')
    # A shared Z gate cannot prove which repeated material layer is visible.
    if any(t.get('assembly', {}).get('id') == assembly['id']
           and t['assembly']['layer_index'] < index - 1
           and t['assembly']['material'] == target['assembly']['material'] for t in tasks):
        raise ValueError('동종 이전 층들의 높이 구분을 검증할 수 없습니다.')
    return dict(target['assembly'], task_id=target['task_id'])


def observation_pose(basket, config, workspace_from_tcp, tcp_to_camera_mm):
    validate_observation_config(config)
    dx, dy = config['observation_camera_xy_offset_mm']
    camera = np.array([number(basket['x'], 'basket.x') + dx,
                       number(basket['y'], 'basket.y') + dy,
                       config['observation_camera_z_mm']])
    tcp = camera - workspace_from_tcp @ np.array(vector(tcp_to_camera_mm, 3, 'TCP camera TF'))
    return dict(zip(('x', 'y', 'z'), (number(v, 'observation TCP') for v in tcp)))


def transit_waypoints(start, end, height, minimum_z):
    """Only travel horizontally at a measured, validated workspace transit height."""
    height = number(height, 'observation_transit_z_mm')
    if height < max(start['z'], end['z'], minimum_z):
        raise ValueError('검증된 관측 이동 높이가 경유점보다 낮습니다.')
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


def previous_layer_center(observation, basket, target_material, config):
    if observation.get('frame_id') != 'workspace_0' or observation.get('error'):
        raise ValueError(f"이전 층 관측 실패: {observation.get('error', '좌표계 오류')}")
    candidates = []
    hx, hy = config['basket_half_extent_mm']
    zmin, zmax = config['target_z_range_mm']
    for name, pose, confidence in observation['detections']:
        if object_class(name) != target_material:
            continue
        x, y, z = (number(v, 'previous layer xyz') for v in pose[:3])
        if abs(x - basket['x']) <= hx and abs(y - basket['y']) <= hy and zmin <= z <= zmax:
            candidates.append((x, y, z, confidence))
    if len(candidates) != 1:
        raise ValueError(f'바구니 내부 이전 층 후보가 정확히 하나여야 합니다: {len(candidates)}')
    x, y, z, confidence = candidates[0]
    if number(confidence, 'confidence', 0, 1) < config['min_confidence']:
        raise ValueError('이전 층 검출 신뢰도가 부족합니다.')
    return x, y, z


def calculate_stack_tcp_pose(detected_center, material_center_offset, release_rotation,
                             nominal_place_pose):
    """Use observed XY directly; retain nominal Z/yaw and all release attitude settings."""
    center = vector(detected_center, 3, 'detected center')
    offset = np.array(vector(material_center_offset, 3, 'material center offset'))
    rotation = np.asarray(release_rotation, dtype=float)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not math.isclose(np.linalg.det(rotation), 1.0, abs_tol=1e-6)):
        raise ValueError('배치 자세 회전행렬이 유효하지 않습니다.')
    rotated = rotation @ offset
    return dict(nominal_place_pose, x=center[0] - rotated[0], y=center[1] - rotated[1])


def corrected_place(nominal, centers, offset, rotation, config):
    if len(centers) < config['samples']:
        raise ValueError('서로 다른 새 프레임 관측이 부족합니다.')
    if any(math.dist(a, b) > config['stability_mm'] for a in centers for b in centers):
        raise ValueError('이전 층 중심 검출이 불안정합니다.')
    targets = [calculate_stack_tcp_pose(list(p), offset, rotation, nominal) for p in centers]
    if any(math.hypot(p['x'] - nominal['x'], p['y'] - nominal['y']) >
           config['max_correction_mm'] for p in targets):
        raise ValueError('적층 TCP 보정량이 허용 범위를 초과합니다.')
    return dict(nominal, **{a: sum(p[a] for p in targets) / len(targets) for a in ('x', 'y')})
