"""ROS-free stack placement geometry and fail-closed observation checks (mm)."""

import copy
import json
import math
from pathlib import Path
import uuid
import numpy as np

try:
    from .material_handling import material_profile, number, thickness
    from .object_memory import object_class
    from . import object_profiles as profiles
except ImportError:
    from material_handling import material_profile, number, thickness
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
}

# Fixed limits, intentionally not ROS parameters.
# In-place look rotation: maximum angle between link6 (= TCP) +X and Base +Z while
# holding material. +X is the hand-eye camera side, so the camera stays above the
# gripper and the held material does not cover the view.
MAX_LOOK_X_UP_DEG = 30.0
# The look rotation pivots about the camera center, so the TCP moves; it may not
# drop more than this below the Pick lift (2026-11-20 layout: ~47 mm).
MAX_LOOK_TCP_DROP_MM = 100.0


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


def validate_observation_config(config, agent_id):
    if config.get('observation_validated') is not True:
        raise ValueError('관측 자세/높이/경로 실측 검증 필요: observation_validated=false')
    if config['tcp_frame'] != f'{agent_id}_tcp':
        raise ValueError(f'{agent_id} 양상추 관측에는 {agent_id}_tcp TF가 필요합니다.')


def previous_layer_z_range(task, tasks, config):
    """target_z_range_mm is the first layer's top; raise it by the layers stacked since."""
    assembly = task['assembly']
    rise = sum(thickness(t['assembly']['material']) for t in tasks
               if t.get('assembly', {}).get('id') == assembly['id']
               and 1 <= t['assembly']['layer_index'] < assembly['layer_index'])
    low, high = vector(config['target_z_range_mm'], 2, 'target_z_range_mm')
    return [low + rise, high + rise]


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


def _axis_rotation(axis, angle):
    """Rodrigues rotation matrix about a unit axis."""
    x, y, z = axis
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def _align_rotation(current, wanted):
    """Smallest rotation taking unit vector current onto unit vector wanted."""
    axis = np.cross(current, wanted)
    sin, cos = np.linalg.norm(axis), float(np.dot(current, wanted))
    if sin < 1e-12:
        if cos > 0:
            return np.eye(3)
        axis = np.cross(current, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(current, [0.0, 1.0, 0.0])
        return _axis_rotation(axis / np.linalg.norm(axis), math.pi)
    return _axis_rotation(axis / sin, math.atan2(sin, cos))


def _x_up_attitude(rotation, optical, direction):
    """Point the optical axis along direction, then spin about it so TCP +X is as
    close to Base +Z as the direction allows. Looking straight up/down keeps the spin."""
    rotation = _align_rotation(rotation @ optical, direction) @ rotation
    up = np.array([0.0, 0.0, 1.0])
    x_perp = rotation[:, 0] - float(np.dot(rotation[:, 0], direction)) * direction
    up_perp = up - float(np.dot(up, direction)) * direction
    if np.linalg.norm(x_perp) < 1e-9 or np.linalg.norm(up_perp) < 1e-9:
        return rotation
    angle = math.atan2(float(np.dot(direction, np.cross(x_perp, up_perp))),
                       float(np.dot(x_perp, up_perp)))
    return _axis_rotation(direction, angle) @ rotation


def _x_up_deg(rotation):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rotation[2, 0])))))


def look_at_attitude(tcp_rotation, tcp_position, camera_rotation, camera_translation,
                     target, max_x_up_deg=MAX_LOOK_X_UP_DEG):
    """Rotate about the camera center so the optical (+Z) axis passes through target
    with TCP (= link6) +X pointing up, i.e. the camera above the gripper.

    Robot Base frame, mm. camera_rotation/translation is TCP <- camera optical frame
    (hand-eye TF, no manual offset). The camera center stays where it is after the
    Pick lift and the TCP moves around it; pivoting about the TCP instead raised the
    camera ~135 mm, which needed a ~49 deg look and left the basket at the image edge
    under the 30 deg limit (2026-11-20). The spin about the optical axis is chosen so
    +X is closest to Base +Z. If +X would be more than max_x_up_deg from Base +Z,
    the optical axis is raised toward horizontal with the same azimuth until it is
    exactly at the limit; the target then lies below the image center.
    Returns (rotation, tcp_position, x_up_deg, clamped).
    """
    rotation = np.array(tcp_rotation, dtype=float)
    offset = np.array(camera_translation, dtype=float)
    optical = np.array(camera_rotation, dtype=float)[:, 2]
    camera = np.array(tcp_position, dtype=float) + rotation @ offset
    wanted = np.array(target, dtype=float) - camera
    if np.linalg.norm(wanted) < 1e-6:
        raise ValueError('관측 대상이 카메라 위치와 겹칩니다.')
    wanted /= np.linalg.norm(wanted)
    look = _x_up_attitude(rotation, optical, wanted)
    x_up = _x_up_deg(look)
    clamped = x_up > max_x_up_deg
    if clamped:
        horizontal = np.array([wanted[0], wanted[1], 0.0])
        if np.linalg.norm(horizontal) < 1e-6:
            raise ValueError('관측 방위를 정할 수 없습니다: 대상이 카메라 바로 아래입니다.')
        horizontal /= np.linalg.norm(horizontal)
        elevation = math.asin(max(-1.0, min(1.0, float(wanted[2]))))

        def raised(fraction):
            e = elevation * (1.0 - fraction)
            u = math.cos(e) * horizontal + np.array([0.0, 0.0, math.sin(e)])
            return _x_up_attitude(rotation, optical, u)

        if _x_up_deg(raised(1.0)) > max_x_up_deg:
            raise ValueError(f'link6 +X를 Base +Z {max_x_up_deg:.0f}deg 이내로 두는 관측 자세가 없습니다.')
        low, high = 0.0, 1.0
        for _ in range(60):
            middle = 0.5 * (low + high)
            if _x_up_deg(raised(middle)) > max_x_up_deg:
                low = middle
            else:
                high = middle
        look = raised(high)
        x_up = _x_up_deg(look)
    return look, camera - look @ offset, x_up, clamped


def look_speed(material, agent_id, action_speed):
    """Rotation speed while holding material: the material's existing tilt speed cap."""
    return min(number(action_speed, 'action speed', 0),
               material_profile(material, agent_id)['place_tilt_speed'])


def compose_transforms(parent, child):
    """Compose workspace<-TCP and TCP<-camera at the same acquisition time."""
    def values(transform):
        t, q = transform.translation, transform.rotation
        xyz = vector([t.x, t.y, t.z], 3, 'TF translation')
        quat = vector([q.x, q.y, q.z, q.w], 4, 'TF quaternion')
        norm = sum(v * v for v in quat)
        if abs(norm - 1.0) > 0.01:
            raise ValueError('TF 회전이 유효하지 않습니다.')
        return np.array(xyz), np.array(quat) / math.sqrt(norm)

    pt, pq = values(parent)
    ct, cq = values(child)
    # Quaternion rotation of the camera translation, followed by composition.
    xyz = pt + ct + 2 * np.cross(pq[:3], np.cross(pq[:3], ct) + pq[3] * ct)
    qxyz = pq[3] * cq[:3] + cq[3] * pq[:3] + np.cross(pq[:3], cq[:3])
    qw = pq[3] * cq[3] - np.dot(pq[:3], cq[:3])
    result = copy.deepcopy(parent)
    result.translation.x, result.translation.y, result.translation.z = map(float, xyz)
    result.rotation.x, result.rotation.y, result.rotation.z = map(float, qxyz)
    result.rotation.w = float(qw)
    return result


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


class NoPreviousLayerCandidate(ValueError):
    """A valid frame with zero eligible candidates may be observed again."""


def save_observation_debug(task_id, reason, observation, directory=None):
    """Persist a matching RGB/detection pair without changing the failure policy."""
    import cv2

    folder = Path(directory) if directory is not None else Path.cwd() / 'log' / 'stack_place_vision'
    folder.mkdir(parents=True, exist_ok=True)
    # Do not use external task identifiers as filenames.
    stem = folder / uuid.uuid4().hex
    payload = {k: v for k, v in observation.items() if k not in ('rgb', 'annotated_rgb')}
    payload.update(task_id=task_id, failure=str(reason))
    for key in ('rgb', 'annotated_rgb'):
        if observation.get(key) is not None:
            path = stem.with_name(f'{stem.name}_{key}.png')
            if not cv2.imwrite(str(path), observation[key]):
                raise OSError(f'관측 디버그 영상 저장 실패: {path}')
            payload[f'{key}_file'] = path.name
    path = stem.with_suffix('.json')
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
    return path


def previous_layer_center(observation, basket, target_material, config, diagnostics=None,
                          base_z=None):
    """One bread candidate inside the basket XY and the target_z_range_mm gate.

    base_z(x, y, z) maps a workspace point to robot Base Z. The Z gate uses Base Z:
    the mount camera -> TCP -> Base chain does not depend on the ArUco workspace lock,
    which moved the same bread from -58 to 37 to 15 mm in workspace Z across locks
    while Base Z stayed 67-70 mm (2026-11-06). Without base_z the workspace Z is used.
    """
    if observation.get('frame_id') != 'workspace_0' or observation.get('error'):
        raise ValueError(f"이전 층 관측 실패: {observation.get('error', '좌표계 오류')}")
    candidates = []
    hx, hy = config['basket_half_extent_mm']
    zmin, zmax = config['target_z_range_mm']
    for name, pose, confidence in observation['detections']:
        if object_class(name) != target_material:
            continue
        x, y, z = (number(v, 'previous layer xyz') for v in pose[:3])
        in_xy = abs(x - basket['x']) <= hx and abs(y - basket['y']) <= hy
        height = number(base_z(x, y, z), 'previous layer base z') if base_z else z
        in_z = zmin <= height <= zmax
        confidence = number(confidence, 'confidence', 0, 1)
        if diagnostics is not None:
            diagnostics.append(dict(
                name=name, workspace_xyz_mm=[x, y, z], confidence=confidence,
                basket_xy=dict(passed=in_xy, center=[basket['x'], basket['y']], half_extent=[hx, hy]),
                z_range=dict(passed=in_z, value=height, frame='robot_base' if base_z else 'workspace_0',
                             limits=[zmin, zmax]),
                confidence_check=dict(passed=confidence >= config['min_confidence'],
                                      minimum=config['min_confidence'])))
        if in_xy and in_z:
            candidates.append((x, y, z, confidence))
    if not candidates:
        raise NoPreviousLayerCandidate('바구니 내부 이전 층 후보가 없습니다: 0')
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


def stack_release_z(nominal_z, centers, material, agent_id):
    """Release TCP Z from the measured previous-layer top, never below the nominal Z.

    The nominal Z assumes floor + layer thicknesses; the observed top and the release
    point share the same XY, so workspace calibration offset/tilt cancels. The rise is
    bounded by the target_z_range_mm gate the centers already passed.
    (2026-11-06: nominal top 25 mm vs measured 37 mm -> 18 mm clearance -> C31 contact.)
    """
    measured_top = float(np.mean([c[2] for c in centers]))
    offset = material_profile(material, agent_id, for_assembly=True)['place_tcp_offset_mm']
    return max(number(nominal_z, 'nominal release z'), measured_top + offset)


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
