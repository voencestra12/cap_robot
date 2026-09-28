"""ROS/로봇 연결 없이 검증 가능한 재료 파지 및 빨간 점 기반 바구니 적층 계산."""

import math

try:
    from . import object_profiles as settings
except ImportError:
    import object_profiles as settings


def number(value, label, minimum=None, maximum=None):
    if value is None or isinstance(value, bool):
        raise ValueError(f'{label}: object_profiles.py에 실험값을 입력하세요.')
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{label}: 숫자가 필요합니다.') from error
    if not math.isfinite(result):
        raise ValueError(f'{label}: 유한한 숫자가 필요합니다.')
    if minimum is not None and result < minimum:
        raise ValueError(f'{label}: {minimum} 이상이어야 합니다.')
    if maximum is not None and result > maximum:
        raise ValueError(f'{label}: {maximum} 이하여야 합니다.')
    return result


def material_profile(material, agent_id=None, for_assembly=False):
    if material not in settings.OBJECT_PROFILES:
        raise ValueError(f'등록되지 않은 재료: {material}')
    common = settings.OBJECT_PROFILES[material]
    profile = dict(common)
    overrides = settings.AGENT_PROFILE_OVERRIDES.get(agent_id, {}).get(material, {})
    if set(overrides) - {
        'grip_position', 'place_tcp_offset_mm', 'place_roll_offset_deg',
        'place_pitch_offset_deg', 'place_tilt_speed',
    }:
        raise ValueError('Agent별 설정은 파지/TCP 보정/배치 기울임 항목만 지원합니다.')
    profile.update(overrides)
    profile['grip_position'] = number(
        profile.get('grip_position'), f'{material}.grip_position', 0, 850
    )
    for key in ('place_roll_offset_deg', 'place_pitch_offset_deg'):
        profile[key] = number(profile.get(key, 0.0), f'{material}.{key}', -90, 90)
    profile['place_tilt_speed'] = number(
        profile.get('place_tilt_speed', 20.0), f'{material}.place_tilt_speed', 1, 150
    )
    if for_assembly:
        profile['thickness_mm'] = thickness(material)
        profile['place_tcp_offset_mm'] = number(
            profile.get('place_tcp_offset_mm'), f'{material}.place_tcp_offset_mm'
        )
    return profile


def thickness(material):
    value = number(
        settings.OBJECT_PROFILES[material].get('thickness_mm'),
        f'{material}.thickness_mm', 0,
    )
    return value


def bind_gripper_actions(actions, target, agent_id=None):
    """검증기가 붙인 mode를 이용하여 숫자형 구버전 명령도 재료 설정으로 치환."""
    profile = (
        material_profile(target, agent_id)
        if target in settings.OBJECT_PROFILES else None
    )
    result = []
    for source in actions:
        action = dict(source)
        if action['api'] == 'control_gripper':
            mode = action.get('mode')
            if profile is not None and mode == 'grasp':
                action['position'] = profile['grip_position']
            elif mode in ('open', 'release') and (
                profile is not None or 'position' not in action
            ):
                action['position'] = number(
                    settings.GRIPPER_OPEN_POSITION, 'GRIPPER_OPEN_POSITION', 700, 850
                )
            if 'position' not in action:
                raise ValueError(f'{target}: 재료 설정이 없으므로 position이 필요합니다.')
        result.append(action)
    return result


def validate_assembly_tasks(tasks):
    """LLM은 순서만 지정하며 두께/좌표는 설정 파일에서 결정한다."""
    groups = {}
    for task in tasks:
        if 'assembly' not in task:
            continue
        assembly = task['assembly']
        if not isinstance(assembly, dict):
            raise ValueError('assembly는 JSON 객체여야 합니다.')
        if task.get('execution_mode', 'single_agent') != 'single_agent':
            raise ValueError('재료 적층은 single_agent Task여야 합니다.')
        assembly_id = assembly.get('id')
        if not isinstance(assembly_id, str) or not assembly_id.strip():
            raise ValueError('assembly.id가 필요합니다.')
        if assembly.get('container') != 'basket':
            raise ValueError('현재 적층은 basket만 지원합니다.')
        if assembly.get('material') not in settings.OBJECT_PROFILES:
            raise ValueError('assembly.material은 등록된 재료여야 합니다.')
        index = assembly.get('layer_index')
        if type(index) is not int or index < 0:
            raise ValueError('assembly.layer_index는 0 이상의 정수여야 합니다.')
        groups.setdefault(assembly_id, []).append(task)
    if len(groups) > 1:
        raise ValueError('현재 공통 바구니 조립점에는 미션당 하나의 assembly만 지원합니다.')
    for layers in groups.values():
        layers.sort(key=lambda task: task['assembly']['layer_index'])
        if [t['assembly']['layer_index'] for t in layers] != list(range(len(layers))):
            raise ValueError('적층 순서는 중복/누락 없이 0부터 연속이어야 합니다.')
        for previous, current in zip(layers, layers[1:]):
            if previous['task_id'] not in current.get('depends_on', []):
                raise ValueError('각 적층 Task는 직전 층 Task에 의존해야 합니다.')
    return groups


def assembly_placement(plan_task, tasks, agent_id=None, *, basket_center):
    """workspace 기준 해제 TCP pose. 40 mm 기본 오프셋을 포함하지 않는다."""
    groups = validate_assembly_tasks(tasks)
    assembly = plan_task['assembly']
    layers = groups[assembly['id']]
    material = assembly['material']
    profile = material_profile(material, agent_id, for_assembly=True)
    # 시작 전에 전체 레시피의 두께를 확인하여 중간에 미설정 값으로 막히지 않게 한다.
    heights = [thickness(t['assembly']['material']) for t in layers]
    offset = sum(heights[:assembly['layer_index']])
    floor_z = number(settings.BASKET_FLOOR_Z_MM, 'BASKET_FLOOR_Z_MM')
    pose = {
        'x': number(basket_center.get('x'), 'basket_center.x'),
        'y': number(basket_center.get('y'), 'basket_center.y'),
        'z': floor_z + offset + profile['place_tcp_offset_mm'],
        'yaw': math.radians(number(settings.BASKET_YAW_DEG, 'BASKET_YAW_DEG')),
    }
    return {
        'reference_place_pose': pose,
        'basket_center_workspace': dict(basket_center),
        'stack_offset_mm': offset,
        'assembly': dict(assembly),
    }


def assembly_move_pose(release_pose, z_offset, pick_place_z_offset_mm):
    """기존 z_offset=40은 해제 위치, 200은 그보다 160 mm 위로 해석."""
    clearance = number(z_offset, 'z_offset') - pick_place_z_offset_mm
    if clearance < 0:
        raise ValueError('적층 배치 경로가 해제 위치 아래로 내려갈 수 없습니다.')
    return dict(release_pose, z=release_pose['z'] + clearance)


def basket_center_from_perception(state, workspace_frame, max_age_sec, now_sec):
    """새로운 두 빨간 점의 3D 중심(mm). 이 중심의 Z는 바구니 바닥 Z가 아니다."""
    if not isinstance(state, dict) or state.get('valid') is not True:
        reason = state.get('reason', '') if isinstance(state, dict) else ''
        raise ValueError(f'바구니 빨간 점 인식이 유효하지 않습니다: {reason}')
    if state.get('frame_id') != workspace_frame:
        raise ValueError('빨간 점 좌표계가 workspace_frame과 다릅니다.')
    if state.get('source') != 'red_handle_detector':
        raise ValueError('바구니 중심에는 red_handle_detector 결과가 필요합니다.')
    age = number(state.get('received_age_sec'), '빨간 점 수신 age', 0)
    stamp = number(state.get('stamp_sec'), '빨간 점 stamp_sec')
    source_age = now_sec - stamp
    if age > max_age_sec or source_age > max_age_sec or source_age < -0.1:
        raise ValueError('바구니 빨간 점 좌표가 오래되었거나 시각이 올바르지 않습니다.')
    objects = state.get('objects', {})
    handles = []
    for name in ('basket_handle_0', 'basket_handle_1'):
        point = objects.get(name) if isinstance(objects, dict) else None
        if not isinstance(point, dict):
            raise ValueError(f'바구니 빨간 점이 없습니다: {name}')
        handles.append({
            axis: number(point.get(f'{axis}_mm'), f'{name}.{axis}_mm')
            for axis in ('x', 'y', 'z')
        })
    if math.dist(tuple(handles[0].values()), tuple(handles[1].values())) < 1e-6:
        raise ValueError('바구니 빨간 점 두 개가 같은 좌표입니다.')
    center = {axis: (handles[0][axis] + handles[1][axis]) / 2.0 for axis in ('x', 'y', 'z')}
    return dict(center, frame_id=workspace_frame, stamp_sec=stamp)


def placement_tilt(target, agent_id=None):
    if target not in settings.OBJECT_PROFILES:
        return None
    profile = material_profile(target, agent_id)
    if not (profile['place_roll_offset_deg'] or profile['place_pitch_offset_deg']):
        return None
    return profile
