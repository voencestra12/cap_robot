"""LLM이 사용할 수 있는 로봇 API 정의와 정책 검증 기능.

실제 로봇 동작 실행은 robot_agent.py가 담당합니다.
이 파일은 API 이름, 기본 Pick-and-Place 시퀀스, 허용 범위 검증만 관리합니다.
"""

API_CONTROL_GRIPPER = 'control_gripper'
API_MOVE_TO_OBJECT = 'move_to_object'
API_MOVE_TO_PLACE = 'move_to_place'
API_WAIT = 'wait'
API_RETURN_HOME = 'return_home'
API_COOPERATIVE_MOVE_RELATIVE = 'cooperative_move_relative'

ALLOWED_APIS = {
    API_CONTROL_GRIPPER,
    API_MOVE_TO_OBJECT,
    API_MOVE_TO_PLACE,
    API_WAIT,
    API_RETURN_HOME,
}

COOPERATIVE_ALLOWED_APIS = {
    API_CONTROL_GRIPPER,
    API_MOVE_TO_OBJECT,
    API_WAIT,
    API_RETURN_HOME,
    API_COOPERATIVE_MOVE_RELATIVE,
}

# [MERGED] 세트 1의 바구니 협동 시퀀스를 cap_robot API 형식으로 표현합니다.
# 수치는 Workstation LLM이 선택하며, 두 Agent는 검증된 동일 배열을 공유합니다.
DEFAULT_COOPERATIVE_ACTIONS = [
    {"api": API_CONTROL_GRIPPER, "position": 850},
    {"api": API_MOVE_TO_OBJECT, "z_offset": 200, "speed": 60},
    {"api": API_MOVE_TO_OBJECT, "z_offset": 10, "speed": 40},
    {"api": API_CONTROL_GRIPPER, "position": 50},
    {
        "api": API_COOPERATIVE_MOVE_RELATIVE,
        "x_offset": 0,
        "y_offset": 0,
        "z_offset": 100,
        "speed": 40,
    },
    {
        "api": API_COOPERATIVE_MOVE_RELATIVE,
        "x_offset": 0,
        "y_offset": -100,
        "z_offset": 0,
        "speed": 40,
    },
    {
        "api": API_COOPERATIVE_MOVE_RELATIVE,
        "x_offset": 0,
        "y_offset": 0,
        "z_offset": -100,
        "speed": 40,
    },
    {"api": API_CONTROL_GRIPPER, "position": 850},
    {"api": API_MOVE_TO_OBJECT, "z_offset": 150, "speed": 50},
    {"api": API_RETURN_HOME},
]

DEFAULT_PNP_ACTIONS = [
    {"api": API_CONTROL_GRIPPER, "position": 850},
    {"api": API_MOVE_TO_OBJECT, "z_offset": 200, "speed": 100},
    {"api": API_MOVE_TO_OBJECT, "z_offset": 40, "speed": 80},
    {"api": API_CONTROL_GRIPPER, "position": 300},
    {"api": API_MOVE_TO_OBJECT, "z_offset": 200, "speed": 100},
    {"api": API_MOVE_TO_PLACE, "z_offset": 200, "speed": 100},
    {"api": API_MOVE_TO_PLACE, "z_offset": 40, "speed": 90},
    {"api": API_CONTROL_GRIPPER, "position": 850},
    {"api": API_MOVE_TO_PLACE, "z_offset": 200, "speed": 100},
]


def validate_actions(raw_actions, pick_place_z_offset_mm):
    """LLM 정책의 API 순서와 수치 범위를 검증합니다.

    기존 robot_agent.py의 검증 로직을 그대로 옮긴 함수입니다.
    """
    if not isinstance(raw_actions, list) or not raw_actions or len(raw_actions) > 20:
        raise ValueError('actions는 1~20개의 JSON 배열이어야 합니다.')

    pick_place_z_offset_mm = float(pick_place_z_offset_mm)
    phase = 'START'
    actions = []

    for index, raw in enumerate(raw_actions):
        if not isinstance(raw, dict):
            raise ValueError(f'actions[{index}]는 JSON 객체여야 합니다.')

        api = str(raw.get('api', '')).strip()

        if api == API_CONTROL_GRIPPER:
            position = float(raw.get('position'))
            if not 0.0 <= position <= 850.0:
                raise ValueError('그리퍼 위치는 0~850이어야 합니다.')
            if phase == 'START' and position >= 700:
                phase = 'OPEN'
            elif phase == 'AT_PICK' and position <= 450:
                phase = 'GRASPED'
            elif phase == 'AT_PLACE' and position >= 700:
                phase = 'RELEASED'
            else:
                raise ValueError(f'actions[{index}] 그리퍼 순서가 올바르지 않습니다.')
            actions.append({'api': api, 'position': position})
            continue

        if api in (API_MOVE_TO_OBJECT, API_MOVE_TO_PLACE):
            z_offset = float(raw.get('z_offset'))
            speed = float(raw.get('speed'))
            if not pick_place_z_offset_mm <= z_offset <= 300.0:
                raise ValueError(
                    f'z_offset은 {pick_place_z_offset_mm:.0f}~300 mm이어야 합니다.'
                )
            if not 20.0 <= speed <= 150.0:
                raise ValueError('speed는 20~150이어야 합니다.')

            if api == API_MOVE_TO_OBJECT:
                if phase == 'OPEN' and z_offset >= 150:
                    phase = 'ABOVE_OBJECT'
                elif phase == 'ABOVE_OBJECT' and z_offset == pick_place_z_offset_mm:
                    phase = 'AT_PICK'
                elif phase == 'GRASPED' and z_offset >= 150:
                    phase = 'LIFTED'
                else:
                    raise ValueError(f'actions[{index}] 객체 이동 순서/높이가 올바르지 않습니다.')
            else:
                if phase == 'LIFTED' and z_offset >= 150:
                    phase = 'ABOVE_PLACE'
                elif phase == 'ABOVE_PLACE' and z_offset == pick_place_z_offset_mm:
                    phase = 'AT_PLACE'
                elif phase == 'RELEASED' and z_offset >= 150:
                    phase = 'RETREATED'
                else:
                    raise ValueError(f'actions[{index}] 배치 이동 순서/높이가 올바르지 않습니다.')

            actions.append({'api': api, 'z_offset': z_offset, 'speed': speed})
            continue

        if api == API_WAIT:
            seconds = float(raw.get('seconds'))
            if not 0.0 <= seconds <= 2.0:
                raise ValueError('wait는 0~2초이어야 합니다.')
            actions.append({'api': api, 'seconds': seconds})
            continue

        if api == API_RETURN_HOME:
            if phase != 'RETREATED':
                raise ValueError('return_home은 배치 후 후퇴한 다음에만 가능합니다.')
            phase = 'HOME'
            actions.append({'api': api})
            continue

        raise ValueError(f"허용되지 않은 API: '{api}'")

    if phase not in ('RETREATED', 'HOME'):
        raise ValueError('정책이 물체 해제 후 안전 후퇴까지 완료되지 않았습니다.')

    return actions


def validate_cooperative_actions(raw_actions):
    """양팔 바구니 작업의 공유 action 배열을 검증합니다.

    Workstation LLM은 값과 순서를 선택할 수 있지만, 두 팔이 물체를 잡은 뒤에는
    cooperative_move_relative만으로 workspace 기준 이동해야 합니다.
    """
    if not isinstance(raw_actions, list) or not raw_actions or len(raw_actions) > 30:
        raise ValueError('협동 actions는 1~30개의 JSON 배열이어야 합니다.')

    phase = 'START'
    actions = []
    cooperative_move_count = 0

    for index, raw in enumerate(raw_actions):
        if not isinstance(raw, dict):
            raise ValueError(f'actions[{index}]는 JSON 객체여야 합니다.')

        api = str(raw.get('api', '')).strip()
        if api not in COOPERATIVE_ALLOWED_APIS:
            raise ValueError(f"협동 작업에서 허용되지 않은 API: '{api}'")

        if api == API_WAIT:
            seconds = float(raw.get('seconds'))
            if not 0.0 <= seconds <= 2.0:
                raise ValueError('협동 wait는 0~2초이어야 합니다.')
            actions.append({'api': api, 'seconds': seconds})
            continue

        if api == API_CONTROL_GRIPPER:
            position = float(raw.get('position'))
            if not 0.0 <= position <= 850.0:
                raise ValueError('그리퍼 위치는 0~850이어야 합니다.')
            if phase == 'START' and position >= 700.0:
                phase = 'OPEN'
            elif phase == 'AT_HANDLE' and position <= 450.0:
                phase = 'GRASPED'
            elif phase == 'COOPERATIVE_MOVING' and position >= 700.0:
                phase = 'RELEASED'
            else:
                raise ValueError(f'actions[{index}] 협동 그리퍼 순서가 올바르지 않습니다.')
            actions.append({'api': api, 'position': position})
            continue

        if api == API_MOVE_TO_OBJECT:
            z_offset = float(raw.get('z_offset'))
            speed = float(raw.get('speed'))
            if not 0.0 <= z_offset <= 300.0:
                raise ValueError('협동 손잡이 z_offset은 0~300 mm이어야 합니다.')
            if not 20.0 <= speed <= 100.0:
                raise ValueError('협동 접근 speed는 20~100이어야 합니다.')

            if phase == 'OPEN' and z_offset >= 100.0:
                phase = 'ABOVE_HANDLE'
            elif phase == 'ABOVE_HANDLE' and z_offset <= 60.0:
                phase = 'AT_HANDLE'
            elif phase == 'RELEASED' and z_offset >= 100.0:
                phase = 'RETREATED'
            else:
                raise ValueError(f'actions[{index}] 손잡이 접근/후퇴 순서가 올바르지 않습니다.')

            actions.append({'api': api, 'z_offset': z_offset, 'speed': speed})
            continue

        if api == API_COOPERATIVE_MOVE_RELATIVE:
            if phase not in ('GRASPED', 'COOPERATIVE_MOVING'):
                raise ValueError(
                    f'actions[{index}] cooperative_move_relative는 양팔 파지 후에만 가능합니다.'
                )
            x_offset = float(raw.get('x_offset', 0.0))
            y_offset = float(raw.get('y_offset', 0.0))
            z_offset = float(raw.get('z_offset', 0.0))
            speed = float(raw.get('speed', 40.0))
            offsets = (x_offset, y_offset, z_offset)
            if not all(-300.0 <= value <= 300.0 for value in offsets):
                raise ValueError('협동 상대 이동의 축별 offset은 -300~300 mm이어야 합니다.')
            if abs(x_offset) + abs(y_offset) + abs(z_offset) < 1e-6:
                raise ValueError('협동 상대 이동은 하나 이상의 0이 아닌 offset이 필요합니다.')
            if not 10.0 <= speed <= 80.0:
                raise ValueError('협동 이동 speed는 10~80이어야 합니다.')
            cooperative_move_count += 1
            phase = 'COOPERATIVE_MOVING'
            actions.append({
                'api': api,
                'x_offset': x_offset,
                'y_offset': y_offset,
                'z_offset': z_offset,
                'speed': speed,
            })
            continue

        if api == API_RETURN_HOME:
            if phase != 'RETREATED':
                raise ValueError('협동 return_home은 해제 후 안전 후퇴한 다음에만 가능합니다.')
            phase = 'HOME'
            actions.append({'api': api})
            continue

    if cooperative_move_count < 1:
        raise ValueError('협동 계획에는 cooperative_move_relative가 하나 이상 필요합니다.')
    if phase not in ('RETREATED', 'HOME'):
        raise ValueError('협동 계획이 동시 해제 후 안전 후퇴까지 완료되지 않았습니다.')
    return actions


# Workstation LLM이 작업 가이드북의 required_capabilities를 만들 때 사용하는
# 추상 capability 목록입니다. 일반 Task 순서는 Agent LLM, 협동 Task 순서는
# Workstation LLM이 정하고 robot_agent.py가 모두 결정론적으로 재검증합니다.
API_REGISTRY = {
    API_CONTROL_GRIPPER: {
        'description': '그리퍼를 열고 닫아 물체를 파지하거나 해제합니다.',
        'provides': [
            'gripper_control',
        ],
    },
    API_MOVE_TO_OBJECT: {
        'description': '검출된 객체의 작업 위치로 로봇을 접근시킵니다.',
        'provides': [
            'robot_motion',
            'object_approach',
        ],
    },
    API_MOVE_TO_PLACE: {
        'description': '목표 위치 또는 기준 객체에 대해 물체를 배치합니다.',
        'provides': [
            'robot_motion',
            'object_placement',
            'relative_positioning',
        ],
    },
    API_WAIT: {
        'description': '정해진 짧은 시간 동안 다음 동작을 기다립니다.',
        'provides': [
            'timing_control',
        ],
    },
    API_RETURN_HOME: {
        'description': '작업 종료 후 로봇을 기본 위치로 복귀시킵니다.',
        'provides': [
            'home_return',
        ],
    },
    API_COOPERATIVE_MOVE_RELATIVE: {
        'description': (
            '두 Agent가 물체를 함께 파지한 상태에서 workspace_0 기준 상대 이동을 '
            '동일한 공유 step으로 수행합니다.'
        ),
        'provides': [
            'dual_arm_grasp',
            'cooperative_transport',
            'synchronized_release',
        ],
    },
}


CAPABILITY_DESCRIPTIONS = {
    'gripper_control': '그리퍼를 열고 닫아 물체 파지·해제를 수행하는 능력',
    'robot_motion': '작업 위치 사이를 이동하는 능력',
    'object_approach': '대상 객체에 안전하게 접근하는 능력',
    'object_placement': '대상 위치에 물체를 배치하는 능력',
    'relative_positioning': '다른 객체나 기준 위치에 대한 상대 관계로 배치하는 능력',
    'timing_control': '협업 순서나 동기화를 위해 대기 시간을 적용하는 능력',
    'home_return': '작업 후 기본 위치로 복귀하는 능력',
    'dual_arm_grasp': '두 로봇이 서로 다른 손잡이를 동기화하여 파지하는 능력',
    'cooperative_transport': '두 로봇이 공유 물체를 workspace 기준으로 함께 이동하는 능력',
    'synchronized_release': '두 로봇이 공유 물체를 동기화하여 해제하는 능력',
}


def get_capability_catalog():
    """API_REGISTRY에서 capability와 이를 제공하는 API를 자동으로 생성합니다."""
    catalog = {}

    for api_name, api_info in API_REGISTRY.items():
        for capability in api_info.get('provides', []):
            item = catalog.setdefault(
                capability,
                {
                    'description': CAPABILITY_DESCRIPTIONS.get(capability, ''),
                    'provided_by': [],
                },
            )
            item['provided_by'].append(api_name)

    for item in catalog.values():
        item['provided_by'] = sorted(set(item['provided_by']))

    return dict(sorted(catalog.items()))
