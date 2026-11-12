"""재료별 실험값. 수정 후 빌드하고 두 Agent를 재시작하세요.

grip_position: SDK 그리퍼 위치(0~850).
thickness_mm: 내려놓은 재료의 수직 두께(mm).
place_tcp_offset_mm: 빈 바구니에 놓는 TCP의 workspace Z - 바구니 바닥 Z.
place_xy_offset_mm: 적층 시 바구니 중심 대비 TCP의 workspace [dX, dY](mm). 생략 시 [0, 0].
stack_tcp_to_material_center_mm: TCP 로컬 XYZ → 파지 재료 중심(mm), Agent별 dict 지원.
place_roll_offset_deg: 기본 배치 자세 대비 roll 보정(도). Agent별 값은 {agent_id: 보정값}.
두 Agent에 동일한 두께/바닥 높이를 배포해야 합니다. 바구니 중심은 파란 점 두 개로 계산합니다.
None은 미설정이며 해당 작업은 로봇 이동 전에 거부됩니다.
"""

OBJECT_PROFILES = {
    "빵": {
        # Agent1은 거의 중앙 파지(2026-11-12, Z 미측정).
        # Agent2는 빵 중심이 손가락 끝 아래(+Z) 약 50 mm(2026-11-12 실측).
        "stack_tcp_to_material_center_mm": {"agent1": [0.0, 0.0, 0.0], "agent2": [0.0, 0.0, 50.0]},
        "grip_position": 150.0,
        "thickness_mm": 20.0,
        # 기울인 배치 자세에서 실측할 TCP 높이 보정값.
        "place_tcp_offset_mm": 90.0,
        "place_roll_offset_deg": {"agent1": 30.0, "agent2": -30.0},
        "place_pitch_offset_deg": 0.0,
        "place_tilt_speed": 75.0,
    },
    "양상추": {
        "stack_tcp_to_material_center_mm": [0.0, 0.0, 0.0],  # 실측 필요 (중심 파지 가정)
        "grip_position": 100.0,
        "thickness_mm": 0.0,
        # 손가락 끝(TCP) 아래로 늘어진 양상추 때문에 30 mm(18 mm·40 mm 여유)에서 빵과 접촉(C31,
        # 2026-11-06). 놓는 높이를 빵 윗면 위 60 mm로. 늘어짐을 실측하면 다시 조정.
        "place_tcp_offset_mm": 60.0,
        # 0층 빵이 바구니 중심에서 어긋나 놓이므로 공칭 위치를 빵 중심 쪽으로 보정.
        # 적층 비전 보정 시에는 공칭값과 max_correction_mm 기준으로만 쓰입니다.
        "place_xy_offset_mm": [0.0, -30.0],
        # 미설정 시 기본 20 → 제자리 관측 회전과 적층 Place 이동이 느려짐. 빵/햄과 동일하게.
        "place_tilt_speed": 75.0,
    },
    "햄": {
        # Agent1은 거의 중앙 파지(2026-11-12, Z 미측정).
        # Agent2는 빵과 같게 햄 중심이 손가락 끝 아래(+Z) 약 50 mm(2026-11-12).
        "stack_tcp_to_material_center_mm": {"agent1": [0.0, 0.0, 0.0], "agent2": [0.0, 0.0, 50.0]},
        "grip_position": 30.0,
        "thickness_mm": 5.0,
        "place_tcp_offset_mm": 100.0,
        "place_roll_offset_deg": {"agent1": 30.0, "agent2": -30.0},
        "place_pitch_offset_deg": 0.0,
        "place_tilt_speed": 75.0,
    },
    "바나나": {
        "grip_position": 330.0,
        "thickness_mm": 35.0,
        "place_tcp_offset_mm": 0.0,
    },
}

GRIPPER_OPEN_POSITION = 850.0

# workspace_0 원점은 마커 0 중심. 마커 평면과 바구니 내부 바닥은 같다고 가정하지 않습니다.
# 파란 점 평균 Z도 손잡이 높이이므로, 내부 바닥 Z는 별도 실측값을 입력합니다.
BASKET_FLOOR_Z_MM = 5.0

# 조립 X/Y는 /perception/yolo_extra의 두 basket_handle 좌표 평균을 사용합니다.
BASKET_YAW_DEG = 0.0

