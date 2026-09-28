"""재료별 실험값. 수정 후 빌드하고 두 Agent를 재시작하세요.

grip_position: SDK 그리퍼 위치(0~850).
thickness_mm: 내려놓은 재료의 수직 두께(mm).
place_tcp_offset_mm: 빈 바구니에 놓는 TCP의 workspace Z - 바구니 바닥 Z.
두 Agent에 동일한 두께/바닥 높이를 배포해야 합니다. 바구니 중심은 빨간 점 두 개로 계산합니다.
None은 미설정이며 해당 작업은 로봇 이동 전에 거부됩니다.
"""

OBJECT_PROFILES = {
    "빵": {
        "grip_position": 150.0,
        "thickness_mm": 20.0,
        # 기울인 배치 자세에서 실측할 TCP 높이 보정값.
        "place_tcp_offset_mm": 0.0,
        "place_roll_offset_deg": 0.0,
        "place_pitch_offset_deg": 0.0,
        "place_tilt_speed": 75.0,
    },
    "양상추": {
        "grip_position": 400.0,
        "thickness_mm": 0.0,
        "place_tcp_offset_mm": 0.0,
    },
    "바나나": {
        "grip_position": 330.0,
        "thickness_mm": 35.0,
        "place_tcp_offset_mm": 0.0,
    },
}

GRIPPER_OPEN_POSITION = 850.0

# workspace_0 원점은 마커 0 중심. 마커 평면과 바구니 내부 바닥은 같다고 가정하지 않습니다.
# 빨간 점 평균 Z도 손잡이 높이이므로, 내부 바닥 Z는 별도 실측값을 입력합니다.
BASKET_FLOOR_Z_MM = 0.0

# 조립 X/Y는 /perception/yolo_extra의 두 basket_handle 좌표 평균을 사용합니다.
BASKET_YAW_DEG = 0.0

# offset 방식은 workspace의 기본 roll=180°, pitch=0°에 더하는 시험 설정입니다.
# 빵의 공통 offset은 0으로 해제합니다. 실측 Base 자세는 해당 Agent에만 적용합니다.
# *_rpy_robot_deg는 UFactory Base 기준 [Roll, Pitch, Yaw] 절대각(degree)입니다.
# 두 자세를 함께 지정하면 offset 방식보다 우선하며 workspace 자세 TF를 적용하지 않습니다.
# TCP 위치는 인식/적층 계산을 사용합니다. 사진의 XYZ와 J1~J6는 재사용하지 않습니다.
AGENT_PROFILE_OVERRIDES = {
    "agent2": {  # UFactory 사진의 robot IP: 192.168.1.198
        "빵": {
            "pick_rpy_robot_deg": [179.4, 0.0, -0.1],
            "place_rpy_robot_deg": [141.5, -0.1, -0.1],
        },
    },
}
