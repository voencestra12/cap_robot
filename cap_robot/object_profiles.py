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
    "햄": {
        "grip_position": 30.0,
        "thickness_mm": 5.0,
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

