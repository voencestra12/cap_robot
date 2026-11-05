# Agent1 양상추 Place 비전 보정

두 Agent의 `stack_place_vision.enabled` 기본값은 `false`입니다.
활성화 대상은 **Agent1, basket, 양상추, layer_index=1**이며,
동일 assembly의 layer 0 빵 Task가 `SUCCEEDED`여야 합니다.
Agent2, 다른 재료, 첫 빵 및 다른 층은 기존 Pick & Place를 수행합니다.

## 이동·인식 순서

1. 기존 양상추 Pick 재확인, 파지, 안전 상승을 수행합니다.
2. 기존 선행 Task 대기와 공용 구역 Token 확보 후 고정 카메라의 파란 점 두 개를 새로 읽습니다.
   두 점의 workspace XY 평균을 TCP 관측 XY로 사용합니다. 손잡이 Z는 사용하지 않습니다.
3. 관측 Z는 기존 첫 높은 `move_to_place`의 `assembly_move_pose()` 결과를 재사용합니다.
   Pick 상승 위치가 더 높으면 그 높이에서 수평 이동한 후 기존 높은 Place Z로 수직 이동합니다.
   낮은 위치에서 대각선으로 바구니에 진입하지 않습니다.
4. blocking 이동 성공과 controller의 정상 정지를 확인한 뒤 `perception_settle_sec`를 적용합니다.
   촬영 시각과 callback 수신 시각이 모두 그 이후인 새 RGB/Depth만 YOLO 관측에 사용합니다.
5. 정렬된 Depth와 카메라 내부 파라미터로 빵 중심 픽셀을 카메라 XYZ(m)로 역투영합니다.
   RGB 촬영 시각에 다음 두 TF를 조회하고 합성합니다. 최신 TF나 cached pose fallback은 없습니다.

   ```text
   workspace_0 <- agent1_tcp <- CameraInfo optical frame
   p_workspace_mm = 1000 * (T_workspace_tcp(t_rgb) @ T_tcp_camera(t_rgb) @ p_camera_m)
   ```

   기존 SDK TCP와 Hand-eye TF를 사용합니다. 카메라와 TCP의 상대 XYZ/회전은 TF에서 구하며
   별도의 사용자 카메라 XY Offset, 카메라 Z, 관측 transit 높이/속도를 받지 않습니다.
6. 바구니 내부 XY 범위, 빵 중심 Z 범위, 신뢰도를 만족하는 빵 후보가 정확히 하나여야 합니다.
   서로 다른 RGB/Depth 표본들의 XYZ 안정성을 확인합니다.
7. 기존 중앙 파지 가정 `stack_tcp_to_material_center_mm=[0,0,0]`에서 최종 TCP XY는
   관측한 빵 중심 XY의 평균입니다. 기존 양상추 `place_xy_offset_mm=[0,-30]`를 다시 더하지 않습니다.
   nominal Place XY는 허용 보정량 검사 기준으로만 사용합니다.
8. 기존 높은 Place Z에서 보정 XY로 수평 이동합니다. 이후 원래 Place 접근·하강·해제·후퇴를 실행합니다.
   기존 Place Z, Roll/Pitch/Yaw, 그리퍼 명령은 유지합니다.
   한 번 확정한 XY를 모든 후속 Place 동작에서 재사용합니다.

## 경로 검사와 실패 처리

관측 진입 전에는 진입 경로 전체를 검사합니다. 인식 후에는 수평 보정부터
기존 Place 접근·자세 변경(설정 시)·하강·후퇴까지 모두 검사한 후 보정 이동을 시작합니다.

- 각 SDK Cartesian 경유점의 유한수와 기존 SDK Safety Box를 검사합니다.
  Safety Box는 볼록하므로 직선 구간도 내부에 있어야 합니다.
- xArm의 `only_check_type` 무동작 계획으로 연결된 전체 경로의 관절/Cartesian 한계,
  자체 충돌, 속도를 검사합니다. 첫 구간은 실제 상태에서 시작하고 중간 가상 상태를 연결합니다.
  검사 요청에 `only_check_type=1/3/2`, `wait=false`를 명시하며 전역 SDK 설정은 바꾸지 않습니다.
  지원 SDK와 펌웨어 **1.11.100 이상**이 없거나 검사 실패/오류이면 실행을 거부합니다.
- 기존 Token 보호를 유지하고 경유점 사이와 해제 전에 Token·선행 Task 상태를 재확인합니다.
- 경로 검사 후 보정 이동 직전에 관측값의 신선도를 다시 확인합니다.
- 정지 실패, 빵 미검출/복수 후보, Depth/TF 오류, 오래된/중복 영상, XYZ 불안정,
  허용 범위 초과, 경로 거부 시 원래 XY로 임의 Place하지 않습니다.
- 관측 보호 진입 후 실패하면 현재 그리퍼를 유지하고 자동 Home·Token 반납을 하지 않습니다.
  기존 `_place_observation_fault`가 후속 단독/협동 Task를 차단합니다.

SDK의 자체 충돌 검사는 외부 바구니·파지한 양상추·카메라·다른 장애물의 형상 모델을 포함하지 않습니다.
이들 간섭과 파지 상태 카메라 시야는 현장에서 확인해야 하며, 이를 나타내는 기존
`observation_validated=false` 보호를 유지합니다. 단위 테스트는 실환경 충돌 검증을 대신하지 않습니다.

## 설정

아래는 `stack_place_vision.` 접두사를 사용하는 설정입니다.

| 이름 | 기본값/현재 설정 | 용도 |
|---|---|---|
| `enabled` | `false` | Agent1 양상추 보정 활성화 |
| `materials` | `[양상추]` | 허용 목록; 다른 Agent/재료로 적용 범위를 넓히지 않음 |
| `observation_validated` | `false` | 높은 Place 위치의 시야·바구니·도구 간섭 현장 확인 |
| `tcp_frame` | Agent1 YAML: `agent1_tcp` | 기존 활성 SDK TCP에 해당하는 TF |
| `basket_half_extent_mm` | `[.nan,.nan]` | 빵 후보의 바구니 내부 XY 반폭 |
| `target_z_range_mm` | `[.nan,.nan]` | 빵 중심 workspace 절대 Z 허용 범위 |
| `max_correction_mm` | `.nan` | nominal TCP XY 대비 최대 보정 거리 |
| `stability_mm` | `3.0` | 표본 사이 최대 XYZ 거리 |
| `samples` | `3` | 서로 다른 새 RGB/Depth 표본 수(2~8) |
| `min_confidence` | `0.6` | 빵 검출 최소 신뢰도 |
| `timeout_sec` | `5.0` | 새 관측 대기 한도 |
| `max_age_sec` | `1.0` | 촬영·수신 후 최대 경과 시간 |
| `sync_tolerance_sec` | `0.08` | RGB/Depth 최대 시각 차이 |

`.nan`은 미설정 값이며 실행을 거부합니다. 기존 nominal 양상추 XY Offset에서
실제 빵 중심까지의 거리도 `max_correction_mm`에 포함됩니다.
기존 안전 접근 높이, action 속도, settle 시간, Token, Safety Box 설정을 재사용합니다.
수동 카메라 XY/Z와 별도 관측 transit 높이/속도 설정 및 관측 위치 역산 코드는 제거했습니다.

## 검증

```bash
cd ~/colcon_ws/src/cap_robot
python3 -m pytest -q
python3 -m compileall -q cap_robot test
git diff --check
```

ROS Node 생성, SDK 연결, 실카메라, ROS launch, 실제 로봇 실행 없이 실제 메서드 본문과
mock SDK/TF/영상으로 검사합니다. 일반 PnP·다른 Agent·Token·Guidebook 회귀 테스트도 함께 실행합니다.
추가 검증에는 높은 관측 좌표, 수평 보정 후 수직 하강, 촬영 시각 TF 합성, XYZ 변환,
Offset 중복 방지, 정지 확인, 전체 경로 무동작 검사, 인식/Depth/TF/경로 실패 중단이 포함됩니다.

검증 결과: **282 passed, 95 subtests passed**. compileall 및 `git diff --check` 통과.
기존 실행/SDK 이동/인식 루프/협동 실행 메서드의 AST와 비전 설정 외 Agent YAML 값도 원본과 동일합니다.
환경에 기존 NumPy/SciPy 버전 호환 경고 1개가 있으며 테스트 실패는 없습니다.
