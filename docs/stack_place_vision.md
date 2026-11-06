# Agent1 양상추 Place 비전 보정

코드의 `stack_place_vision.enabled` 기본값은 `false`입니다.
현재 Agent1 YAML은 현장 설정으로 `enabled=true`, `observation_validated=true`이며,
이번 수정은 YAML 및 실측 제한값을 변경하지 않습니다.
활성화 대상은 **Agent1, basket, 양상추, layer_index=1**이며,
동일 assembly의 layer 0 빵 Task가 `SUCCEEDED`여야 합니다.
Agent2, 다른 재료, 첫 빵 및 다른 층은 기존 Pick & Place를 수행합니다.

## 이동·인식 순서

1. 기존 양상추 Pick 재확인, 파지, 안전 상승을 수행합니다.
2. 기존 선행 Task 대기와 공용 구역 Token 확보 후 고정 카메라의 파란 점 두 개를 새로 읽습니다.
   두 점의 workspace XY 평균을 TCP 관측 XY로 사용합니다. 손잡이 Z는 사용하지 않습니다.
3. 관측 높이는 로봇 Base(SDK) Z 기준으로 정합니다.
   `max(Pick 상승 실제 SDK Z, 바구니 XY의 기존 높은 Place SDK Z)`이며 그보다 더 올리지 않습니다.
   상승량이 `POSITION_TOLERANCE_MM`(1 mm) 미만이면 별도 상승 구간을 만들지 않고 실제 높이를 유지합니다.
   workspace_0(ArUco)는 Base와 평행하지 않으므로(현장 측정 0.5~2.7°) workspace Z를 유지한
   이동은 로봇 기준으로 오르내립니다. 따라서 수평 이동은 SDK Z를 고정하고, 수직 이동은 SDK XY를 고정합니다.
   경로는 Pick 상승 위치 → (필요 시 수직 상승) → 관측 높이에서 수평 이동이며, 관측 전 하강은 없습니다.
   관측점의 workspace XY는 정확히 파란 점 평균이고, workspace Z는 기존 높은 Place Z 이상입니다
   (위의 1 mm 미만 생략분 제외).
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
   후보가 0개인 정상 프레임은 건너뛰고 다음 새 RGB/Depth를 기다립니다.
   후보가 없는 프레임도 처리 시각을 기록하므로 같은 영상이나 한쪽 영상만 바뀐 것을
   다음 관측 표본으로 재사용하지 않습니다. confidence 부족, 복수 후보, Depth/TF 오류는
   기존대로 실패합니다.
   서로 다른 최신 RGB/Depth에서 유효 XYZ 표본 `samples`개(기본 3개)를 모으며,
   새 표본과 기존 모든 표본 사이의 XYZ 거리가 `stability_mm` 이하여야 합니다.
   초과하면 이전 묶음을 모두 버리고 새 표본부터 안정성 검사를 다시 시작합니다.
   평균에는 마지막 안정 표본 묶음만 사용합니다. 재시도 시에도 settle 이후
   `timeout_sec=5.0`의 원래 마감 시각을 연장하지 않습니다.
7. 기존 중앙 파지 가정 `stack_tcp_to_material_center_mm=[0,0,0]`에서 최종 TCP XY는
   관측한 빵 중심 XY의 평균입니다. 기존 양상추 `place_xy_offset_mm=[0,-30]`를 다시 더하지 않습니다.
   nominal Place XY는 허용 보정량 검사 기준으로만 사용합니다.
8. 관측 SDK Z에서 보정 XY로 수평 이동한 뒤 기존 높은 Place 접근 위치로 수직 하강합니다.
   기울기 때문에 보정된 접근 위치가 관측 높이보다 높으면 먼저 수직 상승합니다.
   관측 후 workspace TF가 바뀌었으면 보정 이동을 거부합니다.
   이후 원래 Place 접근·하강·해제·후퇴를 실행합니다.
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
- 계획한 경로는 구간마다 SDK Z 또는 SDK XY 중 하나만 바뀌어야 합니다. 수평 이동에는
  1 mm 미만의 Z 변화만 허용하며, 1 mm 미만 Z 단계를 단독 구간으로 만들지 않습니다.
- 무동작 검사는 로봇이 서 있는 현재 위치에서 시작해 실제로 실행할 이동만 검사합니다.
  같은 자세를 반복하는 길이 0 명령은 검사 목록에서 뺍니다. 2026-11-05 실기에서
  0.00008 mm 구간과 길이 0 구간이 포함된 경로가 C24(Speed Exceeds Limit)로 거부되었습니다.
  실패 시 오류에 구간 번호와 명령을 함께 기록합니다.
- 무동작 검사에 보내는 roll/pitch/yaw는 측정 TCP 자세(첫 구간)와 직전 명령(이후 구간)과 같은
  ±180° 표기로 이어서 보냅니다. 컨트롤러 검사는 RPY 차이를 그대로 계산하여, 2026-11-06 실기에서
  측정 roll −179.5° → 명령 +180°를 손목 한 바퀴로 보고 C24로 거부했습니다(−180°는 통과,
  실제 +180° 이동은 0.5°만 회전하며 정상). 같은 물리 자세이므로 실제 이동 명령은 바꾸지 않습니다.
- 실제 TCP(`get_position`)는 Pick 상승 후, 경로 진입 전, 각 관측 경유점 도착 후에
  명령 위치와 `stack_place.POSITION_TOLERANCE_MM`(1.0 mm) 이내여야 합니다.
- workspace_0와 Base의 기울기가 `stack_place.MAX_WORKSPACE_TILT_DEG`(10°)를 넘으면
  캘리브레이션 이상으로 보고 관측 전에 거부합니다. 두 값은 ROS 파라미터가 아닌 코드 상수입니다.
- 기존 Token 보호를 유지하고 경유점 사이와 해제 전에 Token·선행 Task 상태를 재확인합니다.
- 경로 검사 후 보정 이동 직전에 관측값의 신선도를 다시 확인합니다.
- 정지 실패, 복수 후보, Depth/TF 오류, 오래된/중복 영상, 허용 범위 초과,
  경로 거부 시 원래 XY로 임의 Place하지 않습니다. 빵 미검출 또는 XYZ 불안정이
  이어져 마감 시각까지 안정 표본을 모으지 못한 경우에도 실패합니다.
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

## 관측 디버깅

관측 프레임마다 번호, RGB/Depth 촬영·수신 시각, 원시 YOLO 모델/클래스/confidence/
Bounding Box, 각 객체의 Depth 유효 여부와 TF 결과, 빵의 workspace XYZ를 로그에 남깁니다.
후보별 바구니 XY·Z·confidence 검사 결과와 기준값, 유효 표본 수, 안정성 결과 및 재수집 이유,
최종 평균 빵 XYZ와 nominal TCP 대비 XY 보정량도 기록합니다.

관측 실패 시 실행 작업 디렉터리의 `log/stack_place_vision/`에 고유 ID별로
`*_rgb.png`(원본 RGB), `*_annotated_rgb.png`(원시 Box 포함), `.json`(검출·시각·실패 이유)을
저장하고 JSON 경로를 오류 로그에 출력합니다. 완료된 YOLO 관측이 없으면 최신 RGB와
메타데이터를 저장하고 검출 미완료를 명시합니다. 영상이 아예 없으면 JSON만 남깁니다.
디버그 파일 저장 자체가 실패해도 원래 관측 실패를 유지합니다.

원시 Bounding Box는 이 관측 중 Depth/TF 검사 전에 주황색 `raw` 라벨로 표시하므로
검증에 실패한 객체도 화면에서 볼 수 있습니다. 원본 추론 영상은 별도로 유지하며,
원시 Box를 객체 선택·ObjectMemory·제어 후보에 추가하지 않습니다.

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

이번 검증 결과: **323 passed** (2026-11-06, roll ±180° 표기 C24 수정 후).
workspace_0가 Base 대비 ±0.5°, ±2.7° 기울어진 경우의 전체 실행(관측 전 하강 없음,
SDK 수직·수평 분리, 보정 후에만 하강), 1 mm 미만 TCP 피드백 오차 허용과
2 mm 오차 거부, 15° 이상 기울기 거부, 실기 C24 경로(0.00008 mm 차이)의 무동작 검사에
1 mm 미만 이동 구간이 없음과, 측정 roll −179.5°·명령 +180° 실기 사례가
RPY 차이를 그대로 판정하는 컨트롤러 모의에서 통과함을 검사합니다. 새 기울기·오차 테스트는 수정 전
코드에서 실패함을 확인했습니다.
기존 검증 항목인 첫 프레임 후보 0개 후 3회 안정 검출·보정, 이상 좌표 후 표본 재수집, 5초 마감,
RGB/Depth 중복 거부, settle 이후 촬영, 실패 영상 저장, Depth/TF 실패 시 원시 Box 표시도
계속 포함됩니다. compileall 및 `git diff --check` 통과.
단위 테스트는 실환경 충돌·시야 검증을 대신하지 않습니다.
