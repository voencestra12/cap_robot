# Agent1 양상추 Place 비전 보정

코드의 `stack_place_vision.enabled` 기본값은 `false`입니다.
현재 Agent1 YAML은 현장 설정으로 `enabled=true`, `observation_validated=true`이며,
이번 수정은 YAML 및 실측 제한값을 변경하지 않습니다.
활성화 대상은 **Agent1, basket, 양상추, layer_index=1**이며,
동일 assembly의 layer 0 빵 Task가 `SUCCEEDED`여야 합니다.
Agent2, 다른 재료, 첫 빵 및 다른 층은 기존 Pick & Place를 수행합니다.

## 이동·인식 순서

```text
기본 위치 → 양상추 위(+회전) → 하강 → 집기 → 상승            (기존 Action 1~5)
→ 제자리 회전으로 카메라를 바구니 쪽으로 향함 → 빵 인식      (Action 6 직전, Agent 내부)
→ 보정 XY의 높은 Place 접근(수직 자세로 복귀) → 하강 → 해제 → 후퇴   (기존 Action 6~9)
```

1. 기존 양상추 Pick 재확인, 파지, 안전 상승을 수행합니다.
2. 기존 선행 Task 대기와 공용 구역 Token 확보 후 고정 카메라의 파란 점 두 개를 새로 읽습니다.
3. **제자리 관측 회전**: Pick 상승 위치(실제 TCP)를 그대로 두고 자세만 바꿉니다.
   카메라 광축이 바구니 중심 XY(파란 점 평균)와 빵 Z 범위(로봇 Base 기준)의 가운데를 지나도록
   Hand-eye TF(`agent1_tcp` ← 카메라 optical frame)로 회전 자세를 계산합니다. 수동 카메라 Offset은 없습니다.
   현재 자세에서 가장 작은 회전을 사용하며, 도구 기울기(수직 아래 기준)는
   `stack_place.MAX_LOOK_TILT_DEG`(45°, 코드 상수)로 제한합니다. 제한되면 같은 방향으로 덜 기울입니다.
   회전 속도는 재료 프로필의 기존 `place_tilt_speed`(양상추 20)로 제한합니다.
   2026-11-06 실기 배치 기준 필요한 기울기는 약 47.7°이며, 45° 제한 시 빵은 광축에서 약 2.2° 벗어납니다.
4. blocking 이동 성공과 controller의 정상 정지를 확인한 뒤 `perception_settle_sec`를 적용합니다.
   촬영 시각과 callback 수신 시각이 모두 그 이후인 새 RGB/Depth만 YOLO 관측에 사용합니다.
5. 정렬된 Depth와 카메라 내부 파라미터로 빵 중심 픽셀을 카메라 XYZ(m)로 역투영합니다.
   RGB 촬영 시각에 다음 두 TF를 조회하고 합성합니다. 최신 TF나 cached pose fallback은 없습니다.
   카메라가 어느 위치·방향을 보든 같은 계산을 사용합니다.

   ```text
   workspace_0 <- agent1_tcp <- CameraInfo optical frame
   p_workspace_mm = 1000 * (T_workspace_tcp(t_rgb) @ T_tcp_camera(t_rgb) @ p_camera_m)
   ```

6. 바구니 내부 XY 범위, 빵 중심 Z 범위, 신뢰도를 만족하는 빵 후보가 정확히 하나여야 합니다.
   Z 범위는 **robot_1_base 기준**입니다. 마운트 카메라 → TCP → Base 경로는 ArUco workspace lock과
   무관하여, 2026-11-06 같은 빵이 lock마다 workspace Z −58/37/15 mm로 바뀌는 동안 Base Z는 67~70 mm로
   일정했습니다. 정지 이후 검사에서 거부된 새 프레임은 이유별로 세어 관측 시간 초과 오류에 함께 기록합니다.
   후보가 0개인 정상 프레임은 건너뛰고 다음 새 RGB/Depth를 기다립니다.
   후보가 없는 프레임도 처리 시각을 기록하므로 같은 영상이나 한쪽 영상만 바뀐 것을
   다음 관측 표본으로 재사용하지 않습니다. confidence 부족, 복수 후보, Depth/TF 오류는 실패합니다.
   서로 다른 최신 RGB/Depth에서 유효 XYZ 표본 `samples`개(기본 3개)를 모으며,
   새 표본과 기존 모든 표본 사이의 XYZ 거리가 `stability_mm` 이하여야 합니다.
   초과하면 이전 묶음을 모두 버리고 새 표본부터 다시 시작합니다.
   재시도 시에도 settle 이후 `timeout_sec=5.0`의 원래 마감 시각을 연장하지 않습니다.
7. 기존 중앙 파지 가정 `stack_tcp_to_material_center_mm=[0,0,0]`에서 최종 TCP XY는
   관측한 빵 중심 XY의 평균입니다. 기존 양상추 `place_xy_offset_mm=[0,-30]`를 다시 더하지 않습니다.
   nominal Place XY는 허용 보정량(`max_correction_mm`) 검사 기준으로만 사용합니다.
   해제 TCP Z는 `max(계산값, 측정한 빵 윗면 Z 평균 + 재료 place_tcp_offset_mm)`입니다.
   계산값(바닥 + 아래층 두께 + offset)보다 낮아지지 않으며, 상승 폭은 빵 Z 허용 범위
   (`target_z_range_mm`, Base 기준)로 제한됩니다. 측정 지점과 해제 지점이 같은 XY라 workspace 캘리브레이션의
   높이·기울기 오차가 상쇄됩니다. (2026-11-06: 계산 윗면 25 mm, 실측 37.3 mm → 여유 18 mm로 C31 접촉. 이후 여유 40 mm에서도
   늘어진 양상추가 빵에 닿아 양상추 `place_tcp_offset_mm`를 30 → 60으로 조정)
8. 보정 XY와 해제 Z를 한 번 확정한 뒤 기존 Place Action 6~9를 그대로 실행합니다.
   Action 6(높은 Place 접근)이 관측 자세에서 기존 Place 자세로 돌아오며 보정 XY로 이동하고,
   이후 수직 하강·해제·후퇴합니다. 기존 Place Z, Roll/Pitch/Yaw, 그리퍼 명령은 유지합니다.
   이 Place 이동(접근·하강·후퇴)의 속도는 관측 회전과 같은 재료 `place_tilt_speed`(양상추 20)로 제한합니다.
   (2026-11-06: 관측 후 접근이 93 mm/s로, 빵 접근의 실제 23 mm/s보다 4배 빨랐음)

## 안전과 실패 처리

- 컨트롤러 무동작 사전검사(`only_check_type`)는 사용하지 않습니다(팀 합의, 2026-11-06).
  이 검사는 실제 이동 직후 첫 검사에서 실제로는 문제없는 경로를 C24로 거부했습니다.
  기존 9단계 PnP와 같은 수준의 보호를 사용합니다: 기존 SDK Safety Box, 실제 이동 중 컨트롤러 정지.
- 제자리 관측 회전은 TCP 위치를 바꾸지 않습니다. 회전 전 controller 정상 정지와 실제 TCP 읽기를 확인합니다.
- 기존 Token 보호를 유지하고 관측 전후와 각 Place Action 전에 Token·선행 Task 상태를 재확인합니다.
- 정지 실패, 복수 후보, Depth/TF 오류, 오래된/중복 영상, 허용 범위 초과 시 원래 XY로 임의 Place하지 않습니다.
  빵 미검출 또는 XYZ 불안정이 이어져 마감 시각까지 안정 표본을 모으지 못한 경우에도 실패합니다.
- 관측 보호 진입 후에는 각 이동 뒤 컨트롤러 오류 코드를 확인하여, 오류(예: C31 접촉)가 보고되면
  해제·후퇴 등 다음 동작을 보내지 않습니다. (2026-11-06: 하강 직후 C31 후에도 해제·후퇴를 시도했음)
- 관측 보호 진입 후 실패하면 현재 그리퍼를 유지하고 자동 Home·Token 반납을 하지 않습니다.
  기존 `_place_observation_fault`가 후속 단독/협동 Task를 차단합니다.

파지한 양상추의 미끄러짐, 카메라 시야 가림, 바구니·도구 간섭은 현장에서 확인해야 하며,
이를 나타내는 기존 `observation_validated` 보호를 유지합니다. 단위 테스트는 실환경 검증을 대신하지 않습니다.

## 설정

아래는 `stack_place_vision.` 접두사를 사용하는 설정입니다.

| 이름 | 기본값/현재 설정 | 용도 |
|---|---|---|
| `enabled` | `false` | Agent1 양상추 보정 활성화 |
| `materials` | `[양상추]` | 허용 목록; 다른 Agent/재료로 적용 범위를 넓히지 않음 |
| `observation_validated` | `false` | 제자리 관측 회전의 시야·양상추 파지·간섭 현장 확인 |
| `tcp_frame` | Agent1 YAML: `agent1_tcp` | 기존 활성 SDK TCP에 해당하는 TF |
| `basket_half_extent_mm` | `[.nan,.nan]` | 빵 후보의 바구니 내부 XY 반폭 |
| `target_z_range_mm` | `[.nan,.nan]` (Agent1: `[55,85]`) | 빵 윗면 **robot_1_base 기준** Z 허용 범위 |
| `max_correction_mm` | `.nan` | nominal TCP XY 대비 최대 보정 거리 |
| `stability_mm` | `3.0` | 표본 사이 최대 XYZ 거리 |
| `samples` | `3` | 서로 다른 새 RGB/Depth 표본 수(2~8) |
| `min_confidence` | `0.6` | 빵 검출 최소 신뢰도 |
| `timeout_sec` | `5.0` | 새 관측 대기 한도 |
| `max_age_sec` | `1.0` | 촬영·수신 후 최대 경과 시간 |
| `sync_tolerance_sec` | `0.08` | RGB/Depth 최대 시각 차이 |

`.nan`은 미설정 값이며 실행을 거부합니다. 기존 nominal 양상추 XY Offset에서
실제 빵 중심까지의 거리도 `max_correction_mm`에 포함됩니다.
기존 action 속도, 재료 기울임 속도, settle 시간, Token, Safety Box 설정을 재사용합니다.
수동 카메라 XY/Z, 관측 transit 높이/속도 설정은 없습니다.

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

이번 검증 결과: **282 passed** (2026-11-06, 이동 후 컨트롤러 오류 중단·양상추 해제 여유 60 mm).
제자리 회전 후 카메라 광축이 관측점을 지나는지, TCP 위치가 그대로인지, 기울기 제한 시 같은 방향을
유지하는지, 회전 속도 제한, 회전 → 관측 → 기존 Place Action 순서와 보정 XY, 실제 TCP 읽기 실패 시
회전 전 중단을 검사합니다. 기존 검증 항목인 첫 프레임 후보 0개 후 3회 안정 검출·보정, 이상 좌표 후
표본 재수집, 5초 마감, RGB/Depth 중복 거부, settle 이후 촬영, 실패 영상 저장, Depth/TF 실패 시
원시 Box 표시, Token 상실·이동 실패 시 중단도 계속 포함됩니다. compileall 및 `git diff --check` 통과.
