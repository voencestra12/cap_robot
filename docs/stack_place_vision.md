# 공통 적층 위치 보정 — 구현 및 현장 검증 안내

새 기능은 두 Agent 모두 기본 OFF입니다. 실제 로봇 구동, SDK 연결, ROS2 launch는 수행하지 않았습니다.
미측정 기하 값은 `.nan`, TCP 프레임은 빈 문자열, 햄·빵 파지 중심 Offset은 `None`입니다.
기능을 ON으로 바꿔도 미설정 또는 미검증 상태면 실행을 거부합니다.

## A. 기존 전용 기능 정리

변경 전 `git status --short`, `git diff`는 모두 빈 출력이었습니다.
로컬 추적 파일 전체를 저장소 외부 `/tmp/cap_robot_before_stack_l7Vztptb/tracked_files.tar.gz`에 보관했습니다.
같은 디렉터리에 최초 status/diff도 저장했습니다. 원격 소스는 사용하지 않았습니다.

| 기존 구성 | 처리 |
|---|---|
| `cap_robot/place_verification.py` | 참조를 추적한 뒤 삭제, `cap_robot/stack_place.py`로 대체 |
| `needs_place_verification()` | `needs_stack_place()`: 기능·재료 목록·layer_index로 결정 |
| `check_place_bread_predecessors()` | `get_previous_layer_target()`: 동일 assembly의 직전 layer 및 SUCCEEDED 검사 |
| `plan_place_observation()` | `plan_stack_observation()`: 실제 TCP↔카메라 TF 사용 |
| `move_observation_path()` | `move_stack_observation_path()`: 기존 경로 사전검사 및 Token 보호 재사용 |
| `verify_place_target()` | `observe_previous_layer()`와 `prepare_stack_place()`로 관측/계산 연결 |
| `lettuce_place_vision.*` | 양쪽 YAML 및 선언에서 삭제, 공통 네임스페이스로 대체 |
| `baseline_calibrated`, `expected_bread_xy_offset_mm` 및 `[0,10]` | 삭제 |
| 수동 `observation_tcp_to_camera_mm` | 삭제, 활성 TCP에서 CameraInfo optical frame으로 TF 조회 |
| XYZ 손잡이 평균 기준 관측 Offset / 추가 상승 clearance | 카메라 XY 상대 위치, 카메라 절대 Z, 검증된 절대 transit Z로 대체 |
| 예상 빵 중심 대비 delta를 기존 TCP XY에 더하는 식 | 삭제, 관측한 중심에서 현재 파지 중심 Offset을 빼는 식으로 대체 |
| `test/test_place_verification.py` | `test/test_stack_place.py`로 대체, 공통 대상/회전/실패 회귀 확장 |
| `docs/lettuce_place_vision.md` | 이 문서로 대체 |

공유 카메라 callback, RGB/Depth 수신 시각, CameraInfo, 촬영 시각 TF 변환은 보존했습니다.
`place_verification_request`, `place_observations`는 기존 인식 스레드와 실행 스레드 사이의
범용 요청/큐로 재사용했습니다. 모듈 이름이나 양상추 전용 계산을 뜻하는 필드가 아닙니다.
`_place_observation_fault`와 cooperative Task 차단도 기존 수동 복구 보호를 유지하기 위해 남겼습니다.
ObjectMemory ID 관리, 모델 추론 설정, 모델 간 중복 제거 정책은 변경하지 않았습니다.

## B. 공통 API와 실행 순서

`stack_place.py`는 ROS 없이 검증 가능한 계산/검사 모듈입니다.

| 함수 | 역할 |
|---|---|
| `get_previous_layer_target()` | 직전 layer 재료·상태 확인, 동일 재료 이전 층 구분 불가 시 중단 |
| `material_center_offset()` | 재료별/Agent별 TCP 로컬 중심 Offset 조회, 미설정 거부 |
| `observation_pose()` | 원하는 카메라 원점에서 회전한 TCP→카메라 TF 이동량을 빼 TCP 계산 |
| `transit_waypoints()` | 검증된 높이까지 수직 상승 → 수평 이동 → 수직 도착 |
| `validate_frames()` | 정지 이후 촬영/수신, 신선도, 동기화, 정렬된 optical frame 검사 |
| `previous_layer_center()` | 클래스·XY 내부 범위·절대 Z·신뢰도 검사, 유일 후보 선택 |
| `calculate_stack_tcp_pose()` | 실제 중심과 회전한 파지 Offset으로 XY 산출, 기존 Z/자세 보존 |
| `corrected_place()` | 여러 새 프레임의 3D 안정성·최대 TCP XY 이동량 검사 후 평균 |

`robot_agent.py` 내부 API는 `prepare_stack_place(task, last_pick_pose, pick_rpy, action)`입니다.
이 함수가 `plan_stack_observation()`, `move_stack_observation_path()`,
`observe_previous_layer()`, `stack_release_rotation()`을 연결합니다.
`stack_camera_mount()`는 실제 활성 TCP↔카메라 TF만 사용하며 수동 장착 Offset fallback은 없습니다.
새 LLM 호출/API, Guidebook 필드, Task 종류는 추가하지 않았습니다.

실행 순서:

1. 기존 Pick 재확인, 접근, 파지, 상승을 그대로 실행합니다.
2. 첫 `move_to_place` 전에 기존 `wait_for_assembly_predecessors()`를 호출합니다.
3. layer 0, 비전 OFF 또는 대상 재료 목록 밖이면 기존 Place를 그대로 실행합니다.
4. ON 대상은 동일 assembly의 `layer_index - 1`이 SUCCEEDED인지 확인합니다.
5. 기존 Zone Token을 확보하고 두 파란 점을 새로 읽습니다. 긴 대기 뒤 오래된 파란 점도 거부합니다.
6. `get_basket_center_workspace()` → `basket_center_from_perception()`의 XY 평균을 관측 기준으로 사용합니다.
7. 기존 Pick 자세를 유지하며 TCP↔카메라 TF로 관측 위치를 구합니다. 모든 경유점의 TF/SDK Safety Box를 사전 검사합니다.
8. 검증된 transit 높이로 상승 → 수평 이동 → 관측 위치에 수직 도착합니다.
9. 이동 반환 뒤 기존 `perception_settle_sec` 이후 촬영·수신된 RGB/Depth를 수집합니다.
10. 빵→양상추→햄→빵 순서에서 관측 대상은 각각 빵, 양상추, 햄입니다. 그리퍼가 잡은 재료를 관측 대상으로 삼지 않습니다.
11. 기존 YOLO 중심 픽셀·Depth·`camera_point_to_workspace()`와 촬영 시각 TF로 이전 층 중심을 계산합니다.
12. 최종 XY와 기존 높은 접근·해제·후퇴 경유점을 사전 검사합니다. 관측값 신선도는 보정 이동 시작 직전 다시 확인합니다.
13. transit 높이에서 보정 XY로 정렬한 뒤 원래 첫 `move_to_place`와 후속 하강·해제·후퇴·자세 복원·Home을 실행합니다.

```text
R = 실제 Place 자세의 TCP 로컬 축 → workspace_0 회전
c = 새 프레임들에서 관측한 이전 층 중심
o = TCP 로컬 기준 현재 파지 재료 중심 Offset
최종 TCP XY = c.xy - (R @ o).xy
최종 TCP Z, yaw, Roll/Pitch, 그리퍼 = 기존 설정/실행 그대로
```

양상추의 `o=[0,0,0]` 가정에서는 최종 TCP XY가 실제 빵 중심 XY와 같습니다.
기존 `[0,-30]`은 ON 결과에 더하지 않습니다. 기존 nominal TCP는 최대 보정 거리 검사 기준으로만 사용합니다.
이 때문에 `max_correction_mm`는 기존 실험 Offset에서 새 중심으로 바뀌는 거리까지 고려해 실측해야 합니다.
카메라 장착 이동량은 관측 위치 계산에만 사용하며 workspace 중심에 다시 더하거나 빼지 않습니다.

현재 배치 실행의 세 자세 경로를 그대로 반영합니다:
기본 Base 자세, workspace Roll/Pitch 보정, 실측 Base 절대 `place_rpy_robot_deg`.
Agent2의 기존 SDK Base Z `-10mm`도 변경하지 않았습니다. 관측 위치 계획에서만 그 적용을 역산해
카메라가 지정한 절대 위치에 오도록 합니다. 실제 Place에는 기존 보정이 계속 적용되므로
현장 검증에서 TF/TCP/SDK 좌표의 일치도 함께 확인해야 합니다.

한 실행의 첫 Place에서 한 번만 좌표를 확정하고 접근·하강·후퇴가 동일한 XY를 재사용합니다.
새 프레임이 없거나 모호하면 원래 좌표로 임의 Place하는 fallback은 없습니다.
관측 진입 보호를 시작한 뒤 실패하면 그리퍼 유지, 자동 Home/Token 반납 금지,
후속 단독·협동 Task 차단을 유지합니다. 경유점과 기울임의 내부 이동 사이에도 Token/선행 상태를 확인합니다.
Token 상실은 진행 중인 blocking SDK 동작을 취소하는 기능이 아니라, **이후 이동/해제를 차단**하는 기능입니다.

동일한 이전 재료가 더 아래 층에도 존재하면 현재 공통 Z 범위만으로 층을 증명할 수 없어 보수적으로 중단합니다.
이 경우 layer별 실측 공간/높이 분리 검증을 추가하기 전까지 보정 실행을 허용하지 않습니다.
빵→양상추→햄→빵 레시피는 관측 대상의 이전 동종 중복이 없어 이 제한에 걸리지 않습니다.

## C. 파라미터 전체 목록

아래 ROS 파라미터는 모두 `stack_place_vision.` 접두사를 사용합니다.
두 YAML의 현재 값은 동일하지만 관측 기하/속도/TF는 Agent마다 따로 측정해야 합니다.
`.nan`은 미설정 double 값이며 유한수 검사에서 거부됩니다. 0mm 측정값을 뜻하지 않습니다.
설치된 ROS2 Humble의 C YAML parser로 scalar/array가 실제 double NaN으로 읽히는 것을 확인했습니다.

| 이름(접두사 생략) | 역할 | 현재 값 | 실측/검증 필요 | 사용자 수정 |
|---|---|---|---|---|
| `enabled` | 공통 보정 활성화 | `false` | 전체 검증 후 | 활성화 시 |
| `materials` | 현재 Place 재료 허용 목록 | `[양상추]` | 추가 재료 검증 | 햄/빵 추가 시 |
| `observation_validated` | 파지 자세·시야·높이·경로 확인 | `false` | 필수 | 완료 후에만 true |
| `tcp_frame` | SDK 활성 TCP와 동일한 실제 TF 프레임 | `""` | 필수 | 필수 |
| `observation_camera_xy_offset_mm` | 바구니 중심 대비 카메라 원점 XY, workspace 축 | `[.nan,.nan]` | 필수, Agent별 | 필수 |
| `observation_camera_z_mm` | 카메라 optical 원점 workspace 절대 Z | `.nan` | 필수, Agent별 | 필수 |
| `observation_transit_z_mm` | 관측 왕복 수평 이동 workspace TCP 높이 | `.nan` | 필수, Agent별 | 필수 |
| `observation_speed_mm_s` | 추가 관측/정렬 이동 속도 | `.nan` | 필수, Agent별 | 필수 |
| `basket_half_extent_mm` | 바구니 내부 중심 허용 XY 반폭 | `[.nan,.nan]` | 필수 | 필수 |
| `target_z_range_mm` | 이전 층 중심 workspace 절대 Z 최솟값/최댓값 | `[.nan,.nan]` | 필수 | 필수 |
| `max_correction_mm` | 기존 nominal TCP에서 새 TCP까지 최대 XY 거리 | `.nan` | 필수 | 필수 |
| `stability_mm` | 표본 간 최대 3D 거리 | `3.0` | 현장 노이즈 확인 | 필요 시 |
| `samples` | 서로 다른 최신 RGB/Depth 표본 수 | `3` | 현장 안정성 확인 | 필요 시, 2~8 |
| `min_confidence` | YOLO 최소 신뢰도 | `0.6` | 현장 인식 확인 | 필요 시 |
| `timeout_sec` | 정지/settle 이후 관측 대기 한도 | `5.0` | 처리 지연 확인 | 필요 시 |
| `max_age_sec` | 촬영·수신 프레임 최대 age | `1.0` | 처리 지연 확인 | 필요 시 |
| `sync_tolerance_sec` | RGB/Depth 촬영 시각 차이 한도 | `0.08` | 카메라 동기화 확인 | 필요 시 |

재료 설정은 `object_profiles.py`에 필드 하나만 추가했습니다.

| 재료/필드 | 역할 | 현재 값 | 실측 필요 | 사용자 수정 |
|---|---|---|---|---|
| 양상추 `stack_tcp_to_material_center_mm` | TCP 로컬 XYZ 파지 중심 Offset | `[0,0,0]` | 중심 파지 가정 검증 | 오차 확인 시 |
| 햄 `stack_tcp_to_material_center_mm` | 동일 | `None` | 필수 | 활성화 전 필수 |
| 빵 `stack_tcp_to_material_center_mm` | 동일 | `None` | 필수 | 마지막 빵 활성화 전 필수 |

단일 `[x,y,z]` 또는 `{"agent1": [...], "agent2": [...]}`를 지원합니다.
Agent별 dict에서 해당 Agent 값이 없거나 `None`이면 거부합니다.
기존 `perception_settle_sec`, TF timeout, 바구니 인식 max age, 기존 높은 Place 경유점,
Token 및 Safety Box 설정을 재사용합니다. 같은 의미의 새 설정은 만들지 않았습니다.

## D. 기존 기능 보존 증거

백업과 현재 파일의 byte/AST/YAML 비교를 수행했습니다.
기존 로봇 메서드 98개는 AST가 동일합니다. 기존 이름을 유지한 메서드 중 변경된 곳은
초기화의 보정 파라미터, `execute_task`의 보정 연결, `execute_place_action`의 ON 전용 안전 검사,
관측 Token 오류 문구, `run_perception`의 보정 요청 재료 선택뿐입니다.

- `verify_pick_target`, `resolve_task_assembly`, `wait_for_assembly_predecessors`, `assembly_placement` 보존.
- 파란 점 계산·HSV·ArUco·TF 캐싱·기존 TF/SDK 이동 변환·Home·그리퍼 명령 보존.
- LLM/프롬프트/Guidebook 생성/Claim/Agent 배정/상태 관리/병렬 정책/Zone Token Manager 보존.
- `llm_api.py`, `object_memory.py`, `detection_fusion.py`, `yolo_extra_perception.py`, 해당 YAML과 `prompts/` 무변경.
- 비전 블록을 제외한 양쪽 Agent YAML 값은 전부 동일. IP, dry_run, SDK Safety Box도 동일.
- 기존 모든 재료 필드와 상수 동일. 양상추 `[0,-30]`, grip 100, thickness 0, TCP Z 30 그대로.
- 백업의 실제 실행 메서드와 새 메서드를 mock으로 비교: 4층 × 2 Agent의 OFF 경로에서
  Task 결과, 이동 좌표/자세, 그리퍼, Home, SDK mock 호출이 모두 동일.

기존 `test_material_handling.py`는 공통 AST mock loader의 import/메서드 목록만 갱신했습니다.
기존 테스트의 기대값이나 로봇 설정은 바꾸지 않았습니다.

## E. 테스트 및 한계

실행 위치: `~/colcon_ws/src/cap_robot`.

```bash
python3 -m pytest -q
python3 -m compileall -q cap_robot test
python3 /tmp/audit_stack_preservation.py
/tmp/check_stack_yaml config/agent1.yaml /agent1/robot_agent_node
/tmp/check_stack_yaml config/agent2.yaml /agent2/robot_agent_node
git diff --check
```

수정 전: **190 passed, 95 subtests passed**.
수정 후: **229 passed, 0 failed, 95 subtests passed** (6.05초). 기존 대비 39개 증가했습니다.
Python compileall, YAML 파싱/ROS double 타입 확인, 보존 비교, `git diff --check` 모두 통과했습니다.
NumPy 1.26.4와 시스템 SciPy 요구 범위에 대한 기존 경고 1개는 환경을 변경하지 않고 유지했습니다.

테스트에는 일반 PnP/재료/Token/상태 회귀, 실제 관측 중심 정렬, 기존 XY Offset 이중 적용 방지,
모든 이전 층 클래스의 실제 인식 루프, Offset 회전, Agent별 Offset, 미설정 거부,
신선도/동기화/Depth/TF/신뢰도/복수 후보/범위/안정성 오류,
경유점 사전 검사, 수직·수평 분리, 기울임 내부 Token 상실, 실패 후 수동 복구 보호가 포함됩니다.
긴 Token 대기 테스트는 새 파란 점 수신을 모사하도록 수정했고 오래된 파란 점 거부도 별도로 검사합니다.

ROS Node 생성, SDK 연결, 실카메라, 실로봇, ROS2 launch는 수행하지 않았습니다.
실제 TCP 프레임 존재/정확성, 카메라 가림, 바구니 벽/재료 충돌, 관절 한계·특이점·전체 팔 간섭,
통신 및 Depth 품질은 검증하지 못했습니다. Safety Box는 TCP 경유점의 허용 범위 검사이며
팔 전체의 충돌 검증은 아닙니다. 단위 테스트 통과를 실장비 안전 판정으로 사용하지 마세요.

## F. 현장 실측 항목과 방법

1. **카메라 장착/활성 TCP 관계**: 현재 `handeye_agent*.yaml`은 link6→camera_link이며
   설명에 idealized 변환임을 명시합니다. SDK의 실제 활성 TCP 원점/축과 일치하는 TF 프레임이
   있는지 먼저 확인해야 합니다. link6를 TCP라고 가정하지 마세요. 없는 경우 기존 실제 TCP 설정을
   측정해 TF를 제공하는 별도 캘리브레이션이 필요합니다. 그 후 TCP→CameraInfo optical TF의
   단위·축·방향과 알려진 표식의 관측 위치를 비교합니다. 같은 장착 값을 YAML Offset으로 다시 넣지 않습니다.
2. **관측 위치·자세·안전 경로**: Agent별 기존 Pick 후 상승 자세를 유지한 상태에서 바구니 내부 이전 층이
   보이는 카메라 원점 XY/Z를 기록합니다. XY는 파란 점 중심과의 차이, Z는 workspace 절대값입니다.
   파지한 재료와 그리퍼가 시야/Depth를 가리지 않는지 확인합니다. transit 높이는 Pick 상승점,
   관측 TCP 및 기존 높은 Place보다 높고 바구니 벽·전체 팔·다른 로봇과 간섭 없는 값으로 검증합니다.
   속도는 해당 파지 상태의 흔들림/미끄러짐과 정지 안정성을 확인합니다. 이 검증은 이번 작업에서 수행하지 않았습니다.
3. **TCP→재료 중심**: 재현 가능한 기존 파지로 잡은 재료의 실제 중심을 외부 측정하고
   `o_tcp = R_workspace_tcp.T @ (center_workspace - tcp_workspace)`로 계산합니다.
   Place 자세로 회전했을 때 재료 변형·처짐·미끄러짐도 반복 측정합니다. 양상추는 0 가정을 확인하고,
   가장자리 파지인 햄/빵은 실측 XYZ를 넣습니다. 두 Agent의 파지 방향이 다르면 Agent별 dict를 사용합니다.
4. **바구니 내부 범위/이전 층 Z**: workspace 기준 내부 벽 좌표를 측정해 벽 안쪽에 들어가는 XY 반폭을 정합니다.
   YOLO가 사용하는 중심 픽셀의 Depth가 실제 각 이전 층 표면에서 갖는 Z와 노이즈를 기록하여 Z 범위를 정합니다.
   손잡이 평균 Z를 바닥/Place Z로 쓰지 않습니다. 검출 중심 범위 검사는 재료 전체 외곽의 충돌 판정이 아닙니다.
5. **허용 보정량/인식 안정성**: 정상 배치 시 기존 TCP XY와 새 TCP XY의 차이를 기록하고,
   양상추 기존 -30mm가 ON에서 빠지는 효과까지 포함하여 허용 최대 거리를 정합니다.
   confidence, 표본 간 3D 편차, RGB/Depth stamp 차이와 추론 지연을 로그로 확인합니다.
6. **활성화 순서**: 먼저 첫 빵→양상추만 검증합니다. 미설정값을 모두 채우고 경로/자세/시야 검증 후에만
   `observation_validated=true`, `enabled=true`를 설정합니다. 햄·마지막 빵은 Offset 및 해당 자세의 관측 경로를
   추가 검증한 뒤 `materials`에 추가합니다. 기능 ON만으로 미측정 Offset을 0으로 대체하지 않습니다.
