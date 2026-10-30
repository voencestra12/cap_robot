# 양상추 Place 직전 빵 중심 보정

코드 기본값은 비활성입니다. 현재 Agent YAML은 `enabled=true`이지만 새 관측 설정은
`observation_validated=false`라 SDK 연결 전에 시작을 차단합니다. 아래 XYZ와 자세는
장비에서 안전성이 확인되지 않았습니다. 이 변경은 실제 로봇 실행 없이 단위 테스트로만 검증했습니다.

## 기존 좌표 의미와 보정식

`assembly_placement()`는 파란 점 두 개로 구한 바구니 중심에 재료별
`place_xy_offset_mm`을 한 번 더해 workspace 해제 TCP 좌표를 만듭니다.
양상추 `[0, -30]`은 빵 가장자리 파지와 양상추 중앙 파지의 차이를 반영합니다.
`assembly_move_pose()`는 이 해제 좌표의 Z에 기존 접근/후퇴 여유 높이만 더합니다.

정상 빵의 **YOLO 중심**과 빵 해제 TCP 사이의 실측 관계는 기존 코드에 없습니다.
양상추 TCP와 YOLO 중심도 같다고 단정할 수 없습니다. 따라서 정상 빵 중심을
`바구니 중심 + expected_bread_xy_offset_mm`로 별도 교정합니다.
이 값은 정상 배치에서 측정해야 하며 `[0, -30]`을 자동 복사하지 않습니다.

```text
nominal_lettuce_xy = basket_xy + 기존 양상추 place_xy_offset_mm
expected_bread_xy  = basket_xy + 실측 expected_bread_xy_offset_mm
delta_xy           = 새 프레임들의 bread_xy 평균 - expected_bread_xy
final_lettuce_xy    = nominal_lettuce_xy + delta_xy
```

예: 기존 양상추 좌표 `(300,170)`, 정상 예상 빵 중심 `(302,175)`에서
실제 빵 중심이 `(312,185)`가 되면 최종 양상추 좌표는 `(310,180)`입니다.
기준과 일치하면 좌표를 유지합니다. Z, yaw, TCP 높이, 적층 두께, 그리퍼,
재료별 자세 및 Agent2의 기존 SDK Z 보정은 바꾸지 않습니다.

## 실행 시점과 실패 처리

기존 스키마의 `assembly.container=basket`, `material=양상추` 작업에 적용합니다.
동일 조립의 이전 빵 층이 있어야 하고 모두 `SUCCEEDED`여야 합니다.
별도 sandwich task-type 필드는 추가하지 않습니다.

1. 기존 Pick, 그리퍼 파지, 안전 상승을 완료합니다.
2. `wait_for_assembly_predecessors()`와 같은 조립의 선행 빵 `SUCCEEDED` 검사를 수행합니다.
3. 관측 경로용 공용 구역 Token을 확보합니다. 기존 Pick 위치 자체가 공용 구역이면
   기존 규칙대로 Pick 전부터 보유합니다. Token/Safety Box 비활성 시 관측 작업은 거부합니다.
4. 아래 카메라 장착 오프셋을 반영해 관측 TCP를 계산합니다. Pick 상승 위치에서
   workspace 수직 상승 → 일정 높이 수평 이동 → 관측 위치 수직 진입으로 이동합니다.
   기존 첫 `move_to_place(z_offset=200)`은 이 단계에서 실행하지 않습니다.
5. 기존 SDK `wait=True` 이동이 성공한 뒤 정지/settle 시각을 새로 기록합니다.
   `perception_settle_sec` 이후 촬영 및 수신된 RGB + aligned Depth만 사용합니다.
   촬영 시각의 TF, CameraInfo/frame 일치, 시각 동기화, freshness, Depth 유효성을 검사합니다.
   기존 YOLO/Depth 역투영/중복 제거를 사용하고 과거 저장 좌표로 대체하지 않습니다.
6. 실측 바구니 내부에 빵이 정확히 하나이고 서로 다른 새 프레임들의 중심이 안정적일 때,
   기존 양상추 `[0,-30]`에 빵 이동량만 한 번 더합니다. Z/yaw/TCP/그리퍼/Roll은 유지합니다.
7. 보정된 모든 Place 경유점을 사전 검증합니다. 관측 위치에서 다시 수직 상승하고,
   관측 및 높은 Place 위치보다 높은 높이에서 보정된 X/Y로 수평 이동합니다.
   TF/Safety Box 사전 검증을 마친 **첫 보정 이동 직전** freshness를 재검사합니다.
   이후 같은 보정 좌표를 유지하며, 카메라가 관측 위치를 떠난 뒤 재인식하지 않습니다.
8. 원래 첫 높은 Place 접근부터 수직 이동하고 기존 하강, 그리퍼 해제, 후퇴를 수행합니다.
   재료별 높은 위치 자세 변경과 복원, 기존 홈 복귀도 유지합니다.

관측 진입/이탈 각 선분에 기존 공용 구역 기하 판정을 적용하고 Token을 보수적으로
전체 구간에 요구합니다. 각 경유점은 기존 workspace→Base→SDK 및 Agent2 `-10mm`
보정을 거쳐 Safety Box를 검사합니다. 직선 선분의 양 끝이 볼록한 Safety Box 내부이면
그 선분도 내부입니다. 이는 **TCP 위치 검사**이며, 팔 링크/그리퍼/재료의 충돌 및
관절 가동 범위나 실제 경로의 안전성을 보증하지 않습니다.

기존 `latest_poses`나 `ObjectMemory`의 저장값을 Place 검증에 읽지 않습니다.
`ObjectMemory.update()` 자체도 현재 검출만 반환하지만, 새 검증은 그것과 분리된
요청별 관측 큐를 사용합니다. 카메라 callback에서 실제 수신 시각을 기록하므로
일반 `latest_poses_stamp`도 추론 시작 시각 대신 RGB 수신 시각을 가집니다.

검출 누락/복수/불안정/과도한 보정, Depth/TF 오류 또는 timeout은 Task 실패로 끝납니다.
관측 진입 이후 실패하면 자동 하강/해제/홈 복귀를 하지 않고 현재 그리퍼 상태를 유지합니다.
팔이 바구니 근처에 남을 수 있으므로 Token을 자동 반납하지 않고 기존 heartbeat를 유지하며,
해당 Agent의 후속 일반/협동 Task를 차단합니다. 성공 후 홈 복귀 완료 시에는 기존처럼 반납합니다.
Token을 이미 상실했다면 보유를 복구했다고 가정하지 않고 추가 동작을 중단합니다.

실패 복구는 작업자가 두 로봇과 파지물 위치를 확인하고, 다른 Agent의 진입을 막은 상태에서
수행해야 합니다. 팔이 공용 구역에서 벗어난 것을 확인한 뒤 관리 절차로 Token을 반납하고
노드를 재시작해 로컬 실패 latch를 해제합니다. 원인 해결 전 재시작/lease 만료를 안전 복구로
간주하면 안 됩니다. 자동 복구 이동이나 임의 Place 좌표는 제공하지 않습니다.

## 실측 및 활성화

`config/agent1.yaml`, `config/agent2.yaml`의 `lettuce_place_vision.*`를 설정합니다.
파라미터는 시작할 때 읽으므로 변경 후 재빌드/재시작이 필요합니다.

- `expected_bread_xy_offset_mm`: 정상적으로 놓인 빵의 YOLO 중심에서 동일한
  파란 점 바구니 중심을 뺀 workspace_0 `[dx,dy]`(mm).
  두 카메라의 관측 오차와 빵을 배치하는 Agent 조합을 확인해 Agent별로 실측합니다.
  현재 YAML의 `[0,10]`은 기존 값을 유지했으며, 이번 변경에서 실측 확인하지 않았습니다.
- `basket_half_extent_mm`: 파란 점 중심을 기준으로 **실제 바구니 내부에 완전히
  포함되는** workspace 축 방향 직사각형 반폭 `[hx,hy]`.
  바구니가 회전했으면 외부가 포함되지 않도록 보수적으로 줄여야 합니다.
- `bread_z_range_mm`: 바구니에 놓인 빵 중심의 workspace_0 절대 Z 범위.
  손잡이 평균 Z나 TCP Z를 사용하지 않습니다.
- `max_correction_mm`: XY 이동량의 유클리드 거리 상한, 기본 20mm.
- `stability_mm`: 모든 관측 쌍의 3D 거리 상한, 기본 3mm.
- `samples=3`, `min_confidence=0.6`, `timeout_sec=5.0`, `max_age_sec=1.0`,
  `sync_tolerance_sec=0.08`은 카메라 FPS·모델 추론 시간·TF 지연을 측정해 검증합니다.
  첫 관측부터 마지막 관측 및 첫 보정 이동 직전까지 모두 freshness 한도 안이어야 합니다.

정상 배치 기준과 내부 영역, 다음 관측 경로 검증을 모두 완료한 뒤 활성화합니다.

```yaml
lettuce_place_vision.baseline_calibrated: true
lettuce_place_vision.observation_validated: true
lettuce_place_vision.enabled: true
```

`enabled=true`이지만 `baseline_calibrated=false`이면 로봇 연결 전에 설정 오류로
종료합니다. 인식 비활성 또는 workspace_0 이외의 좌표계에서도 보정 Place를 거부합니다.

### 관측 위치 설정과 장비 검증 절차

| 파라미터 | 좌표 의미 |
| --- | --- |
| `observation_validated` | XYZ, 자세, 파지물 시야 및 전체 경로의 장비별 검증 완료 표시. 기본 `false` |
| `observation_camera_offset_mm` | 두 파란 점의 **XYZ 평균**에서 원하는 카메라 optical 원점까지의 workspace 축 `[dx,dy,dz]` |
| `observation_tcp_to_camera_mm` | 현재 활성 TCP 원점에서 카메라 optical 원점까지의 **TCP 로컬 축** 장착 이동량 |
| `observation_transit_clearance_mm` | 시작점/관측점/높은 Place의 최대 workspace Z보다 추가로 올리는 여유. 실측한 양수 필요 |
| `observation_speed_mm_s` | 관측 및 보정 X/Y 정렬 이동 속도. 기본 30 mm/s도 안전하다고 검증된 값은 아님 |

벡터 기본 `[0,0,0]`과 clearance `0`은 실행 가능한 예시가 아닌 미설정 자리표시자입니다.
새 절대 XYZ나 임의 카메라 각도는 코드에 넣지 않습니다. 바구니 중심 Z는 손잡이 평균이며
바닥 Z가 아닙니다. 관측 자세는 해당 Agent의 **기존 Pick Base RPY**를 그대로 유지합니다.
카메라 optical 자세는 TCP 자세와 실제 장착 회전으로 결정되며, 별도로 Roll을 바꾸지 않습니다.

```text
camera_workspace = basket_xyz + observation_camera_offset_mm
R_workspace_tcp  = transpose(R_base_workspace) × R_base_tcp_pick
observation_tcp  = camera_workspace - R_workspace_tcp × observation_tcp_to_camera_mm
transit_z        = max(start_z, observation_z, high_place_z) + transit_clearance
```

위 카메라 목표는 기존 Z 보정 적용 **전 TF 기준**입니다. Agent2의 기존 SDK `-10mm`를
관측에도 그대로 적용하므로 실제 카메라 원점은 Base Z 방향으로 10mm 내려갑니다.
설정 측정과 시야/간섭 검증에 이 차이를 포함해야 하며, 기존 Place Z 보정값은 바꾸지 않습니다.

1. 노드를 실행하지 않은 상태에서 Agent별 활성 TCP, TCP→optical 장착 이동/회전,
   Base↔workspace TF와 기존 Pick RPY를 확인합니다. URDF/측정 결과의 축과 단위를 대조합니다.
2. 바구니 중심과 내부/빵 높이 범위를 실측하고, 그리퍼에 양상추를 든 상태를 고려해
   카메라 목표 상대 XYZ 및 경로 여유를 정합니다. 카메라 intrinsics와 장착 회전으로
   빵/바구니가 시야 안에 있고 유효 Depth 거리 내인지 검토합니다.
3. Pick 상승점→관측점→허용 보정 범위 전체의 높은 접근점에 대해 각 경유점의
   TF/SDK 좌표와 Safety Box, 공용 구역 교차를 확인합니다. 팔 링크와 파지물 전체의
   swept volume, 바구니 벽/손잡이/다른 팔, 관절 한계를 장비 도면 또는 시뮬레이션으로 검토합니다.
   높은 경로라는 이유만으로 안전하다고 간주하지 않습니다.
4. 별도 승인된 현장 검증 절차에서, 설정한 자세로 **정지한** 카메라 영상과 Depth를
   양상추 파지 전후 비교합니다. 로그의 관측 TCP/Base 자세/장착 오프셋,
   RGB/Depth stamp, `시야/가림 진단`의 영상 크기·빵/양상추 픽셀 중심·bbox·Depth·confidence·
   촬영 시각 camera TF를 대조합니다. 빵이 사라지는지, 양상추 bbox와 겹치는지,
   Depth가 가까운 양상추/그리퍼 표면을 읽는지 확인합니다.
5. 로그만으로 가림을 확정하지 않습니다. 그리퍼 검출 모델은 추가하지 않았고,
   부분 가림에 의한 안정적인 중심 편향도 있을 수 있으므로 실제 RGB/Depth 확인이 필요합니다.
   현장 확인 전에는 `observation_validated=false`를 유지합니다.

이 작업에서는 위 현장 확인이나 로봇 구동을 수행하지 않았습니다.

빌드 예시(이 문서 작업 중 실행하지 않음):

```bash
cd ~/colcon_ws
colcon build --packages-select cap_robot --symlink-install
source install/setup.bash
```

실제 구동 전, 양상추를 들어 올린 정지 자세에서 빵이 보이는지, Depth가 그리퍼나
양상추 표면을 읽지 않는지, 카메라 timestamp와 ROS 시계가 일치하는지,
촬영 시각의 TF가 제공되는지 확인해야 합니다. 부분 가림에 의한 YOLO 중심 편향은
confidence/다중 프레임 안정성만으로 완전히 판별할 수 없으므로 실험 검증이 필요합니다.

현재 Agent YAML은 `dry_run: false`이며, Agent는 dry_run이어도 생성자에서
`XArmAPI`에 연결합니다. 하드웨어 없는 검증에는 아래 단위 테스트를 사용합니다.
이 작업에서는 노드 실행/launch/SDK 연결을 하지 않았습니다.

## 단위 테스트

```bash
cd ~/colcon_ws/src/cap_robot
python3 -m pytest -q test/test_place_verification.py test/test_material_handling.py \
  test/test_object_memory.py test/test_detection_fusion.py test/test_shared_zone.py \
  test/test_zone_token.py test/test_llm_api.py test/test_ollama_stream.py
```

실제 Agent 메서드 본문을 AST로 로드하고 카메라·YOLO·ROS·SDK를 mock합니다.
정상 좌표, 이동량만 적용, Offset 비중복, 미검출/복수/외부 후보, 오래된/중복 프레임,
불안정/과도한 이동, Depth/TF 오류, 양쪽 Agent 좌표 변환, Safety Box,
관측 전 Token 확보/상실, 카메라 장착 오프셋 회전, 수직/수평 경로 분리,
미검증 설정 차단, 실패 후 Token 유지와 후속 작업 차단 및 기존 재료 동작을 검증합니다.
이번 변경의 회귀 테스트: 179개 및 하위 검증 80개 통과. Python 문법, YAML의
미검증 설정 차단, `git diff --check`도 확인했습니다. 테스트는 실제 ROS 노드나
SDK 연결을 만들지 않습니다. 환경의 SciPy/NumPy 버전 경고 1종이 남아 있습니다.
