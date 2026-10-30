# 양상추 Place 직전 빵 중심 보정

기본값은 비활성화이며 실제 로봇을 실행하지 않고 단위 테스트로 검증한 기능입니다.

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

1. `execute_task()`가 기존 Pick/파지/안전 상승을 성공시킵니다.
2. 기존 `wait_for_assembly_predecessors()`가 성공을 확인합니다.
3. 첫 `move_to_place` 전에 `verify_place_target()`이 잠시 `is_moving=False`로 전환합니다.
   그 외 이동 중에는 기존처럼 위치 갱신을 제한합니다.
4. `perception_settle_sec` 이후 촬영되고 수신된 RGB와 aligned Depth만 사용합니다.
   기존 카메라 토픽·YOLO 모델·Depth 역투영·모델 간 중복 제거를 재사용합니다.
   RGB/Depth timestamp 차이, 두 프레임의 freshness, CameraInfo/영상 frame,
   depth 유효성 및 **RGB 촬영 시각**의 `workspace_0 <- camera` TF를 검사합니다.
   최신 TF나 이전 좌표로 대체하지 않습니다.
5. 바구니의 실측 내부 XY 영역 및 절대 Z 범위에서 빵 후보가 정확히 하나여야 합니다.
   ID 대신 클래스를 사용하며 기본 3개의 서로 다른 새 RGB/Depth 관측을 요구합니다.
   후보 누락/복수/낮은 confidence/Depth·TF 오류는 즉시 실패합니다.
   빵의 Depth가 무효라 내부/외부를 판별할 수 없는 경우도 보수적으로 실패합니다.
   오래되거나 비동기인 프레임은 사용하지 않고 제한 시간 내 새 프레임을 기다립니다.
6. 관측 간 3D 편차와 각 관측의 XY 보정량 상한을 검사하고 XY만 한 번 변경합니다.
   보정한 접근/하강/후퇴 경유점을 기존 Agent별 Base 변환과 SDK Safety Box로 사전 검사합니다.
   실제 이동에서도 기존 Safety Box 검사가 그대로 적용됩니다.
7. 공용 구역 경로를 다시 판정하고 기존 토큰 획득/상실 검사/반납 경로를 사용합니다.
   토큰 대기 후 관측이 오래되면 첫 Place도 중단합니다. 이후 경유점은 같은 XY를 유지합니다.

기존 `latest_poses`나 `ObjectMemory`의 저장값을 Place 검증에 읽지 않습니다.
`ObjectMemory.update()` 자체도 현재 검출만 반환하지만, 새 검증은 그것과 분리된
요청별 관측 큐를 사용합니다. 카메라 callback에서 실제 수신 시각을 기록하므로
일반 `latest_poses_stamp`도 추론 시작 시각 대신 RGB 수신 시각을 가집니다.

검증 실패는 기존 Task 실패 경로로 이어지며 자동 하강, 그리퍼 해제, 홈 복귀,
재시도/재할당을 추가하지 않습니다. 이미 확보한 토큰은 기존 finally에서 반납합니다.

## 실측 및 활성화

`config/agent1.yaml`, `config/agent2.yaml`의 `lettuce_place_vision.*`를 설정합니다.
파라미터는 시작할 때 읽으므로 변경 후 재빌드/재시작이 필요합니다.

- `expected_bread_xy_offset_mm`: 정상적으로 놓인 빵의 YOLO 중심에서 동일한
  파란 점 바구니 중심을 뺀 workspace_0 `[dx,dy]`(mm).
  두 카메라의 관측 오차와 빵을 배치하는 Agent 조합을 확인해 Agent별로 실측합니다.
  YAML의 `[0,0]`은 자리표시자입니다.
- `basket_half_extent_mm`: 파란 점 중심을 기준으로 **실제 바구니 내부에 완전히
  포함되는** workspace 축 방향 직사각형 반폭 `[hx,hy]`.
  바구니가 회전했으면 외부가 포함되지 않도록 보수적으로 줄여야 합니다.
- `bread_z_range_mm`: 바구니에 놓인 빵 중심의 workspace_0 절대 Z 범위.
  손잡이 평균 Z나 TCP Z를 사용하지 않습니다.
- `max_correction_mm`: XY 이동량의 유클리드 거리 상한, 기본 20mm.
- `stability_mm`: 모든 관측 쌍의 3D 거리 상한, 기본 3mm.
- `samples=3`, `min_confidence=0.6`, `timeout_sec=5.0`, `max_age_sec=1.0`,
  `sync_tolerance_sec=0.08`은 카메라 FPS·모델 추론 시간·TF 지연을 측정해 검증합니다.
  첫 관측부터 마지막 관측 및 첫 Place까지 모두 freshness 한도 안이어야 합니다.

정상 배치 기준과 내부 영역을 실측한 뒤 아래 두 값을 설정합니다.

```yaml
lettuce_place_vision.baseline_calibrated: true
lettuce_place_vision.enabled: true
```

`enabled=true`이지만 `baseline_calibrated=false`이면 로봇 연결 전에 설정 오류로
종료합니다. 인식 비활성 또는 workspace_0 이외의 좌표계에서도 보정 Place를 거부합니다.

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
토큰 재판정/상실/대기 중 freshness 만료 및 기존 재료 동작을 검증합니다.
기존 테스트의 Agent2 Z 기대값만 현재 구현의 `-10mm`에 맞췄습니다.
기존 `-50mm` 기대값 실패는 수정 전 HEAD 코드에서도 재현했습니다.

검증 결과: 위 회귀 테스트 159개 및 하위 검증 80개 통과. ROS Humble 환경의
`test/test_yolo_extra_perception.py`도 11개 및 하위 검증 15개 통과했습니다.
Python 문법 컴파일, YAML 기본값 일치, `git diff --check`도 확인했습니다.
환경의 SciPy/NumPy 권장 버전 불일치 경고 1종이 있으나 테스트 실패는 없습니다.
