# cap_robot merged (ROS 2 Humble)

`cap_robot`의 기존 실행 구조를 유지하면서, 첫 번째 코드 세트의 빨간 손잡이 인식과
양팔 바구니 협동 기능을 통합한 패키지입니다. 두 코드 세트를 동시에 실행하지 않습니다.

## 배치 구조

| 컴퓨터 | 실행 | 역할 |
|---|---|---|
| 연구실 컴퓨터 | `ros2 launch cap_robot workstation.launch.py` | 고정 RealSense, ArUco TF, 손잡이 인식, **공용 구역 토큰 매니저(`zone_token_manager`)** |
| 연구실 컴퓨터 | `ros2 run cap_robot workstation_llm` | 자연어 해석, 전체 계획 작성, Agent 검토 수집, 최대 1회 재계획 |
| 노트북 1 | `ros2 launch cap_robot agent1.launch.py` | Agent 1 LLM, YOLO perception, xArm6(192.168.1.218) 실행 |
| 노트북 2 | `ros2 launch cap_robot agent2.launch.py` | Agent 2 LLM, YOLO perception, xArm6(192.168.1.198) 실행 |

모든 컴퓨터에서 같은 `ROS_DOMAIN_ID`와 호환되는 DDS 설정을 사용해야 합니다.

## 병합 동작

### 일반 Pick-and-Place

기존 흐름을 유지합니다.

1. Workstation LLM이 의미적 Guidebook과 의존성을 작성합니다.
2. READY Task마다 각 Agent LLM이 자신의 perception으로 실행 후보를 만듭니다.
3. 기존 claim 방식으로 실행 Agent를 결정합니다.
4. `/agent_task` 경로와 기존 TF/안전 상자/xArmAPI 실행기를 사용합니다.

### 바구니 양팔 협동

1. `yolo_extra_perception`이 고정 카메라의 빨간 스티커 두 개를 손잡이로 검출합니다.
2. 손잡이 좌표와 각 Agent의 접근 가능성을 `/perception/yolo_extra`에 JSON으로 발행합니다.
3. Workstation LLM이 두 손잡이 배정과 전체 공유 `actions`를 작성합니다.
4. 두 Agent LLM이 계획을 각각 승인 또는 거부합니다.
5. 하나라도 거부하면 Workstation LLM이 전체 계획을 **한 번만** 다시 작성합니다.
6. 승인 후 각 action에서 두 Agent가 `READY → GO → DONE/FAILED` barrier를 통과합니다.
7. 파지 후 이동은 `cooperative_move_relative`만 허용됩니다.

동기화는 ROS 메시지 기반의 단계 barrier이며 실시간 제어 버스는 아닙니다. 두 노트북의
네트워크 지연 편차가 크면 `step_start_delay_sec`를 늘리십시오.

### 공용 구역(A+B) 충돌 방지 — 토큰(Mutex)

두 로봇이 `depends_on` 없이 병렬로 움직일 때 중앙 workspace에서 부딪히는 것을 막기
위해, **공용 구역 점유를 런타임에 직렬화**합니다. 안전 보장은 LLM 출력과 무관하며
`robot_agent`가 `workspace_0` 좌표로 결정론적으로 판정합니다.

- **구역 정의**: `SHARED_ZONE` 사각형(`workspace_0` 절대좌표, mm). 기본값
  `x∈[150,450]`, `y∈[-50,500]`. `agent*.yaml`의 `shared_zone_x_min/x_max/y_min/y_max`,
  `shared_zone_margin_mm`로 조정합니다. A-only / B-only 구역은 자유 진입.
- **판정**(`task_shared_zone_plan`): pick 지점이 구역 내부면 첫 `move_to_object` 전,
  place 지점이 내부이거나 pick→place 직선이 구역을 통과하면 첫 `move_to_place` 전에
  토큰을 확보합니다. 구역에 닿지 않는 Task는 토큰을 전혀 쓰지 않아 완전 병렬로 실행됩니다.
- **Pick**: 각자 구역에서 이뤄지면 토큰 불필요 → 두 로봇 동시 집기 가능.
- **Dual(협동)**: `cooperative_move_relative` 등 양팔 동시 이동은 이미 step barrier로
  동기화되므로 토큰 불필요. 단, coordinator(`min(participants)`)가 belt-and-suspenders로
  협동 구간 전체 토큰을 확보하고, 비-coordinator는 토큰을 요청하지 않습니다(barrier
  상호 대기 데드락 방지).
- **매니저**(`zone_token_manager`, 시스템 전체 1개): `holder` 1명 + FIFO 대기열.
  반납 시 대기 중인 상대에게 우선 이전(공정 큐), 재요청자는 큐 뒤로. grant마다 `epoch`가
  증가하여 lease 회수 후 지각 반납은 무시됩니다.
- **lease / 하트비트**: grant에 `lease_sec`(기본 45초) 만료 시각이 붙고, 보유 중인
  Agent는 `lease_sec/3`마다 재-acquire로 갱신합니다. 홀더 프로세스가 죽으면 lease 만료로
  자동 회수됩니다. 획득 실패(`zone_token_acquire_timeout_sec`, 기본 60초)·모션 중 토큰
  상실 시 해당 Task는 즉시 실패 처리되고 자동 복구 이동은 하지 않습니다.
- **끄기**: `zone_token_enabled: false`로 게이트 전체를 비활성화할 수 있습니다.

> `agent*.yaml`의 `zone_token_lease_sec`와 `workstation.launch.py`의
> `zone_token_manager` `lease_sec`을 같은 값으로 유지하십시오.

## 주요 토픽

| 토픽 | 형식 | 용도 |
|---|---|---|
| `/mission/guidebook` | `std_msgs/String` JSON | Workstation 전체 계획 |
| `/mission/task_claim` | `std_msgs/String` JSON | 일반 Task claim |
| `/mission/task_status` | `std_msgs/String` JSON | `plan`, `task`, `step` 상태 |
| `/agent_task` | `std_msgs/String` JSON | 일반/협동 실행 메시지 |
| `/perception/yolo_extra` | `std_msgs/String` JSON | 손잡이 및 Agent별 접근성 |
| `/basket/handle_0_pose` | `geometry_msgs/PoseStamped` | workspace_0 기준 손잡이 0(m) |
| `/basket/handle_1_pose` | `geometry_msgs/PoseStamped` | workspace_0 기준 손잡이 1(m) |
| `/zone_token/request` | `std_msgs/String` JSON | 공용 구역 토큰 `acquire`/`release`/`abandon` (Agent → 매니저) |
| `/zone_token/state` | `std_msgs/String` JSON (latched) | 현재 `holder`, `queue`, `epoch`, `lease_remaining_sec` (매니저 → 전체) |

## 빌드

```bash
cd ~/capstone_ws/src
# 이 cap_robot 디렉터리를 여기에 배치
cd ~/capstone_ws
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install ultralytics xArm-Python-SDK
colcon build --packages-select cap_robot --symlink-install
source install/setup.bash
```

Ollama에는 연구실 컴퓨터의 Workstation 모델과 각 노트북의 Agent 모델이 준비되어야
합니다. 기본값은 각각 `gemma4:12b`, `gemma4:e4b`이며 YAML/ROS 파라미터로 변경할 수
있습니다.

Agent LLM의 thinking은 각 `agent*.yaml`의 `llm_think`로 제어합니다. 실행 중에도 다음
명령으로 변경할 수 있으며, 변경값은 다음 LLM 요청부터 적용됩니다.
`true`이면 Ollama의 추론 토큰과 최종 JSON 응답이 각 노드를 실행한 터미널에 실시간으로
분리 출력됩니다. Workstation LLM도 기본적으로 thinking이 켜져 있습니다.

```bash
# Agent 1 thinking 켜기/끄기
ros2 param set /agent1/robot_agent_node llm_think true
ros2 param set /agent1/robot_agent_node llm_think false

# Agent 2 thinking 켜기/끄기
ros2 param set /agent2/robot_agent_node llm_think true
ros2 param set /agent2/robot_agent_node llm_think false

# Workstation thinking 켜기/끄기
ros2 param set /workstation_llm llm_think true
ros2 param set /workstation_llm llm_think false
```

## 빵·양상추·햄 모델 통합

두 Agent는 `models/yolo11m-seg.pt`와 `models/yolo-bread-lettuce-ham.pt`를 함께
로드합니다. 추가 모델은 segmentation 모델이며 클래스는 `0: bread`, `1: lettuce`,
`2: ham`입니다. 인식 결과는 각각 **빵**, **양상추**, **햄**으로 기존 Agent
perception에 들어갑니다.
마스크와 정렬 depth로 구한 위치·yaw를 `workspace_0` 좌표로 변환하고,
기존 `latest_poses`와 `current_detected_items`를 통해 Agent LLM의 target 및
relative_object reference 선택에 사용합니다. 각 모델에는 시각화 전 원본 영상을 입력합니다.

모델은 패키지 share 디렉터리 기준 상대 경로로 로드합니다. 새 모델을 배치한 뒤
각 Agent 실행 환경에서 아래 명령으로 설치 목록과 설정을 갱신하세요.

```bash
cd ~/capstone_ws
colcon build --packages-select cap_robot --symlink-install
source install/setup.bash
```

시작 로그의 `YOLO model loaded`에서 새 파일명과 두 클래스를 확인하고,
Agent 카메라 화면에서 `Bread`, `Lettuce`와 좌표 변환 후 표시되는 yaw를 확인합니다.
재료 인식은 각 Agent의 카메라에서 수행하며, `/perception/yolo_extra`는 기존
고정 카메라의 바구니 손잡이 인식용입니다.

### 여러 물체의 개별 기억

같은 종류의 물체도 `빵_1`, `빵_2`, `햄_1`처럼 개별 ID로 `latest_poses`에 저장하며,
`current_detected_items`와 Agent LLM에도 전체 ID 목록 및 개체별 좌표를 전달합니다.
화면의 `Bread_1`, `Bread_2` 표기로 각각 확인할 수 있습니다. 깊이/TF 변환이
성공한 물체만 실행 후보에 들어갑니다.

이전 workspace 위치와 같은 클래스의 새 검출을 일대일로 매칭하므로 검출 순서가
바뀌어도 ID를 유지합니다. 기본 매칭 거리는 60 mm이며, 잠시 가려진 물체의 ID는
마지막 관측 후 3초간 기억합니다. 미검출 물체의 과거 좌표는 현재 실행 후보에서
제외합니다. `agent*.yaml`의 `object_match_distance_mm`,
`object_memory_retention_sec`로 조정할 수 있습니다.

LLM의 `target`과 `relative_object.reference`는 정확한 ID를 선택합니다. 클래스
이름만 받은 구버전 입력은 해당 종류가 하나일 때만 허용하며, 여러 개면 임의 선택하지
않습니다. `빵_2`에도 빵의 그리퍼·배치 자세를 적용하며 `assembly.material`은
계속 `빵`처럼 종류 이름을 사용합니다.

ID는 Agent별 프로세스 내부 값이며 재시작 시 초기화됩니다. 기억 시간이 지나거나
물체가 매칭 거리보다 크게 이동하면 새 ID가 붙습니다. 가까운 동종 물체가 교차하거나
겹치는 경우의 동일성, Agent 간 동일 물체 ID 공유, 집어 옮긴 물체의 재식별은
보장하지 않습니다. 실제 검출 품질과 파지 자세는 카메라·로봇 환경에서 확인해야 합니다.

## 실행 전 점검

```bash
ros2 topic echo /perception/yolo_extra --once
ros2 run tf2_ros tf2_echo robot_1_base workspace_0
ros2 run tf2_ros tf2_echo robot_2_base workspace_0
ros2 topic info /mission/task_status -v
ros2 topic echo /zone_token/state --qos-durability transient_local --once
```

`/zone_token/state`가 `holder: null`로 한 번 이상 발행되어야 `zone_token_manager`가
정상 기동한 것입니다. 이 노드가 없으면 공용 구역을 지나는 Task마다
`zone_token_acquire_timeout_sec`만큼 대기 후 실패합니다.

`/perception/yolo_extra`의 `valid`가 `true`이고 두 손잡이 및 두 Agent의
`within_safety_box`가 올바른지 확인한 다음 협동 명령을 입력하십시오.

> 중요: 제공된 `agent1.yaml`과 `agent2.yaml`은 기존 실기 설정을 유지해
> `dry_run: false`입니다. 최초 통합 시험에서는 두 파일을 `dry_run: true`로 바꾸고
> 계획·TF·좌표·동기화 로그를 검증한 뒤 실기 모드로 전환하십시오.

## 손잡이 인식 주의사항

노드 이름은 요청대로 `yolo_extra_perception`이지만, 현재 구현은 첫 번째 코드 세트의
기능을 보존해 **빨간 스티커 HSV 분할 + 정렬 depth**를 사용합니다. ArUco 검출은
`aruco_calib`에만 남아 있고, 두 노드는 동일 RealSense ROS 스트림을 구독하므로 장치를
중복으로 직접 열지 않습니다. 조명이나 스티커 색이 바뀌면 HSV 임계값을 파라미터화하거나
학습된 손잡이 모델 backend를 추가해야 합니다.

## 제거된 R1~R9

- Agent 직접 자연어 명령 경로와 `agent_policy.txt`
- 실행 코드에서 쓰이지 않던 coordination/startup 설정
- ArUco의 `/robot_1_target`, `/robot_2_target` 계산·발행
- `/mouse_target_pose` 디버그 발행
- 자동 안전대피, 자동 대체 Agent 재할당, legacy retry
- 단독 `aruco_calib.launch.py`
- 사용되지 않던 utility 함수
- identity 이외의 xArm command frame 모드
- 비-TF workspace 실행 모드

첫 번째 세트의 `pick_dynamic`, 별도 brain/executor 노드, `/dual_arm_sync`, robot 2 Y축
강제 반전, 최대 3회 자동 재시도는 포함하지 않았습니다.

## 검증 범위

- `test/test_llm_api.py` — 일반/협동 action 상태기와 금지 API (레거시 토큰 API는 예외
  없이 무시되는지 포함)
- `test/test_zone_token.py` — 토큰 코어(`ZoneTokenCore`): 획득/대기열/공정 이전/epoch/
  lease 회수/하트비트/`abandon`
- `test/test_shared_zone.py` — 공용 구역 기하 판정: 점·선분 vs AABB, pick/place/transit 분류

ROS 2가 설치된 실제 환경에서는 `colcon test --packages-select cap_robot` 후, 반드시
`dry_run` 분산 통합 시험을 별도로 수행하십시오. 토큰 동작은
`ros2 topic echo /zone_token/state`로 두 로봇이 중앙 구역을 두고 순차 진입하는지,
각자 구역만 쓰는 Task에서는 토큰 트래픽이 없는지 확인합니다.


## 재료별 그리퍼 설정과 바구니 적층

실험값은 `cap_robot/object_profiles.py`에서 관리합니다.

| 재료 | 배치된 두께(mm) | 그리퍼 닫기 |
| --- | ---: | ---: |
| 빵 | 20 | 150 |
| 양상추 | 0 | 400 |
| 바나나 | 35 | 330 |
| 햄 | 5 | 30 |

양상추처럼 두께가 무시할 만큼 작으면 0 mm를 허용합니다. 등록된 재료는 일반
이동에서도 해당 그리퍼 값을 사용하고, `assembly` Task에만 적층 높이를 적용합니다.

### 좌표계와 바구니 중심

`config/calibration.yaml`에서 마커 0의 배치 오프셋은 `[0, 0, 0]`입니다.
따라서 `workspace_0`의 원점은 마커 0 중심이며, `aruco_calib.py`는 보이는 여러
마커의 알려진 배치와 코너를 함께 solvePnP하여 workspace 자세를 추정합니다.
마커 0의 개별 TF를 그대로 복사하는 방식은 아닙니다.

고정 카메라·작업대·로봇 base·로봇 마커 조건에서 `camera → workspace_0`,
`camera → marker_1`, `camera → marker_12`는 여러 프레임을 모아 `/tf_static`으로
한 번 확정합니다. 유효한 CameraInfo, 양의 카메라 깊이, 재투영 RMS 오차 2 px 이하를
검사하며, 각 변환은 최소 30개 관측과 1초 이상의 관측 구간이 필요합니다.
최근 5초 이내 최대 90개 관측에서 80% 이상이 위치 10 mm·회전 3도 이내로
일치해야 합니다. 위치는 inlier 평균, 회전은 quaternion 기반 SO(3) 평균을 사용합니다.
작업대는 프레임마다 작업대 마커 4개 이상이 필요합니다. 임계값은
`config/calibration.yaml`의 `calibration_*` 파라미터로 조정합니다.

세 변환이 모두 준비되면 기존 `marker_1 → robot_1_base`,
`marker_12 → robot_2_base`와 함께 총 5개 static TF를 발행합니다.
따라서 늦게 접속한 listener도 전체 체인을 받습니다. 해당 세 카메라 변환은
`/tf`로 발행하지 않으며, 나머지 마커와 로봇 관절 TF는 기존 동작을 유지합니다.

재캘리브레이션은 기존 `aruco_calib` 프로세스를 종료하고 동일 설정으로 재시작합니다.
서비스나 키 입력은 없습니다. 새 관측이 충분해지면 동일 부모·자식의 static TF를
교체하고, 에이전트의 기존 TF 조회/캐시 타이머(기본 0.2초)가 새 값을 반영합니다.
재수집 중에는 이미 실행 중인 listener에 이전 static TF와 캐시가 남을 수 있습니다.
새 노드의 `static TF 확정` 로그와 변환 갱신을 확인한 뒤 작업을 재개합니다.

현장 확인(현재 설정의 카메라 frame 기준):

```bash
# 확정 후 늦게 구독해도 위 5개 변환이 모두 보이는지 확인
ros2 topic echo /tf_static tf2_msgs/msg/TFMessage --qos-durability transient_local --once
ros2 run tf2_ros tf2_echo fixed_camera/camera_color_optical_frame workspace_0
ros2 run tf2_ros tf2_echo fixed_camera/camera_color_optical_frame marker_1
ros2 run tf2_ros tf2_echo fixed_camera/camera_color_optical_frame marker_12
ros2 run tf2_ros tf2_echo robot_1_base workspace_0
ros2 run tf2_ros tf2_echo robot_2_base workspace_0
# 출력에 workspace_0/marker_1/marker_12가 없고 관절 TF는 계속 나오는지 확인
ros2 topic echo /tf tf2_msgs/msg/TFMessage --field transforms
```

`/tf_static`에는 다른 노드의 메시지도 있으므로 `--once`가 다른 발행자의 메시지만
보여주면 해당 옵션 없이 확인합니다. 재시작 전후 `tf2_echo`를 계속 실행하여 새 값
반영을 확인하고, 마커를 가린 뒤에도 확정된 체인을 조회할 수 있는지 확인합니다.
dynamic에서 static으로 최초 전환할 때는 기존 `aruco_calib`를 종료하고 TF 소비 노드도
재시작하여 이전 dynamic 이력과 새 static TF가 섞이지 않게 합니다.

바구니 중심은 `/perception/yolo_extra`의 `basket_handle_0`, `basket_handle_1`
(빨간 점 두 개, workspace 기준 mm) 좌표를 평균하여 계산합니다.

```text
center_x = (red0.x_mm + red1.x_mm) / 2
center_y = (red0.y_mm + red1.y_mm) / 2
center_z = (red0.z_mm + red1.z_mm) / 2
```

조립 X/Y에 center_x/y를 사용합니다. center_z는 빨간 점 높이이며 바구니 내부
바닥 높이가 아닙니다. 내부 바닥은 `BASKET_FLOOR_Z_MM`으로 별도 입력합니다.
마커 평면 Z=0과 바구니 내부 바닥도 같다고 가정하지 않습니다.
고정 `BASKET_CENTER_XY_MM` 설정은 사용하지 않습니다.

두 점 누락, 인식 valid=false, 다른 좌표계, 비정상 좌표, 오래된 수신/센서 시각은
실행 전에 거부합니다. 후보 생성/수신 때뿐 아니라 실제 실행 시작 직전에도 최신
중심을 다시 구합니다. 한 Task의 동작 중에는 그 좌표를 유지하여 팔의 가림이나
인식 노이즈를 따라가지 않습니다. 동작 중 바구니를 움직이지 않는 조건입니다.

### 추가로 입력할 실측값

| 설정 | 입력할 값 |
| --- | --- |
| `place_tcp_offset_mm` | 해당 배치 자세로 빈 바구니에 내려놓는 TCP의 workspace Z − 내부 바닥 Z |
| `BASKET_FLOOR_Z_MM` | workspace_0 기준 바구니 내부 바닥 Z |
| `BASKET_YAW_DEG` | workspace 기준 배치 방향, 기본 0° |

바닥 높이와 각 재료의 TCP 보정은 아직 `None`입니다. 해당 적층 작업은 값을
입력하기 전 이동을 거부하며, LLM이나 빨간 점 높이로 추측하지 않습니다.

최종 배치 TCP 높이는 다음과 같습니다.

```text
내부 바닥 Z + 앞 층들의 thickness_mm 합 + 현재 재료의 place_tcp_offset_mm
```

빵→양상추→바나나 순서의 적층 오프셋은 각각 **0, 20, 20 mm**입니다.
바나나 두께 35 mm는 그 위에 다음 재료를 놓을 때 추가됩니다.
기존 `z_offset=40`은 계산된 최종 배치 높이이고, `z_offset=200`은 그보다
160 mm 위입니다. 40 mm를 중복해서 더하지 않습니다. 파지 높이는 기존 값을 유지합니다.

### 빵 배치 기울임

재료별 공통 파지값, TCP 높이와 배치 기울임은 `cap_robot/object_profiles.py`의
`OBJECT_PROFILES`에서 설정합니다. 현재 빵의 roll/pitch offset은 0°이며,
Agent별 프로필 덮어쓰기는 사용하지 않습니다. 두께와 바구니 바닥 Z도 공통 설정입니다.

각 적층 Task는 `assembly`에 id, container=basket, layer_index, material을 갖고
직전 층 Task에 의존합니다. 앞 층이 SUCCEEDED가 되어야 다음 층을 시작합니다.
이는 동작 시퀀스/홈 복귀 성공이며 카메라로 적층 성공을 확인하는 기능은 아닙니다.
실패 후 새 미션을 시작할 때 실제 적층 상태를 먼저 정리해야 합니다.

설정 수정 후 빌드하고 Workstation과 두 Agent를 재시작하세요.

```bash
cd ~/colcon_ws
colcon build --packages-select cap_robot --symlink-install
source install/setup.bash
```

로봇 없이 검증:

```bash
cd ~/colcon_ws/src/cap_robot
python3 -m unittest discover -s test -p 'test_*.py'
```
