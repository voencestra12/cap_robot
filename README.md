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

## 빵·양상추 모델 통합

두 Agent는 `models/yolo11m-seg.pt`와 `models/yolo-bread-lettuce.pt`를 함께
로드합니다. 추가 모델은 segmentation 모델이며 클래스는 `0: bread`, `1: lettuce`입니다.
인식 결과는 각각 **빵**, **양상추**로 기존 Agent perception에 들어갑니다.
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

이번 통합은 인식 입력 연결까지입니다. 현재 물체 상태는 클래스 이름당 좌표 한 개를
저장하므로 여러 빵 조각을 개별 식별하지 않습니다. 샌드위치 조립에는 이후 개체 구분,
쌓을 위치·높이, 재료별 파지 설정을 정해야 합니다. 실제 재료의 검출 품질과 파지 자세는
카메라·로봇 환경에서 별도로 확인해야 합니다.

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
