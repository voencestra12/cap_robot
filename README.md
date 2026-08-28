# cap_robot merged (ROS 2 Humble)

`cap_robot`의 기존 실행 구조를 유지하면서, 첫 번째 코드 세트의 빨간 손잡이 인식과
양팔 바구니 협동 기능을 통합한 패키지입니다. 두 코드 세트를 동시에 실행하지 않습니다.

## 배치 구조

| 컴퓨터 | 실행 | 역할 |
|---|---|---|
| 연구실 컴퓨터 | `ros2 launch cap_robot workstation.launch.py` | 고정 RealSense, ArUco TF, 손잡이 인식 |
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

## 실행 전 점검

```bash
ros2 topic echo /perception/yolo_extra --once
ros2 run tf2_ros tf2_echo robot_1_base workspace_0
ros2 run tf2_ros tf2_echo robot_2_base workspace_0
ros2 topic info /mission/task_status -v
```

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

`test/test_llm_api.py`는 일반/협동 action 상태기와 금지 API를 검사합니다. ROS 2가 설치된
실제 환경에서는 `colcon test --packages-select cap_robot` 후, 반드시 `dry_run` 분산 통합
시험을 별도로 수행하십시오.
