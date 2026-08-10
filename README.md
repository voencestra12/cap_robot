# cap_robot

ROS 2 Humble 기반의 xArm6 다중 로봇 협업 패키지입니다. 두 Robot Agent가 각자의 RealSense 카메라로 객체를 인식하고 xArm을 제어하며, Workstation의 고정 카메라와 ArUco 마커가 공통 `workspace_0` 좌표계를 제공합니다. 자연어 임무는 Workstation LLM이 구조화된 가이드북으로 변환해 Agent에 전달합니다.

## 시스템 구성

주요 역할은 다음과 같습니다.

- `robot_agent`: YOLO 기반 객체 인식, depth 좌표 변환, 작업 계획 및 XArmAPI 제어
- `aruco_calib`: 고정 카메라에서 ArUco 마커를 검출하고 workspace 및 robot base TF 발행
- `workstation_llm`: 자연어 임무를 JSON 가이드북으로 변환해 `/mission/guidebook`에 발행
- `mount_tf`: YAML의 hand-eye 결과를 static TF로 발행
- `xarm_driver_node`: 로봇 제어가 아니라 joint state와 robot state 공급에 사용
- `robot_state_publisher`: joint state를 xArm 링크 TF로 변환

로봇의 실제 동작 명령은 기존과 동일하게 `robot_agent.py`의 XArmAPI가 담당합니다. 이 패키지는 MoveIt, `ros2_control` trajectory controller 또는 MoveGroup을 실행하지 않습니다.

## 패키지 구조

```text
cap_robot/
├── cap_robot/
│   ├── aruco_calib.py
│   ├── llm_api.py
│   ├── mount_tf.py
│   ├── robot_agent.py
│   ├── utils.py
│   └── workstation_llm.py
├── config/
│   ├── agent1.yaml
│   ├── agent2.yaml
│   ├── calibration.yaml
│   ├── handeye_agent1.yaml
│   └── handeye_agent2.yaml
├── launch/
│   ├── agent1.launch.py
│   ├── agent2.launch.py
│   ├── aruco_calib.launch.py
│   └── workstation.launch.py
├── prompts/
│   ├── agent_policy.txt
│   └── workstation_prompt.txt
├── urdf/
│   └── xarm_tf_only.urdf.xacro
├── package.xml
├── requirements.txt
└── setup.py
```

## TF 구조

전체 좌표계는 다음과 같이 연결됩니다.

```text
fixed_camera/camera_color_optical_frame
├── workspace_0
├── marker_1
│   └── robot_1_base
│       └── agent1_link_base → ... → agent1_link6
│                                   └── agent1/camera_link
│                                       └── ... → agent1/camera_color_optical_frame
└── marker_12
    └── robot_2_base
        └── agent2_link_base → ... → agent2_link6
                                    └── agent2/camera_link
                                        └── ... → agent2/camera_color_optical_frame
```

정확한 상위 marker 경로는 ArUco 검출 상태에 따라 달라질 수 있습니다. 중요한 연결은 다음과 같습니다.

- Workstation ArUco TF: `workspace_0 ↔ robot_1_base`, `workspace_0 ↔ robot_2_base`
- xArm URDF TF: `robot_N_base → agentN_link_base → ... → agentN_link6`
- hand-eye static TF: `agentN_link6 → agentN/camera_link`
- RealSense 내부 TF: `agentN/camera_link → ... → agentN/camera_color_optical_frame`

`robot_1_base`와 `robot_2_base`는 각 URDF의 root attachment frame입니다. 외부 ArUco TF가 이 프레임을 `workspace_0`에 연결하므로 인식 결과는 workspace 좌표로 표현할 수 있고, 로봇 명령은 각 물리 base 좌표로 변환할 수 있습니다.

### Hand-eye TF 규칙

원본 hand-eye 결과는 `link6 → camera_color_optical_frame` 기준이었습니다. RealSense가 자체적으로 `camera_link → camera_color_optical_frame`을 발행하므로 원본 optical TF를 그대로 발행하면 하나의 child frame에 부모가 중복될 수 있습니다.

현재 패키지는 원본 결과에서 RealSense 내부 변환을 분리한 `link6 → camera_link` 결과만 `mount_tf`로 발행합니다. 변환 과정에서는 다음과 같은 이상화된 RealSense 내부 변환을 사용했습니다.

- `camera_link → camera_color_optical_frame`
- translation 약 `[0, 0.015, 0] m`
- camera link 축을 optical 축으로 바꾸는 이상적인 회전

따라서 원본 optical-frame Y 값 `0.035 m`에서 내부 오프셋 약 `0.015 m`를 분리해 camera-link 기준 Y 값은 `0.020 m`입니다.

현재 hand-eye 행렬은 다음과 같습니다.

Agent 1 (`agent1_link6 → agent1/camera_link`):

```text
[ 0  0 -1  0.105 ]
[ 0  1  0  0.020 ]
[ 1  0  0  0.085 ]
[ 0  0  0  1     ]
```

Agent 2 (`agent2_link6 → agent2/camera_link`):

```text
[ 0  0 -1  0.105 ]
[ 0  1  0  0.020 ]
[ 1  0  0  0.025 ]
[ 0  0  0  1     ]
```

실제 장착 상태나 카메라 모델이 바뀌면 `config/handeye_agent1.yaml`과 `config/handeye_agent2.yaml`을 다시 측정한 값으로 갱신해야 합니다.

## 요구 사항

- ROS 2 Humble
- xArm ROS 2 패키지: `xarm_api`, `xarm_description`, `uf_ros_lib`
- `robot_state_publisher`, `xacro`, `tf2_ros`
- `realsense2_camera`, `cv_bridge`
- Ollama와 사용할 로컬 모델
- Python 패키지: NumPy, SciPy, Requests, OpenCV, Ultralytics, xArm Python SDK

Python 의존성은 다음과 같이 설치합니다.

```bash
python3 -m pip install -r src/cap_robot/requirements.txt
```

## 빌드

xArm workspace가 별도로 있다면 먼저 source합니다.

```bash
source ~/xarm_ws/install/setup.bash
cd ~/capstone_ws
colcon build --symlink-install --packages-select cap_robot
source install/setup.bash
```

실행 파일이나 launch 파일의 이름을 변경했는데 설치 공간에 이전 파일이 남아 있다면 해당 패키지의 `build/cap_robot`과 `install/cap_robot`을 정리한 뒤 다시 빌드합니다.

설치된 실행 파일은 다음 명령으로 확인할 수 있습니다.

```bash
ros2 pkg executables cap_robot
```

현재 제공되는 실행 파일은 `aruco_calib`, `mount_tf`, `robot_agent`, `workstation_llm`입니다.

## 실행

각 터미널에서 필요한 workspace를 source합니다.

```bash
source ~/xarm_ws/install/setup.bash
source ~/capstone_ws/install/setup.bash
```

### Workstation

고정 RealSense와 ArUco 캘리브레이션 노드를 함께 실행합니다.

```bash
ros2 launch cap_robot workstation.launch.py
```

고정 카메라가 이미 실행 중이면 ArUco 노드만 실행할 수 있습니다.

```bash
ros2 launch cap_robot aruco_calib.launch.py
```

직접 실행할 수도 있습니다.

```bash
ros2 run cap_robot aruco_calib --ros-args \
  --params-file $(ros2 pkg prefix cap_robot)/share/cap_robot/config/calibration.yaml
```

### Agent 1

Agent 1의 xArm state driver, robot state publisher, RealSense, hand-eye TF 및 Robot Agent를 실행합니다.

```bash
ros2 launch cap_robot agent1.launch.py
```

### Agent 2

```bash
ros2 launch cap_robot agent2.launch.py
```

두 Agent 모두 hand-eye TF와 Robot Agent가 기본 활성화되어 있습니다. 지원하는 launch argument는 다음과 같습니다.

| Argument | 기본값 | 설명 |
|---|---:|---|
| `enable_robot_agent` | `true` | `robot_agent` 실행 여부 |
| `enable_handeye_tf` | `true` | `mount_tf` 실행 여부 |
| `handeye_yaml` | Agent별 YAML | 사용할 hand-eye YAML 절대 경로 |

예를 들어 Robot Agent 없이 TF 구성만 실행하려면 다음과 같이 사용합니다.

```bash
ros2 launch cap_robot agent1.launch.py enable_robot_agent:=false
ros2 launch cap_robot agent2.launch.py enable_robot_agent:=false
```

hand-eye TF만 직접 실행하는 방법은 다음과 같습니다.

```bash
ros2 run cap_robot mount_tf --ros-args \
  -p result_yaml:=$(ros2 pkg prefix cap_robot)/share/cap_robot/config/handeye_agent1.yaml
```

동일한 hand-eye TF를 Agent launch와 직접 실행 명령으로 동시에 발행하지 마십시오.

## TF 검증

처음 연결할 때는 `enable_robot_agent:=false`로 TF만 검증하는 것을 권장합니다.

### Agent 1

```bash
ros2 topic echo --once /agent1/xarm/joint_states
ros2 run tf2_ros tf2_echo robot_1_base agent1_link_base
ros2 run tf2_ros tf2_echo robot_1_base agent1_link6
ros2 run tf2_ros tf2_echo agent1_link6 agent1/camera_link
ros2 run tf2_ros tf2_echo agent1/camera_link agent1/camera_color_optical_frame
ros2 run tf2_ros tf2_echo workspace_0 agent1/camera_color_optical_frame
```

예상되는 직접 hand-eye translation은 `[0.105, 0.020, 0.085] m`입니다.

### Agent 2

```bash
ros2 topic echo --once /agent2/xarm/joint_states
ros2 run tf2_ros tf2_echo robot_2_base agent2_link_base
ros2 run tf2_ros tf2_echo robot_2_base agent2_link6
ros2 run tf2_ros tf2_echo agent2_link6 agent2/camera_link
ros2 run tf2_ros tf2_echo agent2/camera_link agent2/camera_color_optical_frame
ros2 run tf2_ros tf2_echo workspace_0 agent2/camera_color_optical_frame
```

예상되는 직접 hand-eye translation은 `[0.105, 0.020, 0.025] m`입니다.

로봇 자세를 바꿨을 때 `link6`과 카메라가 함께 움직이고, `link6 → camera_link` 값은 일정해야 합니다. `workspace_0 → camera_color_optical_frame` 조회가 성공하면 Workstation부터 장착 카메라까지 TF tree가 연결된 것입니다.

## LLM 설정

Robot Agent와 Workstation은 서로 다른 기본 모델과 Ollama endpoint 형식을 사용합니다.

| 구성요소 | 기본 모델 | 기본 URL |
|---|---|---|
| Agent 1/2 | `gemma4:e4b` | `http://localhost:11434/api/generate` |
| Workstation LLM | `gemma4:12b` | `http://localhost:11434` |

Workstation LLM은 다음과 같이 실행합니다.

```bash
ros2 run cap_robot workstation_llm
```

파라미터 변경 예시:

```bash
ros2 run cap_robot workstation_llm --ros-args \
  -p llm_model:=다른_모델명 \
  -p ollama_url:=http://localhost:11434 \
  -p guidebook_topic:=/mission/guidebook
```

대화 명령은 `/reset`과 `/quit`입니다. 필요한 정보가 충분해 응답 상태가 `ready`가 되면 가이드북을 검증하고 저장한 뒤 `/mission/guidebook`에 발행합니다.

기본 로그 구조는 다음과 같습니다.

```text
~/capstone_ws/log/cap_robot/
├── workstation_YYYYMMDD_HHMMSS.jsonl
└── missions/
    └── mission_YYYYMMDD_HHMMSS_ffffff.json
```

## 주요 설정 파일

- `config/agent1.yaml`: Agent 1 ID, 로봇 IP, TF, 카메라 토픽, LLM 및 안전 범위
- `config/agent2.yaml`: Agent 2 ID, 로봇 IP, TF, 카메라 토픽, LLM 및 안전 범위
- `config/calibration.yaml`: 고정 카메라, ArUco 마커 배치 및 robot base offset
- `config/handeye_agent1.yaml`: Agent 1 `link6 → camera_link` static TF
- `config/handeye_agent2.yaml`: Agent 2 `link6 → camera_link` static TF
- `prompts/agent_policy.txt`: Robot Agent 정책 프롬프트
- `prompts/workstation_prompt.txt`: 자연어 임무를 가이드북으로 변환하는 프롬프트

설정 YAML의 최상위 노드 이름은 실제 전체 노드 이름과 일치해야 합니다.

```text
/agent1/robot_agent_node
/agent2/robot_agent_node
/aruco_calib
```

## 안전 및 운영 주의사항

현재 `agent1.yaml`과 `agent2.yaml`의 `dry_run`은 모두 `false`이므로 명령을 받으면 실제 로봇이 움직일 수 있습니다. 처음 설정하거나 TF, 카메라 또는 hand-eye 값을 변경한 경우 다음 순서로 검증하십시오.

1. Agent YAML의 `dry_run`을 `true`로 변경합니다.
2. `enable_robot_agent:=false`로 joint state와 전체 TF chain을 확인합니다.
3. 로봇 IP, Agent별 base frame과 카메라 frame을 확인합니다.
4. SDK 안전 박스와 변환된 목표 좌표를 검증합니다.
5. 작은 이동으로 실제 좌표 오차를 확인합니다.
6. 모든 검증 후에만 `dry_run: false`로 전환합니다.

`xarm_driver_node`와 Python XArmAPI가 같은 컨트롤러에 동시에 연결되므로 실제 장비에서 연결 안정성을 확인해야 합니다.

여러 컴퓨터에서 실행할 경우 모든 장비가 동일한 ROS domain과 호환되는 DDS 네트워크 설정을 사용해야 합니다. 같은 컴퓨터에 복수의 RealSense를 연결한다면 serial number를 지정해 장치를 명시적으로 구분하는 것이 안전합니다.
