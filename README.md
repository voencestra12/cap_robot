# cap_robot

ROS 2 기반 xArm6 다중 로봇 협업 패키지입니다. 각 Robot Agent가 전용 RealSense 카메라로 객체를 인식하고 작업을 수행하며, 고정 카메라의 ArUco 마커로 workspace 및 robot base TF를 생성합니다. Workstation LLM은 자연어 임무를 구조화된 JSON 가이드북으로 변환해 Agent에 발행합니다.

## 구성

```text
cap_robot/
├── cap_robot/
│   ├── aruco_calib.py       # ArUco 검출 및 TF 발행
│   ├── llm_api.py           # 로봇 API, capability 목록과 정책 검증
│   ├── robot_agent.py       # 객체 인식, 작업 계획과 로봇 제어
│   └── workstation_llm.py   # 자연어 임무 수집 및 가이드북 발행
├── config/
│   ├── agent1.yaml
│   ├── agent2.yaml
│   └── calibration.yaml
├── launch/
│   ├── agent1.launch.py     # Agent 1 카메라와 Robot Agent
│   ├── agent2.launch.py     # Agent 2 카메라와 Robot Agent
│   ├── aruco_calib.launch.py # ArUco 노드만 실행
│   └── workstation.launch.py # 고정 카메라와 ArUco 노드
├── prompts/
│   ├── agent_policy.txt
│   └── workstation_prompt.txt
├── package.xml
├── requirements.txt
└── setup.py
```

## 실행 구조

| 실행 파일 | ROS 노드 | 카메라 토픽/네임스페이스 |
|---|---|---|
| `agent1.launch.py` | `/agent1/robot_agent_node` | `/agent1/camera/...` |
| `agent2.launch.py` | `/agent2/robot_agent_node` | `/agent2/camera/...` |
| `workstation.launch.py` | `/aruco_calib` | `/fixed_camera/camera/...` |
| `workstation_llm` | `/workstation_llm` | 가이드북 `/mission/guidebook` 발행 |

Agent launch와 Workstation launch는 각각 `realsense2_camera`의 `rs_launch.py`를 포함합니다. 카메라 frame에는 `agent1/`, `agent2/`, `fixed_camera/` TF prefix가 적용됩니다.

## 요구 사항

- ROS 2 Humble
- `realsense2_camera`, `cv_bridge`, `tf2_ros`
- Python 패키지: NumPy, SciPy, Requests, OpenCV, Ultralytics, xArm Python SDK
- Workstation LLM 사용 시 Ollama와 사용할 모델

Python 의존성은 다음과 같이 설치합니다.

```bash
python3 -m pip install -r src/cap_robot/requirements.txt
```

## 빌드

```bash
cd ~/capstone_ws
colcon build --packages-select cap_robot --symlink-install
source install/setup.bash
```

launch 파일을 삭제하거나 이름을 바꾼 뒤 기존 설치 공간에 파일이 남아 있다면 `build/cap_robot`과 `install/cap_robot`을 정리한 후 다시 빌드해야 합니다.

## 실행

각 명령을 실행하기 전에 워크스페이스를 source합니다.

```bash
cd ~/capstone_ws
source install/setup.bash
```

### Agent 1

Agent 1의 RealSense 카메라와 Robot Agent를 함께 실행합니다.

```bash
ros2 launch cap_robot agent1.launch.py
```

### Agent 2

Agent 2의 RealSense 카메라와 Robot Agent를 함께 실행합니다.

```bash
ros2 launch cap_robot agent2.launch.py
```

### 고정 카메라 및 캘리브레이션

Workstation의 고정 RealSense 카메라와 ArUco 캘리브레이션 노드를 함께 실행합니다.

```bash
ros2 launch cap_robot workstation.launch.py
```

이미 카메라가 실행 중이라면 ArUco 노드만 실행할 수 있습니다.

```bash
ros2 launch cap_robot aruco_calib.launch.py
```

또는 실행 파일을 직접 호출할 수 있습니다.

```bash
ros2 run cap_robot aruco_calib --ros-args \
  --params-file $(ros2 pkg prefix cap_robot)/share/cap_robot/config/calibration.yaml
```

### Workstation LLM

Ollama를 실행하고 기본 모델 `gemma4:12b`를 준비한 뒤 별도 터미널에서 실행합니다.

```bash
ros2 run cap_robot workstation_llm
```

기본 설정은 다음과 같습니다.

- Ollama 주소: `http://localhost:11434`
- 모델: `gemma4:12b`
- 가이드북 토픽: `/mission/guidebook`
- 로그 디렉터리: `~/capstone_ws/log/cap_robot`
- Ollama 응답 제한 시간: 300초

파라미터 변경 예시:

```bash
ros2 run cap_robot workstation_llm --ros-args \
  -p llm_model:=다른_모델명 \
  -p ollama_url:=http://localhost:11434 \
  -p guidebook_topic:=/mission/guidebook
```

대화 중 사용할 수 있는 명령은 다음 두 가지입니다.

- `/reset`: 현재 대화 초기화
- `/quit`: 종료

Workstation LLM은 필요한 정보가 충분해져 응답 상태가 `ready`가 되면 가이드북을 자동으로 검증·저장·발행하고 대화를 초기화합니다. 별도의 `/confirm` 명령은 사용하지 않습니다.

## 로그 및 가이드북

기본 저장 구조는 다음과 같습니다.

```text
~/capstone_ws/log/cap_robot/
├── workstation_YYYYMMDD_HHMMSS.jsonl
└── missions/
    └── mission_YYYYMMDD_HHMMSS_ffffff.json
```

- JSONL 파일에는 사용자 입력, LLM 응답, 오류와 발행 이력이 기록됩니다.
- `missions/`에는 검증을 통과해 발행된 최종 JSON 가이드북이 저장됩니다.
- `log_dir` 생성이나 파일 기록에 실패하면 오류를 출력하며, 다른 경로로 자동 대체하지 않습니다.

## 주요 설정

- `config/agent1.yaml`: Agent 1 ID, 로봇 IP, 작업 frame, 카메라 토픽과 안전 범위
- `config/agent2.yaml`: Agent 2 ID, 로봇 IP, 작업 frame, 카메라 토픽과 안전 범위
- `config/calibration.yaml`: 고정 카메라 토픽, ArUco 크기, marker 배치와 robot base offset
- `prompts/agent_policy.txt`: Robot Agent의 작업 정책 프롬프트
- `prompts/workstation_prompt.txt`: 자연어 임무를 가이드북으로 변환하는 프롬프트

설정 YAML의 최상위 노드 이름은 launch에서 생성되는 전체 노드 이름과 일치해야 합니다.

```text
/agent1/robot_agent_node
/agent2/robot_agent_node
/aruco_calib
```

## 안전 주의사항

현재 `agent1.yaml`과 `agent2.yaml`의 `dry_run` 값은 모두 `false`입니다. 따라서 명령을 받으면 실제 로봇을 구동할 수 있습니다.

처음 설정하거나 좌표·TF·카메라 구성을 변경한 경우 다음 순서를 권장합니다.

1. 두 Agent의 `dry_run`을 `true`로 변경합니다.
2. 로봇 IP와 `agent_specs`, base/workspace frame을 확인합니다.
3. 카메라 토픽과 TF가 각 네임스페이스에서 정상 발행되는지 확인합니다.
4. SDK 안전 박스와 이동 좌표를 검증합니다.
5. 실제 실험 준비가 끝난 뒤에만 `dry_run: false`로 변경합니다.

두 Agent와 Workstation을 서로 다른 컴퓨터에서 실행할 경우 동일한 ROS domain 및 네트워크 검색 설정을 사용해야 합니다. 같은 컴퓨터에 복수의 RealSense를 연결할 때는 필요한 경우 serial number를 launch 인자로 추가해 장치를 명시적으로 구분해야 합니다.
