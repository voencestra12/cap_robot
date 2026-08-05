# cap_robot

정상 동작이 확인된 `aruco_calib.py`, `robot_agent.py`를 기반으로 최소한만 분리한 ROS2 Python 패키지입니다.

## 구조

```text
cap_robot/
├── cap_robot/
│   ├── aruco_calib.py
│   ├── robot_agent.py
│   └── llm_api.py
├── config/
│   ├── calibration.yaml
│   ├── agent1.yaml
│   └── agent2.yaml
├── prompts/
│   └── agent_policy.txt
└── launch/
    ├── aruco_calib.launch.py
    └── robot_agent.launch.py
```

- `aruco_calib.py`: ArUco 검출, workspace/robot base TF 발행
- `robot_agent.py`: 기존의 나머지 전체 동작
- `llm_api.py`: LLM이 사용하는 API 이름, 기본 PnP 동작, 정책 검증
- `agent_policy.txt`: `robot_agent.py`가 직접 읽는 프롬프트
- `config/*.yaml`: 기존 ROS 파라미터와 캘리브레이션 값

## 빌드

```bash
cd ~/capstone_ws/src
unzip cap_robot.zip
cd ~/capstone_ws
colcon build --packages-select cap_robot --symlink-install
source install/setup.bash
```

Python 의존성은 기존 환경에 설치되어 있어야 합니다.

```bash
pip install -r src/cap_robot/requirements.txt
```

ROS용 `cv_bridge`, RealSense 드라이버, xArm 환경은 기존 설치 상태를 사용합니다.

## 실행

### 캘리브레이션

```bash
ros2 launch cap_robot aruco_calib.launch.py
```

또는:

```bash
ros2 run cap_robot aruco_calib --ros-args \
  --params-file $(ros2 pkg prefix cap_robot)/share/cap_robot/config/calibration.yaml
```

### Agent 1

```bash
ros2 launch cap_robot robot_agent.launch.py \
  config_file:=$(ros2 pkg prefix cap_robot)/share/cap_robot/config/agent1.yaml
```

### Agent 2

```bash
ros2 launch cap_robot robot_agent.launch.py \
  config_file:=$(ros2 pkg prefix cap_robot)/share/cap_robot/config/agent2.yaml
```

## 주의

- `agent1.yaml`, `agent2.yaml`은 안전을 위해 `dry_run: true`입니다. 로그와 좌표를 확인한 뒤 실제 실험에서만 `false`로 변경하세요.
- 프롬프트는 `robot_agent.py`가 직접 `prompts/agent_policy.txt`를 읽습니다. 별도 prompt loader 파일은 없습니다.
- 핵심 좌표변환, YOLO, 작업 Queue, 로봇 동작 로직은 기존 코드에서 수정하지 않았습니다.
- `agent_specs`와 카메라 토픽은 실제 실행 환경에 맞게 YAML에서 확인하세요.
