<!-- 역할: 재현 가능한 설치·실행·실기 전환·평가 절차. 인터페이스: CLI/launch/config. -->
# 설치와 운용

## 1. 패키지 배치와 빌드

대상은 Ubuntu 22.04 / ROS 2 Humble / Python 3.10이다.
패키지 이름은 `cap_robot`이며, 원본 구버전 노드와 동시에 같은 로봇을 제어하면 안 된다.
원본의 SDK 직접 제어 노드와 xarm driver를 함께 켜던 구성을 제거했다.

```bash
mkdir -p ~/capstone_new_ws/src
# 배포 ZIP의 cap_robot 디렉터리를 ~/capstone_new_ws/src/ 아래에 풀기
source /opt/ros/humble/setup.bash
# 기존에 설치한 xarm_description이 있으면 해당 workspace도 source
source ~/xarm_ws/install/setup.bash
cd ~/capstone_new_ws
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r src/cap_robot/requirements.txt
colcon build --packages-select cap_robot
source install/setup.bash
```

`xarm_description`은 xArm 공식 ROS 2 Humble workspace에서 준비한다. RealSense, cv_bridge,
tf2, rosbag 등은 `package.xml`의 rosdep 의존성이다. 모의 실행은 xArm SDK나 YOLO를 import하지 않지만,
실기 프로필은 두 라이브러리가 필요하다. 패키지를 pip로만 설치하는 대신 ament/colcon을 사용한다.

모든 PC의 패키지·설정·LLM 프롬프트 버전을 맞춘다. 설정은 일반 YAML 매핑이며 노드의
`config_file` 파라미터로 읽는다. ROS의 `ros__parameters` 중첩 YAML이 아니다.
YAML을 바꾼 후 다시 빌드하거나, 수정한 YAML의 절대경로를 launch 인자로 전달한다.
agent launch는 `config_file:=...`, workstation launch는 `workstation_config_file:=...`를 사용한다.
노드 자체의 ROS 파라미터 이름은 모두 `config_file`이다.
1.0.0의 `ros2 launch cap_robot workstation.launch.py config_file:=...` 호출은
1.0.1에서 `ros2 launch cap_robot workstation.launch.py workstation_config_file:=...`로 바꾼다.
<!-- [변경] 1.0.1에서 workstation launch 인자를 분리해 include 간 이름 충돌을 제거했다. -->

## 2. 먼저 로봇 없는 전체 실행

```bash
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
ros2 launch cap_robot demo.launch.py
```

이 launch는 두 모의 로봇·두 모의 장면·워크스테이션·coordinator·CSV 기록과
**실기에서 사용하면 안 되는 예시 world/base TF**를 실행한다. 로봇 IP로 연결하지 않는다.
각 include의 인자는 별도 범위에 있고 패키지의 `agent1.yaml`, `agent2.yaml`,
`workstation.yaml`을 명시적으로 사용한다. 외부 launch의 `config_file`, `camera` 값은
demo 내부로 상속하지 않는다. 사용자 지정 설정은 아래의 개별 launch에서 전달한다.
그래도 LLM은 실제 Ollama endpoint를 호출한다. 테스트용 하드코딩 모델이 자동 선택되지는 않는다.

다른 터미널에서 같은 환경을 source한다.

```bash
ros2 run cap_robot cli monitor
ros2 run cap_robot cli mission '보이는 빵의 위치를 두 로봇이 차례로 확인해줘.'
ros2 run cap_robot cli estop
ros2 run cap_robot cli kill
```

LLM 모델명은 원본의 `gemma4:12b`(워크스테이션), `gemma4:e4b`(각 agent)를 그대로 설정했다.
해당 모델이 존재/설치한다고 검증한 것은 아니다. 각 PC의 `ollama list`로 확인하고 실제 설치 모델명을
YAML `llm.model`에 설정한다. 기본 endpoint는 로컬 `http://localhost:11434/api/generate`다.
없는 모델, 잘못된 JSON, 시간 초과는 명시적 실패로 처리된다.

모의 장면의 객체는 물리 시뮬레이션에 따라 이동하지 않는다. 따라서 전체 실제 샌드위치/커피
완성도를 모의 launch로 판정할 수 없다. 모의 실행은 코드·메시지·안전 제한을 검사하는 용도다.

## 3. 세 컴퓨터로 분산 실행

같은 LAN에서 동일 ROS_DOMAIN_ID, 호환 DDS, 허용된 방화벽 설정을 사용한다.
분산 모드에서는 `ROS_LOCALHOST_ONLY=0`이어야 한다. ROS_DOMAIN_ID는 접근 인증이 아니다.
명령 토픽의 신뢰 경계가 필요한 환경에서는 별도 격리 네트워크/DDS Security를 구성한다.

공동 운반의 `execute_at`은 ROS 시스템시각을 비교하므로 chrony/NTP로 두 노트북과 워크스테이션의
시각 차이를 측정해야 한다. 예시 `max_start_skew=.05`초보다 충분히 작아야 한다.
DDS나 NTP가 하드 실시간 동기화를 제공한다는 뜻은 아니다.

```bash
# 각 PC 공통
source /opt/ros/humble/setup.bash
source ~/capstone_new_ws/install/setup.bash
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=0

# 노트북 1
ros2 launch cap_robot agent.launch.py agent:=agent1

# 노트북 2
ros2 launch cap_robot agent.launch.py agent:=agent2

# 워크스테이션
ros2 launch cap_robot workstation.launch.py
```

이 예시는 아직 dry_run이다. 분산 공동 운반 시험에서는 별도 모의 TF를 공급하거나 검증된
실제 TF를 읽어야 한다. 일반 노트북 launch는 임의 world/base TF를 만들지 않는다.
각 agent의 모델은 해당 노트북에서 돌아간다. 워크스테이션의 agent YAML 복사본은 공동 운반에서
각 로봇의 제한·frame·commissioning을 검사하므로 실제 노트북 설정과 일치해야 한다.

## 4. 실기 프로필

`agent1_hardware.yaml`, `agent2_hardware.yaml`은 명시적으로 선택하는 참고 프로필이다.
IP는 원본의 `192.168.1.218`, `192.168.1.198`을 유지했다. 자동 arm은 하지 않는다.
프로필 안의 박스·도구 반경·자세/회전 제한·탐색 좌표는 **실측값이 아니다**.

| 설정 | 확인할 실제 값 |
|---|---|
| `base_frame`, ArUco offset, hand-eye | SDK base 좌표, 마커 방향, 실제 link6-camera 변환 |
| `safety.min_xyz/max_xyz` | 테이블과 설치 구역에 맞는 TCP XYZ 경계 |
| `safety.tcp_radius` | 그리퍼·집은 물체·용기의 최대 회전 반경까지 포함 |
| `safety.max_speed/max_accel` | 로봇·payload·지그에 맞춰 시험한 값 |
| `planning.search_*` | 실제 카메라가 추가 영역을 볼 수 있는 안전한 자세/위치 |
| `perception.model_paths/label_aliases` | 빵·양상추·컵·용기 클래스가 있는 모델/이름 |
| `commissioning.*` | 해당 기하·제어 동작을 실제로 확인했을 때만 true |

사용자가 확인한 그리퍼는 **xArm Gripper**다. SDK의 position pulse 0..850을 사용한다.
설치된 SDK의 `get_gripper_status`는 그리퍼 firmware 3.4.3 이상에서 제공된다고 명시되어 있다.
정확한 그리퍼 세대/firmware는 아직 미확인이며, 위치/상태 피드백을 얻지 못하면 동작 성공을 선언하지 않는다.
SDK API만 성공해도 그리퍼가 실제 물체를 안전하게 잡았다고 보장하지 않는다.

손목 카메라와 원본 hand-eye를 사용하려면 다음과 같이 실행한다. 파일은 실제 교정 후 선택한다.

```bash
# 노트북별로 번호/경로 변경
ros2 launch cap_robot agent.launch.py agent:=agent1 \
  config_file:=/absolute/path/agent1_hardware.yaml \
  camera:=true handeye:=true \
  handeye_file:=/absolute/path/handeye_agent1.yaml
```

안전 노드가 SDK 보고에서 `/agentN/joint_states`를 게시하고 robot_state_publisher가
원본 link6 체인을 구성한다. 기존 `xarm_driver_node`를 추가로 켜지 않는다.
SDK의 TCP pose는 별도 `agentN_tool0` TF로 게시하며 link6와 같다고 간주하지 않는다.
RealSense의 프레임 prefix는 `agent1/`, `agent2/`, `fixed_camera/`로 분리한다.

그 후 고정 카메라의 ArUco TF가 필요하면 워크스테이션 `camera:=true`를 사용한다.

```bash
ros2 launch cap_robot workstation.launch.py camera:=true \
  agent1_config_file:=/absolute/path/agent1_hardware.yaml \
  agent2_config_file:=/absolute/path/agent2_hardware.yaml
```

장면·TF·controller 상태를 확인하고 검증된 프로필에서 운용자가 명시적으로 arm한다.

```bash
ros2 run tf2_ros tf2_echo robot_1_base workspace_0
ros2 topic echo /agent1/observations --once
ros2 topic echo /agent1/gate_state --once
ros2 run cap_robot cli arm agent1
ros2 run cap_robot cli arm agent2
```

초기 TCP가 설정 펜스 밖이면 arm을 거절한다. 코드가 자동 홈 동작으로 안전 검사를 우회하지 않는다.
운용자가 로봇 제어기/티칭 도구로 실제 안전 자세와 오류 원인을 먼저 확인해야 한다.

## 5. 빨간 손잡이 바구니

사용자가 확인한 원본 바구니는 손잡이가 빨간색이다. 원본 HSV 범위(0..10, 170..180 hue)를 보존했다.
새 비전 모듈은 각 빨간 영역을 `basket_handle` 인스턴스로 게시한다. 조명·반사·다른 빨간 물체로
잘못 검출할 수 있으므로 색만으로 같은 바구니에 속한 손잡이라는 사실을 확정할 수는 없다.

- 손목 카메라: hardware 프로필의 `detect_red_handles:true`가 YOLO 결과에 빨간 손잡이를 더한다.
- 원본 고정 카메라: `agent1_basket_fixed.yaml`, `agent2_basket_fixed.yaml`을 선택한다.
  각 agent는 고정 RGB-D를 **각자 인식**하고, 해당 촬영 시각에 자기 base 좌표로 변환한다.
  워크스테이션 `camera:=true`만 장치를 열고 두 agent는 영상 토픽을 공유한다.

실기 공동 운반에서는 두 agent와 WS에 일치하는 프로필을 사용하고,
`cooperative.enabled:true`, `commissioning.cooperative_verified:true` 등 검증 조건을 실제로 충족해야 한다.
기본 hardware/basket 프로필에서는 공동 운반이 꺼져 있다.

손잡이 간 거리, 바구니 무게, 최대 payload, 여유 공간과 비상정지 시 그리퍼 유지 거동은 아직 미확인이다.
방향을 유지한 작은 병진만 구현했다. 양팔 공유 물체의 회전/비틀기나 힘 동기화는 지원하지 않는다.
자연어 목표에는 실제로 검증한 손잡이와 옮길 위치 또는 작은 변위를 명시한다.

## 6. 샌드위치와 커피 목표 작성

아래 문장은 프롬프트 사용 예시이며 코드에 레시피로 등록되어 있지 않다.

> 각자 가까이에 있는 재료를 사용해, 관측된 조립 위치에 빵 한 장 → 양상추 → 빵 한 장 순서로 쌓아줘.
> 앞 단계가 실제 완료됐다는 피드백을 확인하고 다음 재료를 옮겨줘.

조립 위치는 두 로봇이 같은 물리적 장소로 해석할 수 있는 관측 대상/지그 또는 명시된 각 base 좌표로 제공한다.
workspace 숫자를 아무 변환 없이 agent2의 base 위치로 넣으면 안 된다.
두 빵은 서로 다른 ID로 추적되지만, 쌓은 후 가림이나 재검출 때문에 ID가 바뀔 수 있다.
필요하면 최종 위치에서 label+position을 검증하도록 LLM이 계획한다.

> 샌드위치가 끝나면, 관측된 컵 위에서 커피 용기를 작은 회전 단계로 기울였다가 다시 세워줘.
> 지정한 각도와 도구/용기 여유 공간을 지키고, 지원하지 않는 액체량 측정은 주장하지 마.

회전량/속도/그립·용기/컵 위치를 검증하지 않은 실제 따르기는 자동으로 안전하다고 인정되지 않는다.
주어진 클래스 가중치가 양상추·커피 용기를 실제 인식한다고 확인되지 않았다. 추가 학습된 모델이나
명확한 label alias를 설정해야 할 수 있다. 미검출을 가짜 좌표로 대신하지 않고 제한 탐색으로 처리한다.

## 7. 상태, 중지, 복구

정상 흐름: `PLANNING → WAITING_FOR_GATES → ASSIGNED → EXECUTING → SUCCEEDED`.
오류 시 `BLOCKED`를 포함한 이유가 `/cap/mission_state`에 남는다. LLM 형식 오류는 물리 권한 발행 전 `REJECTED`가 된다.
활성 임무 중 새로운 목표는 거절하고, 같은 request_id/token/command_id는 중복 실행하지 않는다.

```bash
ros2 run cap_robot cli estop
ros2 run cap_robot cli kill
ros2 run cap_robot cli reset-stop agent1
```

`reset-stop`은 gate의 E-stop 래치만 리셋하며 arm하지 않는다. Kill은 해당 프로세스에서 리셋할 수 없다.
실행 결과가 불명확한 BLOCKED 임무를 자동 이어서 수행하지 않는다. 물리 상태·물체 지지·오류 원인을
확인하고 전체 스택을 정지/재시작하여 새 임무로 시작한다. 에이전트/coordinator의 정지 상태도 재시작으로
정리한다. 하드웨어는 재시작해도 다시 명시적인 arm을 요구한다. 정지 메시지 false로 재개하지 않는다.

## 8. 로깅과 정량 평가

기본 CSV는 `~/.ros/cap_robot/metrics_<timestamp>.csv`에 생기며 매 이벤트 flush한다.
event_id로 중복 이벤트를 제거한다. 로그 기록 실패는 ROS error로 보인다.

```bash
ros2 launch cap_robot workstation.launch.py record:=true \
  bag_path:=/absolute/path/new_bag_directory
ros2 run cap_robot cli grade <CLI가-출력한-request_id> \
  --recognized yes --prompt-compliant yes --task-completed no --note '목표는 이해했지만 마지막 파지가 실패'
ros2 run cap_robot metrics_report /absolute/path/metrics_*.csv
```

| 지표 | 정의 |
|---|---|
| 추론 지연 평균/p95 | HTTP 호출 + 응답/계획 검증 경과 시간. 실패 호출 포함 |
| 구조화 응답 수용률 | JSON·허용 동작 검증에 통과한 추론 / 전체 추론 시도 |
| 명령 인식률 | 사람이 자연어 의도와 계획이 맞다고 평가한 request / 평가한 request |
| 프롬프트 의미 준수율 | 사람이 조건·순서·물체 수 등을 준수했다고 평가한 request / 평가한 request |
| 태스크 성공률 | 사람이 물리 목표 완료를 확인한 request / 평가한 request |
| 단독 제어 성공률 | 실제 완료 피드백을 받은 단독 명령 / 단독 명령 결과 |
| 공동 운반 단계 | 완료 단계 수와 호스트 dispatch 시각 차이, 실패 이유 |

사람 평가가 없으면 인식률·의미 준수율은 `null`이다. JSON 성공률을 명령 이해 정확도나 식품/액체
작업 성공률로 바꾸어 보고하지 않는다. 음성인식(STT) 정확도는 이 패키지의 측정 대상이 아니다.

rosbag에는 영상 원본을 기본 기록하지 않고 상태·관측·명령·결과·정지·TF를 담는다.
영상도 필요하면 별도로 명시한 영상 토픽을 기록한다. **실기 ROS domain에서 명령 토픽이 포함된 bag을
재생하지 않는다.** 재생은 격리된 무접속 환경이나 명령 토픽 제외 방식으로 한다.

## 9. 재현 시험

```bash
cd ~/capstone_new_ws
source install/setup.bash
colcon test --packages-select cap_robot
colcon test-result --verbose

# 로컬 DDS/HTTP를 사용하는 추가 통합 시험. 실제 LLM이나 로봇 연결 없음.
CAP_ROBOT_ROS_TESTS=1 python3 -m pytest src/cap_robot/test -q
```

제어 통합 시험은 독립 노드 프로세스, 로컬 HTTP 모의 Ollama, ROS domain 174를 사용한다.
launch 회귀 시험은 domain 176에서 설치된 `demo.launch.py`와 `workstation.launch.py`를
실제 실행해 설정 경로·9개 앱 노드·2개 TF·상태 피드백을 확인한다. workstation의 빈 인자와
사용자 지정 설정도 별도 확인한다. launch 시험에서는 임무를 보내지 않으므로 Ollama를 호출하지 않는다.
두 시험은 localhost 통신만 사용하며 해당 domain에서 다른 실험을 동시에 실행하지 않는다.
기본 colcon 시험은 네트워크 시험을 건너뛴다.
패키지와 별도로 제공되는 `VALIDATION.md`에 실제 수행한 결과와 미수행 실기 검증을 구분해 기록한다.
