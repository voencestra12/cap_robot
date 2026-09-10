# 실기 통합 체크리스트

아래 순서를 건너뛰지 않는 것을 권장합니다.

## 1. 공통 통신

- 세 컴퓨터의 `ROS_DOMAIN_ID`, DDS 구현, 네트워크 인터페이스를 동일하게 맞춘다.
- `ros2 node list`에서 연구실/노트북 1/노트북 2 노드가 서로 보이는지 확인한다.
- 방화벽과 Wi-Fi 절전으로 DDS multicast/unicast가 끊기지 않는지 확인한다.

## 2. Dry-run 강제

- `config/agent1.yaml`, `config/agent2.yaml`의 `dry_run`을 모두 `true`로 바꾼다.
- 실제 바구니와 주변 장애물을 작업공간에서 치운다.
- Agent 1/2가 올바른 IP 로봇에 연결되는지 로그로 확인한다.

## 3. TF와 카메라

- `workstation.launch.py` 실행 후 `workspace_0`가 안정적으로 갱신되는지 확인한다.
- 아래 두 TF의 translation/rotation이 실제 설치 방향과 맞는지 확인한다.
  - `robot_1_base <- workspace_0`
  - `robot_2_base <- workspace_0`
- 각 노트북의 hand-eye TF와 RealSense optical frame이 끊기지 않는지 확인한다.

## 4. 손잡이 검출

- `/perception/yolo_extra`에서 `valid=true`, object 2개가 유지되는지 확인한다.
- 손잡이를 움직였을 때 workspace 좌표의 축과 부호가 실제 방향과 맞는지 확인한다.
- 각 Agent에 대해 선택 가능한 손잡이의 `within_safety_box=true`를 확인한다.
- 빨간 물체가 주변에 있으면 잘못된 두 contour가 선택될 수 있으므로 제거한다.

## 5. LLM 계획

- Workstation에 짧고 수치가 명확한 협동 명령을 입력한다.
- Guidebook의 `execution_mode=cooperative`, 두 participants, 서로 다른 target,
  검증 가능한 actions를 확인한다.
- `/mission/task_status`에서 두 Agent의 `ACCEPTED` 후 Workstation `APPROVED`가 오는지 확인한다.
- 고의로 접근 불가능한 설정을 넣은 시험에서는 한 번만 revision이 증가하고 이후
  `FAILED`로 끝나는지 확인한다.

## 6. Barrier dry-run

- 각 action index에서 두 Agent의 `READY`가 모인 뒤 `GO`가 한 번만 나오는지 확인한다.
- 한 Agent를 중지했을 때 다른 Agent가 `step_sync_timeout_sec` 뒤 실패하고 추가 action을
  실행하지 않는지 확인한다.
- `cooperative_move_relative`의 workspace offset이 두 Agent에서 각자의 TF를 거쳐 서로
  다른 base 좌표 명령으로 변환되는지 로그를 비교한다.

## 7. 제한된 실기 시험

- 속도를 낮추고, 비상정지 버튼을 잡은 작업자 두 명이 각 로봇을 감시한다.
- 먼저 그리퍼 열기와 높은 접근까지만 검증한다.
- 다음으로 손잡이 접근과 동기 파지를 검증한다.
- 마지막으로 작은 Z 이동부터 시작해 X/Y/Z 협동 이동 범위를 단계적으로 늘린다.
- 한쪽 실패 시 자동 대피가 없으므로 두 팔과 바구니를 수동으로 안전 상태로 만든다.

## 알려진 실기 리스크

- 이 패키지의 barrier는 ROS/DDS 단계 동기화이지 하드 실시간 서보 동기화가 아니다.
- xArm driver와 직접 `XArmAPI` 연결을 함께 쓰는 기존 구조를 유지했다. driver를 상태
  공급 외 제어에 사용하지 않는다.
- identity command frame 가정이 실제 SDK 좌표와 다르면 이동 전 반드시 TF/SDK 좌표를
  다시 교정해야 한다.
- 손잡이 검출은 현재 빨간색 HSV 방식이라 조명, 반사, 유사 색상에 영향을 받는다.
- 자동 재시도·자동 대피를 제거했으므로 실패 후 재시작 전에 로봇과 파지 상태를 직접
  확인해야 한다.
