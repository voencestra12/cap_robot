<!-- 역할: 1.0.0 실행 결함의 원인과 1.0.1 수정 범위. 인터페이스: launch 인자 및 회귀 시험. -->
# 1.0.1 수정 기록 — 2026-09-10

1.0.0의 기본 명령 `ros2 launch cap_robot demo.launch.py`에서
`workstation_node`, `cooperative_node`, `metrics`가 모두
`IsADirectoryError: [Errno 21] Is a directory: '.'`로 종료되는 문제를 실제 Humble에서 재현했다.

agent와 workstation launch가 같은 `config_file` 인자를 선언했다. include는 자동으로
설정 범위를 분리하지 않으며, `DeclareLaunchArgument.execute`는 이미 설정된 값을 유지한다.
먼저 선언된 agent의 빈 기본값 때문에 workstation의 기본 경로가 적용되지 않았다.
결과적으로 `load_config("")`가 현재 디렉터리를 파일로 열려고 했다.

이전의 42개 일반 시험과 1개 DDS/HTTP 통합 시험은 개별 노드 main을 직접 실행했으며,
실제 launch 파일을 실행하지 않았다. 이전 통과 수치는 이 launch 경로의 정상 동작을 입증하지 못했다.

| 수정 | 결과 |
|---|---|
| workstation launch 인자를 `workstation_config_file`로 분리 | agent 인자와 이름 충돌 제거. 노드의 `config_file` 파라미터는 유지 |
| 빈 workstation 인자에 기본 YAML 경로 적용 | 빈 경로가 현재 디렉터리로 해석되는 오류 방지 |
| demo의 각 include를 `GroupAction(scoped=True, forwarding=False)`로 격리 | agent/config/camera 값의 include 간 전파 차단 |
| demo에서 세 설정 경로 명시 | 양쪽 agent와 workstation이 각각 지정된 기본 YAML 사용 |
| 유휴 노드의 executor 대기를 0.1초씩 제한 | 타이머 없는 metrics도 종료 신호를 처리할 기회 확보 |
| 실제 설치본 launch 회귀 시험 3개 | 기본 demo, include로 전달한 빈 workstation 인자, 사용자 지정 workstation 설정 및 정상 종료 검증 |
| 큰 Ollama 외피도 엄격 JSON 파서 사용 | 128KiB 초과 외피의 중복 키·NaN/Infinity·숫자 오버플로 거부 |
| 외피 파싱 시험 11개 추가 | generate/chat 호환과 계획 128KiB·외피 1MiB 제한 유지 검증 |

새 demo 회귀 시험이 수정 전 설치본에서 `IsADirectoryError`를 검출해 실패하는 것도 확인했다.
수정 후 실행 결과와 시험 원본은 [검증 기록](VALIDATION.md)에 기록했다.

launch 기동 결함을 수정한 후의 종료 시험에서 유휴 metrics가 SIGINT/SIGTERM을 처리하지
못하는 문제도 확인했다. rclpy 자체 신호 처리를 끈 상태에서 `executor.spin()`의 native DDS
대기가 끝나지 않아 Python 신호 처리가 지연될 수 있었다. 공통 실행 루프를
`spin_once(timeout_sec=0.1)` 반복으로 바꿔 종료 신호를 처리하게 했다. 이 대기 제한을
전체 종료 시간이나 물리 정지 시간의 보장으로 해석하지 않는다.

워크스테이션에 사용자 YAML을 전달하던 실행 명령은 다음과 같이 변경한다.

```bash
ros2 launch cap_robot workstation.launch.py workstation_config_file:=/absolute/path/workstation.yaml
```

LLM 응답 내부의 계획은 이전에도 엄격 파서를 통과했다. 큰 외피의 파싱 불일치를
수정한 사실을 안전 게이트 우회 결함의 입증으로 해석하지 않는다.
