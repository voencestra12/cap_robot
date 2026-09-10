<!-- 역할: 실제 수행한 검증과 미수행 영역 기록. 인터페이스: pytest/colcon 결과. -->
# 검증 기록

재검증일: **2026-09-10**, 버전 **1.0.1**. 결과 원본은 [test-results.xml](test-results.xml),
[colcon-pytest.xml](colcon-pytest.xml), [모의 지표 집계](metrics-smoke-summary.json)에 포함했다.

| 검증 | 결과 |
|---|---|
| ROS 2 Humble `colcon build` | 패키지 1개 빌드/설치 성공 |
| 기본 `colcon test` | 57개 수집, **53 passed / 4 skipped** |
| 네트워크 시험을 켠 전체 pytest | **57 passed**, 27.19초 |
| 실제 설치본 `demo.launch.py` | 9개 앱 노드·2개 TF 기동, 모든 설정 경로·양팔 관측·워크스테이션 피드백 확인 |
| workstation launch 설정 | include의 빈 값은 기본 YAML 사용, 사용자 지정 YAML은 3개 노드에 동일 전달 |
| launch 종료 | 3개 실행 경우의 모든 자식 프로세스 정상 종료, 강제 종료·미처리 Traceback 없음 |
| 수정 전 실패 재현 | 새 demo 회귀 시험이 1.0.0 설치본에서 `IsADirectoryError`를 검출해 실패 |
| 큰 HTTP 응답 외피 | generate/chat 호환, 중복 키·비유한 수 거부, 내부/외피 크기 제한 11개 시험 통과 |
| 실제 localhost DDS + 독립 프로세스 | 두 agent 순차 태스크, 중복 목표 억제, 공동 운반, 동작 중 E-stop 통과 |
| 공동 운반 좌표 | agent2 base를 180도 회전; 반대 local X 이동으로 같은 world X 병진 검증 |
| CSV 정지 실패 집계 | actuator 결과 9개 중 8 성공, 의도적 중단 1 실패까지 기록 |
| 빨간 손잡이 | 합성 영상의 빨간 손잡이 두 개를 YOLO+HSV 경로에서 독립 검출 |
| xArm6 URDF (09-08 검증 기록) | 원본 xacro 생성 성공, agent1_joint1..6 이름 일치 |
| 설치 파일·설정 | Python 구문, YAML, package.xml 검증 성공 |
| 모델 보존 (09-08 검증 기록) | 원본과 두 모델 SHA-256 동일, 중복 .pt.1 제외 |
| 종료 로그 | 통합 시험 자식 노드 로그에 미처리 Traceback 없음 |

기본 colcon 시험은 로컬 네트워크 사용을 명시적으로 켜지 않아 제어 통합 시험 1개와
launch 회귀 시험 3개를 건너뛴다. 추가 전체 실행에서는 그 시험까지 수행하여 총 57개가 통과했다.

09-08의 **42 passed / 1 skipped**, 네트워크 포함 **43 passed** 기록은 당시 실제 결과지만
launch 파일을 실행하지 않아 기본 demo 결함을 놓쳤다. 이번에는 동일 회귀 시험이 수정 전에는
실패하고 수정 후에는 통과함을 확인했다. 자세한 원인과 수정 범위는 [1.0.1 수정 기록](LAUNCH_FIX.md),
실제 로그는 [수정 전 오류](launch-before.txt), [수정 후 demo](demo-launch.log),
[빈 값 include](workstation-empty-launch.log), [사용자 지정 설정](workstation-custom-launch.log)에 있다.

주요 회귀 범위: Z 하한·XYZ/도구 여유·NaN/Inf·속도·토큰/ID·시각 검증,
명령 중복 실행 억제, 비상정지/영구 Kill, 동작 중 권한 회수/heartbeat 상실,
GO 없는 공동 운반 차단, 공유 물체의 상대 변위/회전 불일치 거절,
DAG 순환/의존성, 제한 탐색, 정확한 객체 ID, CSV 중복/수동 평가 분모 처리.

모의 모델 요청은 총 8회였다. 측정된 수 밀리초 HTTP 지연과 100% 구조화 응답 수용률은
시험용 고정 응답 서버의 결과이며 실제 LLM 성능이 아니다.
명령 의미 인식률·프롬프트 의미 준수율·물리 태스크 성공률은 사람 평가가 없어 `null`이다.

환경: Python 3.10.12, ROS 2 Humble, pytest 6.2.5, numpy 1.24.4,
scipy 1.8.0, requests 2.32.5, PyYAML 5.4.1.
API 검토 환경에는 xArm Python SDK 1.17.3, ultralytics 8.4.7, OpenCV 4.13.0이 설치되어 있었다.

실제 xArm, RealSense, 첨부 YOLO 가중치, 실제 Ollama 모델, 샌드위치 쌓기,
커피 따르기, 실제 하중이 있는 바구니 공동 운반은 이 환경에서 실행하지 않았다.
실기 활성화를 허용한 적도 없으며, SDK 무접속 backend와 테스트 전용 HTTP 응답을 사용했다.

실제 바구니 무게·손잡이 간 거리·그리퍼 firmware·hand-eye 교정·도구 외곽·작업 공간은
별도로 확인해야 한다. TCP 펜스는 로봇 링크 전체의 충돌 검사나 물리 비상정지 회로를 대체하지 않는다.
공동 운반의 시각 장벽은 하드 실시간 궤적/힘 동기화를 보증하지 않는다.
