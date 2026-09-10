<!-- 역할: 설치·운용·설계 안내. 인터페이스: 문서 링크와 ros2 명령. -->
# cap_robot — ROS 2 Humble xArm6 듀얼 LLM 협업

최신 `cap_robot-260907.zip`을 전체 검토한 후 새 패키지로 재구성했습니다.
각 로봇의 독립 비전/LLM, 워크스테이션 DAG, 실시간 상태 피드백,
별도 안전 최종 게이트, 제한된 양팔 공동 운반, 예외 기반 재계획과 탐색을 제공합니다.

**기본값은 로봇에 연결하지 않는 dry_run과 모의 관측입니다.** 기본 LLM은
원본 설정의 모델명을 유지했으며 해당 Ollama 모델을 각 컴퓨터에 준비해야 합니다.
실제 카메라·로봇·용기에서의 검증은 자동 소프트웨어 시험과 별개입니다.

상세 내용은 아래 문서에 있습니다.

- [디렉터리 구조 → 파일별 전체 코드](docs/FILE_BY_FILE.md)
- [설치·분산 실행·실기 프로필·로그 평가](docs/OPERATIONS.md)
- [원본 분석·설계·요구사항 비교표](docs/DESIGN_REVIEW.md)
- [ROS 토픽·서비스 프로토콜](docs/PROTOCOL.md)
- [검증 결과: 전체 57개 시험 통과](docs/VALIDATION.md)
- [1.0.1 수정: demo launch 충돌·유휴 종료·큰 JSON 외피](docs/LAUNCH_FIX.md)
