# cap_robot Workstation LLM 패치

## 포함 파일

- `cap_robot/workstation_llm.py` — 터미널 자연어 대화, Ollama 호출, 최종 JSON 가이드북 검증·저장·발행
- `prompts/workstation_prompt.txt` — Workstation LLM 자연어 대화 프롬프트
- `cap_robot/llm_api.py` — 기존 코드를 유지하고 API 기반 capability catalog만 추가
- `setup.py` — `workstation_llm` 실행 항목만 추가
- `package.xml` — 기존 파일과 동일하며 수정 불필요

## 적용

현재 패키지 루트에서 대응하는 파일을 복사하거나 교체합니다.

```bash
cp cap_robot/workstation_llm.py ~/capstone_ws/src/cap_robot/cap_robot/
cp cap_robot/llm_api.py ~/capstone_ws/src/cap_robot/cap_robot/
cp prompts/workstation_prompt.txt ~/capstone_ws/src/cap_robot/prompts/
cp setup.py ~/capstone_ws/src/cap_robot/setup.py
```

`package.xml`은 현재 의존성이 충분하므로 교체할 필요가 없습니다.

## 빌드

```bash
cd ~/capstone_ws
colcon build --packages-select cap_robot --symlink-install
source install/setup.bash
```

## 실행

```bash
ros2 run cap_robot workstation_llm
```

기본값:

- 모델: `gemma4:12b`
- Ollama: `http://localhost:11434/api/chat`
- 발행 토픽: `/mission/guidebook`
- 로그: `~/capstone_ws/log/cap_robot`

모델 변경 예시:

```bash
ros2 run cap_robot workstation_llm --ros-args \
  -p llm_model:=다른_모델명
```

## 터미널 명령

- `/confirm` — 자연어로 합의한 계획을 JSON 가이드북으로 변환·발행
- `/show` — 현재 대화와 상태 표시
- `/reset` — 새 임무 시작
- `/help` — 도움말
- `/quit` — 종료

## 로그 구조

```text
~/capstone_ws/log/cap_robot/
└── workstation/
    ├── sessions/
    │   └── workstation_YYYYMMDD_HHMMSS.jsonl
    ├── missions/
    │   └── mission_YYYYMMDD_HHMMSS.json
    └── latest_session.txt
```

기본 로그 경로 생성에 실패하면 `~/.ros/cap_robot/log`를 사용합니다.
