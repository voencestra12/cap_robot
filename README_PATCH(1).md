# cap_robot Workstation LLM 간소화 패치

## 포함 파일

- `cap_robot/workstation_llm.py`
- `cap_robot/llm_api.py`
- `prompts/workstation_prompt.txt`
- `setup.py`

## 적용

```bash
cp cap_robot/workstation_llm.py ~/capstone_ws/src/cap_robot/cap_robot/
cp cap_robot/llm_api.py ~/capstone_ws/src/cap_robot/cap_robot/
cp prompts/workstation_prompt.txt ~/capstone_ws/src/cap_robot/prompts/
cp setup.py ~/capstone_ws/src/cap_robot/setup.py

cd ~/capstone_ws
colcon build --packages-select cap_robot --symlink-install
source install/setup.bash
```

## 실행

```bash
ros2 run cap_robot workstation_llm
```

기본 모델은 `gemma4:12b`, Ollama 주소는 `http://localhost:11434`입니다.

## 명령

- `/confirm`: 현재 대화를 JSON 가이드북으로 변환, 저장, 발행
- `/show`: 현재 대화 표시
- `/reset`: 현재 대화 초기화
- `/quit`: 종료

## 로그

- 세션 로그: `~/capstone_ws/log/cap_robot/workstation_날짜_시간.jsonl`
- 최종 가이드북: `~/capstone_ws/log/cap_robot/missions/mission_날짜_시간.json`
