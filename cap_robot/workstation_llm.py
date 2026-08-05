"""정보가 충분해지면 가이드북을 자동 발행하는 Workstation LLM 노드."""

import json
import time
from datetime import datetime
from pathlib import Path

import requests
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from cap_robot.llm_api import get_capability_catalog


class WorkstationLLM(Node):
    def __init__(self):
        super().__init__('workstation_llm')

        self.declare_parameter('llm_model', 'gemma4:12b')
        self.declare_parameter(
            'ollama_url',
            'http://localhost:11434',
        )
        self.declare_parameter(
            'guidebook_topic',
            '/mission/guidebook',
        )
        self.declare_parameter(
            'log_dir',
            '~/capstone_ws/log/cap_robot',
        )
        self.declare_parameter(
            'timeout_sec',
            300.0,
        )

        self.model = str(
            self.get_parameter('llm_model').value
        )

        self.ollama_url = str(
            self.get_parameter('ollama_url').value
        ).rstrip('/')

        self.topic = str(
            self.get_parameter('guidebook_topic').value
        )

        self.timeout = float(
            self.get_parameter('timeout_sec').value
        )

        # llm_api.py에서 capability 목록을 가져온다.
        self.catalog = get_capability_catalog()
        self.capabilities = set(self.catalog)

        self.system_prompt = self._load_prompt()

        # 늦게 실행된 Agent도 마지막 가이드북을 받을 수 있도록 설정한다.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(
            String,
            self.topic,
            qos,
        )

        # 로그 저장 경로
        self.log_dir = Path(
            str(self.get_parameter('log_dir').value)
        ).expanduser()

        self.mission_dir = self.log_dir / 'missions'

        self.mission_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        stamp = datetime.now().strftime(
            '%Y%m%d_%H%M%S'
        )

        self.log_file = (
            self.log_dir
            / f'workstation_{stamp}.jsonl'
        )

        self._reset()
        self._log(
            'SESSION_STARTED',
            model=self.model,
            topic=self.topic,
        )

    def _load_prompt(self):
        """설치된 workstation_prompt.txt를 읽는다."""

        prompt_path = (
            Path(
                get_package_share_directory(
                    'cap_robot'
                )
            )
            / 'prompts'
            / 'workstation_prompt.txt'
        )

        template = prompt_path.read_text(
            encoding='utf-8'
        )

        capability_text = '\n'.join(
            (
                f"- {name}: "
                f"{info.get('description', '')}"
            )
            for name, info
            in sorted(self.catalog.items())
        )

        return template.replace(
            '{{CAPABILITY_CATALOG}}',
            capability_text,
        )

    def _log(self, event, **data):
        """한 사건을 JSONL 로그에 저장한다."""

        record = {
            'time': (
                datetime.now()
                .astimezone()
                .isoformat(timespec='seconds')
            ),
            'event': event,
            **data,
        }

        try:
            with self.log_file.open(
                'a',
                encoding='utf-8',
            ) as file:
                file.write(
                    json.dumps(
                        record,
                        ensure_ascii=False,
                    )
                    + '\n'
                )

        except OSError as exc:
            self.get_logger().error(
                f'로그 저장 실패: {exc}'
            )

    def _reset(self):
        """대화 이력을 초기화한다."""

        self.messages = [
            {
                'role': 'system',
                'content': self.system_prompt,
            }
        ]

    def _ask_ollama(self):
        """현재 대화 이력을 Ollama에 전달한다."""

        payload = {
            'model': self.model,
            'messages': self.messages,
            'stream': False,

            # 충분한 추론을 허용한다.
            'think': False,

            # 모든 응답을 JSON으로 받는다.
            'format': 'json',

            # 요청 사이에 모델을 메모리에 유지한다.
            'keep_alive': '10m',

            'options': {
                'num_ctx': 8192,
                'num_predict': 4096,
                'temperature': 0.2,
            },
        }

        print(
            '\nLLM 응답 생성 중...',
            flush=True,
        )

        started = time.monotonic()

        try:
            response = requests.post(
                f'{self.ollama_url}/api/chat',
                json=payload,
                timeout=(10, self.timeout),
            )

            response.raise_for_status()
            data = response.json()

        except requests.RequestException as exc:
            if exc.response is not None:
                detail = exc.response.text.strip()
            else:
                detail = str(exc)

            raise RuntimeError(
                f'Ollama 요청 실패: {detail}'
            ) from exc

        except ValueError as exc:
            raise RuntimeError(
                f'Ollama 응답 해석 실패: {exc}'
            ) from exc

        message = data.get(
            'message',
            {},
        )

        content = str(
            message.get('content', '')
        ).strip()

        thinking = str(
            message.get('thinking', '')
        ).strip()

        if not content:
            raise RuntimeError(
                'Ollama가 최종 답변을 '
                '반환하지 않았습니다. '
                f'thinking 길이={len(thinking)}'
            )

        self._log(
            'LLM_RESPONSE',
            latency_sec=round(
                time.monotonic() - started,
                2,
            ),
            content=content,
        )

        return content

    @staticmethod
    def _parse(raw):
        """LLM 응답에서 상태, 메시지, 가이드북을 읽는다."""

        try:
            result = json.loads(raw)

        except json.JSONDecodeError:
            # JSON 앞뒤에 문장이 붙은 경우를 위한 최소 보정
            start = raw.find('{')
            end = raw.rfind('}')

            if start < 0 or end <= start:
                raise RuntimeError(
                    'LLM 응답에서 JSON을 '
                    '찾을 수 없습니다.'
                )

            result = json.loads(
                raw[start:end + 1]
            )

        if not isinstance(result, dict):
            raise RuntimeError(
                'LLM 응답은 JSON 객체여야 합니다.'
            )

        status = result.get('status')

        message = str(
            result.get('message', '')
        ).strip()

        guidebook = result.get('guidebook')

        if status not in (
            'need_info',
            'ready',
        ):
            raise RuntimeError(
                'status는 need_info 또는 '
                'ready여야 합니다.'
            )

        if not message:
            raise RuntimeError(
                'message가 비어 있습니다.'
            )

        if (
            status == 'need_info'
            and guidebook is not None
        ):
            raise RuntimeError(
                'need_info일 때 guidebook은 '
                'null이어야 합니다.'
            )

        if (
            status == 'ready'
            and not isinstance(guidebook, dict)
        ):
            raise RuntimeError(
                'ready일 때 guidebook이 필요합니다.'
            )

        return status, message, guidebook

    def _publish(self, guidebook):
        """가이드북을 최소 검증한 뒤 저장하고 발행한다."""

        for key in (
            'goal',
            'tasks',
            'completion_condition',
        ):
            if key not in guidebook:
                raise RuntimeError(
                    f'가이드북 필드 누락: {key}'
                )

        tasks = guidebook['tasks']

        if (
            not isinstance(tasks, list)
            or not tasks
        ):
            raise RuntimeError(
                'tasks는 비어 있지 않은 '
                '배열이어야 합니다.'
            )

        for index, task in enumerate(tasks):
            if not isinstance(task, dict):
                raise RuntimeError(
                    f'tasks[{index}]는 '
                    '객체여야 합니다.'
                )

            required_fields = (
                'task_id',
                'description',
                'depends_on',
                'required_capabilities',
                'success_condition',
            )

            for key in required_fields:
                if key not in task:
                    raise RuntimeError(
                        f'tasks[{index}] '
                        f'필드 누락: {key}'
                    )

            capabilities = task[
                'required_capabilities'
            ]

            if not isinstance(
                capabilities,
                list,
            ):
                raise RuntimeError(
                    f'tasks[{index}].'
                    'required_capabilities는 '
                    '배열이어야 합니다.'
                )

            unknown = (
                set(capabilities)
                - self.capabilities
            )

            if unknown:
                raise RuntimeError(
                    '지원되지 않는 capability: '
                    f'{sorted(unknown)}'
                )

        mission_id = (
            'mission_'
            + datetime.now().strftime(
                '%Y%m%d_%H%M%S_%f'
            )
        )

        # LLM이 만든 mission_id가 있더라도
        # 실제 ID는 프로그램에서 다시 지정한다.
        guidebook['mission_id'] = mission_id

        mission_path = (
            self.mission_dir
            / f'{mission_id}.json'
        )

        mission_path.write_text(
            json.dumps(
                guidebook,
                ensure_ascii=False,
                indent=2,
            ),
            encoding='utf-8',
        )

        msg = String()
        msg.data = json.dumps(
            guidebook,
            ensure_ascii=False,
        )

        self.publisher.publish(msg)

        self._log(
            'PUBLISHED',
            mission_id=mission_id,
            file=str(mission_path),
            guidebook=guidebook,
        )

        print(
            f'발행 완료: {self.topic}'
        )
        print(
            f'저장 위치: {mission_path}'
        )

    def _handle_user(self, text):
        """사용자 입력 하나를 처리한다."""

        self.messages.append(
            {
                'role': 'user',
                'content': text,
            }
        )

        self._log(
            'USER',
            content=text,
        )

        try:
            raw = self._ask_ollama()

            status, message, guidebook = (
                self._parse(raw)
            )

        except (
            RuntimeError,
            json.JSONDecodeError,
        ) as exc:
            # 실패한 입력은 대화 이력에서 제거한다.
            self.messages.pop()

            self._log(
                'ERROR',
                message=str(exc),
            )

            print(
                f'오류> {exc}'
            )
            return

        # 모델이 이전에 어떤 질문을 했는지
        # 기억하도록 JSON 원문을 저장한다.
        self.messages.append(
            {
                'role': 'assistant',
                'content': raw,
            }
        )

        self._log(
            'ASSISTANT',
            status=status,
            message=message,
        )

        # 사용자에게는 자연어 message만 보여준다.
        print(
            f'\nLLM> {message}'
        )

        if status == 'ready':
            try:
                self._publish(guidebook)

                # 발행이 끝나면 다음 임무를 위해 초기화한다.
                self._reset()

            except RuntimeError as exc:
                self._log(
                    'ERROR',
                    message=str(exc),
                )

                print(
                    f'오류> {exc}'
                )

    def run(self):
        print(
            f'모델: {self.model}'
        )
        print(
            f'Ollama: {self.ollama_url}'
        )
        print(
            '/reset: 대화 초기화  '
            '/quit: 종료'
        )

        while rclpy.ok():
            try:
                text = input(
                    '\n사용자> '
                ).strip()

            except (
                EOFError,
                KeyboardInterrupt,
            ):
                break

            if not text:
                continue

            if text == '/quit':
                break

            if text == '/reset':
                self._reset()
                self._log('RESET')

                print(
                    '대화를 초기화했습니다.'
                )
                continue

            self._handle_user(text)


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = WorkstationLLM()
        node.run()

    except Exception as exc:
        print(
            f'시작 실패> {exc}'
        )

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()