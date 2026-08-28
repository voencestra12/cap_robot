"""자연어 임무를 해석하고 전체 가이드북을 발행하는 Workstation LLM 노드."""

from __future__ import annotations

import copy
import json
import threading
import time
from datetime import datetime
from pathlib import Path

import requests
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from cap_robot.llm_api import get_capability_catalog, validate_cooperative_actions


class WorkstationLLM(Node):
    """중앙 LLM 계획, Agent 검토 수집, 최대 1회 재계획을 담당합니다."""

    def __init__(self):
        super().__init__('workstation_llm')

        self.declare_parameter('llm_model', 'gemma4:12b')
        self.declare_parameter('ollama_url', 'http://localhost:11434')
        self.declare_parameter('guidebook_topic', '/mission/guidebook')
        self.declare_parameter('task_status_topic', '/mission/task_status')
        self.declare_parameter('extra_perception_topic', '/perception/yolo_extra')
        self.declare_parameter('workspace_frame', 'workspace_0')
        self.declare_parameter('extra_perception_max_age_sec', 2.0)
        self.declare_parameter('plan_review_timeout_sec', 90.0)
        self.declare_parameter('log_dir', '~/capstone_ws/log/cap_robot')
        self.declare_parameter('timeout_sec', 300.0)

        self.model = str(self.get_parameter('llm_model').value)
        self.ollama_url = str(self.get_parameter('ollama_url').value).rstrip('/')
        self.topic = str(self.get_parameter('guidebook_topic').value)
        self.task_status_topic = str(self.get_parameter('task_status_topic').value)
        self.extra_perception_topic = str(
            self.get_parameter('extra_perception_topic').value
        )
        self.workspace_frame = str(self.get_parameter('workspace_frame').value).strip()
        self.extra_perception_max_age_sec = float(
            self.get_parameter('extra_perception_max_age_sec').value
        )
        self.plan_review_timeout_sec = float(
            self.get_parameter('plan_review_timeout_sec').value
        )
        self.timeout = float(self.get_parameter('timeout_sec').value)

        self.catalog = get_capability_catalog()
        self.capabilities = set(self.catalog)
        self.system_prompt = self._load_prompt()

        guidebook_qos = QoSProfile(depth=1)
        guidebook_qos.reliability = ReliabilityPolicy.RELIABLE
        guidebook_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(String, self.topic, guidebook_qos)
        self.status_publisher = self.create_publisher(
            String,
            self.task_status_topic,
            20,
        )
        self.create_subscription(
            String,
            self.task_status_topic,
            self._task_status_callback,
            20,
        )
        self.create_subscription(
            String,
            self.extra_perception_topic,
            self._extra_perception_callback,
            10,
        )
        self.create_timer(1.0, self._check_review_timeout)

        self.log_dir = Path(str(self.get_parameter('log_dir').value)).expanduser()
        self.mission_dir = self.log_dir / 'missions'
        self.mission_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = self.log_dir / f'workstation_{stamp}.jsonl'

        self._state_lock = threading.Lock()
        self._llm_lock = threading.Lock()
        self._latest_extra_state = None
        self._latest_extra_received_monotonic = 0.0
        self._active_review = None
        self._executor = None
        self._spin_thread = None
        self._reset()
        self._log(
            'SESSION_STARTED',
            model=self.model,
            topic=self.topic,
            task_status_topic=self.task_status_topic,
            extra_perception_topic=self.extra_perception_topic,
        )

    def _load_prompt(self):
        prompt_path = (
            Path(get_package_share_directory('cap_robot'))
            / 'prompts'
            / 'workstation_prompt.txt'
        )
        template = prompt_path.read_text(encoding='utf-8')
        capability_text = '\n'.join(
            f"- {name}: {info.get('description', '')}"
            for name, info in sorted(self.catalog.items())
        )
        return template.replace('{{CAPABILITY_CATALOG}}', capability_text)

    def _log(self, event, **data):
        record = {
            'time': datetime.now().astimezone().isoformat(timespec='seconds'),
            'event': event,
            **data,
        }
        try:
            with self.log_file.open('a', encoding='utf-8') as file:
                file.write(json.dumps(record, ensure_ascii=False) + '\n')
        except OSError as error:
            self.get_logger().error(f'로그 저장 실패: {error}')

    def _reset(self):
        self.messages = [{'role': 'system', 'content': self.system_prompt}]

    def _extra_perception_callback(self, msg):
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict):
                raise ValueError('인식 상태는 JSON 객체여야 합니다.')
            with self._state_lock:
                self._latest_extra_state = state
                self._latest_extra_received_monotonic = time.monotonic()
        except (ValueError, json.JSONDecodeError) as error:
            self.get_logger().warn(f'추가 인식 상태 해석 실패: {error}')

    def _world_snapshot(self):
        with self._state_lock:
            state = copy.deepcopy(self._latest_extra_state)
            age = time.monotonic() - self._latest_extra_received_monotonic
        if state is None:
            return {
                'valid': False,
                'frame_id': self.workspace_frame,
                'reason': 'yolo_extra_perception 데이터를 아직 받지 못함',
            }
        state['received_age_sec'] = round(age, 3)
        if age > self.extra_perception_max_age_sec:
            state['valid'] = False
            state['reason'] = (
                f'추가 인식 데이터가 오래됨: {age:.2f}s > '
                f'{self.extra_perception_max_age_sec:.2f}s'
            )
        return state

    def _ask_ollama(self, messages=None):
        payload = {
            'model': self.model,
            'messages': messages if messages is not None else self.messages,
            'stream': False,
            'think': False,
            'format': 'json',
            'keep_alive': '10m',
            'options': {
                'num_ctx': 8192,
                'num_predict': 4096,
                'temperature': 0.2,
            },
        }
        print('\nLLM 응답 생성 중...', flush=True)
        started = time.monotonic()
        try:
            with self._llm_lock:
                response = requests.post(
                    f'{self.ollama_url}/api/chat',
                    json=payload,
                    timeout=(10, self.timeout),
                )
                response.raise_for_status()
                data = response.json()
        except requests.RequestException as error:
            detail = (
                error.response.text.strip()
                if error.response is not None
                else str(error)
            )
            raise RuntimeError(f'Ollama 요청 실패: {detail}') from error
        except ValueError as error:
            raise RuntimeError(f'Ollama 응답 해석 실패: {error}') from error

        message = data.get('message', {})
        content = str(message.get('content', '')).strip()
        thinking = str(message.get('thinking', '')).strip()
        if not content:
            raise RuntimeError(
                'Ollama가 최종 답변을 반환하지 않았습니다. '
                f'thinking 길이={len(thinking)}'
            )
        self._log(
            'LLM_RESPONSE',
            latency_sec=round(time.monotonic() - started, 2),
            content=content,
        )
        return content

    @staticmethod
    def _parse(raw):
        try:
            result = json.loads(raw)
        except json.JSONDecodeError:
            start, end = raw.find('{'), raw.rfind('}')
            if start < 0 or end <= start:
                raise RuntimeError('LLM 응답에서 JSON을 찾을 수 없습니다.')
            result = json.loads(raw[start:end + 1])
        if not isinstance(result, dict):
            raise RuntimeError('LLM 응답은 JSON 객체여야 합니다.')
        status = result.get('status')
        message = str(result.get('message', '')).strip()
        guidebook = result.get('guidebook')
        if status not in ('need_info', 'ready'):
            raise RuntimeError('status는 need_info 또는 ready여야 합니다.')
        if not message:
            raise RuntimeError('message가 비어 있습니다.')
        if status == 'need_info' and guidebook is not None:
            raise RuntimeError('need_info일 때 guidebook은 null이어야 합니다.')
        if status == 'ready' and not isinstance(guidebook, dict):
            raise RuntimeError('ready일 때 guidebook이 필요합니다.')
        return status, message, guidebook

    @staticmethod
    def _validate_dependencies(tasks):
        task_ids = [str(task.get('task_id', '')).strip() for task in tasks]
        if any(not task_id for task_id in task_ids):
            raise RuntimeError('모든 task_id는 비어 있지 않아야 합니다.')
        if len(set(task_ids)) != len(task_ids):
            raise RuntimeError('task_id는 중복될 수 없습니다.')
        completed = set()
        for index, task in enumerate(tasks):
            depends_on = task.get('depends_on')
            if not isinstance(depends_on, list):
                raise RuntimeError(f'tasks[{index}].depends_on은 배열이어야 합니다.')
            unknown = set(map(str, depends_on)) - set(task_ids)
            if unknown:
                raise RuntimeError(f'존재하지 않는 의존 task: {sorted(unknown)}')
            if not set(map(str, depends_on)).issubset(completed):
                raise RuntimeError('tasks는 의존성 순서대로 정렬되어야 하며 순환할 수 없습니다.')
            completed.add(task_ids[index])

    def _validate_cooperative_task(self, task, index, world_state):
        required_cooperative = {
            'dual_arm_grasp',
            'cooperative_transport',
            'synchronized_release',
        }
        missing_capabilities = required_cooperative - set(
            task.get('required_capabilities', [])
        )
        if missing_capabilities:
            raise RuntimeError(
                f'tasks[{index}] 협동 capability 누락: '
                f'{sorted(missing_capabilities)}'
            )
        participants = task.get('participants')
        if (
            not isinstance(participants, list)
            or len(participants) != 2
            or len(set(map(str, participants))) != 2
        ):
            raise RuntimeError(
                f'tasks[{index}].participants는 서로 다른 Agent 2개여야 합니다.'
            )
        participants = [str(value).strip() for value in participants]
        if any(not value for value in participants):
            raise RuntimeError(f'tasks[{index}].participants 값이 비어 있습니다.')

        targets = task.get('targets_by_agent')
        if not isinstance(targets, dict) or set(targets) != set(participants):
            raise RuntimeError(
                f'tasks[{index}].targets_by_agent는 각 participant를 정확히 포함해야 합니다.'
            )
        target_names = [str(targets[agent]).strip() for agent in participants]
        if len(set(target_names)) != 2:
            raise RuntimeError('양팔은 서로 다른 두 손잡이를 선택해야 합니다.')
        if not world_state.get('valid'):
            raise RuntimeError(
                '협동 계획에는 최신 유효 손잡이 인식이 필요합니다: '
                f"{world_state.get('reason', 'unknown')}"
            )
        objects = world_state.get('objects', {})
        reachability = world_state.get('reachability', {})
        for agent_id, target_name in zip(participants, target_names):
            if target_name not in objects:
                raise RuntimeError(f'인식되지 않은 협동 target: {target_name}')
            agent_reach = reachability.get(agent_id, {})
            target_reach = agent_reach.get('handles', {}).get(target_name, {})
            if not target_reach.get('within_safety_box', False):
                raise RuntimeError(
                    f'{agent_id}가 선택된 {target_name}에 안전하게 접근할 수 없습니다.'
                )

        task['participants'] = participants
        task['targets_by_agent'] = {
            agent: str(targets[agent]).strip() for agent in participants
        }
        task['coordinator_id'] = min(participants)
        task['actions'] = validate_cooperative_actions(task.get('actions'))

    def _validate_guidebook(self, guidebook):
        if not isinstance(guidebook, dict):
            raise RuntimeError('guidebook은 객체여야 합니다.')
        for key in ('goal', 'tasks', 'completion_condition'):
            if key not in guidebook:
                raise RuntimeError(f'가이드북 필드 누락: {key}')
        tasks = guidebook['tasks']
        if not isinstance(tasks, list) or not tasks:
            raise RuntimeError('tasks는 비어 있지 않은 배열이어야 합니다.')
        self._validate_dependencies(tasks)
        world_state = self._world_snapshot()
        cooperative_count = 0
        for index, task in enumerate(tasks):
            if not isinstance(task, dict):
                raise RuntimeError(f'tasks[{index}]는 객체여야 합니다.')
            for key in (
                'task_id',
                'description',
                'depends_on',
                'required_capabilities',
                'success_condition',
            ):
                if key not in task:
                    raise RuntimeError(f'tasks[{index}] 필드 누락: {key}')
            capabilities = task['required_capabilities']
            if not isinstance(capabilities, list):
                raise RuntimeError(
                    f'tasks[{index}].required_capabilities는 배열이어야 합니다.'
                )
            unknown = set(capabilities) - self.capabilities
            if unknown:
                raise RuntimeError(f'지원되지 않는 capability: {sorted(unknown)}')
            mode = str(task.get('execution_mode', 'single_agent')).strip()
            if mode not in ('single_agent', 'cooperative'):
                raise RuntimeError(f'지원하지 않는 execution_mode: {mode}')
            task['execution_mode'] = mode
            if mode == 'cooperative':
                cooperative_count += 1
                self._validate_cooperative_task(task, index, world_state)
        return cooperative_count

    def _publish_status(self, status, mission_id, revision, **fields):
        payload = {
            'scope': 'plan',
            'status': status,
            'mission_id': mission_id,
            'plan_revision': int(revision),
            'agent_id': 'workstation',
            'stamp_sec': self.get_clock().now().nanoseconds * 1e-9,
            **fields,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(message)
        self._log('PLAN_STATUS', **payload)

    def _publish(
        self,
        guidebook,
        *,
        original_command,
        mission_id=None,
        revision=0,
        replan_count=0,
    ):
        """[MERGED] 전체 계획을 검증하고 Agent LLM 검토 대상으로 발행합니다."""
        guidebook = copy.deepcopy(guidebook)
        cooperative_count = self._validate_guidebook(guidebook)
        mission_id = mission_id or (
            'mission_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        )
        guidebook['mission_id'] = mission_id
        guidebook['plan_revision'] = int(revision)
        guidebook['coordinate_frame'] = self.workspace_frame
        guidebook['requires_plan_review'] = cooperative_count > 0
        guidebook['perception_snapshot'] = (
            self._world_snapshot() if cooperative_count > 0 else None
        )

        expected = set()
        for task in guidebook['tasks']:
            if task['execution_mode'] == 'cooperative':
                expected.update(
                    (str(task['task_id']), agent_id)
                    for agent_id in task['participants']
                )

        mission_path = self.mission_dir / f'{mission_id}_r{revision}.json'
        mission_path.write_text(
            json.dumps(guidebook, ensure_ascii=False, indent=2),
            encoding='utf-8',
        )
        # Agent가 매우 빠르게 응답해도 feedback을 잃지 않도록 발행 전에 상태를 설치합니다.
        with self._state_lock:
            self._active_review = (
                {
                    'mission_id': mission_id,
                    'revision': int(revision),
                    'guidebook': guidebook,
                    'original_command': original_command,
                    'replan_count': int(replan_count),
                    'expected': expected,
                    'accepted': set(),
                    'rejections': {},
                    'started_monotonic': time.monotonic(),
                    'replanning': False,
                    'closed': False,
                }
                if expected
                else None
            )

        message = String()
        message.data = json.dumps(guidebook, ensure_ascii=False)
        self.publisher.publish(message)

        self._log(
            'PUBLISHED',
            mission_id=mission_id,
            plan_revision=revision,
            file=str(mission_path),
            guidebook=guidebook,
        )
        print(f'발행 완료: {self.topic} (mission={mission_id}, revision={revision})')
        print(f'저장 위치: {mission_path}')
        if not expected:
            self._publish_status('APPROVED', mission_id, revision, reason='검토 불필요')

    def _task_status_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if payload.get('scope') != 'plan':
            return
        status = str(payload.get('status', '')).upper()
        if status not in ('ACCEPTED', 'REJECTED'):
            return
        mission_id = str(payload.get('mission_id', '')).strip()
        revision = int(payload.get('plan_revision', -1))
        task_id = str(payload.get('task_id', '')).strip()
        agent_id = str(payload.get('agent_id', '')).strip()
        key = (task_id, agent_id)
        trigger_replan = False
        approve = False
        with self._state_lock:
            review = self._active_review
            if (
                not review
                or review['closed']
                or review['mission_id'] != mission_id
                or review['revision'] != revision
                or key not in review['expected']
            ):
                return
            if status == 'ACCEPTED':
                review['accepted'].add(key)
                review['rejections'].pop(key, None)
            else:
                review['rejections'][key] = str(
                    payload.get('reason', 'Agent가 계획을 거부함')
                )
                if not review['replanning']:
                    review['replanning'] = True
                    trigger_replan = True
            if not review['rejections'] and review['accepted'] >= review['expected']:
                review['closed'] = True
                approve = True
        if approve:
            self._publish_status('APPROVED', mission_id, revision)
            print(f'Agent 검토 완료: mission={mission_id} revision={revision}')
        elif trigger_replan:
            threading.Thread(
                target=self._replan_or_fail,
                args=(mission_id, revision),
                daemon=True,
            ).start()

    def _check_review_timeout(self):
        trigger = None
        with self._state_lock:
            review = self._active_review
            if (
                review
                and not review['closed']
                and not review['replanning']
                and time.monotonic() - review['started_monotonic']
                >= self.plan_review_timeout_sec
            ):
                missing = review['expected'] - review['accepted']
                for key in missing:
                    review['rejections'][key] = 'Agent 계획 검토 시간 초과'
                review['replanning'] = True
                trigger = (review['mission_id'], review['revision'])
        if trigger:
            threading.Thread(
                target=self._replan_or_fail,
                args=trigger,
                daemon=True,
            ).start()

    def _replan_or_fail(self, mission_id, revision):
        with self._state_lock:
            review = copy.deepcopy(self._active_review)
        if (
            not review
            or review['mission_id'] != mission_id
            or review['revision'] != revision
        ):
            return
        reasons = [
            {'task_id': task_id, 'agent_id': agent_id, 'reason': reason}
            for (task_id, agent_id), reason in review['rejections'].items()
        ]
        if review['replan_count'] >= 1:
            with self._state_lock:
                if self._active_review:
                    self._active_review['closed'] = True
            reason = f'재계획 1회 후에도 거부됨: {reasons}'
            self._publish_status('FAILED', mission_id, revision, reason=reason)
            print(f'계획 실패> {reason}')
            return

        self._publish_status(
            'SUPERSEDED',
            mission_id,
            revision,
            reason='Agent 거부로 1회 재계획 시작',
        )
        world_state = self._world_snapshot()
        replan_instruction = {
            'instruction': (
                '이것은 허용된 단 한 번의 재계획이다. 거부 사유와 최신 인식을 '
                '반영해 전체 가이드북을 다시 작성하라. status=ready JSON만 반환하라.'
            ),
            'original_command': review['original_command'],
            'previous_guidebook': review['guidebook'],
            'rejection_reasons': reasons,
            'current_world_state': world_state,
        }
        messages = [
            {'role': 'system', 'content': self.system_prompt},
            {
                'role': 'user',
                'content': json.dumps(replan_instruction, ensure_ascii=False),
            },
        ]
        try:
            raw = self._ask_ollama(messages)
            status, _, guidebook = self._parse(raw)
            if status != 'ready':
                raise RuntimeError('재계획 LLM이 ready 계획을 반환하지 않았습니다.')
            self._publish(
                guidebook,
                original_command=review['original_command'],
                mission_id=mission_id,
                revision=revision + 1,
                replan_count=1,
            )
        except (RuntimeError, ValueError, json.JSONDecodeError) as error:
            with self._state_lock:
                if self._active_review:
                    self._active_review['closed'] = True
            self._publish_status(
                'FAILED',
                mission_id,
                revision,
                reason=f'1회 재계획 생성/검증 실패: {error}',
            )
            print(f'재계획 실패> {error}')

    def _handle_user(self, text):
        world_state = self._world_snapshot()
        content = (
            f'{text}\n\n[현재 추가 인식 상태 - 좌표 단위 mm, 기준 {self.workspace_frame}]\n'
            + json.dumps(world_state, ensure_ascii=False)
        )
        self.messages.append({'role': 'user', 'content': content})
        self._log('USER', content=text, world_state=world_state)
        try:
            raw = self._ask_ollama()
            status, message, guidebook = self._parse(raw)
        except (RuntimeError, json.JSONDecodeError) as error:
            self.messages.pop()
            self._log('ERROR', message=str(error))
            print(f'오류> {error}')
            return

        self.messages.append({'role': 'assistant', 'content': raw})
        self._log('ASSISTANT', status=status, message=message)
        print(f'\nLLM> {message}')
        if status == 'ready':
            command_context = '\n'.join(
                message_item['content']
                for message_item in self.messages
                if message_item['role'] == 'user'
            )
            try:
                self._publish(guidebook, original_command=command_context)
                self._reset()
            except (RuntimeError, ValueError) as error:
                self._log('ERROR', message=str(error))
                print(f'오류> {error}')

    def start_ros_spin_thread(self):
        # [MERGED] input()/Ollama 대기 중에도 Agent 검토와 인식 콜백을 처리합니다.
        if self._spin_thread is not None:
            return
        self._executor = MultiThreadedExecutor(num_threads=3)
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def stop_ros_spin_thread(self):
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.0)
            self._executor = None
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None

    def run(self):
        self.start_ros_spin_thread()
        print(f'모델: {self.model}')
        print(f'Ollama: {self.ollama_url}')
        print('/reset: 대화 초기화  /quit: 종료')
        while rclpy.ok():
            try:
                text = input('\n사용자> ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not text:
                continue
            if text == '/quit':
                break
            if text == '/reset':
                self._reset()
                self._log('RESET')
                print('대화를 초기화했습니다.')
                continue
            self._handle_user(text)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WorkstationLLM()
        node.run()
    except Exception as error:
        print(f'시작 실패> {error}')
    finally:
        if node is not None:
            node.stop_ros_spin_thread()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
