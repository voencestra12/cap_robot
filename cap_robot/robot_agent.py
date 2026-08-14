import json
import math
import threading
import time
from pathlib import Path
from string import Template

import numpy as np
import requests
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import Buffer, TransformListener
from xarm.wrapper import XArmAPI

try:
    from .llm_api import DEFAULT_PNP_ACTIONS as LLM_DEFAULT_PNP_ACTIONS
    from .llm_api import validate_actions as validate_llm_actions
except ImportError:
    # 소스 디렉터리에서 직접 실행할 때를 위한 fallback
    from llm_api import DEFAULT_PNP_ACTIONS as LLM_DEFAULT_PNP_ACTIONS
    from llm_api import validate_actions as validate_llm_actions

try:
    import cv2
    from cv_bridge import CvBridge
    from ultralytics import YOLO
except ImportError:
    cv2 = None
    CvBridge = None
    YOLO = None


class RobotAgentNode(Node):
    # 공통 설계값은 코드에 유지하고, 에이전트마다 달라지는 값만 ROS 파라미터로 받습니다.
    # workspace_0 기준 작업 구역(mm).
    # 실제 xArm 이동 좌표는 TF(robot_1_base <- workspace_0)로 변환해서 생성합니다.
    ZONES = {
        'A': {'x_min': 0.0, 'x_max': 300.0, 'y_min': 0.0, 'y_max': 400.0},
        'B': {'x_min': 400.0, 'x_max': 700.0, 'y_min': 0.0, 'y_max': 400.0},
    }
    ZONE_MARGIN_MM = 50.0
    MIN_OBJECT_SPACING_MM = 120.0
    PICK_PLACE_Z_OFFSET_MM = 40.0
    DEFAULT_PNP_ACTIONS = LLM_DEFAULT_PNP_ACTIONS
    HOME_POSE = (200.0, 0.0, 300.0, 0.0)
    SAFE_RETREAT_POSE = (100.0, 350.0, 400.0, 0.0)
    CLAIM_WAIT_SEC = 2.0
    STARTUP_GUIDEBOOK_IGNORE_SEC = 5.0

    def __init__(self):
        super().__init__('robot_agent_node')
        self.node_started_wall_time = time.time()

        self.declare_parameter('agent_id', 'agent1')
        self.declare_parameter('robot_ip', '192.168.1.218')
        self.declare_parameter('enable_perception', True)
        # 예: ['agent1:A|B'] 또는 ['agent1:A', 'agent2:B']
        # 이번 TF 테스트는 robot_1_base 하나로 A/B구역을 모두 검증할 수 있게 기본값을 agent1:A|B로 둡니다.
        self.declare_parameter('agent_specs', ['agent1:A|B'])
        # ArUco 노드가 broadcast하는 TF 이름을 그대로 사용합니다.
        # target_frame <- source_frame = robot_1_base <- workspace_0
        self.declare_parameter('use_tf_workspace', True)
        self.declare_parameter('robot_base_frame', 'robot_1_base')
        self.declare_parameter('workspace_frame', 'workspace_0')
        self.declare_parameter('tf_lookup_timeout_sec', 1.0)
        self.declare_parameter('agent_base_frames', ['agent1:robot_1_base', 'agent2:robot_2_base'])
        self.declare_parameter('tf_cache_period_sec', 0.2)
        self.declare_parameter('tf_cache_max_age_sec', 60.0)
        self.declare_parameter('tf_buffer_cache_sec', 60.0)
        # TF latest(Time(0))가 동적 체인에서 오래된 latest-common-time을 잡는 경우가 있어,
        # 최신 시각보다 약간 과거의 명시적 시각으로 조회한다.
        self.declare_parameter('tf_lookup_delay_sec', 0.5)
        self.declare_parameter('tf_debug_log_period_sec', 2.0)
        # TF의 robot_1_base 좌표계와 xArm SDK set_position 좌표계가 다를 수 있으므로
        # 실제 이동 직전에 command frame 변환을 한 번 더 적용한다.
        # agent_legacy: agent1=neg_xy_z, agent2=identity
        self.declare_parameter('xarm_command_frame_mode', 'identity')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('enable_sdk_safety_box', True)
        self.declare_parameter('sdk_x_min', 50.0)
        self.declare_parameter('sdk_x_max', 750.0)
        self.declare_parameter('sdk_y_min', -600.0)
        self.declare_parameter('sdk_y_max', 600.0)
        self.declare_parameter('sdk_z_min', 20.0)
        self.declare_parameter('sdk_z_max', 700.0)
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('llm_model', 'gemma4:e4b')
        self.declare_parameter('ollama_url', 'http://localhost:11434/api/generate')
        self.declare_parameter('prompt_file', 'agent_policy.txt')
        # Guidebook 기반 정책 생성은 기존 사용자 명령용 prompt와 분리합니다.
        # 기존 실행 경로를 보존하면서 새 경로를 단계적으로 검증하기 위한 설정입니다.
        self.declare_parameter('guidebook_prompt_file', 'agent_guidebook_policy.txt')
        self.declare_parameter('guidebook_policy_enabled', True)
        self.declare_parameter('guidebook_policy_poll_sec', 1.0)
        self.declare_parameter('guidebook_policy_retry_sec', 5.0)
        self.declare_parameter('yolo_model_path', 'yolo11m-seg.pt')
        self.declare_parameter('guidebook_topic', '/mission/guidebook')
        self.declare_parameter('task_claim_topic', '/mission/task_claim')
        self.declare_parameter('task_status_topic', '/mission/task_status')

        self.agent_id = str(self.get_parameter('agent_id').value).strip()
        self.robot_ip = str(self.get_parameter('robot_ip').value).strip()
        self.enable_perception = bool(self.get_parameter('enable_perception').value)
        self.agent_specs = self.parse_agent_specs(self.get_parameter('agent_specs').value)
        self.use_tf_workspace = bool(self.get_parameter('use_tf_workspace').value)
        self.robot_base_frame = str(self.get_parameter('robot_base_frame').value).strip()
        self.workspace_frame = str(self.get_parameter('workspace_frame').value).strip()
        self.tf_lookup_timeout_sec = float(self.get_parameter('tf_lookup_timeout_sec').value)
        self.agent_base_frames = self.parse_agent_base_frames(
            self.get_parameter('agent_base_frames').value
        )
        self.tf_cache_period_sec = float(self.get_parameter('tf_cache_period_sec').value)
        self.tf_cache_max_age_sec = float(self.get_parameter('tf_cache_max_age_sec').value)
        self.tf_buffer_cache_sec = float(self.get_parameter('tf_buffer_cache_sec').value)
        self.tf_lookup_delay_sec = float(self.get_parameter('tf_lookup_delay_sec').value)
        self.tf_debug_log_period_sec = float(self.get_parameter('tf_debug_log_period_sec').value)
        self.xarm_command_frame_mode = str(self.get_parameter('xarm_command_frame_mode').value).strip().lower()
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.enable_sdk_safety_box = bool(self.get_parameter('enable_sdk_safety_box').value)
        self.sdk_x_min = float(self.get_parameter('sdk_x_min').value)
        self.sdk_x_max = float(self.get_parameter('sdk_x_max').value)
        self.sdk_y_min = float(self.get_parameter('sdk_y_min').value)
        self.sdk_y_max = float(self.get_parameter('sdk_y_max').value)
        self.sdk_z_min = float(self.get_parameter('sdk_z_min').value)
        self.sdk_z_max = float(self.get_parameter('sdk_z_max').value)
        self.color_topic = str(self.get_parameter('color_topic').value).strip()
        self.depth_topic = str(self.get_parameter('depth_topic').value).strip()
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value).strip()
        self.llm_model = str(self.get_parameter('llm_model').value).strip()
        self.ollama_url = str(self.get_parameter('ollama_url').value).strip()
        self.prompt_file = str(self.get_parameter('prompt_file').value).strip()
        self.guidebook_prompt_file = str(
            self.get_parameter('guidebook_prompt_file').value
        ).strip()
        self.guidebook_policy_enabled = bool(
            self.get_parameter('guidebook_policy_enabled').value
        )
        self.guidebook_policy_poll_sec = float(
            self.get_parameter('guidebook_policy_poll_sec').value
        )
        self.guidebook_policy_retry_sec = float(
            self.get_parameter('guidebook_policy_retry_sec').value
        )
        self.yolo_model_path = str(self.get_parameter('yolo_model_path').value).strip()
        self.guidebook_topic = str(self.get_parameter('guidebook_topic').value).strip()
        self.task_claim_topic = str(self.get_parameter('task_claim_topic').value).strip()
        self.task_status_topic = str(self.get_parameter('task_status_topic').value).strip()
        self.prompt_path = self.resolve_prompt_path(self.prompt_file)
        self.guidebook_prompt_path = self.resolve_prompt_path(
            self.guidebook_prompt_file
        )

        if not self.agent_id or not self.robot_ip:
            raise ValueError('agent_id와 robot_ip는 비어 있을 수 없습니다.')
        if self.agent_id not in self.agent_specs:
            raise ValueError(f'{self.agent_id}가 agent_specs에 없습니다: {self.agent_specs}')
        if self.use_tf_workspace and not self.workspace_frame:
            raise ValueError('TF workspace 사용 시 workspace_frame은 비어 있을 수 없습니다.')
        if self.use_tf_workspace and not self.agent_base_frames:
            raise ValueError('TF workspace 사용 시 agent_base_frames는 비어 있을 수 없습니다.')
        if self.enable_perception and (not self.color_topic or not self.depth_topic or not self.camera_info_topic):
            raise ValueError('perception 사용 시 color/depth/camera_info topic은 비어 있을 수 없습니다.')
        if self.enable_perception and not self.use_tf_workspace:
            raise ValueError('기존 T_OBJ 보정을 제거했으므로 perception 좌표 변환에는 TF workspace가 필요합니다.')


        self.tf_buffer = None
        self.tf_listener = None
        # workspace_tf_cache[agent_id] = (TransformStamped, cache_update_wall_time_sec)
        self.workspace_tf_cache = {}
        self.tf_cache_lock = threading.Lock()
        self.tf_ready_agents = set()
        self.tf_cache_timer = None
        if self.use_tf_workspace:
            self.tf_buffer = Buffer(cache_time=Duration(seconds=self.tf_buffer_cache_sec))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.tf_cache_timer = self.create_timer(
                self.tf_cache_period_sec, self.update_workspace_tf_cache
            )
            frame_text = ', '.join(
                f'{agent}:{base} <- {self.workspace_frame}'
                for agent, base in self.agent_base_frames.items()
            )
            self.get_logger().info(
                f'🧭 [{self.agent_id}] TF workspace 활성화: {frame_text}'
            )

        self.get_logger().info(f'🔌 [{self.agent_id}] 로봇 연결 중... IP={self.robot_ip}')
        self.arm = XArmAPI(self.robot_ip, is_radian=False)
        self.get_logger().info(f'✅ [{self.agent_id}] 로봇 연결 완료!')

        # Workstation이 발행한 동일한 가이드북을 모든 Agent가 받습니다.
        # 현재 단계에서는 가이드북을 저장하고 depends_on에 따라 READY/BLOCKED만 계산하며,
        # 실제 LLM 호출이나 로봇 실행에는 아직 연결하지 않습니다.
        self.current_mission_id = None
        self.current_guidebook = {}
        self.guidebook_tasks = {}
        self.guidebook_task_status = {}
        self.guidebook_lock = threading.Lock()

        # Step 1: READY Guidebook Task를 Agent LLM 정책 후보로 변환하는 경로만 추가합니다.
        # 아직 claim/status 또는 실제 /agent_task 발행에는 연결하지 않습니다.
        self.guidebook_policy_candidates = {}
        self.guidebook_policy_inflight = set()
        self.guidebook_policy_last_attempt = {}
        self.guidebook_policy_lock = threading.Lock()

        # Step 2 최소 coordination 상태: Task별 claim 후보와 이미 결정된 winner만 저장합니다.
        self.guidebook_claims = {}
        self.guidebook_claim_started = set()
        self.guidebook_claim_lock = threading.Lock()

        guidebook_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.guidebook_sub = self.create_subscription(
            String,
            self.guidebook_topic,
            self.guidebook_callback,
            guidebook_qos,
        )
        self.guidebook_policy_timer = self.create_timer(
            max(0.2, self.guidebook_policy_poll_sec),
            self.poll_ready_guidebook_tasks,
        )

        # 두 Agent가 모두 실행 중인 상태에서 쓰는 최소 claim/status 채널입니다.
        # 이번 단계는 별도 custom msg 없이 std_msgs/String JSON을 사용합니다.
        self.task_claim_pub = self.create_publisher(String, self.task_claim_topic, 10)
        self.task_claim_sub = self.create_subscription(
            String, self.task_claim_topic, self.task_claim_callback, 10
        )
        self.task_status_pub = self.create_publisher(String, self.task_status_topic, 10)
        self.task_status_sub = self.create_subscription(
            String, self.task_status_topic, self.task_status_callback, 10
        )

        # 기존 /agent_task 기반 실행 경로는 회귀 방지를 위해 그대로 유지합니다.
        # 모든 Agent가 같은 토픽을 구독하고 assignee_id가 자신인 작업만 실행합니다.
        self.task_pub = self.create_publisher(String, '/agent_task', 10)
        self.task_sub = self.create_subscription(String, '/agent_task', self.task_callback, 10)
        self.pose_publisher = self.create_publisher(Float32MultiArray, '/mouse_target_pose', 10)

        self.latest_poses = {}
        self.current_detected_items = []
        self.is_moving = False
        self.task_queue = []
        self.task_queue_lock = threading.Lock()
        self.task_worker_running = False
        self.placed_points = {'A': [], 'B': []}
        self.placed_points_lock = threading.Lock()

        self.model = None
        self.bridge = None
        self.color_sub = None
        self.depth_sub = None
        self.camera_info_sub = None
        self.latest_color_image = None
        self.latest_depth_image = None
        self.camera_intrinsics = None
        self.camera_frame = None
        self.perception_lock = threading.Lock()
        self.last_wait_log_time = 0.0
        self.last_tf_error_log_time = {}
        self._ros_executor = None
        self._ros_spin_thread = None
        if self.enable_perception:
            if cv2 is None or CvBridge is None or YOLO is None:
                raise ImportError('perception 사용 시 OpenCV, cv_bridge, ultralytics가 필요합니다.')
            self.model = YOLO(self.yolo_model_path)
            self.bridge = CvBridge()
            image_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.color_sub = self.create_subscription(
                Image, self.color_topic, self.color_callback, image_qos
            )
            self.depth_sub = self.create_subscription(
                Image, self.depth_topic, self.depth_callback, image_qos
            )
            self.camera_info_sub = self.create_subscription(
                CameraInfo, self.camera_info_topic, self.camera_info_callback, image_qos
            )
            self.get_logger().info(
                f'📷 [{self.agent_id}] ROS camera topic 구독: '
                f'color={self.color_topic}, depth={self.depth_topic}, info={self.camera_info_topic}'
            )

        self.init_robot()
        self.get_logger().info(
            f'🧠 LLM 설정: model={self.llm_model}, url={self.ollama_url}, '
            f'prompt={self.prompt_path}'
        )
        self.get_logger().info(
            f'📘 Guidebook policy: enabled={self.guidebook_policy_enabled}, '
            f'prompt={self.guidebook_prompt_path}, '
            f'poll={self.guidebook_policy_poll_sec:.1f}s, '
            f'retry={self.guidebook_policy_retry_sec:.1f}s'
        )
        self.get_logger().info(
            f'🧭 xArm command frame mode={self.xarm_command_frame_mode}, '
            f'dry_run={self.dry_run}, sdk_safety_box={self.enable_sdk_safety_box}, '
            f'x=[{self.sdk_x_min:.0f},{self.sdk_x_max:.0f}], '
            f'y=[{self.sdk_y_min:.0f},{self.sdk_y_max:.0f}], '
            f'z=[{self.sdk_z_min:.0f},{self.sdk_z_max:.0f}]'
        )
        self.get_logger().info(
            f'✅ [{self.agent_id}] 준비 완료 '
            f'(perception={self.enable_perception}, '
            f'zones={self.agent_specs[self.agent_id]}, '
            f'use_tf_workspace={self.use_tf_workspace}, '
            f'guidebook_topic={self.guidebook_topic})'
        )

    def guidebook_callback(self, msg):
        """Workstation guidebook을 저장하고 각 Task의 초기 READY/BLOCKED 상태를 계산합니다.

        이 단계에서는 가이드북을 실제 정책 생성이나 로봇 실행에 연결하지 않습니다.
        """
        try:
            guidebook = json.loads(msg.data)
            if not isinstance(guidebook, dict):
                raise ValueError('guidebook은 JSON 객체여야 합니다.')

            mission_id = str(guidebook.get('mission_id', '')).strip()
            tasks = guidebook.get('tasks')
            if not mission_id:
                raise ValueError('mission_id가 비어 있습니다.')

            # /mission/guidebook이 TRANSIENT_LOCAL이라 launch 직후 이전 mission이 다시 올 수 있습니다.
            # 이번 최소 Step 2에서는 두 Agent를 먼저 띄운 뒤 새 mission을 보내는 방식으로 안전하게 테스트합니다.
            if time.time() - self.node_started_wall_time < self.STARTUP_GUIDEBOOK_IGNORE_SEC:
                self.get_logger().warn(
                    f'🛑 [{self.agent_id}] startup 직후 Guidebook 무시: mission_id={mission_id}. '
                    '두 Agent 준비 후 새 mission을 발행하세요.'
                )
                return
            if not isinstance(tasks, list) or not tasks:
                raise ValueError('tasks는 비어 있지 않은 배열이어야 합니다.')

            task_map = {}
            for index, task in enumerate(tasks):
                if not isinstance(task, dict):
                    raise ValueError(f'tasks[{index}]는 JSON 객체여야 합니다.')

                task_id = str(task.get('task_id', '')).strip()
                depends_on = task.get('depends_on')
                if not task_id:
                    raise ValueError(f'tasks[{index}].task_id가 비어 있습니다.')
                if task_id in task_map:
                    raise ValueError(f'중복 task_id입니다: {task_id}')
                if not isinstance(depends_on, list):
                    raise ValueError(f'{task_id}.depends_on은 배열이어야 합니다.')

                normalized_task = dict(task)
                normalized_task['task_id'] = task_id
                normalized_task['depends_on'] = [str(dep).strip() for dep in depends_on]
                task_map[task_id] = normalized_task

            known_task_ids = set(task_map)
            for task_id, task in task_map.items():
                unknown_dependencies = [
                    dep for dep in task['depends_on']
                    if dep not in known_task_ids
                ]
                if unknown_dependencies:
                    raise ValueError(
                        f'{task_id}가 존재하지 않는 선행 Task를 참조합니다: '
                        f'{unknown_dependencies}'
                    )

            with self.guidebook_lock:
                if self.current_mission_id == mission_id:
                    self.get_logger().info(
                        f'📘 [{self.agent_id}] 동일 mission 재수신 무시: {mission_id}'
                    )
                    return
                self.current_mission_id = mission_id
                self.current_guidebook = dict(guidebook)
                self.guidebook_tasks = task_map
                self.guidebook_task_status = {
                    task_id: 'BLOCKED' for task_id in task_map
                }
                self.refresh_guidebook_task_states_locked()
                status_snapshot = dict(self.guidebook_task_status)

            with self.guidebook_policy_lock:
                self.guidebook_policy_candidates.clear()
                self.guidebook_policy_inflight.clear()
                self.guidebook_policy_last_attempt.clear()

            with self.guidebook_claim_lock:
                self.guidebook_claims.clear()
                self.guidebook_claim_started.clear()

            self.get_logger().info(
                f'📘 [{self.agent_id}] Guidebook 수신: '
                f'mission_id={mission_id}, tasks={len(task_map)}'
            )
            for task_id, task in task_map.items():
                self.get_logger().info(
                    f'  - {task_id}: {status_snapshot[task_id]} | '
                    f'depends_on={task["depends_on"]} | '
                    f'{task.get("description", "")}'
                )

        except Exception as error:
            self.get_logger().error(f'❌ Guidebook 수신/해석 실패: {error}')

    def refresh_guidebook_task_states_locked(self):
        """guidebook_lock을 잡은 상태에서 depends_on만으로 READY/BLOCKED를 계산합니다."""
        for task_id, task in self.guidebook_tasks.items():
            current = self.guidebook_task_status.get(task_id)
            if current in ('CLAIMED', 'EXECUTING', 'SUCCEEDED', 'FAILED'):
                continue

            dependencies = task.get('depends_on', [])
            ready = all(
                self.guidebook_task_status.get(dep) == 'SUCCEEDED'
                for dep in dependencies
            )
            self.guidebook_task_status[task_id] = 'READY' if ready else 'BLOCKED'


    def task_claim_callback(self, msg):
        """READY Task에 대해 실행 가능한 Agent가 보낸 claim을 모읍니다."""
        try:
            claim = json.loads(msg.data)
            mission_id = str(claim.get('mission_id', '')).strip()
            task_id = str(claim.get('task_id', '')).strip()
            agent_id = str(claim.get('agent_id', '')).strip()
            if not mission_id or not task_id or not agent_id:
                raise ValueError('task_claim 필수 필드가 없습니다.')

            with self.guidebook_lock:
                if mission_id != self.current_mission_id:
                    return
                if self.guidebook_task_status.get(task_id) != 'READY':
                    return

            with self.guidebook_claim_lock:
                self.guidebook_claims.setdefault(task_id, set()).add(agent_id)

            self.get_logger().info(
                f'📨 [{self.agent_id}] CLAIM 수신: {task_id} <- {agent_id}'
            )
        except Exception as error:
            self.get_logger().error(f'❌ task_claim 해석 실패: {error}')

    def publish_task_claim(self, task_id):
        """자기 Policy 후보가 유효할 때 claim을 1회 발행하고 winner 결정을 예약합니다."""
        with self.guidebook_lock:
            mission_id = self.current_mission_id
            if not mission_id or self.guidebook_task_status.get(task_id) != 'READY':
                return

        claim = {
            'mission_id': mission_id,
            'task_id': task_id,
            'agent_id': self.agent_id,
        }

        with self.guidebook_claim_lock:
            self.guidebook_claims.setdefault(task_id, set()).add(self.agent_id)
            if task_id in self.guidebook_claim_started:
                return
            self.guidebook_claim_started.add(task_id)

        msg = String()
        msg.data = json.dumps(claim, ensure_ascii=False)
        self.task_claim_pub.publish(msg)
        self.get_logger().info(f'📣 [{self.agent_id}] CLAIM 발행: {task_id}')

        threading.Thread(
            target=self.finalize_task_claim,
            args=(mission_id, task_id),
            daemon=True,
        ).start()

    def finalize_task_claim(self, mission_id, task_id):
        """짧은 claim window 뒤 claimant의 agent_id 사전순으로 winner를 정합니다.

        현재 Step 2의 목적은 중복 실행 방지와 status 흐름 검증입니다.
        더 복잡한 협상/우선순위 정책은 이 경로가 안정된 뒤 확장합니다.
        """
        time.sleep(self.CLAIM_WAIT_SEC)

        with self.guidebook_lock:
            if mission_id != self.current_mission_id:
                return
            if self.guidebook_task_status.get(task_id) != 'READY':
                return

        with self.guidebook_claim_lock:
            claimants = sorted(self.guidebook_claims.get(task_id, set()))

        if not claimants:
            return

        winner = claimants[0]
        self.get_logger().info(
            f'🏁 [{self.agent_id}] CLAIM WINNER: {task_id} -> {winner} '
            f'(claimants={claimants})'
        )
        if winner != self.agent_id:
            return

        with self.guidebook_policy_lock:
            task = self.guidebook_policy_candidates.get(task_id)
        if not isinstance(task, dict):
            self.get_logger().error(
                f'❌ [{self.agent_id}] {task_id} winner이지만 실행 후보가 없습니다.'
            )
            return

        self.publish_task_status(task_id, 'CLAIMED')

        dispatch = dict(task)
        dispatch['assignee_id'] = self.agent_id
        msg = String()
        msg.data = json.dumps(dispatch, ensure_ascii=False)
        self.task_pub.publish(msg)
        self.get_logger().info(
            f'➡️ [{self.agent_id}] /agent_task 발행: {task_id} / target={dispatch["target"]}'
        )

    def publish_task_status(self, task_id, status):
        """Guidebook Task 상태를 로컬에 먼저 반영하고 다른 Agent에도 공유합니다."""
        with self.guidebook_lock:
            mission_id = self.current_mission_id
        if not mission_id:
            return

        payload = {
            'mission_id': mission_id,
            'task_id': task_id,
            'agent_id': self.agent_id,
            'status': status,
        }
        self.apply_task_status(payload)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.task_status_pub.publish(msg)

    def task_status_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            self.apply_task_status(payload)
        except Exception as error:
            self.get_logger().error(f'❌ task_status 해석 실패: {error}')

    def apply_task_status(self, payload):
        """CLAIMED/EXECUTING/SUCCEEDED/FAILED를 로컬 Guidebook 상태에 반영합니다."""
        mission_id = str(payload.get('mission_id', '')).strip()
        task_id = str(payload.get('task_id', '')).strip()
        agent_id = str(payload.get('agent_id', '')).strip()
        status = str(payload.get('status', '')).strip().upper()
        if status not in ('CLAIMED', 'EXECUTING', 'SUCCEEDED', 'FAILED'):
            raise ValueError(f'지원하지 않는 task status: {status}')

        with self.guidebook_lock:
            if mission_id != self.current_mission_id:
                return
            if task_id not in self.guidebook_tasks:
                return

            current = self.guidebook_task_status.get(task_id)
            if current in ('SUCCEEDED', 'FAILED') or current == status:
                return
            status_rank = {'BLOCKED': 0, 'READY': 0, 'CLAIMED': 1, 'EXECUTING': 2,
                           'SUCCEEDED': 3, 'FAILED': 3}
            if status_rank.get(status, 0) < status_rank.get(current, 0):
                return

            self.guidebook_task_status[task_id] = status
            before = dict(self.guidebook_task_status)
            self.refresh_guidebook_task_states_locked()
            after = dict(self.guidebook_task_status)

        self.get_logger().info(
            f'📡 [{self.agent_id}] {task_id} status={status} by {agent_id}'
        )
        for changed_id in after:
            if before.get(changed_id) != after.get(changed_id):
                self.get_logger().info(
                    f'🔄 [{self.agent_id}] {changed_id}: '
                    f'{before.get(changed_id)} -> {after.get(changed_id)}'
                )

    @staticmethod
    def parse_agent_specs(raw_specs):
        specs = {}
        for raw in raw_specs:
            text = str(raw).strip()
            if ':' not in text:
                raise ValueError(f"잘못된 agent_specs 형식: '{text}'")
            agent_id, zones_text = text.split(':', 1)
            zones = []
            for raw_zone in zones_text.split('|'):
                zone = raw_zone.strip().upper().replace('구역', '')
                if zone not in ('A', 'B'):
                    raise ValueError(f"지원하지 않는 구역: '{raw_zone}'")
                if zone not in zones:
                    zones.append(zone)
            if not agent_id.strip() or not zones:
                raise ValueError(f"잘못된 agent_specs 형식: '{text}'")
            specs[agent_id.strip()] = zones
        if not specs:
            raise ValueError('agent_specs는 하나 이상의 Agent를 포함해야 합니다.')
        return specs

    @staticmethod
    def parse_agent_base_frames(raw_frames):
        if isinstance(raw_frames, str):
            text = raw_frames.strip()
            if text.startswith('[') and text.endswith(']'):
                text = text[1:-1]
            raw_items = [item.strip().strip("'\"") for item in text.split(',') if item.strip()]
        else:
            raw_items = [str(item).strip() for item in raw_frames]

        frames = {}
        for raw in raw_items:
            if ':' not in raw:
                raise ValueError(f"잘못된 agent_base_frames 형식: '{raw}'")
            agent_id, frame_id = raw.split(':', 1)
            agent_id = agent_id.strip()
            frame_id = frame_id.strip()
            if not agent_id or not frame_id:
                raise ValueError(f"잘못된 agent_base_frames 형식: '{raw}'")
            frames[agent_id] = frame_id
        return frames

    def init_robot(self):
        self.arm.clean_error()
        self.arm.motion_enable(True)
        self.arm.clean_gripper_error()
        self.arm.set_mode(0)
        self.arm.set_state(0)
        self.arm.set_collision_sensitivity(3)
        self.arm.set_gripper_enable(True)
        self.get_logger().info('✅ 로봇 및 그리퍼 초기화 완료!')

    @staticmethod
    def quaternion_to_rotation_matrix(qx, qy, qz, qw):
        """geometry_msgs Quaternion(x,y,z,w)을 3x3 회전행렬로 변환합니다."""
        q = np.array([float(qx), float(qy), float(qz), float(qw)], dtype=float)
        norm = np.linalg.norm(q)
        if norm < 1e-9:
            raise ValueError('TF quaternion norm이 0에 가깝습니다.')
        qx, qy, qz, qw = q / norm

        return np.array([
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ], dtype=float)

    def get_agent_base_frame(self, agent_id=None):
        agent_id = self.agent_id if agent_id is None else str(agent_id).strip()
        frame = self.agent_base_frames.get(agent_id)
        if frame:
            return frame
        if agent_id == self.agent_id and self.robot_base_frame:
            return self.robot_base_frame
        raise ValueError(f'{agent_id}에 대응하는 robot base frame이 없습니다.')

    def transform_stamp_to_sec(self, transform_msg):
        stamp = transform_msg.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def now_sec(self):
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def tf_lookup_time(self):
        """TF를 조회할 명시적 시각을 반환합니다.

        rclpy.time.Time() 또는 Time(0)은 tf2에서 최신 공통 시각을 자동 선택합니다.
        그런데 camera->marker_1, camera->workspace_0처럼 여러 동적 TF가 섞이면
        최신 공통 시각이 과거에 고정되어 extrapolation into the past가 날 수 있습니다.
        그래서 현재 시각보다 조금 과거의 명시적 시각으로 조회합니다.
        """
        delay_ns = int(max(0.0, self.tf_lookup_delay_sec) * 1e9)
        query_ns = max(0, int(self.get_clock().now().nanoseconds) - delay_ns)
        return RclpyTime(nanoseconds=query_ns)

    def warn_tf_throttled(self, key, message):
        now = time.time()
        last = self.last_tf_error_log_time.get(key, 0.0)
        if now - last >= self.tf_debug_log_period_sec:
            self.last_tf_error_log_time[key] = now
            self.get_logger().warn(message)

    def get_cached_workspace_transform(self, agent_id=None):
        if not self.use_tf_workspace:
            return None

        cache_agent_id = self.agent_id if agent_id is None else str(agent_id).strip()
        target_frame = self.get_agent_base_frame(cache_agent_id)

        with self.tf_cache_lock:
            cached_entry = self.workspace_tf_cache.get(cache_agent_id)

        if cached_entry is None:
            raise RuntimeError(
                f'cached TF가 없습니다: {cache_agent_id} {target_frame} <- {self.workspace_frame}'
            )

        # 새 구조: (TransformStamped, cache_update_time_sec).
        # 혹시 이전 구조가 남아 있어도 TransformStamped 단독 저장을 허용한다.
        if isinstance(cached_entry, tuple):
            cached, cache_update_sec = cached_entry
        else:
            cached = cached_entry
            cache_update_sec = self.transform_stamp_to_sec(cached)

        age_sec = self.now_sec() - float(cache_update_sec)
        if age_sec < 0.0:
            age_sec = 0.0

        if age_sec > self.tf_cache_max_age_sec:
            raise RuntimeError(
                f'cached TF가 너무 오래되었습니다: {cache_agent_id} '
                f'{target_frame} <- {self.workspace_frame}, '
                f'age={age_sec:.2f}s > limit={self.tf_cache_max_age_sec:.2f}s'
            )

        self.get_logger().warn(
            f'⚠️ 최신 TF 조회 실패 → cached TF 사용: '
            f'{cache_agent_id} {target_frame} <- {self.workspace_frame}, '
            f'cache_age={age_sec:.2f}s, tf_stamp={self.transform_stamp_to_sec(cached):.3f}'
        )
        return cached

    def lookup_workspace_transform(self, agent_id=None):
        """agent별 robot_base 기준 workspace_frame의 최신 TF를 조회합니다.

        최신 lookup이 timestamp mismatch 등으로 실패하면,
        최근 정상적으로 조회된 cached TF를 fallback으로 사용합니다.
        """
        if not self.use_tf_workspace:
            return None

        cache_agent_id = self.agent_id if agent_id is None else str(agent_id).strip()
        target_frame = self.get_agent_base_frame(cache_agent_id)

        try:
            query_time = self.tf_lookup_time()
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                self.workspace_frame,
                query_time,
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )

            # 직접 조회 성공 시 cache도 갱신합니다.
            with self.tf_cache_lock:
                self.workspace_tf_cache[cache_agent_id] = (transform, self.now_sec())

            if cache_agent_id not in self.tf_ready_agents:
                self.tf_ready_agents.add(cache_agent_id)
                self.get_logger().info(
                    f'🧭 TF OK: {cache_agent_id} {target_frame} <- {self.workspace_frame}'
                )

            return transform

        except Exception as error:
            try:
                return self.get_cached_workspace_transform(cache_agent_id)
            except Exception as cache_error:
                raise RuntimeError(
                    f'TF 조회 실패: {target_frame} <- {self.workspace_frame}. '
                    f'ArUco 노드가 켜져 있고 마커가 보이는지 확인하세요. '
                    f'직접 조회 원인: {error} / cache fallback 원인: {cache_error}'
                ) from error

    def update_workspace_tf_cache(self):
        if not self.use_tf_workspace:
            return
        for agent_id, base_frame in self.agent_base_frames.items():
            try:
                query_time = self.tf_lookup_time()
                transform = self.tf_buffer.lookup_transform(
                    base_frame,
                    self.workspace_frame,
                    query_time,
                    timeout=Duration(seconds=0.01),
                )
                with self.tf_cache_lock:
                    self.workspace_tf_cache[agent_id] = (transform, self.now_sec())
                if agent_id not in self.tf_ready_agents:
                    self.tf_ready_agents.add(agent_id)
                    self.get_logger().info(
                        f'🧭 TF OK: {agent_id} {base_frame} <- {self.workspace_frame}'
                    )
            except Exception as error:
                self.warn_tf_throttled(
                    f'tf_cache_{agent_id}',
                    f'⏳ TF cache 대기/실패: {agent_id} {base_frame} <- {self.workspace_frame} / {error}'
                )
                continue

    def workspace_point_to_robot(self, x_mm, y_mm, z_mm=0.0, agent_id=None):
        """workspace_0 기준 좌표(mm)를 agent별 robot base 기준 좌표(mm)로 변환합니다."""
        if not self.use_tf_workspace:
            return float(x_mm), float(y_mm), float(z_mm)

        transform = self.lookup_workspace_transform(agent_id).transform
        translation_m = np.array([
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        ], dtype=float)
        rotation = transform.rotation
        rotation_robot_workspace = self.quaternion_to_rotation_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        point_workspace_m = np.array([
            float(x_mm) / 1000.0,
            float(y_mm) / 1000.0,
            float(z_mm) / 1000.0,
        ], dtype=float)
        point_robot_m = rotation_robot_workspace @ point_workspace_m + translation_m
        return tuple((point_robot_m * 1000.0).tolist())

    def workspace_yaw_to_robot(self, yaw_rad, agent_id=None):
        """workspace_0 평면 yaw(rad)를 agent별 robot base 평면 yaw(rad)로 변환합니다."""
        if not self.use_tf_workspace:
            return float(yaw_rad)

        transform = self.lookup_workspace_transform(agent_id).transform
        rotation = transform.rotation
        rotation_robot_workspace = self.quaternion_to_rotation_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        direction_workspace = np.array([
            math.cos(float(yaw_rad)),
            math.sin(float(yaw_rad)),
            0.0,
        ], dtype=float)
        direction_robot = rotation_robot_workspace @ direction_workspace
        return float(math.atan2(direction_robot[1], direction_robot[0]))

    def workspace_place_to_robot_place(self, place_pose, agent_id=None):
        """배치 좌표를 workspace 기준에서 agent별 xArm base 기준으로 변환합니다."""
        x_robot, y_robot, _ = self.workspace_point_to_robot(
            place_pose['x'], place_pose['y'], 0.0, agent_id=agent_id
        )
        yaw_robot = self.workspace_yaw_to_robot(
            place_pose.get('yaw', 0.0), agent_id=agent_id
        )
        return {'x': float(x_robot), 'y': float(y_robot), 'yaw': float(yaw_robot)}

    def workspace_object_to_robot_object(self, object_pose, agent_id=None):
        """workspace 기준 object pose를 agent별 xArm base 기준으로 변환합니다."""
        x_robot, y_robot, z_robot = self.workspace_point_to_robot(
            object_pose['x'], object_pose['y'], object_pose.get('z', 0.0), agent_id=agent_id
        )
        yaw_robot = self.workspace_yaw_to_robot(
            object_pose.get('yaw', 0.0), agent_id=agent_id
        )
        return {
            'x': float(x_robot),
            'y': float(y_robot),
            'z': float(z_robot),
            'yaw': float(yaw_robot),
        }

    def camera_point_to_workspace(self, x_m, y_m, z_m):
        """camera_color_optical_frame 기준 3D 점(m)을 workspace_0 기준 좌표(mm)로 변환합니다."""
        if not self.use_tf_workspace:
            raise RuntimeError('camera point를 workspace로 변환하려면 TF workspace가 필요합니다.')
        if not self.camera_frame:
            raise RuntimeError('CameraInfo header.frame_id를 아직 받지 못했습니다.')

        transform = self.tf_buffer.lookup_transform(
            self.workspace_frame,
            self.camera_frame,
            self.tf_lookup_time(),
            timeout=Duration(seconds=self.tf_lookup_timeout_sec),
        ).transform
        translation_m = np.array([
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        ], dtype=float)
        rotation = transform.rotation
        rotation_workspace_camera = self.quaternion_to_rotation_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        point_camera_m = np.array([float(x_m), float(y_m), float(z_m)], dtype=float)
        point_workspace_m = rotation_workspace_camera @ point_camera_m + translation_m
        return tuple((point_workspace_m * 1000.0).tolist())

    @staticmethod
    def normalize_angle_rad(angle):
        return math.atan2(math.sin(float(angle)), math.cos(float(angle)))

    def get_xarm_command_frame_mode(self, agent_id=None):
        agent_id = self.agent_id if agent_id is None else str(agent_id).strip()
        mode = self.xarm_command_frame_mode
        if mode == 'agent_legacy':
            # 기존 aruco_calib.py의 /robot_1_target은 agent1에만 x,y 마이너스를 적용했고,
            # agent2는 TF translation을 그대로 사용했다.
            return 'neg_xy_z' if agent_id == 'agent1' else 'identity'
        return mode

    def tf_pose_to_sdk_pose(self, x, y, z, yaw=0.0, agent_id=None):
        """robot_N_base TF 좌표(mm, rad)를 xArm SDK set_position 좌표(mm, rad)로 변환한다.

        주의:
            workspace_point_to_robot()의 결과는 TF의 robot_N_base 좌표이다.
            xArm SDK set_position()은 별도의 command/UI 좌표계를 쓰므로 이 변환을 거쳐야 한다.
        """
        mode = self.get_xarm_command_frame_mode(agent_id)
        x_tf, y_tf, z_tf = float(x), float(y), float(z)
        yaw_tf = float(yaw)

        if mode in ('identity', 'none', ''):
            x_sdk, y_sdk, z_sdk = x_tf, y_tf, z_tf
            yaw_sdk = yaw_tf
        elif mode in ('neg_xy', 'neg_xy_z', 'agent1_legacy'):
            x_sdk, y_sdk, z_sdk = -x_tf, -y_tf, z_tf
            yaw_sdk = yaw_tf + math.pi
        elif mode in ('neg_x',):
            x_sdk, y_sdk, z_sdk = -x_tf, y_tf, z_tf
            yaw_sdk = math.atan2(math.sin(yaw_tf), -math.cos(yaw_tf))
        elif mode in ('neg_y',):
            x_sdk, y_sdk, z_sdk = x_tf, -y_tf, z_tf
            yaw_sdk = math.atan2(-math.sin(yaw_tf), math.cos(yaw_tf))
        elif mode in ('ui_inverse', 'ui_to_tf_inverse'):
            # 예전 offset 추출 코드에 있었던 x=ui_y, y=-ui_x, z=-ui_z의 역변환 후보.
            # 실제 z축까지 반전되므로, 검증 전 실기 이동에는 쓰지 않는 것을 권장한다.
            x_sdk, y_sdk, z_sdk = -y_tf, x_tf, -z_tf
            direction = np.array([math.cos(yaw_tf), math.sin(yaw_tf), 0.0], dtype=float)
            mapped = np.array([-direction[1], direction[0], -direction[2]], dtype=float)
            yaw_sdk = math.atan2(mapped[1], mapped[0])
        else:
            raise ValueError(
                f'지원하지 않는 xarm_command_frame_mode={mode!r}. '
                '사용 가능: agent_legacy, identity, neg_xy_z, neg_x, neg_y, ui_inverse'
            )

        return {
            'x': float(x_sdk),
            'y': float(y_sdk),
            'z': float(z_sdk),
            'yaw': self.normalize_angle_rad(yaw_sdk),
            'mode': mode,
        }

    def check_sdk_pose_or_raise(self, x, y, z, label=''):  # xArm SDK command 좌표 기준
        if not self.enable_sdk_safety_box:
            return
        x, y, z = float(x), float(y), float(z)
        ok = (
            self.sdk_x_min <= x <= self.sdk_x_max and
            self.sdk_y_min <= y <= self.sdk_y_max and
            self.sdk_z_min <= z <= self.sdk_z_max
        )
        if not ok:
            raise RuntimeError(
                f'xArm SDK safety box 밖으로 이동하려고 합니다{f" ({label})" if label else ""}: '
                f'cmd=({x:.1f}, {y:.1f}, {z:.1f}), '
                f'allowed x=[{self.sdk_x_min:.1f},{self.sdk_x_max:.1f}], '
                f'y=[{self.sdk_y_min:.1f},{self.sdk_y_max:.1f}], '
                f'z=[{self.sdk_z_min:.1f},{self.sdk_z_max:.1f}]'
            )

    def move_to_sdk(self, x, y, z, yaw=0.0, speed=100.0, label=''):
        """이미 xArm SDK command 좌표인 pose를 set_position으로 보낸다."""
        x, y, z = float(x), float(y), float(z)
        yaw = self.normalize_angle_rad(yaw)
        self.check_sdk_pose_or_raise(x, y, z, label=label)

        if self.dry_run:
            self.get_logger().warn(
                f'🧪 DRY_RUN move_to_sdk{f"[{label}]" if label else ""}: '
                f'x={x:.1f}, y={y:.1f}, z={z:.1f}, yaw={np.degrees(yaw):.1f}deg, speed={float(speed):.1f}'
            )
            return True

        ret = self.arm.set_position(
            x=x, y=y, z=z, roll=180.0, pitch=0.0,
            yaw=float(np.degrees(yaw)), speed=float(speed), wait=True
        )
        if ret == 0:
            time.sleep(0.5)
            return True
        self.get_logger().error(f'🚨 로봇 이동 거절됨! 에러 코드: {ret}')
        return False

    def move_to_robot_tf(self, x, y, z, yaw=0.0, speed=100.0, label=''):
        """robot_N_base TF 좌표를 xArm SDK command 좌표로 변환한 뒤 이동한다."""
        cmd = self.tf_pose_to_sdk_pose(x, y, z, yaw=yaw, agent_id=self.agent_id)
        self.get_logger().info(
            f'🧭 TF→SDK{f"[{label}]" if label else ""}: '
            f'tf=({float(x):.1f}, {float(y):.1f}, {float(z):.1f}, yaw={np.degrees(float(yaw)):.1f}deg) '
            f'-> sdk=({cmd["x"]:.1f}, {cmd["y"]:.1f}, {cmd["z"]:.1f}, '
            f'yaw={np.degrees(cmd["yaw"]):.1f}deg), mode={cmd["mode"]}'
        )
        return self.move_to_sdk(cmd['x'], cmd['y'], cmd['z'], yaw=cmd['yaw'], speed=speed, label=label)

    def move_to(self, x, y, z, yaw=0.0, speed=100.0):
        # 기존 HOME_POSE / SAFE_RETREAT_POSE는 xArm SDK command 좌표로 간주한다.
        return self.move_to_sdk(x, y, z, yaw=yaw, speed=speed, label='sdk_direct')

    def control_gripper(self, position):
        if self.dry_run:
            self.get_logger().warn(f'🧪 DRY_RUN gripper position={float(position):.1f}')
            return True
        ret = self.arm.set_gripper_position(float(position), wait=True)
        if ret not in (0, None):
            self.get_logger().error(f'🚨 그리퍼 명령 거절됨! 에러 코드: {ret}')
            return False
        time.sleep(0.5)
        return True

    @staticmethod
    def normalize_zone(destination):
        text = str(destination).upper().replace('구역', '').strip()
        if text == 'A':
            return 'A'
        if text == 'B':
            return 'B'
        return None

    @staticmethod
    def find_detected_target(raw_target, poses_dict):
        target = str(raw_target).strip()
        if not target:
            return None
        if target in poses_dict:
            return target
        return next((key for key in poses_dict if target in key or key in target), None)

    def convert_object_pose(self, reference_pose, assignee_id):
        pose = dict(reference_pose)
        if self.use_tf_workspace:
            return self.workspace_object_to_robot_object(pose, agent_id=assignee_id)
        if assignee_id != self.agent_id:
            raise ValueError('TF workspace 없이 다른 Agent 좌표로 object pose를 변환할 수 없습니다.')
        return pose

    def convert_place_pose(self, reference_pose, assignee_id):
        pose = dict(reference_pose)
        if self.use_tf_workspace:
            return self.workspace_place_to_robot_place(pose, agent_id=assignee_id)
        if assignee_id != self.agent_id:
            raise ValueError('TF workspace 없이 다른 Agent 좌표로 place pose를 변환할 수 없습니다.')
        return pose

    def zone_uv_to_xy(self, zone, u, v):
        bounds = self.ZONES[zone]
        x_min = bounds['x_min'] + self.ZONE_MARGIN_MM
        x_max = bounds['x_max'] - self.ZONE_MARGIN_MM
        y_min = bounds['y_min'] + self.ZONE_MARGIN_MM
        y_max = bounds['y_max'] - self.ZONE_MARGIN_MM
        return x_min + u * (x_max - x_min), y_min + v * (y_max - y_min)

    def point_is_free(self, x, y, reserved_points):
        return all(
            math.hypot(x - px, y - py) >= self.MIN_OBJECT_SPACING_MM
            for px, py in reserved_points
        )

    def select_place_point(self, zone, u, v, reserved_points):
        if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
            raise ValueError('place_u와 place_v는 0.0~1.0이어야 합니다.')
        requested = self.zone_uv_to_xy(zone, u, v)
        if not self.point_is_free(*requested, reserved_points):
            raise ValueError(
                f'{zone}구역에서 LLM이 선택한 점 {requested}은 '
                f'다른 배치점과 {self.MIN_OBJECT_SPACING_MM:.0f} mm 이상 떨어져 있지 않습니다.'
            )
        return requested

    def validate_actions(self, raw_actions):
        """llm_api.py에 정의된 정책 검증 함수를 사용합니다."""
        return validate_llm_actions(raw_actions, self.PICK_PLACE_Z_OFFSET_MM)

    @staticmethod
    def resolve_prompt_path(prompt_file):
        """설치된 패키지 또는 소스 디렉터리에서 프롬프트 파일을 찾습니다."""
        prompt_path = Path(str(prompt_file)).expanduser()
        if prompt_path.is_absolute():
            return prompt_path

        try:
            share_dir = Path(get_package_share_directory('cap_robot'))
            installed_path = share_dir / 'prompts' / prompt_path
            if installed_path.exists():
                return installed_path
        except Exception:
            pass

        source_path = Path(__file__).resolve().parent.parent / 'prompts' / prompt_path
        if source_path.exists():
            return source_path

        raise FileNotFoundError(f'프롬프트 파일을 찾을 수 없습니다: {prompt_file}')


    @staticmethod
    def poses_for_prompt(poses_dict):
        """workspace_0 기준 perception pose를 LLM 입력용 JSON 객체로 바꿉니다."""
        result = {}
        for name, pose in poses_dict.items():
            if not isinstance(pose, (list, tuple)) or len(pose) < 4:
                continue
            result[str(name)] = {
                'x_mm': round(float(pose[0]), 1),
                'y_mm': round(float(pose[1]), 1),
                'z_mm': round(float(pose[2]), 1),
                'yaw_deg': round(float(np.degrees(float(pose[3]))), 1),
            }
        return result

    def load_guidebook_policy_prompt(self, guidebook, task, detected_items, poses_dict):
        """Guidebook READY Task 하나를 위한 Agent LLM prompt를 생성합니다."""
        template_text = self.guidebook_prompt_path.read_text(encoding='utf-8')
        template = Template(template_text)
        zone_frame_text = (
            f'{self.workspace_frame} 기준' if self.use_tf_workspace else f'{self.agent_id} 기준'
        )
        return template.substitute(
            local_agent_id=self.agent_id,
            mission_id=str(guidebook.get('mission_id', '')),
            mission_goal=str(guidebook.get('goal', '')),
            guidebook_json=json.dumps(guidebook, ensure_ascii=False),
            task_json=json.dumps(task, ensure_ascii=False),
            detected_items=json.dumps(detected_items, ensure_ascii=False),
            detected_poses=json.dumps(
                self.poses_for_prompt(poses_dict),
                ensure_ascii=False,
            ),
            agents=json.dumps(self.agent_specs, ensure_ascii=False),
            zone_frame=zone_frame_text,
            zones=json.dumps(self.ZONES, ensure_ascii=False),
            pick_place_z_offset_mm=f'{self.PICK_PLACE_Z_OFFSET_MM:.1f}',
            default_pnp_actions=json.dumps(
                self.DEFAULT_PNP_ACTIONS,
                ensure_ascii=False,
            ),
        )

    def ask_llm_for_guidebook_policy(self, guidebook, task, detected_items, poses_dict):
        """READY Guidebook Task에 대한 이 Agent의 PnP 정책 후보를 생성합니다."""
        prompt = self.load_guidebook_policy_prompt(
            guidebook,
            task,
            detected_items,
            poses_dict,
        )
        payload = {
            'model': self.llm_model,
            'prompt': prompt,
            'format': 'json',
            'stream': False,
            'options': {'temperature': 0.0, 'num_predict': 2048},
        }
        task_id = str(task.get('task_id', ''))
        self.get_logger().info(
            f'🧠 [{self.agent_id}] Guidebook {task_id} 정책 후보 생성 중...'
        )
        try:
            response = requests.post(self.ollama_url, json=payload, timeout=300.0)
            response.raise_for_status()
            result = json.loads(response.json()['response'].strip())
            if not isinstance(result, dict):
                raise ValueError('Agent LLM 응답은 JSON 객체여야 합니다.')
            self.get_logger().info(
                f'🤖 [{self.agent_id}] Guidebook {task_id} Policy 후보: {result}'
            )
            return result
        except Exception as error:
            self.get_logger().error(
                f'🚨 [{self.agent_id}] Guidebook {task_id} Policy 생성 실패: {error}'
            )
            return {}

    def validate_guidebook_policy_candidate(self, result, task, poses_dict):
        """Step 1 정책 후보를 기존 build_tasks()가 이해할 수 있는 형태로 검증합니다.

        현재 단계에서는 기존 executor를 그대로 재사용하기 위해 destination을
        A/B zone + u/v 형식으로 제한합니다. 이 제한은 임시 호환 계층이며,
        향후 zone 하드코딩 제거 시 Target Resolver로 대체할 예정입니다.
        """
        if not isinstance(result, dict):
            raise ValueError('Guidebook 정책 후보는 JSON 객체여야 합니다.')

        can_execute = result.get('can_execute')
        if not isinstance(can_execute, bool):
            raise ValueError('can_execute는 true 또는 false여야 합니다.')

        if not can_execute:
            reason = str(result.get('reason', '')).strip() or '실행 불가'
            return None, reason

        target = self.find_detected_target(result.get('target', ''), poses_dict)
        if target is None:
            raise ValueError(
                f"정책의 target을 현재 perception에서 찾을 수 없습니다: {result.get('target')}"
            )

        destination = result.get('destination')
        if not isinstance(destination, dict):
            raise ValueError('destination은 JSON 객체여야 합니다.')

        destination_type = str(destination.get('type', '')).strip().lower()
        if destination_type != 'zone':
            raise ValueError(
                "Step 1에서는 destination.type='zone'만 지원합니다. "
                '향후 Target Resolver 단계에서 relative/workspace 표현을 추가합니다.'
            )

        zone = self.normalize_zone(destination.get('zone', ''))
        if zone is None:
            raise ValueError('destination.zone은 A 또는 B여야 합니다.')
        if zone not in self.agent_specs[self.agent_id]:
            return None, f'{self.agent_id}가 {zone}구역에 도달할 수 없습니다.'

        try:
            u = float(destination.get('u'))
            v = float(destination.get('v'))
        except (TypeError, ValueError) as error:
            raise ValueError('destination.u/v에는 숫자가 필요합니다.') from error

        actions = self.validate_actions(result.get('actions'))

        legacy_policy = {
            'generated_policy': [
                {
                    'agent_id': self.agent_id,
                    'target': target,
                    'destination': f'{zone}구역',
                    'place_u': u,
                    'place_v': v,
                    'actions': actions,
                }
            ]
        }
        return legacy_policy, ''

    def poll_ready_guidebook_tasks(self):
        """READY Task 중 아직 후보를 만들지 않은 하나를 비동기로 계획합니다."""
        if not self.guidebook_policy_enabled:
            return
        if not self.latest_poses:
            return

        with self.guidebook_lock:
            mission_id = self.current_mission_id
            if not mission_id:
                return
            ready_ids = [
                task_id
                for task_id, status in self.guidebook_task_status.items()
                if status == 'READY'
            ]

        if not ready_ids:
            return

        now = time.time()
        selected = None
        with self.guidebook_policy_lock:
            for task_id in ready_ids:
                if task_id in self.guidebook_policy_candidates:
                    continue
                if task_id in self.guidebook_policy_inflight:
                    continue
                last = self.guidebook_policy_last_attempt.get(task_id, 0.0)
                if now - last < self.guidebook_policy_retry_sec:
                    continue
                self.guidebook_policy_last_attempt[task_id] = now
                self.guidebook_policy_inflight.add(task_id)
                selected = task_id
                break

        if selected is not None:
            threading.Thread(
                target=self.plan_one_guidebook_task,
                args=(selected,),
                daemon=True,
            ).start()

    def plan_one_guidebook_task(self, task_id):
        """READY Guidebook Task 하나를 기존 task 형식의 실행 후보까지 변환합니다."""
        try:
            with self.guidebook_lock:
                if self.guidebook_task_status.get(task_id) != 'READY':
                    return
                guidebook = dict(self.current_guidebook)
                task = dict(self.guidebook_tasks[task_id])
                mission_id = self.current_mission_id

            with self.perception_lock:
                poses_dict = dict(self.latest_poses)
                detected_items = list(self.current_detected_items)

            if not poses_dict:
                return

            result = self.ask_llm_for_guidebook_policy(
                guidebook,
                task,
                detected_items,
                poses_dict,
            )
            if not result:
                return

            legacy_policy, reason = self.validate_guidebook_policy_candidate(
                result,
                task,
                poses_dict,
            )
            if legacy_policy is None:
                # 동일 mission/task에서 실행 불가 판정을 매 poll마다 다시 LLM에 묻지 않습니다.
                with self.guidebook_policy_lock:
                    self.guidebook_policy_candidates[task_id] = None
                self.get_logger().info(
                    f'ℹ️ [{self.agent_id}] {task_id} 후보 제외: {reason}'
                )
                return

            # 핵심: 새 Guidebook 정책을 기존 build_tasks()에 맞춰 변환합니다.
            # TF / zone 계산 / actions 검증 / 기존 task 구조를 그대로 재사용합니다.
            built = self.build_tasks(legacy_policy, poses_dict)
            if len(built) != 1:
                raise ValueError(
                    f'Guidebook Task 하나에서 실행 후보 {len(built)}개가 생성되었습니다.'
                )

            execution_task = built[0]
            execution_task['mission_id'] = mission_id
            execution_task['guidebook_task_id'] = task_id
            execution_task['guidebook_description'] = str(task.get('description', ''))

            # LLM 응답을 기다리는 동안 다른 Agent가 먼저 Task를 가져갔을 수 있습니다.
            with self.guidebook_lock:
                if mission_id != self.current_mission_id:
                    return
                if self.guidebook_task_status.get(task_id) != 'READY':
                    self.get_logger().info(
                        f'ℹ️ [{self.agent_id}] {task_id}는 Policy 생성 중 이미 처리되어 후보를 폐기합니다.'
                    )
                    return

            with self.guidebook_policy_lock:
                self.guidebook_policy_candidates[task_id] = execution_task

            self.get_logger().info(
                f'✅ [{self.agent_id}] {task_id} Guidebook→기존 Task 변환 완료: '
                f'target={execution_task["target"]}, '
                f'destination={execution_task["destination"]}, '
                f'actions={len(execution_task["actions"])}'
            )
            self.publish_task_claim(task_id)

        except Exception as error:
            self.get_logger().error(
                f'❌ [{self.agent_id}] {task_id} Guidebook 정책 후보 처리 실패: {error}'
            )
        finally:
            with self.guidebook_policy_lock:
                self.guidebook_policy_inflight.discard(task_id)
                # 긴 timeout 뒤 즉시 재호출되지 않도록 종료 시각을 기준으로 retry합니다.
                self.guidebook_policy_last_attempt[task_id] = time.time()

    def load_policy_prompt(self, user_cmd, detected_items):
        """robot_agent.py에서 .txt 프롬프트를 직접 로드하고 값을 채웁니다."""
        template_text = self.prompt_path.read_text(encoding='utf-8')
        template = Template(template_text)
        zone_frame_text = (
            f'{self.workspace_frame} 기준' if self.use_tf_workspace else 'agent1 기준'
        )
        return template.substitute(
            detected_items=json.dumps(detected_items, ensure_ascii=False),
            user_command=str(user_cmd),
            agents=json.dumps(self.agent_specs, ensure_ascii=False),
            zone_frame=zone_frame_text,
            zones=json.dumps(self.ZONES, ensure_ascii=False),
        )

    def ask_llm_for_policy(self, user_cmd, detected_items):
        prompt = self.load_policy_prompt(user_cmd, detected_items)
        payload = {
            'model': self.llm_model, 'prompt': prompt, 'format': 'json', 'stream': False,
            'options': {'temperature': 0.0, 'num_predict': 1024}
        }
        self.get_logger().info('🧠 Gemma가 역할·순서·배치점·저수준 동작을 생성 중입니다...')
        try:
            response = requests.post(self.ollama_url, json=payload, timeout=60.0)
            response.raise_for_status()
            result = json.loads(response.json()['response'].strip())
            self.get_logger().info(f'🤖 [AI 생성 오리지널 Policy]: {result}')
            return result
        except Exception as error:
            self.get_logger().error(f'🚨 Code as Policy 생성 실패: {error}')
            return {}

    def build_tasks(self, policy_result, poses_dict):
        raw_tasks = policy_result.get('generated_policy', [])
        if not isinstance(raw_tasks, list) or not raw_tasks:
            raise ValueError('generated_policy는 비어 있지 않은 배열이어야 합니다.')

        if self.use_tf_workspace:
            # TF listener가 최신 변환을 받을 수 있게 cache 갱신을 한 번 시도합니다.
            self.update_workspace_tf_cache()

        with self.placed_points_lock:
            reserved = {zone: list(points) for zone, points in self.placed_points.items()}
        used_targets = set()
        tasks = []
        plan_stamp = int(time.time() * 1000)

        for order, raw in enumerate(raw_tasks, start=1):
            if not isinstance(raw, dict):
                raise ValueError(f'정책 작업 #{order}는 JSON 객체여야 합니다.')

            assignee = str(raw.get('agent_id', '')).strip()
            zone = self.normalize_zone(raw.get('destination', ''))
            if assignee not in self.agent_specs:
                raise ValueError(f'정책 작업 #{order}: 사용할 수 없는 Agent {assignee}')
            if zone is None or zone not in self.agent_specs[assignee]:
                raise ValueError(
                    f'정책 작업 #{order}: {assignee}가 도달할 수 없는 목적지입니다.'
                )

            target = self.find_detected_target(raw.get('target', ''), poses_dict)
            if target is None or target in used_targets:
                raise ValueError(
                    f'정책 작업 #{order}: 인식되지 않았거나 중복된 target '
                    f'{raw.get("target")}'
                )

            try:
                u = float(raw.get('place_u'))
                v = float(raw.get('place_v'))
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f'정책 작업 #{order}: place_u/place_v에는 숫자가 필요합니다.'
                ) from error

            place_x, place_y = self.select_place_point(zone, u, v, reserved[zone])
            raw_actions = raw.get('actions', self.DEFAULT_PNP_ACTIONS)
            actions = self.validate_actions(raw_actions)
            rx, ry, rz, yaw = poses_dict[target]

            ref_object = {
                'x': float(rx), 'y': float(ry),
                'z': float(rz), 'yaw': float(yaw),
            }
            ref_place = {'x': float(place_x), 'y': float(place_y), 'yaw': 0.0}
            robot_place = self.convert_place_pose(ref_place, assignee)
            if self.use_tf_workspace:
                base_frame = self.get_agent_base_frame(assignee)
                self.get_logger().info(
                    f"🧭 TF place 변환: {assignee}/{zone}구역 workspace="
                    f"({ref_place['x']:.1f}, {ref_place['y']:.1f}, yaw={np.degrees(ref_place['yaw']):.1f}deg) -> "
                    f"{base_frame}=({robot_place['x']:.1f}, {robot_place['y']:.1f}, "
                    f"yaw={np.degrees(robot_place['yaw']):.1f}deg)"
                )
            tasks.append({
                'task_id': f'{plan_stamp}_{order:02d}',
                'order': order,
                'coordinator_id': self.agent_id,
                'assignee_id': assignee,
                'target': target,
                'destination': f'{zone}구역',
                'place_u': u,
                'place_v': v,
                'reference_object_pose': ref_object,
                'reference_place_pose': ref_place,
                'object_pose': self.convert_object_pose(ref_object, assignee),
                'place_pose': robot_place,
                'actions': actions,
                'retry_count': 0,
            })
            used_targets.add(target)
            reserved[zone].append((place_x, place_y))

        return tasks

    def command_callback(self, msg):
        if self.is_moving:
            self.get_logger().warn('⚠️ 현재 작업 또는 정책 생성이 진행 중입니다.')
            return
        if not self.latest_poses:
            self.get_logger().warn('⚠️ 사용할 수 있는 객체 좌표가 없습니다.')
            return
        self.is_moving = True
        threading.Thread(
            target=self.plan_and_dispatch,
            args=(msg.data, self.latest_poses.copy(), self.current_detected_items.copy()),
            daemon=True,
        ).start()

    def plan_and_dispatch(self, user_cmd, poses_dict, detected_items):
        local_count = 0
        try:
            self.get_logger().info(f'🧾 user_cmd = {user_cmd}')
            self.get_logger().info(f'🧾 detected_items = {detected_items}')
            self.get_logger().info(f'🧾 poses keys = {list(poses_dict.keys())}')
            self.get_logger().info(f'🧾 agent_specs = {self.agent_specs}')
            if self.use_tf_workspace:
                self.get_logger().info(
                    f'🧭 TF workspace mode = {self.agent_base_frames}, '
                    f'workspace={self.workspace_frame}, ZONES(workspace mm) = {self.ZONES}'
                )

            policy_result = self.ask_llm_for_policy(user_cmd, detected_items)
            tasks = self.build_tasks(policy_result, poses_dict)

            if not tasks:
                self.get_logger().warn('⚠️ 실행 가능한 정책이 없습니다.')
                return

            local_count = sum(task['assignee_id'] == self.agent_id for task in tasks)
            for task in tasks:  # LLM 배열 순서 그대로 발행
                msg = String()
                msg.data = json.dumps(task, ensure_ascii=False)
                self.task_pub.publish(msg)
                self.get_logger().info(
                    f"➡️ [{task['task_id']}] {task['target']} -> "
                    f"{task['assignee_id']} / {task['destination']} / "
                    f"place=({task['place_pose']['x']:.1f}, {task['place_pose']['y']:.1f}, "
                    f"yaw={np.degrees(task['place_pose']['yaw']):.1f}deg)"
                )
                time.sleep(0.05)
            self.get_logger().info('✅ 정책 배포 완료. Agent별 Queue를 순차 실행합니다.')
        except Exception as error:
            self.get_logger().error(f'🚨 정책 배포 오류: {error}')
        finally:
            if local_count == 0:
                self.is_moving = False

    def validate_received_task(self, task):
        required = (
            'task_id', 'assignee_id', 'target', 'destination', 'reference_object_pose',
            'reference_place_pose', 'object_pose', 'place_pose', 'actions'
        )
        if any(key not in task for key in required):
            raise ValueError('작업 메시지에 필수 필드가 없습니다.')
        if str(task['assignee_id']) != self.agent_id:
            return False

        zone = self.normalize_zone(task['destination'])
        if zone not in self.agent_specs[self.agent_id]:
            raise ValueError(f'{self.agent_id}가 도달할 수 없는 목적지입니다.')
        for key in ('x', 'y', 'z', 'yaw'):
            float(task['object_pose'][key])
        for key in ('x', 'y', 'yaw'):
            float(task['place_pose'][key])
        task['actions'] = self.validate_actions(task['actions'])
        return True

    def task_callback(self, msg):
        try:
            task = json.loads(msg.data)
            if not self.validate_received_task(task):
                return
            with self.task_queue_lock:
                self.task_queue.append(task)
                start_worker = not self.task_worker_running
                if start_worker:
                    self.task_worker_running = True
            self.is_moving = True
            self.get_logger().info(f"📩 [{task['task_id']}] [{task['target']}] Queue 추가")
            if start_worker:
                threading.Thread(target=self.process_task_queue, daemon=True).start()
        except Exception as error:
            self.get_logger().error(f'❌ 작업 메시지 분석 실패: {error}')

    def process_task_queue(self):
        while rclpy.ok():
            with self.task_queue_lock:
                if not self.task_queue:
                    self.task_worker_running = False
                    self.is_moving = False
                    return
                task = self.task_queue.pop(0)

            guidebook_task_id = str(task.get('guidebook_task_id', '')).strip()
            if guidebook_task_id:
                self.publish_task_status(guidebook_task_id, 'EXECUTING')

            success = self.execute_task(task)

            if guidebook_task_id:
                self.publish_task_status(
                    guidebook_task_id,
                    'SUCCEEDED' if success else 'FAILED'
                )

            if not success:
                self.handle_task_failure(task)
                return

    def execute_task(self, task):
        obj = task['object_pose']
        place = task['place_pose']
        self.get_logger().info(
            f"🦾 [{task['task_id']}] [{task['target']}] -> {task['destination']} 시작 | "
            f"obj=({float(obj['x']):.1f}, {float(obj['y']):.1f}, {float(obj['z']):.1f}) | "
            f"place=({float(place['x']):.1f}, {float(place['y']):.1f}, "
            f"yaw={np.degrees(float(place['yaw'])):.1f}deg)"
        )
        try:
            self.arm.clean_error()
            self.arm.set_state(0)
            for index, action in enumerate(task['actions'], start=1):
                api = action['api']
                self.get_logger().info(f"💻 [{task['task_id']}] Action {index}: {action}")
                if api == 'control_gripper':
                    success = self.control_gripper(action['position'])
                elif api == 'move_to_object':
                    success = self.move_to_robot_tf(
                        obj['x'], obj['y'], obj['z'] + action['z_offset'],
                        yaw=obj['yaw'], speed=action['speed'],
                        label=f"{task['task_id']} object"
                    )
                elif api == 'move_to_place':
                    success = self.move_to_robot_tf(
                        place['x'], place['y'], obj['z'] + action['z_offset'],
                        yaw=place['yaw'], speed=action['speed'],
                        label=f"{task['task_id']} place"
                    )
                elif api == 'wait':
                    time.sleep(action['seconds'])
                    success = True
                elif api == 'return_home':
                    success = self.move_to(*self.HOME_POSE[:3], yaw=self.HOME_POSE[3])
                else:
                    success = False
                if not success:
                    return False

            msg = Float32MultiArray()
            msg.data = [
                float(place['x']), float(place['y']),
                float(obj['z'] + self.PICK_PLACE_Z_OFFSET_MM), float(place['yaw'])
            ]
            self.pose_publisher.publish(msg)
            zone = self.normalize_zone(task['destination'])
            reference_place = task['reference_place_pose']
            with self.placed_points_lock:
                self.placed_points[zone].append((
                    float(reference_place['x']), float(reference_place['y'])
                ))
            self.get_logger().info(f"✅ [{task['task_id']}] 작업 완료")
            return True
        except Exception as error:
            self.get_logger().error(f"🚨 [{task['task_id']}] 작업 오류: {error}")
            return False

    def recover_to_safe_pose(self):
        try:
            self.arm.clean_error()
            time.sleep(0.5)
            self.arm.motion_enable(True)
            time.sleep(0.2)
            self.arm.set_state(0)
            time.sleep(0.5)
            x, y, z, yaw = self.SAFE_RETREAT_POSE
            self.get_logger().info(f'🛡️ [{self.agent_id}] 안전 위치 ({x}, {y}, {z})로 대피')
            return self.move_to(x, y, z, yaw=yaw)
        except Exception as error:
            self.get_logger().error(f'🚨 대피 동작 오류: {error}')
            return False

    def find_alternative_agent(self, task):
        zone = self.normalize_zone(task['destination'])
        return next(
            (
                agent_id for agent_id, zones in self.agent_specs.items()
                if agent_id != self.agent_id and zone in zones
            ),
            None,
        )

    def handle_task_failure(self, failed_task):
        self.get_logger().error(f"🚨 [{failed_task['task_id']}] 실패. 즉시 복구·대피합니다.")
        self.recover_to_safe_pose()

        with self.task_queue_lock:
            remaining = [failed_task] + self.task_queue
            self.task_queue.clear()
            self.task_worker_running = False

        for task in remaining:
            if int(task.get('retry_count', 0)) >= 1:
                self.get_logger().error(f"❌ [{task['task_id']}] 재시도 한도 초과")
                continue
            alternative = self.find_alternative_agent(task)
            if alternative is None:
                self.get_logger().error(
                    f"❌ [{task['task_id']}] {task['destination']} 대체 Agent가 없습니다."
                )
                continue
            try:
                reassigned = dict(task)
                reassigned['task_id'] = f"{task['task_id']}_retry"
                reassigned['assignee_id'] = alternative
                reassigned['retry_count'] = int(task.get('retry_count', 0)) + 1
                reassigned['object_pose'] = self.convert_object_pose(
                    task['reference_object_pose'], alternative
                )
                reassigned['place_pose'] = self.convert_place_pose(
                    task['reference_place_pose'], alternative
                )
                msg = String()
                msg.data = json.dumps(reassigned, ensure_ascii=False)
                self.task_pub.publish(msg)
                self.get_logger().warn(
                    f"🔄 [{task['task_id']}] {alternative}에게 재할당"
                )
            except Exception as error:
                self.get_logger().error(f"🚨 [{task['task_id']}] 재할당 실패: {error}")
        self.is_moving = False

    def color_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.perception_lock:
                self.latest_color_image = image.copy()
        except Exception as error:
            self.get_logger().warn(f'⚠️ color image 변환 실패: {error}')

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self.perception_lock:
                self.latest_depth_image = np.array(depth_image, copy=True)
        except Exception as error:
            self.get_logger().warn(f'⚠️ depth image 변환 실패: {error}')

    def camera_info_callback(self, msg):
        k = msg.k
        frame_id = str(msg.header.frame_id).strip()
        with self.perception_lock:
            self.camera_intrinsics = {
                'fx': float(k[0]),
                'fy': float(k[4]),
                'cx': float(k[2]),
                'cy': float(k[5]),
            }
            if frame_id:
                self.camera_frame = frame_id

    def get_latest_perception_frames(self):
        with self.perception_lock:
            if (
                self.latest_color_image is None
                or self.latest_depth_image is None
                or self.camera_intrinsics is None
                or self.camera_frame is None
            ):
                return None
            return (
                self.latest_color_image.copy(),
                self.latest_depth_image.copy(),
                dict(self.camera_intrinsics),
            )

    @staticmethod
    def deproject_pixel_to_point(u, v, depth_m, intrinsics):
        x = (float(u) - intrinsics['cx']) / intrinsics['fx'] * float(depth_m)
        y = (float(v) - intrinsics['cy']) / intrinsics['fy'] * float(depth_m)
        z = float(depth_m)
        return x, y, z

    @staticmethod
    def median_depth_m(depth_image, u, v):
        if depth_image is None or depth_image.ndim != 2:
            return None
        height, width = depth_image.shape
        if not (0 <= u < width and 0 <= v < height):
            return None
        x0, x1 = max(0, u - 5), min(width, u + 6)
        y0, y1 = max(0, v - 5), min(height, v + 6)
        patch = depth_image[y0:y1, x0:x1]
        if np.issubdtype(patch.dtype, np.integer):
            valid = patch[patch > 0]
            if valid.size == 0:
                return None
            return float(np.median(valid)) / 1000.0
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def run_perception(self):
        class_map_ko = {46: '바나나', 47: '사과', 49: '오렌지', 64: '마우스'}
        class_map_en = {46: 'Banana', 47: 'Apple', 49: 'Orange', 64: 'Mouse'}
        try:
            while rclpy.ok():
                # ROS callback(/tf, /tf_static, camera topics, command, timer)은
                # 별도 executor thread에서 계속 처리한다. 여기서는 YOLO만 수행한다.
                frames = self.get_latest_perception_frames()
                if frames is None:
                    now = time.time()
                    if now - self.last_wait_log_time > 5.0:
                        self.get_logger().info(
                            '⏳ camera topic 대기 중... '
                            f'color={self.color_topic}, depth={self.depth_topic}, info={self.camera_info_topic}'
                        )
                        self.last_wait_log_time = now
                    continue

                img, depth_img, intrinsics = frames
                results = self.model.predict(
                    source=img, classes=[46, 47, 49, 64], conf=0.3, verbose=False
                )
                current_poses, current_items = {}, []

                for result in results:
                    if result.masks is None or self.is_moving:
                        continue
                    for index, mask_array in enumerate(result.masks.xy):
                        mask_np = mask_array.astype(np.float32)
                        if len(mask_np) < 5:
                            continue

                        class_id = int(result.boxes.cls[index])
                        name_ko = class_map_ko.get(class_id, f'물체_{class_id}')
                        name_en = class_map_en.get(class_id, f'Obj_{class_id}')
                        if name_ko not in current_items:
                            current_items.append(name_ko)

                        moments = cv2.moments(mask_np)
                        if moments['m00'] == 0:
                            continue
                        cx = int(moments['m10'] / moments['m00'])
                        cy = int(moments['m01'] / moments['m00'])

                        box = np.int32(cv2.boxPoints(cv2.minAreaRect(mask_np)))
                        d01 = np.linalg.norm(box[0] - box[1])
                        d12 = np.linalg.norm(box[1] - box[2])
                        if d01 > d12:
                            pt1, pt2 = (box[1] + box[2]) / 2, (box[0] + box[3]) / 2
                        else:
                            pt1, pt2 = (box[0] + box[1]) / 2, (box[2] + box[3]) / 2
                        center_px = np.array([cx, cy])
                        front_pt = pt1 if np.linalg.norm(center_px - pt1) > np.linalg.norm(center_px - pt2) else pt2

                        dist_z = self.median_depth_m(depth_img, cx, cy)
                        if dist_z is None:
                            continue

                        p3d = self.deproject_pixel_to_point(cx, cy, dist_z, intrinsics)
                        try:
                            wx, wy, wz = self.camera_point_to_workspace(*p3d)
                            p3d_front = self.deproject_pixel_to_point(
                                int(front_pt[0]), int(front_pt[1]), dist_z, intrinsics
                            )
                            wx_front, wy_front, _ = self.camera_point_to_workspace(*p3d_front)
                        except Exception:
                            continue
                        yaw = np.arctan2(wy_front - wy, wx_front - wx)
                        current_poses[name_ko] = (wx, wy, wz, yaw)

                        cv2.drawContours(img, [box], 0, (255, 0, 0), 2)
                        cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.arrowedLine(
                            img, (cx, cy), (int(front_pt[0]), int(front_pt[1])),
                            (0, 255, 0), 3, tipLength=0.3
                        )
                        cv2.putText(
                            img, f'[{name_en}] WS Yaw:{int(np.degrees(yaw))}',
                            (cx - 30, cy - 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 255), 2
                        )

                if not self.is_moving:
                    self.latest_poses = current_poses
                    self.current_detected_items = current_items
                cv2.imshow(f'{self.agent_id} Pick & Place', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            cv2.destroyAllWindows()

    def start_ros_spin_thread(self):
        """ROS callback 처리를 perception/YOLO 루프와 분리해서 계속 spin한다.

        기존 구조는 run_perception() 내부에서 spin_once()를 한 번 수행한 뒤
        YOLO 추론을 실행했다. YOLO가 오래 걸리거나 이미지 callback이 밀리면
        /tf callback과 TF cache timer가 충분히 처리되지 않아 TF buffer가 멈출 수 있다.
        """
        if self._ros_executor is not None:
            return
        self._ros_executor = MultiThreadedExecutor(num_threads=2)
        self._ros_executor.add_node(self)
        self._ros_spin_thread = threading.Thread(
            target=self._ros_executor.spin,
            daemon=True,
        )
        self._ros_spin_thread.start()
        self.get_logger().info('🧵 ROS callback spin thread 시작: /tf, camera, guidebook, task, timer 처리 분리')

    def run(self):
        if self.enable_perception:
            self.start_ros_spin_thread()
            self.run_perception()
        else:
            rclpy.spin(self)

    def shutdown(self):
        if self._ros_executor is not None:
            try:
                self._ros_executor.shutdown()
            except Exception:
                pass
            self._ros_executor = None
        if cv2 is not None:
            cv2.destroyAllWindows()
        if getattr(self, 'arm', None) is not None:
            self.arm.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotAgentNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
