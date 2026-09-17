import json
import math
import threading
import time
import uuid
from pathlib import Path
from string import Template

import numpy as np
import requests
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from xarm.wrapper import XArmAPI

try:
    from .llm_api import DEFAULT_PNP_ACTIONS as LLM_DEFAULT_PNP_ACTIONS
    from .llm_api import validate_cooperative_actions
    from .llm_api import validate_actions as validate_llm_actions
    from .ollama_stream import consume_ollama_stream
    from .shared_zone import classify_shared_zone_entry, expand_aabb
except ImportError:
    # 소스 디렉터리에서 직접 실행할 때를 위한 fallback
    from llm_api import DEFAULT_PNP_ACTIONS as LLM_DEFAULT_PNP_ACTIONS
    from llm_api import validate_cooperative_actions
    from llm_api import validate_actions as validate_llm_actions
    from ollama_stream import consume_ollama_stream
    from shared_zone import classify_shared_zone_entry, expand_aabb

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
        # 순차 협업 실험에서 두 로봇의 배치 위치가 충분히 떨어지도록
        # 기존 구역을 조금 더 분리합니다.
        'A': {'x_min': 0.0, 'x_max': 250.0, 'y_min': 0.0, 'y_max': 400.0},
        'B': {'x_min': 450.0, 'x_max': 700.0, 'y_min': 0.0, 'y_max': 400.0},
    }
    ZONE_MARGIN_MM = 50.0
    MIN_OBJECT_SPACING_MM = 120.0
    PICK_PLACE_Z_OFFSET_MM = 40.0
    DEFAULT_PNP_ACTIONS = LLM_DEFAULT_PNP_ACTIONS
    # 첨부된 xArm 기본 자세:
    # TCP ≈ (198.0, -2.2, 264.1) mm, RPY ≈ (178.4, 0.0, 0.2) deg
    # Joint = [0, -60, -30, 0, 90, 0] deg
    # Cartesian 좌표보다 joint 자세가 재현성이 높아 기본 자세 복귀는 joint 명령을 사용합니다.
    HOME_JOINT_ANGLES_DEG = [0.0, -60.0, -30.0, 0.0, 90.0, 0.0]
    HOME_JOINT_SPEED_DEG_S = 20.0
    CLAIM_WAIT_SEC = 2.0
     
    def __init__(self):
        super().__init__('robot_agent_node')
        
        self.declare_parameter('agent_id', 'agent1')
        self.declare_parameter('robot_ip', '192.168.1.218')
        self.declare_parameter('enable_perception', True)
        # 예: ['agent1:A|B'] 또는 ['agent1:A', 'agent2:B']
        # 이번 TF 테스트는 robot_1_base 하나로 A/B구역을 모두 검증할 수 있게 기본값을 agent1:A|B로 둡니다.
        self.declare_parameter('agent_specs', ['agent1:A|B'])
        # ArUco 노드가 broadcast하는 TF 이름을 그대로 사용합니다.
        # target_frame <- source_frame = robot_1_base <- workspace_0
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
        self.declare_parameter('llm_model', 'qwen3.8:27b')
        self.declare_parameter('ollama_url', 'http://localhost:11434/api/generate')
        # 매 LLM 요청 직전에 읽으므로 실행 중 ros2 param set으로 변경할 수 있습니다.
        self.declare_parameter('llm_think', True)
        self.declare_parameter('guidebook_prompt_file', 'agent_guidebook_policy.txt')
        # [MERGED] Workstation이 완성한 협동 계획은 별도 Agent LLM 프롬프트로 검토합니다.
        self.declare_parameter(
            'cooperative_review_prompt_file',
            'agent_cooperative_review.txt',
        )
        self.declare_parameter('guidebook_policy_enabled', True)
        self.declare_parameter('guidebook_policy_poll_sec', 1.0)
        self.declare_parameter('guidebook_policy_retry_sec', 5.0)
        self.declare_parameter(
            'yolo_model_paths',
            ['models/yolo11m-seg.pt', 'models/yolo-bread-lettuce.pt'],
        )
        self.declare_parameter('guidebook_topic', '/mission/guidebook')
        self.declare_parameter('task_claim_topic', '/mission/task_claim')
        self.declare_parameter('task_status_topic', '/mission/task_status')
        self.declare_parameter('extra_perception_topic', '/perception/yolo_extra')
        self.declare_parameter('extra_perception_max_age_sec', 2.0)
        self.declare_parameter('cooperative_review_timeout_sec', 60.0)
        self.declare_parameter('step_sync_timeout_sec', 30.0)
        self.declare_parameter('step_start_delay_sec', 0.25)

        # A+B 공용 구역 토큰 / 기하 게이트 파라미터
        self.declare_parameter('zone_token_enabled', True)
        self.declare_parameter('zone_token_request_topic', '/zone_token/request')
        self.declare_parameter('zone_token_state_topic', '/zone_token/state')
        self.declare_parameter('zone_token_acquire_timeout_sec', 60.0)
        self.declare_parameter('zone_token_lease_sec', 45.0)
        self.declare_parameter('shared_zone_x_min', 150.0)
        self.declare_parameter('shared_zone_x_max', 450.0)
        self.declare_parameter('shared_zone_y_min', -50.0)
        self.declare_parameter('shared_zone_y_max', 500.0)
        self.declare_parameter('shared_zone_margin_mm', 0.0)

        self.agent_id = str(self.get_parameter('agent_id').value).strip()
        self.robot_ip = str(self.get_parameter('robot_ip').value).strip()
        self.enable_perception = bool(self.get_parameter('enable_perception').value)
        self.agent_specs = self.parse_agent_specs(self.get_parameter('agent_specs').value)
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
        self.guidebook_prompt_file = str(
            self.get_parameter('guidebook_prompt_file').value
        ).strip()
        self.cooperative_review_prompt_file = str(
            self.get_parameter('cooperative_review_prompt_file').value
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
        self.yolo_model_paths = [
            str(path).strip()
            for path in self.get_parameter('yolo_model_paths').value
            if str(path).strip()
        ]
        self.guidebook_topic = str(self.get_parameter('guidebook_topic').value).strip()
        self.task_claim_topic = str(self.get_parameter('task_claim_topic').value).strip()
        self.task_status_topic = str(self.get_parameter('task_status_topic').value).strip()
        self.extra_perception_topic = str(
            self.get_parameter('extra_perception_topic').value
        ).strip()
        self.extra_perception_max_age_sec = float(
            self.get_parameter('extra_perception_max_age_sec').value
        )
        self.cooperative_review_timeout_sec = float(
            self.get_parameter('cooperative_review_timeout_sec').value
        )
        self.step_sync_timeout_sec = float(
            self.get_parameter('step_sync_timeout_sec').value
        )
        self.step_start_delay_sec = float(
            self.get_parameter('step_start_delay_sec').value
        )
        self.zone_token_enabled = bool(
            self.get_parameter('zone_token_enabled').value
        )
        self.zone_token_request_topic = str(
            self.get_parameter('zone_token_request_topic').value
        ).strip()
        self.zone_token_state_topic = str(
            self.get_parameter('zone_token_state_topic').value
        ).strip()
        self.zone_token_acquire_timeout_sec = float(
            self.get_parameter('zone_token_acquire_timeout_sec').value
        )
        self.zone_token_lease_sec = float(
            self.get_parameter('zone_token_lease_sec').value
        )
        self.shared_zone_box = {
            'x_min': float(self.get_parameter('shared_zone_x_min').value),
            'x_max': float(self.get_parameter('shared_zone_x_max').value),
            'y_min': float(self.get_parameter('shared_zone_y_min').value),
            'y_max': float(self.get_parameter('shared_zone_y_max').value),
        }
        self.shared_zone_margin_mm = float(
            self.get_parameter('shared_zone_margin_mm').value
        )
        self.guidebook_prompt_path = self.resolve_prompt_path(
            self.guidebook_prompt_file
        )
        self.cooperative_review_prompt_path = self.resolve_prompt_path(
            self.cooperative_review_prompt_file
        )
        
        if not self.agent_id or not self.robot_ip:
            raise ValueError('agent_id와 robot_ip는 비어 있을 수 없습니다.')
        if self.agent_id not in self.agent_specs:
            raise ValueError(f'{self.agent_id}가 agent_specs에 없습니다: {self.agent_specs}')
        if not self.workspace_frame:
            raise ValueError('workspace_frame은 비어 있을 수 없습니다.')
        if not self.agent_base_frames:
            raise ValueError('agent_base_frames는 비어 있을 수 없습니다.')
        if self.enable_perception and (not self.color_topic or not self.depth_topic or not self.camera_info_topic):
            raise ValueError('perception 사용 시 color/depth/camera_info topic은 비어 있을 수 없습니다.')
            
        # [MERGED] R8/R9 제거: workspace TF와 identity SDK frame만 지원합니다.
        # workspace_tf_cache[agent_id] = (TransformStamped, cache_update_wall_time_sec)
        self.workspace_tf_cache = {}
        self.tf_cache_lock = threading.Lock()
        self.tf_ready_agents = set()
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
            f'🧭 [{self.agent_id}] TF workspace 필수: {frame_text}'
        )
        
        self.get_logger().info(f'🔌 [{self.agent_id}] 로봇 연결 중... IP={self.robot_ip}')
        self.arm = XArmAPI(self.robot_ip, is_radian=False)
        self.get_logger().info(f'✅ [{self.agent_id}] 로봇 연결 완료!')
        
        # Workstation이 발행한 동일한 가이드북을 모든 Agent가 받아 상태를 공유합니다.
        self.current_mission_id = None
        self.current_plan_revision = -1
        self.current_guidebook = {}
        self.guidebook_tasks = {}
        self.guidebook_task_status = {}
        self.guidebook_lock = threading.Lock()
        
        # [MERGED] 협동 계획 검토와 action 단위 barrier 상태입니다.
        self.plan_approved_revision = -1
        self.cooperative_review_inflight = set()
        self.cooperative_reviewed = set()
        self.cooperative_dispatched = set()
        self.extra_perception_state = None
        self.extra_perception_received_monotonic = 0.0
        self.extra_perception_lock = threading.Lock()
        self.step_sync_condition = threading.Condition()
        self.step_sync_state = {}
        self.step_go_sent = set()
        self.cooperative_aborted = set()
        self.llm_request_lock = threading.Lock()
        
        # 일반 READY Task를 Agent LLM 실행 후보로 변환합니다.
        self.guidebook_policy_candidates = {}
        self.guidebook_policy_inflight = set()
        self.guidebook_policy_last_attempt = {}
        self.guidebook_policy_lock = threading.Lock()
        
        # 일반 Task의 claim 후보와 결정된 winner를 저장합니다.
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
        
        # 별도 custom msg 없이 std_msgs/String JSON으로 claim/plan/task/step을 공유합니다.
        self.task_claim_pub = self.create_publisher(String, self.task_claim_topic, 10)
        self.task_claim_sub = self.create_subscription(
            String, self.task_claim_topic, self.task_claim_callback, 10
        )
        self.task_status_pub = self.create_publisher(String, self.task_status_topic, 10)
        self.task_status_sub = self.create_subscription(
            String, self.task_status_topic, self.task_status_callback, 10
        )
        self.extra_perception_sub = self.create_subscription(
            String,
            self.extra_perception_topic,
            self.extra_perception_callback,
            10,
        )
        
        # 기존 /agent_task 기반 실행 경로는 회귀 방지를 위해 그대로 유지합니다.
        # 모든 Agent가 같은 토픽을 구독하고 assignee_id가 자신인 작업만 실행합니다.
        self.task_pub = self.create_publisher(String, '/agent_task', 10)
        self.task_sub = self.create_subscription(String, '/agent_task', self.task_callback, 10)
        
        # ==============================================================
        # A+B 공용 구역 토큰: 토픽 기반 클라이언트 (zone_token_manager 노드와 통신)
        # 안전 보장은 execute_task 의 기하 게이트가 담당하고, 여기서는 요청/상태만.
        # ==============================================================
        self._zone_cbg = ReentrantCallbackGroup()
        self.zone_token_pub = self.create_publisher(
            String, self.zone_token_request_topic, 10
        )
        zone_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.zone_token_state_sub = self.create_subscription(
            String,
            self.zone_token_state_topic,
            self.zone_token_state_callback,
            zone_state_qos,
            callback_group=self._zone_cbg,
        )
        self._zone_cond = threading.Condition()
        self._zone_state = {'holder': None, 'queue': [], 'epoch': 0}
        self._zone_token_held = False
        self._zone_token_epoch = None
        if self.zone_token_enabled:
            self._zone_hb_timer = self.create_timer(
                max(5.0, self.zone_token_lease_sec / 3.0),
                self._zone_token_heartbeat,
                callback_group=self._zone_cbg,
            )
        # ==============================================================

        self.latest_poses = {}
        self.current_detected_items = []
        self.is_moving = False
        self.task_queue = []
        self.task_queue_lock = threading.Lock()
        self.task_worker_running = False
        self.placed_points = {'A': [], 'B': []}
        self.placed_points_lock = threading.Lock()
        
        self.models = []
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
            package_share = Path(get_package_share_directory('cap_robot'))
            for configured_path in self.yolo_model_paths:
                model_path = Path(configured_path).expanduser()
                if not model_path.is_absolute():
                    model_path = package_share / model_path
                if not model_path.is_file():
                    raise FileNotFoundError(f'YOLO model not found: {model_path}')
                model = YOLO(str(model_path))
                self.models.append((model_path.name, model))
                self.get_logger().info(
                    f'YOLO model loaded: {model_path} '
                    f'(task={model.task}, classes={model.names})'
                )
            self.bridge = CvBridge()
            image_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
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
            f'think={bool(self.get_parameter("llm_think").value)}, '
            f'normal_prompt={self.guidebook_prompt_path}, '
            f'cooperative_review_prompt={self.cooperative_review_prompt_path}'
        )
        self.get_logger().info(
            f'📘 Guidebook policy: enabled={self.guidebook_policy_enabled}, '
            f'prompt={self.guidebook_prompt_path}, '
            f'poll={self.guidebook_policy_poll_sec:.1f}s, '
            f'retry={self.guidebook_policy_retry_sec:.1f}s'
        )
        self.get_logger().info(
            '🧭 xArm command frame mode=identity, '
            f'dry_run={self.dry_run}, sdk_safety_box={self.enable_sdk_safety_box}, '
            f'x=[{self.sdk_x_min:.0f},{self.sdk_x_max:.0f}], '
            f'y=[{self.sdk_y_min:.0f},{self.sdk_y_max:.0f}], '
            f'z=[{self.sdk_z_min:.0f},{self.sdk_z_max:.0f}]'
        )
        self.get_logger().info(
            f'✅ [{self.agent_id}] 준비 완료 '
            f'(perception={self.enable_perception}, '
            f'zones={self.agent_specs[self.agent_id]}, '
            'workspace_tf=required, '
            f'guidebook_topic={self.guidebook_topic})'
        )

    def guidebook_callback(self, msg):
        """Workstation guidebook을 저장하고 일반/협동 Task 경로를 준비합니다."""
        try:
            guidebook = json.loads(msg.data)
            if not isinstance(guidebook, dict):
                raise ValueError('guidebook은 JSON 객체여야 합니다.')

            mission_id = str(guidebook.get('mission_id', '')).strip()
            plan_revision = int(guidebook.get('plan_revision', 0))
            tasks = guidebook.get('tasks')
            if not mission_id:
                raise ValueError('mission_id가 비어 있습니다.')

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
                normalized_task['execution_mode'] = str(
                    task.get('execution_mode', 'single_agent')
                ).strip()
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
                if (
                    self.current_mission_id == mission_id
                    and plan_revision <= self.current_plan_revision
                ):
                    self.get_logger().info(
                        f'📘 [{self.agent_id}] 이미 처리한 계획 무시: '
                        f'{mission_id}/r{plan_revision}'
                    )
                    return
                self.current_mission_id = mission_id
                self.current_plan_revision = plan_revision
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

            with self.step_sync_condition:
                self.plan_approved_revision = -1
                self.cooperative_review_inflight.clear()
                self.cooperative_reviewed.clear()
                self.cooperative_dispatched.clear()
                self.step_sync_state.clear()
                self.step_go_sent.clear()
                self.cooperative_aborted.clear()
                self.step_sync_condition.notify_all()

            # 이전 mission의 배치 기록이 다음 mission의 120 mm 간격 검사에
            # 영향을 주지 않도록 새 mission마다 초기화합니다.
            with self.placed_points_lock:
                self.placed_points = {'A': [], 'B': []}

            self.get_logger().info(
                f'📘 [{self.agent_id}] Guidebook 수신: '
                f'mission_id={mission_id}, revision={plan_revision}, '
                f'tasks={len(task_map)}'
            )
            for task_id, task in task_map.items():
                self.get_logger().info(
                    f'  - {task_id}: {status_snapshot[task_id]} | '
                    f'depends_on={task["depends_on"]} | '
                    f'{task.get("description", "")}'
                )

            # [MERGED] 협동 Task는 READY 여부와 무관하게 이동 전에 전체 계획을 검토합니다.
            for task_id, task in task_map.items():
                if (
                    self.is_cooperative_task(task)
                    and self.agent_id in task.get('participants', [])
                ):
                    key = (mission_id, plan_revision, task_id)
                    with self.step_sync_condition:
                        self.cooperative_review_inflight.add(key)
                    threading.Thread(
                        target=self.review_cooperative_task,
                        args=(mission_id, plan_revision, task_id),
                        daemon=True,
                    ).start()

        except Exception as error:
            self.get_logger().error(f'❌ Guidebook 수신/해석 실패: {error}')

    def refresh_guidebook_task_states_locked(self):
        """guidebook_lock을 잡은 상태에서 상태를 계산합니다."""
        for task_id, task in self.guidebook_tasks.items():
            current = self.guidebook_task_status.get(task_id)
            if current in ('CLAIMED', 'EXECUTING', 'SUCCEEDED', 'FAILED'):
                continue
            dependencies = task.get('depends_on', [])
            
            # [수정] SUCCEEDED 뿐만 아니라 EXECUTING 상태여도 선행 조건 통과로 인정
            ready = all(
                self.guidebook_task_status.get(dep) in ('SUCCEEDED', 'EXECUTING')
                for dep in dependencies
            )
            self.guidebook_task_status[task_id] = 'READY' if ready else 'BLOCKED'

    @staticmethod
    def is_cooperative_task(task):
        return str(task.get('execution_mode', 'single_agent')).strip() == 'cooperative'


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

        일반 Task의 기존 중복 실행 방지를 위해 고정된 사전순 winner 규칙을 사용합니다.
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
            'scope': 'task',
            'mission_id': mission_id,
            'plan_revision': self.current_plan_revision,
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
            scope = str(payload.get('scope', 'task')).strip().lower()
            if scope == 'plan':
                self.apply_plan_status(payload)
            elif scope == 'step':
                self.apply_step_status(payload)
            elif scope == 'task':
                self.apply_task_status(payload)
            else:
                raise ValueError(f'지원하지 않는 status scope: {scope}')
        except Exception as error:
            self.get_logger().error(f'❌ task_status 해석 실패: {error}')

    def extra_perception_callback(self, msg):
        """[MERGED] Workstation과 동일한 손잡이 인식 스냅샷을 보관합니다."""
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict):
                raise ValueError('extra perception은 JSON 객체여야 합니다.')
            with self.extra_perception_lock:
                self.extra_perception_state = state
                self.extra_perception_received_monotonic = time.monotonic()
        except Exception as error:
            self.get_logger().warn(f'⚠️ 추가 인식 상태 해석 실패: {error}')

    def get_extra_perception_snapshot(self):
        with self.extra_perception_lock:
            state = (
                None
                if self.extra_perception_state is None
                else json.loads(json.dumps(self.extra_perception_state))
            )
            age = time.monotonic() - self.extra_perception_received_monotonic
        if state is None:
            return {'valid': False, 'reason': '추가 인식 데이터를 받지 못함'}
        state['received_age_sec'] = round(age, 3)
        if age > self.extra_perception_max_age_sec:
            state['valid'] = False
            state['reason'] = f'추가 인식 데이터가 오래됨: {age:.2f}s'
        return state

    def publish_plan_feedback(self, mission_id, revision, task_id, accepted, reason):
        payload = {
            'scope': 'plan',
            'mission_id': mission_id,
            'plan_revision': int(revision),
            'task_id': task_id,
            'agent_id': self.agent_id,
            'status': 'ACCEPTED' if accepted else 'REJECTED',
            'reason': str(reason),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.task_status_pub.publish(msg)
        self.get_logger().info(
            f'🧠 [{self.agent_id}] 협동 계획 '
            f'{payload["status"]}: {task_id} / {reason}'
        )

    def apply_plan_status(self, payload):
        """Workstation의 승인/폐기/실패 결정을 반영합니다."""
        mission_id = str(payload.get('mission_id', '')).strip()
        revision = int(payload.get('plan_revision', -1))
        status = str(payload.get('status', '')).strip().upper()
        if status in ('ACCEPTED', 'REJECTED'):
            return
        with self.guidebook_lock:
            if (
                mission_id != self.current_mission_id
                or revision != self.current_plan_revision
            ):
                return
        with self.step_sync_condition:
            if status == 'APPROVED':
                self.plan_approved_revision = revision
            elif status in ('SUPERSEDED', 'FAILED'):
                self.cooperative_aborted.add((mission_id, revision))
                with self.task_queue_lock:
                    self.task_queue = [
                        task for task in self.task_queue
                        if not (
                            task.get('execution_mode') == 'cooperative'
                            and task.get('mission_id') == mission_id
                            and int(task.get('plan_revision', -1)) == revision
                        )
                    ]
            else:
                return
            self.step_sync_condition.notify_all()
        self.get_logger().info(
            f'📡 [{self.agent_id}] 계획 status={status}: {mission_id}/r{revision}'
        )

    def publish_step_status(self, task, step_index, status, reason=''):
        payload = {
            'scope': 'step',
            'mission_id': str(task['mission_id']),
            'plan_revision': int(task['plan_revision']),
            'task_id': str(task['guidebook_task_id']),
            'attempt': int(task.get('attempt', 0)),
            'step_index': int(step_index),
            'agent_id': self.agent_id,
            'status': str(status).upper(),
            'reason': str(reason),
        }
        # DDS loopback 전달을 기다리지 않아도 로컬 상태가 즉시 보이게 합니다.
        self.apply_step_status(payload)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.task_status_pub.publish(msg)

    @staticmethod
    def _step_key(payload):
        return (
            str(payload.get('mission_id', '')),
            int(payload.get('plan_revision', -1)),
            str(payload.get('task_id', '')),
            int(payload.get('attempt', 0)),
            int(payload.get('step_index', -1)),
        )

    def apply_step_status(self, payload):
        status = str(payload.get('status', '')).strip().upper()
        if status not in ('READY', 'GO', 'DONE', 'FAILED'):
            raise ValueError(f'지원하지 않는 step status: {status}')
        key = self._step_key(payload)
        agent_id = str(payload.get('agent_id', '')).strip()
        with self.guidebook_lock:
            task = self.guidebook_tasks.get(key[2])
            current_match = (
                key[0] == self.current_mission_id
                and key[1] == self.current_plan_revision
            )
        if not current_match or not task or not self.is_cooperative_task(task):
            return
        participants = set(map(str, task.get('participants', [])))
        if status != 'GO' and agent_id not in participants:
            return

        send_go = False
        with self.step_sync_condition:
            state = self.step_sync_state.setdefault(
                key,
                {'READY': set(), 'GO': set(), 'DONE': set(), 'FAILED': set()},
            )
            state[status].add(agent_id)
            if status == 'FAILED':
                self.cooperative_aborted.add((key[0], key[1]))
            if (
                status == 'READY'
                and self.agent_id == str(task.get('coordinator_id', min(participants)))
                and state['READY'] >= participants
                and key not in self.step_go_sent
            ):
                self.step_go_sent.add(key)
                send_go = True
            self.step_sync_condition.notify_all()

        if send_go:
            go_payload = {
                **payload,
                'agent_id': self.agent_id,
                'status': 'GO',
                'start_after_sec': self.step_start_delay_sec,
            }
            # 로컬 executor도 동일 GO를 확인합니다.
            with self.step_sync_condition:
                self.step_sync_state[key]['GO'].add(self.agent_id)
                self.step_sync_condition.notify_all()
            msg = String()
            msg.data = json.dumps(go_payload, ensure_ascii=False)
            self.task_status_pub.publish(msg)

    def wait_for_step_go(self, task, step_index):
        self.publish_step_status(task, step_index, 'READY')
        key = (
            str(task['mission_id']),
            int(task['plan_revision']),
            str(task['guidebook_task_id']),
            int(task.get('attempt', 0)),
            int(step_index),
        )
        deadline = time.monotonic() + self.step_sync_timeout_sec
        with self.step_sync_condition:
            while rclpy.ok():
                if (key[0], key[1]) in self.cooperative_aborted:
                    raise RuntimeError('협동 계획이 중단되었습니다.')
                state = self.step_sync_state.get(key, {})
                if state.get('FAILED'):
                    raise RuntimeError('다른 Agent가 이 협동 단계에서 실패했습니다.')
                if state.get('GO'):
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError(
                        f'협동 단계 동기화 시간 초과: step={step_index}'
                    )
                self.step_sync_condition.wait(timeout=min(0.5, remaining))
        time.sleep(max(0.0, self.step_start_delay_sec))

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
        """배치 좌표를 workspace 기준에서 agent별 xArm base 기준으로 변환합니다.

        기존 zone PnP는 place_pose에 z가 없으므로 예전처럼 workspace z=0으로 x/y만
        변환합니다. relative_object/workspace target처럼 z가 명시된 경우에만 z까지
        보존하여 새 Target Resolver 경로에서 사용합니다.
        """
        has_z = 'z' in place_pose
        workspace_z = float(place_pose.get('z', 0.0))
        x_robot, y_robot, z_robot = self.workspace_point_to_robot(
            place_pose['x'], place_pose['y'], workspace_z, agent_id=agent_id
        )
        yaw_robot = self.workspace_yaw_to_robot(
            place_pose.get('yaw', 0.0), agent_id=agent_id
        )
        result = {
            'x': float(x_robot),
            'y': float(y_robot),
            'yaw': float(yaw_robot),
        }
        if has_z:
            result['z'] = float(z_robot)
        return result

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

    def tf_pose_to_sdk_pose(self, x, y, z, yaw=0.0, agent_id=None):
        """[MERGED] robot_N_base와 xArm SDK 좌표계를 identity로 고정합니다."""
        return {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'yaw': self.normalize_angle_rad(yaw),
            'mode': 'identity',
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

        # 실패 원인을 다음 테스트에서 바로 확인할 수 있도록 controller 상태를 함께 남긴다.
        try:
            state = self.arm.state
        except Exception:
            state = 'unknown'
        try:
            error_code = self.arm.error_code
        except Exception:
            error_code = 'unknown'
        try:
            warn_code = self.arm.warn_code
        except Exception:
            warn_code = 'unknown'

        self.get_logger().error(
            f'🚨 로봇 이동 거절됨! ret={ret}, state={state}, '
            f'error_code={error_code}, warn_code={warn_code}, label={label}'
        )
        return False

    def move_to_robot_tf(self, x, y, z, yaw=0.0, speed=100.0, label=''):
        """robot_N_base TF 좌표를 xArm SDK command 좌표로 변환한 뒤 이동한다."""
        
        # [수정된 부분] agent2일 경우에만 Z축을 40mm(4cm) 더 내리도록 보정합니다.
        if self.agent_id == 'agent2':
            z = float(z) - 50.0

        cmd = self.tf_pose_to_sdk_pose(x, y, z, yaw=yaw, agent_id=self.agent_id)
        self.get_logger().info(
            f'🧭 TF→SDK{f"[{label}]" if label else ""}: '
            f'tf=({float(x):.1f}, {float(y):.1f}, {float(z):.1f}, yaw={np.degrees(float(yaw)):.1f}deg) '
            f'-> sdk=({cmd["x"]:.1f}, {cmd["y"]:.1f}, {cmd["z"]:.1f}, '
            f'yaw={np.degrees(cmd["yaw"]):.1f}deg), mode={cmd["mode"]}'
        )
        return self.move_to_sdk(cmd['x'], cmd['y'], cmd['z'], yaw=cmd['yaw'], speed=speed, label=label)
    
    def return_to_home_joint_pose(self):
        """첨부 이미지의 기본 joint 자세로 복귀합니다."""
        target = list(self.HOME_JOINT_ANGLES_DEG)

        if self.dry_run:
            self.get_logger().warn(
                f'🧪 DRY_RUN return_home_joint: angles={target}, '
                f'speed={self.HOME_JOINT_SPEED_DEG_S:.1f} deg/s'
            )
            return True

        ret = self.arm.set_servo_angle(
            angle=target,
            speed=float(self.HOME_JOINT_SPEED_DEG_S),
            wait=True,
        )
        if ret in (0, None):
            time.sleep(0.5)
            self.get_logger().info(
                f'🏠 [{self.agent_id}] 기본 자세 복귀 완료: {target}'
            )
            return True

        try:
            state = self.arm.state
        except Exception:
            state = 'unknown'
        try:
            error_code = self.arm.error_code
        except Exception:
            error_code = 'unknown'
        try:
            warn_code = self.arm.warn_code
        except Exception:
            warn_code = 'unknown'

        self.get_logger().error(
            f'🚨 기본 자세 복귀 실패! ret={ret}, state={state}, '
            f'error_code={error_code}, warn_code={warn_code}'
        )
        return False

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
        return self.workspace_object_to_robot_object(
            dict(reference_pose), agent_id=assignee_id
        )

    def convert_place_pose(self, reference_pose, assignee_id):
        return self.workspace_place_to_robot_place(
            dict(reference_pose), agent_id=assignee_id
        )

    def zone_uv_to_xy(self, zone, u, v):
        bounds = self.ZONES[zone]
        x_min = bounds['x_min'] + self.ZONE_MARGIN_MM
        x_max = bounds['x_max'] - self.ZONE_MARGIN_MM
        y_min = bounds['y_min'] + self.ZONE_MARGIN_MM
        y_max = bounds['y_max'] - self.ZONE_MARGIN_MM
        return x_min + u * (x_max - x_min), y_min + v * (y_max - y_min)

    def resolve_target_destination(self, destination_spec, poses_dict, assignee_id, reserved_points):
        """Agent LLM의 destination을 기존 reference_place_pose 형식으로 변환합니다.

        지원 형식:
        - zone: 기존 A/B + u/v 호환 경로
        - relative_object: workspace_0 기준 reference object pose + offset_mm
        - workspace: workspace_0 절대 x/y/z/yaw

        반환값의 reference_place_pose는 항상 workspace_0 기준입니다.
        실제 robot base 변환은 기존 convert_place_pose()가 담당합니다.
        """
        if not isinstance(destination_spec, dict):
            raise ValueError('destination_spec은 JSON 객체여야 합니다.')

        destination_type = str(destination_spec.get('type', '')).strip().lower()
        if destination_type not in ('zone', 'relative_object', 'workspace'):
            raise ValueError(
                f'지원하지 않는 destination.type={destination_type!r}. '
                '사용 가능: zone, relative_object, workspace'
            )

        if destination_type == 'zone':
            zone = self.normalize_zone(destination_spec.get('zone', ''))
            if zone is None:
                raise ValueError('zone target의 zone은 A 또는 B여야 합니다.')
            if zone not in self.agent_specs[assignee_id]:
                raise ValueError(f'{assignee_id}가 {zone}구역에 도달할 수 없습니다.')

            try:
                u = float(destination_spec.get('u'))
                v = float(destination_spec.get('v'))
            except (TypeError, ValueError) as error:
                raise ValueError('zone target의 u/v에는 숫자가 필요합니다.') from error

            place_x, place_y = self.select_place_point(
                zone, u, v, reserved_points.get(zone, [])
            )
            return {
                'type': 'zone',
                'label': f'{zone}구역',
                'reference_place_pose': {
                    'x': float(place_x),
                    'y': float(place_y),
                    'yaw': 0.0,
                },
                'reservation_zone': zone,
                'place_u': u,
                'place_v': v,
            }

        if destination_type == 'relative_object':
            reference = self.find_detected_target(
                destination_spec.get('reference', ''), poses_dict
            )
            if reference is None:
                raise ValueError(
                    'relative_object의 reference를 현재 perception에서 찾을 수 없습니다: '
                    f'{destination_spec.get("reference")}'
                )

            offset = destination_spec.get('offset_mm')
            if not isinstance(offset, (list, tuple)) or len(offset) != 3:
                raise ValueError(
                    'relative_object.offset_mm은 [dx, dy, dz] 3개 숫자 배열이어야 합니다.'
                )
            try:
                dx, dy, dz = (float(value) for value in offset)
                yaw_deg = float(destination_spec.get('yaw_deg', 0.0))
            except (TypeError, ValueError) as error:
                raise ValueError(
                    'relative_object offset_mm/yaw_deg에는 숫자가 필요합니다.'
                ) from error

            if not all(math.isfinite(value) for value in (dx, dy, dz, yaw_deg)):
                raise ValueError('relative_object에 유한한 숫자만 사용할 수 있습니다.')

            ref_x, ref_y, ref_z, _ = poses_dict[reference]
            return {
                'type': 'relative_object',
                'label': f'{reference} 기준 상대 위치',
                'reference_place_pose': {
                    'x': float(ref_x) + dx,
                    'y': float(ref_y) + dy,
                    'z': float(ref_z) + dz,
                    'yaw': float(np.radians(yaw_deg)),
                },
                'reference_object': reference,
                'offset_mm': [dx, dy, dz],
                'reservation_zone': None,
            }

        # destination_type == 'workspace'
        try:
            x_mm = float(destination_spec.get('x_mm'))
            y_mm = float(destination_spec.get('y_mm'))
            z_raw = destination_spec.get('z_mm')
            z_mm = None if z_raw is None else float(z_raw)
            yaw_deg = float(destination_spec.get('yaw_deg', 0.0))
        except (TypeError, ValueError) as error:
            raise ValueError(
                'workspace target의 x_mm/y_mm 및 선택적 z_mm/yaw_deg에는 숫자가 필요합니다.'
            ) from error

        values = [x_mm, y_mm, yaw_deg] + ([] if z_mm is None else [z_mm])
        if not all(math.isfinite(value) for value in values):
            raise ValueError('workspace target에 유한한 숫자만 사용할 수 있습니다.')

        reference_place_pose = {
            'x': x_mm,
            'y': y_mm,
            'yaw': float(np.radians(yaw_deg)),
        }
        # z를 사용자가 주지 않았으면 억지로 생성하지 않습니다.
        # executor의 기존 fallback(place.z가 없으면 object.z)을 그대로 사용합니다.
        if z_mm is not None:
            reference_place_pose['z'] = z_mm

        return {
            'type': 'workspace',
            'label': 'workspace 절대 위치',
            'reference_place_pose': reference_place_pose,
            'reservation_zone': None,
        }

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
    def parse_llm_json_object(raw_text):
        """LLM이 JSON 앞뒤에 짧은 문장을 붙인 경우에도 객체 하나를 추출합니다."""
        raw_text = str(raw_text).strip()
        try:
            result = json.loads(raw_text)
        except json.JSONDecodeError:
            start, end = raw_text.find('{'), raw_text.rfind('}')
            if start < 0 or end <= start:
                raise ValueError('LLM 응답에서 JSON 객체를 찾을 수 없습니다.')
            result = json.loads(raw_text[start:end + 1])
        if not isinstance(result, dict):
            raise ValueError('LLM 응답은 JSON 객체여야 합니다.')
        return result

    def request_ollama_generate(self, payload, *, label, timeout):
        """Ollama generate 응답과 thinking을 스트리밍하고 최종 본문을 반환합니다."""
        request_payload = dict(payload)
        request_payload['stream'] = True
        think_enabled = bool(request_payload.get('think', False))
        with self.llm_request_lock:
            with requests.post(
                self.ollama_url,
                json=request_payload,
                timeout=timeout,
                stream=True,
            ) as response:
                response.raise_for_status()
                content, _thinking = consume_ollama_stream(
                    response,
                    endpoint='generate',
                    label=f'{self.agent_id} {label}',
                    show_thinking=think_enabled,
                )
        if not content:
            raise RuntimeError('Ollama가 최종 응답을 반환하지 않았습니다.')
        return content

    def _cooperative_object_pose(self, state, target_name):
        if not state.get('valid'):
            raise ValueError(
                f"유효한 yolo_extra_perception 상태가 없습니다: {state.get('reason', '')}"
            )
        if str(state.get('frame_id', '')) != self.workspace_frame:
            raise ValueError(
                f'협동 좌표계는 {self.workspace_frame}이어야 합니다: '
                f"{state.get('frame_id')}"
            )
        raw = state.get('objects', {}).get(target_name)
        if not isinstance(raw, dict):
            raise ValueError(f'손잡이 target을 찾을 수 없습니다: {target_name}')
        return {
            'x': float(raw['x_mm']),
            'y': float(raw['y_mm']),
            'z': float(raw['z_mm']),
            'yaw': float(np.radians(float(raw.get('yaw_deg', 0.0)))),
        }

    def _hard_validate_cooperative_task(self, guidebook, task, state):
        """LLM 판단 전에 TF와 안전 상자로 협동 계획을 결정론적으로 검증합니다."""
        participants = [str(value) for value in task.get('participants', [])]
        if len(participants) != 2 or len(set(participants)) != 2:
            raise ValueError('협동 participants는 서로 다른 Agent 2개여야 합니다.')
        if self.agent_id not in participants:
            raise ValueError(f'{self.agent_id}가 협동 participants에 없습니다.')
        targets = task.get('targets_by_agent')
        if not isinstance(targets, dict) or set(targets) != set(participants):
            raise ValueError('targets_by_agent가 participants와 일치하지 않습니다.')
        target_name = str(targets[self.agent_id]).strip()
        if len(set(map(str, targets.values()))) != 2:
            raise ValueError('두 Agent는 서로 다른 손잡이에 할당되어야 합니다.')
        actions = validate_cooperative_actions(task.get('actions'))
        object_pose = self._cooperative_object_pose(state, target_name)

        reach = (
            state.get('reachability', {})
            .get(self.agent_id, {})
            .get('handles', {})
            .get(target_name, {})
        )
        if not reach.get('within_safety_box', False):
            raise ValueError(f'{self.agent_id}가 {target_name}에 안전하게 접근할 수 없습니다.')

        current_workspace = np.array(
            [object_pose['x'], object_pose['y'], object_pose['z']], dtype=float
        )
        grasped = False
        for index, action in enumerate(actions):
            api = action['api']
            if api == 'move_to_object':
                candidate = np.array([
                    object_pose['x'],
                    object_pose['y'],
                    object_pose['z'] + action['z_offset'],
                ])
                current_workspace = candidate
            elif api == 'control_gripper' and action['position'] <= 450:
                grasped = True
                continue
            elif api == 'cooperative_move_relative':
                if not grasped:
                    raise ValueError('상대 협동 이동 전에 파지가 필요합니다.')
                current_workspace = current_workspace + np.array([
                    action['x_offset'], action['y_offset'], action['z_offset']
                ])
            else:
                continue
            bx, by, bz = self.workspace_point_to_robot(*current_workspace)
            robot_yaw = self.workspace_yaw_to_robot(object_pose['yaw'])
            command = self.tf_pose_to_sdk_pose(bx, by, bz, robot_yaw)
            self.check_sdk_pose_or_raise(
                command['x'], command['y'], command['z'],
                label=f'cooperative precheck action {index}',
            )

        normalized = dict(task)
        normalized['participants'] = participants
        normalized['targets_by_agent'] = {
            agent_id: str(targets[agent_id]).strip() for agent_id in participants
        }
        normalized['coordinator_id'] = str(
            task.get('coordinator_id', min(participants))
        )
        normalized['actions'] = actions
        return normalized, target_name, object_pose

    def _ask_llm_for_cooperative_review(self, guidebook, task, state, object_pose):
        template = Template(
            self.cooperative_review_prompt_path.read_text(encoding='utf-8')
        )
        prompt = template.substitute(
            local_agent_id=self.agent_id,
            mission_id=str(guidebook.get('mission_id', '')),
            plan_revision=str(guidebook.get('plan_revision', 0)),
            guidebook_json=json.dumps(guidebook, ensure_ascii=False),
            task_json=json.dumps(task, ensure_ascii=False),
            perception_json=json.dumps(state, ensure_ascii=False),
            local_target_json=json.dumps(object_pose, ensure_ascii=False),
            safety_box_json=json.dumps({
                'x_mm': [self.sdk_x_min, self.sdk_x_max],
                'y_mm': [self.sdk_y_min, self.sdk_y_max],
                'z_mm': [self.sdk_z_min, self.sdk_z_max],
            }, ensure_ascii=False),
        )
        payload = {
            'model': self.llm_model,
            'prompt': prompt,
            'format': 'json',
            'think': bool(self.get_parameter('llm_think').value),
            'options': {'temperature': 0.0, 'num_predict': 512},
        }
        raw = self.request_ollama_generate(
            payload,
            label='협동 계획 검토',
            timeout=self.cooperative_review_timeout_sec,
        )
        result = self.parse_llm_json_object(raw)
        if not isinstance(result.get('accept'), bool):
            raise ValueError('협동 검토 응답은 accept(boolean)를 포함해야 합니다.')
        reason = str(result.get('reason', '')).strip()
        if not reason:
            raise ValueError('협동 검토 응답의 reason이 비어 있습니다.')
        return bool(result['accept']), reason

    def review_cooperative_task(self, mission_id, revision, task_id):
        """[MERGED] 중앙 계획을 이 Agent의 LLM이 승인하거나 거부합니다."""
        key = (mission_id, revision, task_id)
        accepted = False
        reason = ''
        try:
            with self.guidebook_lock:
                if (
                    mission_id != self.current_mission_id
                    or revision != self.current_plan_revision
                ):
                    return
                guidebook = dict(self.current_guidebook)
                task = dict(self.guidebook_tasks[task_id])
            state = self.get_extra_perception_snapshot()
            task, _, object_pose = self._hard_validate_cooperative_task(
                guidebook, task, state
            )
            accepted, reason = self._ask_llm_for_cooperative_review(
                guidebook, task, state, object_pose
            )
        except Exception as error:
            reason = f'코드/LLM 검토 실패: {error}'

        with self.guidebook_lock:
            still_current = (
                mission_id == self.current_mission_id
                and revision == self.current_plan_revision
            )
        with self.step_sync_condition:
            self.cooperative_review_inflight.discard(key)
            if still_current:
                self.cooperative_reviewed.add(key)
        if still_current:
            self.publish_plan_feedback(
                mission_id, revision, task_id, accepted, reason
            )

    def build_cooperative_execution_task(self, guidebook_task):
        """승인된 중앙 계획을 양 Agent가 공유하는 실행 메시지로 고정합니다."""
        state = self.get_extra_perception_snapshot()
        task, _, _ = self._hard_validate_cooperative_task(
            self.current_guidebook,
            guidebook_task,
            state,
        )
        poses = {
            agent_id: self._cooperative_object_pose(state, target_name)
            for agent_id, target_name in task['targets_by_agent'].items()
        }
        return {
            'execution_mode': 'cooperative',
            'mission_id': self.current_mission_id,
            'plan_revision': self.current_plan_revision,
            'guidebook_task_id': str(task['task_id']),
            'task_id': (
                f'{self.current_mission_id}_r{self.current_plan_revision}_'
                f'{task["task_id"]}'
            ),
            'coordinator_id': str(task['coordinator_id']),
            'participants': list(task['participants']),
            'targets_by_agent': dict(task['targets_by_agent']),
            'target_poses_workspace': poses,
            'actions': list(task['actions']),
            'attempt': 0,
            'description': str(task.get('description', '')),
        }

    def dispatch_ready_cooperative_task(self, task_id):
        key = (self.current_mission_id, self.current_plan_revision, task_id)
        with self.step_sync_condition:
            if key in self.cooperative_dispatched:
                return
            self.cooperative_dispatched.add(key)
        try:
            with self.guidebook_lock:
                task = dict(self.guidebook_tasks[task_id])
                if self.guidebook_task_status.get(task_id) != 'READY':
                    return
            execution_task = self.build_cooperative_execution_task(task)
            self.publish_task_status(task_id, 'CLAIMED')
            msg = String()
            msg.data = json.dumps(execution_task, ensure_ascii=False)
            self.task_pub.publish(msg)
            self.get_logger().info(
                f'➡️ [{self.agent_id}] 협동 /agent_task 발행: {task_id}'
            )
        except Exception as error:
            self.publish_task_status(task_id, 'FAILED')
            self.get_logger().error(f'❌ 협동 Task 준비 실패: {task_id}: {error}')


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
        zone_frame_text = f'{self.workspace_frame} 기준'
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
            'think': bool(self.get_parameter('llm_think').value),
            'options': {'temperature': 0.0, 'num_predict': 2048},
        }
        task_id = str(task.get('task_id', ''))
        self.get_logger().info(
            f'🧠 [{self.agent_id}] Guidebook {task_id} 정책 후보 생성 중...'
        )
        try:
            raw = self.request_ollama_generate(
                payload,
                label=f'Guidebook {task_id} 정책',
                timeout=300.0,
            )
            result = self.parse_llm_json_object(raw)
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
        """Guidebook 정책 후보를 검증하고 기존 build_tasks() 입력 형태로 감쌉니다.

        Target Resolver가 zone / relative_object / workspace를 공통
        reference_place_pose로 변환하므로, executor와 TF 코드는 그대로 재사용합니다.
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
        if destination_type not in ('zone', 'relative_object', 'workspace'):
            raise ValueError(
                "destination.type은 'zone', 'relative_object', 'workspace' 중 하나여야 합니다."
            )

        normalized_destination = dict(destination)
        normalized_destination['type'] = destination_type

        # Agent가 claim하기 전에 명백한 입력 오류/인지 부족은 여기서 먼저 차단합니다.
        if destination_type == 'zone':
            zone = self.normalize_zone(destination.get('zone', ''))
            if zone is None:
                raise ValueError('destination.zone은 A 또는 B여야 합니다.')
            if zone not in self.agent_specs[self.agent_id]:
                return None, f'{self.agent_id}가 {zone}구역에 도달할 수 없습니다.'
            try:
                normalized_destination['u'] = float(destination.get('u'))
                normalized_destination['v'] = float(destination.get('v'))
            except (TypeError, ValueError) as error:
                raise ValueError('destination.u/v에는 숫자가 필요합니다.') from error

        elif destination_type == 'relative_object':
            reference = self.find_detected_target(
                destination.get('reference', ''), poses_dict
            )
            if reference is None:
                return None, (
                    'relative_object 기준 물체를 현재 perception에서 찾을 수 없습니다: '
                    f'{destination.get("reference")}'
                )
            offset = destination.get('offset_mm')
            if not isinstance(offset, (list, tuple)) or len(offset) != 3:
                raise ValueError(
                    'relative_object.offset_mm은 [dx, dy, dz] 3개 숫자 배열이어야 합니다.'
                )
            try:
                normalized_destination['reference'] = reference
                normalized_destination['offset_mm'] = [
                    float(value) for value in offset
                ]
                normalized_destination['yaw_deg'] = float(
                    destination.get('yaw_deg', 0.0)
                )
            except (TypeError, ValueError) as error:
                raise ValueError(
                    'relative_object offset_mm/yaw_deg에는 숫자가 필요합니다.'
                ) from error

        else:  # workspace
            try:
                normalized_destination['x_mm'] = float(destination.get('x_mm'))
                normalized_destination['y_mm'] = float(destination.get('y_mm'))
                if destination.get('z_mm') is not None:
                    normalized_destination['z_mm'] = float(destination.get('z_mm'))
                normalized_destination['yaw_deg'] = float(
                    destination.get('yaw_deg', 0.0)
                )
            except (TypeError, ValueError) as error:
                raise ValueError(
                    'workspace x_mm/y_mm 및 선택적 z_mm/yaw_deg에는 숫자가 필요합니다.'
                ) from error

        actions = self.validate_actions(result.get('actions'))

        policy = {
            'generated_policy': [
                {
                    'agent_id': self.agent_id,
                    'target': target,
                    'destination_spec': normalized_destination,
                    'actions': actions,
                }
            ]
        }
        return policy, ''

    def poll_ready_guidebook_tasks(self):
        """READY Task 중 아직 후보를 만들지 않은 하나를 비동기로 계획합니다."""
        if not self.guidebook_policy_enabled:
            return

        with self.guidebook_lock:
            mission_id = self.current_mission_id
            revision = self.current_plan_revision
            requires_plan_review = bool(
                self.current_guidebook.get('requires_plan_review')
            )
            if not mission_id:
                return
            ready_ids = [
                task_id
                for task_id, status in self.guidebook_task_status.items()
                if status == 'READY'
            ]

        if not ready_ids:
            return

        # 협동 Task가 하나라도 포함된 mission은 두 Agent가 전체 계획을 승인한 뒤에만
        # 일반 Task를 포함한 어떤 물리 이동도 시작합니다.
        if (
            requires_plan_review
            and self.plan_approved_revision != revision
        ):
            return

        # [MERGED] 협동 계획은 Workstation이 이미 actions까지 완성했습니다.
        # 두 Agent 승인 후 coordinator만 공통 실행 메시지를 발행합니다.
        for task_id in ready_ids:
            with self.guidebook_lock:
                task = dict(self.guidebook_tasks[task_id])
            if not self.is_cooperative_task(task):
                continue
            if (
                self.plan_approved_revision == revision
                and self.agent_id == str(task.get('coordinator_id', ''))
            ):
                self.dispatch_ready_cooperative_task(task_id)

        if not self.latest_poses:
            return

        now = time.time()
        selected = None
        with self.guidebook_policy_lock:
            for task_id in ready_ids:
                with self.guidebook_lock:
                    if self.is_cooperative_task(self.guidebook_tasks[task_id]):
                        continue
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

            normal_policy, reason = self.validate_guidebook_policy_candidate(
                result,
                task,
                poses_dict,
            )
            if normal_policy is None:
                # 동일 mission/task에서 실행 불가 판정을 매 poll마다 다시 LLM에 묻지 않습니다.
                with self.guidebook_policy_lock:
                    self.guidebook_policy_candidates[task_id] = None
                self.get_logger().info(
                    f'ℹ️ [{self.agent_id}] {task_id} 후보 제외: {reason}'
                )
                return

            # 핵심: 새 Guidebook 정책을 기존 build_tasks()에 맞춰 변환합니다.
            # TF / zone 계산 / actions 검증 / 기존 task 구조를 그대로 재사용합니다.
            built = self.build_tasks(normal_policy, poses_dict)
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

    def build_tasks(self, policy_result, poses_dict):
        raw_tasks = policy_result.get('generated_policy', [])
        if not isinstance(raw_tasks, list) or not raw_tasks:
            raise ValueError('generated_policy는 비어 있지 않은 배열이어야 합니다.')

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
            if assignee not in self.agent_specs:
                raise ValueError(f'정책 작업 #{order}: 사용할 수 없는 Agent {assignee}')

            target = self.find_detected_target(raw.get('target', ''), poses_dict)
            if target is None or target in used_targets:
                raise ValueError(
                    f'정책 작업 #{order}: 인식되지 않았거나 중복된 target '
                    f'{raw.get("target")}'
                )

            destination_spec = raw.get('destination_spec')
            if not isinstance(destination_spec, dict):
                raise ValueError(
                    f'정책 작업 #{order}: destination_spec JSON 객체가 필요합니다.'
                )

            resolved = self.resolve_target_destination(
                destination_spec,
                poses_dict,
                assignee,
                reserved,
            )

            raw_actions = raw.get('actions', self.DEFAULT_PNP_ACTIONS)
            actions = self.validate_actions(raw_actions)
            rx, ry, rz, yaw = poses_dict[target]

            ref_object = {
                'x': float(rx), 'y': float(ry),
                'z': float(rz), 'yaw': float(yaw),
            }
            ref_place = dict(resolved['reference_place_pose'])
            robot_place = self.convert_place_pose(ref_place, assignee)

            base_frame = self.get_agent_base_frame(assignee)
            z_text = f", z={ref_place['z']:.1f}" if 'z' in ref_place else ''
            robot_z_text = f", z={robot_place['z']:.1f}" if 'z' in robot_place else ''
            self.get_logger().info(
                f"🎯 Target Resolver: {assignee}/{resolved['type']} "
                f"workspace=({ref_place['x']:.1f}, {ref_place['y']:.1f}{z_text}, "
                f"yaw={np.degrees(ref_place['yaw']):.1f}deg) -> "
                f"{base_frame}=({robot_place['x']:.1f}, {robot_place['y']:.1f}"
                f"{robot_z_text}, yaw={np.degrees(robot_place['yaw']):.1f}deg)"
            )

            task_data = {
                'task_id': f'{plan_stamp}_{order:02d}',
                'order': order,
                'coordinator_id': self.agent_id,
                'assignee_id': assignee,
                'target': target,
                'destination': resolved['label'],
                'destination_type': resolved['type'],
                'destination_spec': dict(destination_spec),
                'reference_object_pose': ref_object,
                'reference_place_pose': ref_place,
                'object_pose': self.convert_object_pose(ref_object, assignee),
                'place_pose': robot_place,
                'actions': actions,
            }
            if resolved['type'] == 'zone':
                task_data['place_u'] = float(resolved['place_u'])
                task_data['place_v'] = float(resolved['place_v'])

            tasks.append(task_data)
            used_targets.add(target)

            reservation_zone = resolved.get('reservation_zone')
            if reservation_zone is not None:
                reserved[reservation_zone].append(
                    (float(ref_place['x']), float(ref_place['y']))
                )

        return tasks

    def validate_received_task(self, task):
        execution_mode = str(task.get('execution_mode', 'single_agent')).strip()
        if execution_mode == 'cooperative':
            required = (
                'mission_id', 'plan_revision', 'guidebook_task_id', 'task_id',
                'coordinator_id', 'participants', 'targets_by_agent',
                'target_poses_workspace', 'actions',
            )
            if any(key not in task for key in required):
                raise ValueError('협동 작업 메시지에 필수 필드가 없습니다.')
            participants = [str(value) for value in task['participants']]
            if self.agent_id not in participants:
                return False
            with self.guidebook_lock:
                if str(task['mission_id']) != self.current_mission_id:
                    return False
                if int(task['plan_revision']) != self.current_plan_revision:
                    return False
                plan_task = self.guidebook_tasks.get(str(task['guidebook_task_id']))
            if not plan_task or not self.is_cooperative_task(plan_task):
                raise ValueError('현재 guidebook과 일치하는 협동 Task가 없습니다.')
            if set(participants) != set(map(str, plan_task.get('participants', []))):
                raise ValueError('실행 메시지 participants가 승인 계획과 다릅니다.')
            if task['targets_by_agent'] != plan_task.get('targets_by_agent'):
                raise ValueError('실행 메시지 손잡이 할당이 승인 계획과 다릅니다.')
            expected_actions = validate_cooperative_actions(plan_task.get('actions'))
            actual_actions = validate_cooperative_actions(task.get('actions'))
            if actual_actions != expected_actions:
                raise ValueError('실행 메시지 actions가 승인 계획과 다릅니다.')
            poses = task['target_poses_workspace']
            if not isinstance(poses, dict) or set(poses) != set(participants):
                raise ValueError('협동 target_poses_workspace가 올바르지 않습니다.')
            for pose in poses.values():
                for key in ('x', 'y', 'z', 'yaw'):
                    if not math.isfinite(float(pose[key])):
                        raise ValueError('협동 손잡이 pose에 유한한 숫자가 필요합니다.')
            task['actions'] = actual_actions
            task['participants'] = participants
            return True

        required = (
            'task_id', 'assignee_id', 'target', 'destination', 'reference_object_pose',
            'reference_place_pose', 'object_pose', 'place_pose', 'actions'
        )
        if any(key not in task for key in required):
            raise ValueError('작업 메시지에 필수 필드가 없습니다.')
        if str(task['assignee_id']) != self.agent_id:
            return False

        destination_type = str(task.get('destination_type', 'zone')).strip().lower()
        if destination_type == 'zone':
            zone = self.normalize_zone(task['destination'])
            if zone not in self.agent_specs[self.agent_id]:
                raise ValueError(f'{self.agent_id}가 도달할 수 없는 목적지입니다.')
        elif destination_type not in ('relative_object', 'workspace'):
            raise ValueError(f'지원하지 않는 destination_type={destination_type!r}')

        for key in ('x', 'y', 'z', 'yaw'):
            float(task['object_pose'][key])
        for key in ('x', 'y', 'yaw'):
            float(task['place_pose'][key])
        if 'z' in task['place_pose']:
            float(task['place_pose']['z'])
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
            cooperative = task.get('execution_mode') == 'cooperative'
            coordinator = str(task.get('coordinator_id', '')) == self.agent_id
            if guidebook_task_id and (not cooperative or coordinator):
                self.publish_task_status(guidebook_task_id, 'EXECUTING')

            success = (
                self.execute_cooperative_task(task)
                if cooperative
                else self.execute_task(task)
            )

            if guidebook_task_id and (not cooperative or coordinator or not success):
                self.publish_task_status(
                    guidebook_task_id,
                    'SUCCEEDED' if success else 'FAILED'
                )

            if not success:
                self.handle_task_failure(task)
                return

    # ==================================================================
    # A+B 공용 구역 토큰 클라이언트 (zone_token_manager 노드와 토픽 통신)
    # ==================================================================
    def zone_token_state_callback(self, msg):
        """zone_token_manager가 latch로 발행하는 현재 소유 상태를 반영합니다."""
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict):
                return
        except (ValueError, json.JSONDecodeError):
            return
        with self._zone_cond:
            self._zone_state = state
            if self._zone_token_held and state.get('holder') != self.agent_id:
                self.get_logger().warn(
                    f'⚠️ [{self.agent_id}] 공용 구역 토큰을 잃었습니다 '
                    f'(holder={state.get("holder")}). lease 만료/매니저 재시작 가능성.'
                )
                self._zone_token_held = False
                self._zone_token_epoch = None
            self._zone_cond.notify_all()

    def _publish_zone_request(self, action, request_id=None, epoch=None):
        payload = {'agent_id': self.agent_id, 'action': str(action)}
        if request_id is not None:
            payload['request_id'] = str(request_id)
        if epoch is not None:
            payload['epoch'] = int(epoch)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        try:
            self.zone_token_pub.publish(msg)
        except Exception as error:  # 노드 종료 중 publish 실패가 finally 반환값을 가리지 않도록
            self.get_logger().warn(f'⚠️ zone_token 요청 발행 실패({action}): {error}')

    def _shared_zone_box(self):
        return expand_aabb(self.shared_zone_box, self.shared_zone_margin_mm)

    def task_shared_zone_plan(self, task):
        """workspace_0 좌표로 이 Task가 공용 구역에 들어가는지 판정합니다.

        - pick 지점이 공용 구역 안 → 첫 move_to_object 전에 토큰
        - place 지점이 공용 구역 안, 또는 pick→place 직선이 공용 구역을 통과
          → 첫 move_to_place 전에 토큰
        - 좌표를 못 구하면 보수적으로 토큰을 요구
        """
        if not self.zone_token_enabled:
            return {'needed': False, 'acquire_before_api': None, 'reason': 'disabled'}

        box = self._shared_zone_box()
        obj = task.get('reference_object_pose') or {}
        place = task.get('reference_place_pose') or {}
        try:
            ox, oy = float(obj['x']), float(obj['y'])
            px, py = float(place['x']), float(place['y'])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(
                f'⚠️ [{task.get("task_id")}] workspace 좌표 확인 불가 '
                '→ 공용 구역 토큰을 보수적으로 요구합니다.'
            )
            return {
                'needed': True,
                'acquire_before_api': 'move_to_object',
                'reason': 'workspace 좌표 불명',
            }

        acquire_before_api, reason = classify_shared_zone_entry(ox, oy, px, py, box)
        return {
            'needed': acquire_before_api is not None,
            'acquire_before_api': acquire_before_api,
            'reason': reason,
        }

    def acquire_zone_token(self, task_id, timeout_sec=None):
        """공용 구역 토큰을 획득할 때까지(또는 timeout까지) 이 워커 스레드를 블록합니다."""
        if not self.zone_token_enabled:
            return True
        if self._zone_token_held:
            return True

        if timeout_sec is None:
            timeout_sec = self.zone_token_acquire_timeout_sec
        req_id = uuid.uuid4().hex[:8]
        self.get_logger().info(
            f'🎟️ [{task_id}] 공용 구역 토큰 요청 (req={req_id})'
        )
        deadline = time.monotonic() + float(timeout_sec)
        last_pub = 0.0
        with self._zone_cond:
            while rclpy.ok():
                now = time.monotonic()
                if now - last_pub > 2.0:
                    self._publish_zone_request('acquire', request_id=req_id)
                    last_pub = now
                if self._zone_state.get('holder') == self.agent_id:
                    self._zone_token_held = True
                    self._zone_token_epoch = self._zone_state.get('epoch')
                    self.get_logger().info(
                        f'✅ [{task_id}] 공용 구역 토큰 획득 '
                        f'(epoch={self._zone_token_epoch})'
                    )
                    return True
                remaining = deadline - now
                if remaining <= 0.0:
                    self.get_logger().error(
                        f'⏰ [{task_id}] 공용 구역 토큰 획득 timeout '
                        f'({float(timeout_sec):.0f}s). '
                        f'holder={self._zone_state.get("holder")}, '
                        f'queue={self._zone_state.get("queue")} '
                        '(zone_token_manager 노드가 실행 중인지 확인)'
                    )
                    # P5: 대기열에 남은 유령 waiter가 되지 않도록 명시적으로 취소합니다.
                    self._publish_zone_request('abandon', request_id=req_id)
                    return False
                self._zone_cond.wait(timeout=min(1.0, remaining))
        # rclpy 종료 등으로 루프를 빠져나온 경우에도 대기열을 정리합니다.
        self._publish_zone_request('abandon', request_id=req_id)
        return False

    def release_zone_token(self, task_id):
        if not self.zone_token_enabled:
            return
        with self._zone_cond:
            epoch = self._zone_token_epoch
            self._zone_token_held = False
            self._zone_token_epoch = None
        self._publish_zone_request('release', epoch=epoch)
        self.get_logger().info(
            f'🟢 [{task_id}] 공용 구역 토큰 반납 (epoch={epoch})'
        )

    def _zone_token_heartbeat(self):
        """토큰 보유 중에는 주기적으로 acquire를 재발행해 lease를 갱신합니다."""
        if self._zone_token_held:
            self._publish_zone_request('acquire', request_id='hb')

    def execute_task(self, task):
        obj = task['object_pose']
        place = task['place_pose']
        # 기존 zone Task에는 place.z가 없으므로 obj.z를 그대로 사용합니다.
        # relative_object/workspace Target Resolver Task에만 place.z가 들어옵니다.
        place_base_z = float(place.get('z', obj['z']))
        self.get_logger().info(
            f"🦾 [{task['task_id']}] [{task['target']}] -> {task['destination']} 시작 | "
            f"obj=({float(obj['x']):.1f}, {float(obj['y']):.1f}, {float(obj['z']):.1f}) | "
            f"place=({float(place['x']):.1f}, {float(place['y']):.1f}, "
            f"{place_base_z:.1f}, yaw={np.degrees(float(place['yaw'])):.1f}deg)"
        )

        # 공용 구역(A+B) 진입 여부를 workspace_0 좌표로 결정론적으로 판정합니다.
        zone_plan = self.task_shared_zone_plan(task)
        self.get_logger().info(
            f"🚧 [{task['task_id']}] 공용 구역 판정: needed={zone_plan['needed']} "
            f"({zone_plan['reason']})"
            + (
                f", 토큰 확보 시점=첫 {zone_plan['acquire_before_api']}"
                if zone_plan['needed'] else ''
            )
        )
        acquired_here = False
        try:
            self.arm.clean_error()
            self.arm.set_state(0)
            for index, action in enumerate(task['actions'], start=1):
                api = action['api']

                # 공용 구역에 처음 들어가는 모션 직전에 토큰을 확보합니다.
                if (
                    zone_plan['needed']
                    and not acquired_here
                    and api == zone_plan['acquire_before_api']
                ):
                    if not self.acquire_zone_token(task['task_id']):
                        return False
                    acquired_here = True

                # P5: 확보했던 토큰을 모션 도중 상실했으면(매니저 재시작 등)
                # 공용 구역으로 더 움직이지 않고 즉시 중단합니다.
                if (
                    acquired_here
                    and not self._zone_token_held
                    and api in ('move_to_object', 'move_to_place')
                ):
                    self.get_logger().error(
                        f"🛑 [{task['task_id']}] 공용 구역 토큰을 상실한 상태에서 "
                        "모션을 시도하여 작업을 중단합니다. 두 로봇 위치를 확인하세요."
                    )
                    return False

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
                        place['x'], place['y'], place_base_z + action['z_offset'],
                        yaw=place['yaw'], speed=action['speed'],
                        label=f"{task['task_id']} place"
                    )
                elif api == 'wait':
                    time.sleep(action['seconds'])
                    success = True
                elif api == 'return_home':
                    success = self.return_to_home_joint_pose()
                elif api in ('request_token', 'release_token'):
                    # 구버전 계획 호환: 공용 구역 잠금은 이제 기하 게이트가 자동 처리합니다.
                    self.get_logger().debug(
                        f"ℹ️ [{task['task_id']}] {api} 무시 (기하 게이트 사용)"
                    )
                    success = True
                else:
                    success = False

                if not success:
                    return False

            # 일반 PnP가 끝나면 두 Agent 모두 같은 기본 자세로 복귀합니다.
            # 복귀가 성공해야 Guidebook Task를 SUCCEEDED로 처리합니다.
            self.get_logger().info(
                f"🏠 [{task['task_id']}] 작업 완료 후 기본 자세로 복귀"
            )
            if not self.return_to_home_joint_pose():
                return False

            zone = self.normalize_zone(task['destination'])
            reference_place = task['reference_place_pose']
            if zone in self.placed_points:
                with self.placed_points_lock:
                    self.placed_points[zone].append((
                        float(reference_place['x']), float(reference_place['y'])
                    ))
            self.get_logger().info(f"✅ [{task['task_id']}] 작업 완료")
            return True

        except Exception as error:
            self.get_logger().error(f"🚨 [{task['task_id']}] 작업 오류: {error}")
            return False
        finally:
            # 성공/실패/예외 어느 경로로 끝나든 확보한 토큰은 반드시 반납합니다.
            if acquired_here:
                self.release_zone_token(task['task_id'])

    def wait_for_plan_approval(self, task):
        mission_id = str(task['mission_id'])
        revision = int(task['plan_revision'])
        deadline = time.monotonic() + self.step_sync_timeout_sec
        with self.step_sync_condition:
            while rclpy.ok():
                if (mission_id, revision) in self.cooperative_aborted:
                    raise RuntimeError('Workstation이 협동 계획을 폐기/실패 처리했습니다.')
                if self.plan_approved_revision == revision:
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError('Workstation 계획 승인 대기 시간 초과')
                self.step_sync_condition.wait(timeout=min(0.5, remaining))

    def execute_cooperative_task(self, task):
        """[MERGED] 동일 action index마다 DDS barrier를 통과한 뒤 양팔을 이동합니다.

        공용 구역 토큰 규칙(P3):
        - coordinator만 협동 작업 전체 구간 토큰을 확보/반납한다 (belt-and-suspenders).
        - 비-coordinator participant는 토큰을 절대 요청하지 않는다. 양쪽이 단일
          토큰을 잡으면 wait_for_step_go barrier에서 상호 대기 → 데드락이 된다.
        - 같은 Agent에서 단일 task와 협동 task는 직렬 큐(process_task_queue) 덕분에
          동시에 실행되지 않는다.
        """
        task_id = str(task['task_id'])
        is_coordinator = str(task.get('coordinator_id', '')) == self.agent_id
        local_pose = dict(task['target_poses_workspace'][self.agent_id])
        workspace_position = np.array([
            float(local_pose['x']), float(local_pose['y']), float(local_pose['z'])
        ])
        workspace_yaw = float(local_pose['yaw'])

        # 불변식: 협동 진입 시 단일 task 토큰이 남아 있으면 안 된다. 방어적으로 반납.
        if self._zone_token_held:
            self.get_logger().error(
                f'🛑 [{task_id}] 불변식 위반: 협동 진입 시 이미 공용 구역 토큰 보유 중. '
                '방어적으로 반납합니다.'
            )
            self.release_zone_token(task_id)

        coop_token_held = False
        try:
            self.wait_for_plan_approval(task)

            if is_coordinator and self.zone_token_enabled:
                coop_timeout = min(
                    self.zone_token_acquire_timeout_sec, self.step_sync_timeout_sec
                )
                self.get_logger().info(
                    f'🎟️ [{task_id}] coordinator가 협동 구간 공용 구역 토큰 확보 시도'
                )
                if not self.acquire_zone_token(task_id, timeout_sec=coop_timeout):
                    self.publish_step_status(
                        task, -1, 'FAILED', reason='공용 구역 토큰 확보 실패'
                    )
                    return False
                coop_token_held = True

            self.arm.clean_error()
            self.arm.set_state(0)

            for index, action in enumerate(task['actions']):
                self.wait_for_step_go(task, index)
                api = action['api']
                self.get_logger().info(
                    f'🤝 [{task_id}] synchronized action {index}: {action}'
                )

                # P5: coordinator가 협동 구간 토큰을 상실했으면 즉시 중단합니다.
                if (
                    coop_token_held
                    and not self._zone_token_held
                    and api in ('move_to_object', 'cooperative_move_relative')
                ):
                    self.get_logger().error(
                        f'🛑 [{task_id}] 협동 구간 공용 구역 토큰 상실 → 작업 중단'
                    )
                    self.publish_step_status(
                        task, index, 'FAILED', reason='공용 구역 토큰 상실'
                    )
                    return False

                if api == 'control_gripper':
                    success = self.control_gripper(action['position'])
                elif api == 'move_to_object':
                    workspace_position = np.array([
                        float(local_pose['x']),
                        float(local_pose['y']),
                        float(local_pose['z']) + float(action['z_offset']),
                    ])
                    bx, by, bz = self.workspace_point_to_robot(*workspace_position)
                    robot_yaw = self.workspace_yaw_to_robot(workspace_yaw)
                    success = self.move_to_robot_tf(
                        bx, by, bz,
                        yaw=robot_yaw,
                        speed=action['speed'],
                        label=f'{task_id} cooperative handle',
                    )
                elif api == 'cooperative_move_relative':
                    workspace_position = workspace_position + np.array([
                        float(action['x_offset']),
                        float(action['y_offset']),
                        float(action['z_offset']),
                    ])
                    bx, by, bz = self.workspace_point_to_robot(*workspace_position)
                    robot_yaw = self.workspace_yaw_to_robot(workspace_yaw)
                    success = self.move_to_robot_tf(
                        bx, by, bz,
                        yaw=robot_yaw,
                        speed=action['speed'],
                        label=f'{task_id} cooperative relative',
                    )
                elif api == 'wait':
                    time.sleep(float(action['seconds']))
                    success = True
                elif api == 'return_home':
                    success = self.return_to_home_joint_pose()
                else:
                    success = False
                if not success:
                    self.publish_step_status(
                        task, index, 'FAILED', reason=f'{api} 실행 실패'
                    )
                    return False
                self.publish_step_status(task, index, 'DONE')

            # 마지막 action 완료까지 양쪽이 도착했음을 확인한 뒤 Task를 종료합니다.
            final_index = len(task['actions'])
            self.wait_for_step_go(task, final_index)
            self.publish_step_status(task, final_index, 'DONE')
            self.get_logger().info(f'✅ [{task_id}] 양팔 협동 작업 완료')
            return True
        except Exception as error:
            self.get_logger().error(f'🚨 [{task_id}] 양팔 협동 작업 오류: {error}')
            self.publish_step_status(task, -1, 'FAILED', reason=str(error))
            return False
        finally:
            # coordinator가 확보한 협동 구간 토큰을 모든 종료 경로에서 반납합니다.
            if coop_token_held:
                self.release_zone_token(task_id)

    def handle_task_failure(self, failed_task):
        """[MERGED] 실패 시 큐를 정지하고 자동 이동/재할당은 수행하지 않습니다."""
        self.get_logger().error(f"🚨 [{failed_task['task_id']}] 작업 실패")
        with self.task_queue_lock:
            self.task_queue.clear()
            self.task_worker_running = False
        self.is_moving = False
        self.get_logger().warn(
            '🛑 자동 복구 이동/자동 재할당을 하지 않습니다. '
            '두 로봇의 상태와 파지 상태를 확인하세요.'
        )

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
        name_map_ko = {
            'banana': '바나나', 'apple': '사과', 'orange': '오렌지',
            'mouse': '마우스', 'bread': '빵', 'lettuce': '양상추',
        }
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
                # 앞 모델의 시각화가 다음 모델의 추론 입력에 섞이지 않게 합니다.
                inference_image = img.copy()
                current_poses, current_items = {}, []

                for model_filename, model in self.models:
                    wanted_class_ids = [
                        class_id for class_id, class_name in model.names.items()
                        if str(class_name).lower() in name_map_ko
                    ]
                    results = model.predict(
                        source=inference_image,
                        classes=wanted_class_ids or None,
                        conf=0.3,
                        verbose=False,
                    )
                    for result in results:
                        if self.is_moving or result.boxes is None:
                            continue
                        masks = result.masks.xy if result.masks is not None else None
                        for index, detected_box in enumerate(result.boxes):
                            if masks is not None:
                                mask_np = masks[index].astype(np.float32)
                            else:
                                x1, y1, x2, y2 = detected_box.xyxy[0].cpu().numpy()
                                mask_np = np.array(
                                    [[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
                                    dtype=np.float32,
                                )
                            if len(mask_np) < 5:
                                # Detection 모델의 사각형 contour는 점이 4개입니다.
                                if len(mask_np) != 4:
                                    continue

                            class_id = int(detected_box.cls[0])
                            model_class_name = str(model.names[class_id]).lower()
                            name_ko = name_map_ko.get(model_class_name, model_class_name)
                            name_en = model_class_name.title()
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
