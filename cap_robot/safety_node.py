"""역할: 로봇당 단일 SDK 소유자·최종 안전 게이트. 인터페이스: command/result/state,
permit, agent_state, estop/kill, cooperative_go, operator arm/reset_stop, joint_states/TF.

# [변경] 계획 노드에 SDK 참조가 없으며 독립 프로세스가 모든 동작을 승인한다.
"""

import time
import uuid
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from .hardware import make_backend
from .safety import SafetyGate, quaternion
from .protocol import validate_command
from .ros_support import (
    parameter,
    load_config,
    decode,
    publish_json,
    qos_state,
    qos_event,
    run_node,
)


class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_gate")
        self.config = load_config(parameter(self, "config_file", ""))
        self.ident = self.config["agent_id"]
        self.boot = uuid.uuid4().hex
        self.seq = 0
        self.emergency_callbacks = ReentrantCallbackGroup()
        self.result_pub = self.create_publisher(String, "command_result", qos_event())
        self.metric_pub = self.create_publisher(String, "/cap/metrics", qos_event())
        self.state_pub = self.create_publisher(String, "gate_state", qos_state())
        self.joint_pub = self.create_publisher(JointState, "joint_states", qos_event())
        self.tf = TransformBroadcaster(self)
        self.backend = make_backend(self.config)
        self.gate = SafetyGate(
            self.config,
            self.backend,
            self.result,
            ros_clock=lambda: self.get_clock().now().nanoseconds * 1e-9,
        )
        self.create_subscription(String, "command", self.command, qos_event())
        self.create_subscription(
            String, "/cap/permit", self.permit, qos_event(), callback_group=self.emergency_callbacks
        )
        self.create_subscription(
            String,
            "/cap/agent_state",
            self.agent,
            qos_event(),
            callback_group=self.emergency_callbacks,
        )
        self.create_subscription(String, "/cap/cooperative_go", self.go, qos_event())
        # [변경] 느린 operator arm 서비스 중에도 정지와 heartbeat 콜백을 처리한다.
        self.create_subscription(
            Bool,
            "/cap/estop",
            lambda m: self.gate.stop("operator/peer E-stop") if m.data else None,
            qos_event(),
            callback_group=self.emergency_callbacks,
        )
        self.create_subscription(
            Bool,
            "/cap/kill",
            lambda m: self.gate.stop("operator kill switch", kill=True) if m.data else None,
            qos_event(),
            callback_group=self.emergency_callbacks,
        )
        self.create_service(Trigger, "arm", self.arm)
        self.create_service(Trigger, "reset_stop", self.reset)
        self.create_timer(0.1, self.tick, callback_group=self.emergency_callbacks)

    def result(self, data):
        publish_json(self.result_pub, data)
        # [변경] agent가 정지로 기다림을 중단해도 gate가 실패를 직접 CSV로 남긴다.
        publish_json(self.metric_pub, dict(
            schema=1, event_id=f'{self.boot}:{data["command_id"]}:result',
            source=self.ident + '_gate', event='gate_command_result',
            stamp=self.get_clock().now().nanoseconds * 1e-9,
            mission_id=data.get('mission_id',''), task_id=data.get('task_id',''),
            command_id=data['command_id'], success=data['status']=='SUCCEEDED',
            cooperative=data.get('cooperative',False), detail=data.get('reason',''),
        ))

    def command(self, msg):
        data = {}
        try:
            data = decode(msg)
            validate_command(data)
            self.gate.submit(data)
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warning(f"command rejected: {exc}")
            if isinstance(data.get("command_id"), str):
                self.result(
                    dict(
                        schema=1,
                        command_id=data["command_id"],
                        token=data.get("token", ""),
                        agent_id=self.ident,
                        status="FAILED",
                        reason=str(exc),
                        started_at=None,
                    )
                )

    def permit(self, msg):
        try:
            self.gate.update_permit(decode(msg))
        except (ValueError, TypeError):
            pass

    def agent(self, msg):
        try:
            d = decode(msg)
            if type(d.get("seq")) is int and isinstance(d.get("boot_id"), str):
                self.gate.update_agent(d)
        except (ValueError, TypeError):
            pass

    def go(self, msg):
        try:
            self.gate.cooperative_go(decode(msg))
        except (ValueError, TypeError):
            pass

    def arm(self, request, response):
        try:
            self.gate.arm()
            response.success = True
            response.message = "armed by operator"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def reset(self, request, response):
        try:
            self.gate.reset()
            response.success = True
            response.message = "E-stop reset; arm remains required"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def tick(self):
        self.seq += 1
        s = self.gate.snapshot()
        publish_json(self.state_pub, dict(schema=1, boot_id=self.boot, seq=self.seq, **s))
        if not s["telemetry_fresh"] or len(s["position"]) != 3:
            return
        stamp = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.config["base_frame"]
        t.child_frame_id = self.ident + "_tool0"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = s[
            "position"
        ]
        (
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ) = quaternion(s["rpy"])
        self.tf.sendTransform(t)
        if len(s["joints"]) == 6:
            j = JointState()
            j.header.stamp = stamp
            j.name = [f"{self.ident}_joint{i}" for i in range(1, 7)]
            j.position = s["joints"]
            self.joint_pub.publish(j)

    def close(self):
        self.gate.close()


def main(args=None):
    run_node(SafetyNode, args)
