"""역할: LLM 임무 DAG·배타 실행권·피드백 집계. 인터페이스: /cap/mission_* /permit /assignment."""

from __future__ import annotations

import copy
import threading
import time
import uuid

from rclpy.node import Node
from std_msgs.msg import Bool, String

from .llm import OllamaClient
from .metrics import make_event
from .planning import gate_consensus, load_prompt, text, validate_task_graph
from .protocol import validate_observations
from .ros_support import (
    decode,
    load_config,
    parameter,
    publish_json,
    qos_event,
    qos_state,
    run_node,
)


class WorkstationNode(Node):
    def __init__(self):
        super().__init__("workstation")
        self.config = load_config(parameter(self, "config_file", ""))
        self.agents = self.config.get("agents", ["agent1", "agent2"])
        if self.agents != ["agent1", "agent2"]:
            raise ValueError("This dual-arm deployment requires agents [agent1, agent2]")
        self.llm = OllamaClient(self.config.get("llm", {}))
        self.prompt = load_prompt("workstation.txt")
        self.session = uuid.uuid4().hex
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.worker = None
        self.mission = None
        self.last_state = {"status": "IDLE", "detail": "waiting for a goal"}
        self.gates, self.states, self.scenes = {}, {}, {}
        self.requests = set()
        self.permit = self._idle_permit()
        self.assignment = None
        self.permit_pub = self.create_publisher(String, "/cap/permit", qos_state())
        self.assignment_pub = self.create_publisher(String, "/cap/assignment", qos_event())
        self.cooperative_pub = self.create_publisher(
            String, "/cap/cooperative_assignment", qos_event()
        )
        self.state_pub = self.create_publisher(String, "/cap/mission_state", qos_state())
        self.metric_pub = self.create_publisher(String, "/cap/metrics", qos_event())
        self.stop_pub = self.create_publisher(Bool, "/cap/estop", qos_state())
        self.create_subscription(String, "/cap/mission_input", self._input, qos_event())
        self.create_subscription(String, "/cap/agent_state", self._state, qos_event())
        for agent in self.agents:
            self.create_subscription(
                String, f"/{agent}/gate_state", lambda msg, a=agent: self._gate(msg, a), qos_state()
            )
            self.create_subscription(
                String,
                f"/{agent}/observations",
                lambda msg, a=agent: self._scene(msg, a),
                qos_event(),
            )
        self.create_timer(0.2, self._tick)

    def _idle_permit(self):
        return {
            "schema": 1,
            "session": self.session,
            "token": "",
            "owner": "",
            "mission_id": "",
            "task_id": "",
            "enabled": False,
            "cooperative": False,
        }

    def _metric(self, event, **fields):
        if not self.closed.is_set():
            publish_json(self.metric_pub, make_event("workstation", event, **fields))

    def _input(self, msg):
        try:
            request = decode(msg)
            request_id = text(request.get("request_id"), "request_id", 128)
            goal = text(request.get("goal"), "goal", 8192)
            with self.lock:
                if request_id in self.requests:
                    return
                if self.mission is not None:
                    self._metric(
                        "mission_request_rejected",
                        success=False,
                        detail="active mission",
                        request_id=request_id,
                    )
                    return
                self.requests.add(request_id)
                if len(self.requests) > 4096:
                    raise ValueError("request budget exhausted; restart the idle workstation")
                mid = uuid.uuid4().hex
                self.mission = {
                    "id": mid,
                    "request_id": request_id,
                    "goal": goal,
                    "status": "PLANNING",
                    "tasks": [],
                    "completed": [],
                    "active_task": None,
                    "phase": "PLANNING",
                    "detail": "",
                }
                context = {
                    "goal": goal,
                    "agents": self.agents,
                    "local_scenes": self._scene_snapshot(),
                    "agent_states": {a: r["data"] for a, r in self.states.items()},
                    "capabilities": [
                        "cartesian_move_rpy",
                        "gripper",
                        "bounded_search",
                        "visual_position_verification",
                        "calibrated_dual_rigid_carry",
                    ],
                }
                self._metric(
                    "mission_input",
                    mission_id=mid,
                    success=True,
                    detail=goal,
                    request_id=request_id,
                )
                self.worker = threading.Thread(target=self._plan, args=(mid, context), daemon=True)
                self.worker.start()
        except (ValueError, TypeError) as error:
            self.get_logger().error(f"mission request rejected: {error}")

    def _plan(self, mission_id, context):
        started = time.monotonic()
        success, detail = False, ""
        try:
            raw = self.llm.generate(self.prompt, context)
            tasks = validate_task_graph(raw, self.agents)
            success = True
            with self.lock:
                # [변경] 비동기 LLM 응답은 생성 시점의 임무에만 적용한다.
                if self.closed.is_set() or not self.mission or self.mission["id"] != mission_id:
                    return
                self.mission.update(tasks=tasks, status="WAITING_FOR_GATES", phase="READY")
        except Exception as error:
            detail = str(error)
            with self.lock:
                if self.mission and self.mission["id"] == mission_id:
                    self.last_state = {**self.mission, "status": "REJECTED", "detail": detail}
                    self.mission = None  # no physical permit has been issued
        finally:
            self._metric(
                "llm_inference",
                mission_id=mission_id,
                latency_s=time.monotonic() - started,
                success=success,
                detail=detail,
                metric_definition="HTTP+JSON+schema validity proxy",
            )
            self._metric(
                "mission_plan_validated",
                mission_id=mission_id,
                success=success,
                detail=detail,
                metric_definition="schema compliance; human semantic correctness requires separate grade",
            )

    def _scene_snapshot(self):
        result = {}
        now = time.monotonic()
        age_limit = float(self.config.get("planning", {}).get("observation_max_age", 2.0))
        for agent in self.agents:
            record = self.scenes.get(agent)
            if record and now - record["received"] <= age_limit:
                result[agent] = copy.deepcopy(record["data"])
            else:
                result[agent] = {
                    "valid": False,
                    "objects": [],
                    "reason": "local scene missing/stale",
                }
        return result

    def _scene(self, msg, agent):
        try:
            data = validate_observations(decode(msg))
            now_stamp = self.get_clock().now().nanoseconds / 1e9
            if data["agent_id"] != agent or not -0.2 <= now_stamp - data["stamp"] <= 2.0:
                return
            with self.lock:
                self.scenes[agent] = {"data": data, "received": time.monotonic()}
        except (ValueError, TypeError):
            return

    def _gate(self, msg, agent):
        try:
            data = decode(msg)
            if (
                data.get("agent_id") != agent
                or not isinstance(data.get("boot_id"), str)
                or type(data.get("seq")) is not int
            ):
                return
            if any(type(data.get(k)) is not bool for k in ("idle", "armed", "latched")):
                return
            with self.lock:
                old = self.gates.get(agent)
                if (
                    old
                    and old["data"]["boot_id"] == data["boot_id"]
                    and data["seq"] <= old["data"]["seq"]
                ):
                    return
                if old and old["data"]["boot_id"] != data["boot_id"] and self.assignment:
                    self._block(f"{agent} gate restarted while task active", stop=True)
                self.gates[agent] = {"data": data, "received": time.monotonic()}
        except (ValueError, TypeError):
            return

    def _state(self, msg):
        try:
            data = decode(msg)
            agent = data.get("agent_id")
            if (
                agent not in [*self.agents, "both"]
                or not isinstance(data.get("boot_id"), str)
                or type(data.get("seq")) is not int
            ):
                return
            with self.lock:
                old = self.states.get(agent)
                if (
                    old
                    and old["data"]["boot_id"] == data["boot_id"]
                    and data["seq"] <= old["data"]["seq"]
                ):
                    return
                if old and old["data"]["boot_id"] != data["boot_id"] and self.assignment:
                    self._block(f"{agent} agent restarted while task active", stop=True)
                self.states[agent] = {"data": data, "received": time.monotonic()}
                if not self.assignment or self.mission["phase"] == "BLOCKED":
                    return
                task = self.assignment["task"]
                if (
                    agent != task["agent"]
                    or data.get("token") != self.assignment["token"]
                    or data.get("mission_id") != self.assignment["mission_id"]
                    or data.get("task_id") != task["id"]
                ):
                    return
                status = data.get("status")
                if status == "FAILED":
                    self._block(data.get("detail", "agent task failed"), stop=False)
                elif status == "SUCCEEDED":
                    self.mission["phase"] = "CONFIRM_IDLE"
                    self.mission["status"] = "VERIFYING_GATE_IDLE"
                elif status in ("PLANNING", "EXECUTING"):
                    self.mission["status"] = status
                    self.mission["detail"] = data.get("detail", "")
        except (ValueError, TypeError, KeyError):
            return

    def _block(self, reason, *, stop):
        if self.mission is None or self.mission["phase"] == "BLOCKED":
            return
        self.mission.update(status="BLOCKED", phase="BLOCKED", detail=str(reason))
        self.permit["enabled"] = False
        self._metric(
            "mission_blocked",
            mission_id=self.mission["id"],
            task_id=self.permit["task_id"],
            success=False,
            detail=str(reason),
        )
        if stop:
            self.stop_pub.publish(Bool(data=True))
        # [변경] 실패/미확인 결과의 토큰은 다른 로봇에 절대로 재할당하지 않는다.

    def _heartbeats_ok(self, now):
        timeout = float(self.config.get("heartbeat_timeout", 2.0))
        return all(
            a in self.states and now - self.states[a]["received"] <= timeout for a in self.agents
        )

    def _tick(self):
        if self.closed.is_set():
            return
        with self.lock:
            now = time.monotonic()
            timeout = float(self.config.get("heartbeat_timeout", 2.0))
            mission = self.mission
            if mission and mission["phase"] in ("AWAIT_ACK", "RUNNING", "CONFIRM_IDLE"):
                if not self._heartbeats_ok(now) or any(
                    a not in self.gates or now - self.gates[a]["received"] > timeout
                    for a in self.agents
                ):
                    self._block(
                        "agent/gate heartbeat lost; physical outcome requires inspection", stop=True
                    )
                elif any(
                    self.gates[a]["data"]["latched"] or not self.gates[a]["data"]["armed"]
                    for a in self.agents
                ):
                    self._block("a safety gate is latched or unarmed", stop=True)
                elif now - mission["task_started"] > float(self.config.get("task_timeout", 180.0)):
                    self._block(
                        "task deadline exceeded; do not retry unknown physical outcome", stop=True
                    )
                elif mission["phase"] == "RUNNING" and self.assignment["task"]["agent"] == "both":
                    coordinator = self.states.get("both")
                    if now - mission["task_started"] > timeout and (
                        not coordinator or now - coordinator["received"] > timeout
                    ):
                        self._block("cooperative coordinator heartbeat lost", stop=True)
            if mission and mission["phase"] == "CONFIRM_IDLE":
                if gate_consensus(self.gates, self.agents, self.permit["token"], now, timeout):
                    task_id = self.assignment["task"]["id"]
                    mission["completed"].append(task_id)
                    self._metric(
                        "task_result",
                        mission_id=mission["id"],
                        task_id=task_id,
                        success=True,
                        detail="agent verification and both idle gates confirmed",
                    )
                    self.assignment = None
                    self.permit = self._idle_permit()
                    mission.update(phase="READY", active_task=None)
            if mission and mission["phase"] == "READY":
                if len(mission["completed"]) == len(mission["tasks"]):
                    self.last_state = {
                        **mission,
                        "status": "SUCCEEDED",
                        "detail": "all task verifications completed",
                    }
                    self._metric(
                        "mission_result",
                        mission_id=mission["id"],
                        success=True,
                        detail="execution/visual verification; semantic grade is separate",
                    )
                    self.mission = None
                    mission = None
                elif self._heartbeats_ok(now) and gate_consensus(
                    self.gates, self.agents, "", now, timeout, require_token=False
                ):
                    ready = next(
                        t
                        for t in mission["tasks"]
                        if t["id"] not in mission["completed"]
                        and set(t["depends_on"]) <= set(mission["completed"])
                    )
                    token = uuid.uuid4().hex
                    self.permit = {
                        "schema": 1,
                        "session": self.session,
                        "token": token,
                        "owner": ready["agent"],
                        "mission_id": mission["id"],
                        "task_id": ready["id"],
                        "enabled": True,
                        "cooperative": ready["type"] == "cooperative",
                    }
                    self.assignment = {
                        "schema": 1,
                        "session": self.session,
                        "token": token,
                        "mission_id": mission["id"],
                        "task": ready,
                    }
                    mission.update(
                        phase="AWAIT_ACK",
                        status="WAITING_FOR_TOKEN_ACK",
                        active_task=ready["id"],
                        task_started=now,
                    )
            if mission and mission["phase"] == "AWAIT_ACK":
                if gate_consensus(self.gates, self.agents, self.permit["token"], now, timeout):
                    mission.update(phase="RUNNING", status="ASSIGNED")
            publish_json(self.permit_pub, self.permit)
            if self.mission and self.mission["phase"] == "RUNNING" and self.assignment:
                pub = (
                    self.cooperative_pub
                    if self.assignment["task"]["type"] == "cooperative"
                    else self.assignment_pub
                )
                publish_json(pub, self.assignment)
            snapshot = copy.deepcopy(self.mission or self.last_state)
            snapshot.update(
                schema=1,
                session=self.session,
                permit=copy.deepcopy(self.permit),
                agents={a: r["data"] for a, r in self.states.items()},
                gates={a: r["data"] for a, r in self.gates.items()},
            )
            publish_json(self.state_pub, snapshot)

    def close(self):
        self.closed.set()
        with self.lock:
            if self.assignment:
                self.stop_pub.publish(Bool(data=True))
            self.permit["enabled"] = False
            publish_json(self.permit_pub, self.permit)


def main(args=None):
    run_node(WorkstationNode, args)
