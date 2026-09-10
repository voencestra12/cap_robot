"""역할: 독립 로봇 계획 합의, TF 강체 검증, prepare/GO 장벽으로 공동 운반 실행.

인터페이스: /cap/cooperative_assignment, /agentN/cooperative_plan_request/response,
/agentN/command, gate_state, command_result; /cap/cooperative_go, agent_state, estop.
하드 실시간/힘 결합 제어가 아니며 실기 공동 운반은 별도 commissioning 확인이 필수.
"""

import copy
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from .cooperative import distance, pair_plans, prepared_pair, quaternion_matrix, transform_point
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


class CooperativeNode(Node):
    """Bounded translation coordinator; all actuator requests still cross safety gates."""

    def __init__(self):
        super().__init__("cooperative_coordinator")
        self.config = load_config(parameter(self, "config_file", ""))
        self.agents = list(self.config.get("agents", ["agent1", "agent2"]))
        if len(self.agents) != 2 or len(set(self.agents)) != 2:
            raise ValueError("cooperative coordinator requires exactly two different agents")
        self.agent_config = {
            a: load_config(parameter(self, f"{a}_config_file", "")) for a in self.agents
        }
        self.options = self.config.get("cooperative", {})
        self.world_frame = self.options.get("world_frame", "world")
        self.lock = threading.RLock()
        self.quit = threading.Event()
        self.boot_id = str(uuid.uuid4())
        self.seq = 0
        self.active = None
        self.worker = None
        self.seen_tokens = set()
        self.gates, self.scenes, self.responses, self.results = {}, {}, {}, {}
        self.permit, self.permit_received = {}, 0.0
        self.external_stop = ""
        self.transforms = {}
        self.held_relative = None
        self.state = {"status": "IDLE", "token": "", "mission_id": "", "task_id": "", "detail": ""}
        self.state_pub = self.create_publisher(String, "/cap/agent_state", qos_event())
        self.estop_pub = self.create_publisher(Bool, "/cap/estop", qos_state())
        self.go_pub = self.create_publisher(String, "/cap/cooperative_go", qos_event())
        self.metrics_pub = self.create_publisher(String, "/cap/metrics", qos_event())
        self.commands = {
            a: self.create_publisher(String, f"/{a}/command", qos_event()) for a in self.agents
        }
        self.requests = {
            a: self.create_publisher(String, f"/{a}/cooperative_plan_request", qos_event())
            for a in self.agents
        }
        self.create_subscription(String, "/cap/permit", self._on_permit, qos_event())
        self.create_subscription(
            String, "/cap/cooperative_assignment", self._on_assignment, qos_event()
        )
        self.create_subscription(Bool, "/cap/estop", self._on_stop, qos_event())
        self.create_subscription(Bool, "/cap/kill", self._on_stop, qos_event())
        for agent in self.agents:
            self.create_subscription(
                String,
                f"/{agent}/gate_state",
                lambda msg, a=agent: self._on_gate(a, msg),
                qos_state(),
            )
            self.create_subscription(
                String,
                f"/{agent}/observations",
                lambda msg, a=agent: self._on_scene(a, msg),
                qos_state(),
            )
            self.create_subscription(
                String,
                f"/{agent}/cooperative_plan_response",
                lambda msg, a=agent: self._on_response(a, msg),
                qos_event(),
            )
            self.create_subscription(
                String,
                f"/{agent}/command_result",
                lambda msg, a=agent: self._on_result(a, msg),
                qos_event(),
            )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self._publish_state)

    def _ros_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_permit(self, msg):
        try:
            data = decode(msg)
            with self.lock:
                self.permit, self.permit_received = data, time.monotonic()
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def _on_gate(self, agent, msg):
        try:
            data = decode(msg)
            if data.get("agent_id") != agent:
                raise ValueError("gate identity mismatch")
            with self.lock:
                prior = self.gates.get(agent)
                if (
                    prior
                    and prior[0].get("boot_id") == data.get("boot_id")
                    and data.get("seq", -1) <= prior[0].get("seq", -1)
                ):
                    return
                if prior and self.active and prior[0].get("boot_id") != data.get("boot_id"):
                    self.external_stop = f"{agent}: safety gate restarted during shared task"
                self.gates[agent] = (data, time.monotonic())
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def _on_scene(self, agent, msg):
        try:
            data = validate_observations(decode(msg))
            if (
                data["agent_id"] != agent
                or data["frame_id"] != self.agent_config[agent]["base_frame"]
            ):
                raise ValueError("cooperative observation frame/identity mismatch")
            with self.lock:
                prior = self.scenes.get(agent)
                if prior and data.get("seq", -1) <= prior[0].get("seq", -1):
                    return
                self.scenes[agent] = (data, time.monotonic())
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def _on_response(self, agent, msg):
        try:
            data = decode(msg)
            if data.get("agent_id") != agent:
                return
            with self.lock:
                if self.active and data.get("token") == self.active["token"]:
                    self.responses[(agent, data.get("request_id"))] = data
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def _on_result(self, agent, msg):
        try:
            data = decode(msg)
            with self.lock:
                if self.active and data.get("token") == self.active["token"]:
                    self.results[(agent, data.get("command_id"))] = data
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def _on_stop(self, msg):
        if msg.data:
            with self.lock:
                self.external_stop = (
                    "operator or peer stop latched; restart coordinator after operator reset"
                )

    def _on_assignment(self, msg):
        try:
            data = decode(msg)
            task = data.get("task", {})
            if task.get("agent") != "both" or task.get("type") != "cooperative":
                raise ValueError("cooperative assignment requires type=cooperative and agent=both")
            if not isinstance(task.get("goal"), str) or not task["goal"]:
                raise ValueError("cooperative goal required")
            if not isinstance(data.get("token"), str) or not data["token"]:
                raise ValueError("cooperative assignment token required")
            with self.lock:
                if data["token"] in self.seen_tokens or self.active:
                    return
                self.seen_tokens.add(data["token"])
                self.active = data
                self.state.update(
                    status="PLANNING",
                    token=data["token"],
                    mission_id=data.get("mission_id", ""),
                    task_id=task.get("id", ""),
                    detail="independent robot plans requested",
                )
                self.worker = threading.Thread(
                    target=self._run_assignment, args=(data,), daemon=True
                )
                self.worker.start()
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def _publish_state(self):
        with self.lock:
            self.seq += 1
            state = {
                "schema": 1,
                "agent_id": "both",
                "boot_id": self.boot_id,
                "seq": self.seq,
                **self.state,
            }
        publish_json(self.state_pub, state)

    def _set_state(self, status, detail):
        with self.lock:
            self.state.update(status=status, detail=detail)
        self._publish_state()

    def _metric(self, event, **fields):
        publish_json(
            self.metrics_pub,
            {
                "schema": 1,
                "event_id": str(uuid.uuid4()),
                "source": "cooperative",
                "event": event,
                "stamp": self._ros_now(),
                "mission_id": self.state["mission_id"],
                "task_id": self.state["task_id"],
                **fields,
            },
        )

    def _guard(self):
        """Every preparation, GO, wait and telemetry sample rechecks the permit."""
        now = time.monotonic()
        with self.lock:
            if self.quit.is_set() or self.external_stop:
                raise RuntimeError(self.external_stop or "coordinator shutting down")
            active, permit = self.active, self.permit
            if (
                not active
                or not permit.get("enabled")
                or permit.get("owner") != "both"
                or permit.get("cooperative") is not True
            ):
                raise RuntimeError("shared motion permit absent")
            if (
                any(permit.get(k) != active.get(k) for k in ("token", "session", "mission_id"))
                or permit.get("task_id") != active["task"]["id"]
            ):
                raise RuntimeError("shared motion permit identity mismatch")
            if now - self.permit_received > float(self.options.get("permit_timeout", 1.5)):
                raise RuntimeError("shared motion permit heartbeat lost")
            states = {}
            for agent in self.agents:
                sample = self.gates.get(agent)
                if not sample or now - sample[1] > float(self.options.get("gate_timeout", 0.5)):
                    raise RuntimeError(f"{agent}: safety gate telemetry stale")
                state = sample[0]
                if (
                    state.get("latched")
                    or not state.get("armed")
                    or state.get("token") != active["token"]
                ):
                    raise RuntimeError(f"{agent}: safety gate unavailable")
                states[agent] = copy.deepcopy(state)
            held_relative = self.held_relative
        if held_relative is not None:
            worlds = {
                a: transform_point(self.transforms[a], states[a]["position"]) for a in self.agents
            }
            ordered = sorted(self.agents)
            measured = [worlds[ordered[1]][i] - worlds[ordered[0]][i] for i in range(3)]
            if distance(measured, held_relative) > float(
                self.options.get("relative_tolerance", 0.006)
            ):
                raise RuntimeError("shared payload relative TCP error exceeds commissioned bound")
        return states

    def _scene(self, agent):
        with self.lock:
            sample = self.scenes.get(agent)
            if not sample:
                raise RuntimeError(f"{agent}: no independent vision observation")
            data, received = sample
            max_age = float(self.options.get("observation_max_age", 2.0))
            source_age = self._ros_now() - data["stamp"]
            if (
                not data.get("valid")
                or time.monotonic() - received > max_age
                or not -0.25 <= source_age <= max_age
            ):
                raise RuntimeError(f"{agent}: invalid/stale independent observation")
            return copy.deepcopy(data)

    def _load_transforms(self):
        # [변경] 공동 운반에 identity 변환을 묵시적으로 쓰지 않고 보정된 TF가 없으면 거부.
        transforms = {}
        for agent in self.agents:
            base = self.agent_config[agent]["base_frame"]
            transform = self.tf_buffer.lookup_transform(self.world_frame, base, Time())
            stamp = Time.from_msg(transform.header.stamp).nanoseconds / 1e9
            # Static TF has zero stamp; dynamic calibration must be recent.
            if stamp != 0 and not -0.1 <= self._ros_now() - stamp <= 0.5:
                raise RuntimeError(f"{agent}: common-frame calibration TF stale")
            t, q = transform.transform.translation, transform.transform.rotation
            transforms[agent] = quaternion_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
        return transforms

    def _commissioning(self):
        if not self.options.get("enabled", False):
            raise RuntimeError("cooperative execution is disabled in workstation configuration")
        for agent in self.agents:
            config = self.agent_config[agent]
            if not config.get("cooperative", {}).get("enabled", False):
                raise RuntimeError(f"{agent}: cooperative execution disabled")
            if not config.get("dry_run", True):
                required = ("calibration_verified", "workspace_verified", "cooperative_verified")
                if not all(config.get("commissioning", {}).get(k) is True for k in required):
                    raise RuntimeError(f"{agent}: hardware cooperative commissioning incomplete")
        if len({bool(c.get("dry_run", True)) for c in self.agent_config.values()}) != 1:
            raise RuntimeError("mixing hardware and dry-run arms is forbidden")

    def _plan(self, assignment):
        previous, error = {}, ""
        for attempt in range(int(self.options.get("max_replans", 2)) + 1):
            starts = self._guard()
            scenes = {a: self._scene(a) for a in self.agents}
            request_id = str(uuid.uuid4())
            for agent in self.agents:
                peer = next(a for a in self.agents if a != agent)
                request = {
                    "schema": 1,
                    "request_id": request_id,
                    "token": assignment["token"],
                    "mission_id": assignment["mission_id"],
                    "task_id": assignment["task"]["id"],
                    "goal": assignment["task"]["goal"],
                    "agent_id": agent,
                    "peer_id": peer,
                    "local_scene": scenes[agent],
                    "peer_scene": scenes[peer],
                    "world_frame": self.world_frame,
                    "base_to_world": self.transforms[agent],
                    "peer_base_to_world": self.transforms[peer],
                    "local_start": starts[agent],
                    "peer_start": starts[peer],
                    "previous_proposal": previous.get(agent),
                    "peer_proposal": previous.get(peer),
                    "validation_error": error,
                    "attempt": attempt,
                }
                publish_json(self.requests[agent], request)
            deadline = time.monotonic() + float(self.options.get("plan_timeout", 60.0))
            responses = {}
            while time.monotonic() < deadline:
                self._guard()
                with self.lock:
                    responses = {a: self.responses.get((a, request_id)) for a in self.agents}
                if all(responses.values()):
                    break
                self.quit.wait(0.02)
            if not all(responses.values()):
                raise RuntimeError("independent robot proposal timeout")
            failed = [
                r.get("reason", "local LLM proposal rejected")
                for r in responses.values()
                if r.get("status") != "SUCCEEDED"
            ]
            if failed:
                raise RuntimeError("; ".join(failed))
            previous = {a: responses[a]["plan"] for a in self.agents}
            try:
                result = pair_plans(
                    previous, self.transforms, starts, self.agent_config, self.options
                )
                self._metric(
                    "cooperative_plan_validated", success=True, steps=len(result), attempt=attempt
                )
                return result
            except ValueError as exc:
                error = str(exc)
                self._metric(
                    "cooperative_plan_disagreement", success=False, detail=error, attempt=attempt
                )
        raise RuntimeError(
            f"independent proposals failed to agree after bounded replanning: {error}"
        )

    def _execute_pair(self, assignment, index, step):
        states = self._guard()
        # [변경] command_result와 10 Hz gate_state의 DDS 도착 순서는 보장되지 않는다.
        # 완료 결과 이후 새 idle 관측을 기다리며 이전 telemetry로 다음 단계를 거부하지 않는다.
        idle_deadline = time.monotonic() + 1.0
        while not all(s.get("idle") for s in states.values()):
            if time.monotonic() >= idle_deadline:
                raise RuntimeError("both gates must be idle before each paired preparation")
            self.quit.wait(0.01)
            states = self._guard()
        lead = float(self.options.get("start_delay", 1.0))
        if not 0.5 <= lead <= 2.5:
            raise ValueError("cooperative start_delay must be 0.5..2.5 seconds")
        execute_at = self._ros_now() + lead
        group_id = str(uuid.uuid4())
        commands = {
            a: {
                "schema": 1,
                "command_id": str(uuid.uuid4()),
                "agent_id": a,
                "token": assignment["token"],
                "mission_id": assignment["mission_id"],
                "task_id": assignment["task"]["id"],
                "action": step["actions"][a],
                "stamp": self._ros_now(),
                "phase": "prepare",
                "group_id": group_id,
                "step_index": index,
                "execute_at": execute_at,
            }
            for a in self.agents
        }
        # [변경] 먼저 두 gate의 준비만 요청하고, 둘 다 준비되기 전에는 움직임을 시작하지 않는다.
        for agent in self.agents:
            publish_json(self.commands[agent], commands[agent])
        deadline = time.monotonic() + min(
            float(self.options.get("prepare_timeout", 0.6)), lead - 0.2
        )
        ready = False
        while time.monotonic() < deadline:
            states = self._guard()
            self._check_failures(commands)
            if prepared_pair(states, commands):
                ready = True
                break
            self.quit.wait(0.01)
        if not ready or self._ros_now() > execute_at - 0.15:
            raise RuntimeError("two-gate preparation barrier missed the GO cutoff")
        self._guard()
        publish_json(
            self.go_pub,
            {
                "schema": 1,
                "token": assignment["token"],
                "group_id": group_id,
                "step_index": index,
                "execute_at": execute_at,
                "command_ids": {a: commands[a]["command_id"] for a in self.agents},
            },
        )
        deadline = time.monotonic() + lead + float(self.options.get("command_timeout", 25.0))
        results = {}
        while time.monotonic() < deadline:
            self._guard()
            results = self._check_failures(commands)
            if all(results.values()):
                break
            self.quit.wait(0.01)
        if not all(results.values()):
            raise RuntimeError("paired command completion timeout; ownership must not transfer")
        actual_starts = [r.get("started_at") for r in results.values()]
        if any(isinstance(s, bool) or not isinstance(s, (int, float)) for s in actual_starts):
            raise RuntimeError("missing measured paired command start timestamps")
        skew = max(actual_starts) - min(actual_starts)
        if skew > float(self.options.get("max_start_skew", 0.05)):
            raise RuntimeError("measured paired start skew exceeded commissioned limit")
        states = self._guard()
        for agent in self.agents:
            action = step["actions"][agent]
            if action["kind"] == "move" and distance(
                states[agent]["position"], action["position"]
            ) > float(self.options.get("pose_tolerance", 0.015)):
                raise RuntimeError(f"{agent}: paired final pose mismatch")
        self._metric(
            "cooperative_step",
            success=True,
            phase=step["phase"],
            step_index=index,
            dispatch_skew_s=skew,
        )

    def _check_failures(self, commands):
        with self.lock:
            results = {a: self.results.get((a, c["command_id"])) for a, c in commands.items()}
        for agent, result in results.items():
            if result and result.get("status") != "SUCCEEDED":
                raise RuntimeError(f"{agent}: {result.get('reason', 'paired action failed')}")
        return results

    def _verify(self, actions):
        # [변경] SDK 성공만으로 공동 태스크 성공을 선언하지 않고 양쪽 독립 비전으로 검증.
        seen = {a: self._scene(a)["seq"] for a in self.agents}
        deadline = time.monotonic() + float(self.options.get("verification_timeout", 3.0))
        while time.monotonic() < deadline:
            self._guard()
            scenes = {a: self._scene(a) for a in self.agents}
            if all(scenes[a]["seq"] > seen[a] for a in self.agents):
                for agent in self.agents:
                    action = actions[agent]
                    matches = [
                        o for o in scenes[agent]["objects"] if o["id"] == action["object_id"]
                    ]
                    if (
                        len(matches) != 1
                        or distance(matches[0]["position"], action["expected_position"])
                        > action["tolerance"]
                    ):
                        raise RuntimeError(
                            f"{agent}: shared payload final vision verification failed"
                        )
                return
            self.quit.wait(0.02)
        raise RuntimeError("fresh post-action observations missing for both robots")

    def _run_assignment(self, assignment):
        try:
            self._commissioning()
            self._guard()
            self.transforms = self._load_transforms()
            steps = self._plan(assignment)
            self._set_state("EXECUTING", f"validated {len(steps)} synchronized steps")
            for index, step in enumerate(steps):
                self._guard()
                if step["phase"] == "verify":
                    self._verify(step["actions"])
                    continue
                self._execute_pair(assignment, index, step)
                if step["phase"] == "grasp":
                    self.held_relative = step["relative"]
                elif step["phase"] == "release":
                    self.held_relative = None
            self._set_state("SUCCEEDED", "both robot visions verified shared task completion")
        except Exception as exc:
            # [변경] 한쪽 실패/불확실한 결과는 전역 latch; 나머지 팔이나 다음 태스크를 계속하지 않는다.
            self.estop_pub.publish(Bool(data=True))
            self._set_state("FAILED", str(exc))
            self._metric("cooperative_failure", success=False, detail=str(exc))
            self.get_logger().error(str(exc))
        finally:
            with self.lock:
                self.active = None
                self.responses.clear()
                self.results.clear()
                self.held_relative = None

    def close(self):
        self.quit.set()
        if self.active:
            self.estop_pub.publish(Bool(data=True))
        if self.worker:
            self.worker.join(timeout=2.0)


def main(args=None):
    run_node(CooperativeNode, args=args)


if __name__ == "__main__":
    main()
