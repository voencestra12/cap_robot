"""역할: 로봇별 독립 LLM 배치 계획·실행·상태 피드백. 인터페이스: observations,
assignment, command/result, cooperative_plan_request/response, /cap/agent_state.

# [변경] 고정 PnP 레시피·자동 홈 복귀 제거. 성공한 동작 이력을 보존하고 이상 때만 재계획.
"""

import copy
import math
import threading
import time
import uuid
from rclpy.node import Node
from std_msgs.msg import Bool, String
from .llm import OllamaClient
from .planning import (
    load_prompt,
    validate_action_plan,
    resolve_move,
    verify_scene,
    matching_objects,
)
from .cooperative import build_local_prompt, validate_local_plan
from .protocol import validate_observations
from .metrics import make_event
from .ros_support import (
    parameter,
    load_config,
    decode,
    publish_json,
    qos_state,
    qos_event,
    run_node,
)


class ObservationAnomaly(ValueError):
    pass


class PhysicalFailure(RuntimeError):
    pass


class AgentNode(Node):
    def __init__(self):
        super().__init__("agent")
        self.config = load_config(parameter(self, "config_file", ""))
        self.ident = self.config["agent_id"]
        self.options = self.config.get("planning", {})
        self.llm = OllamaClient(self.config.get("llm", {}))
        self.prompt = load_prompt("agent.txt")
        self.boot = uuid.uuid4().hex
        self.seq = 0
        self.lock = threading.RLock()
        self.quit = threading.Event()
        self.scene = None
        self.scene_at = 0.0
        self.gate = None
        self.gate_at = 0.0
        self.permit = {}
        self.permit_at = 0.0
        self.results = {}
        self.peers = {}
        self.seen = set()
        self.coop_seen = set()
        self.active = None
        self.worker = None
        self.stopped = False
        self.state = dict(
            status="IDLE", mission_id="", task_id="", token="", detail="waiting for assignment"
        )
        self.pub = self.create_publisher(String, "/cap/agent_state", qos_event())
        self.commands = self.create_publisher(String, "command", qos_event())
        self.metrics = self.create_publisher(String, "/cap/metrics", qos_event())
        self.stop_pub = self.create_publisher(Bool, "/cap/estop", qos_event())
        self.coop_pub = self.create_publisher(String, "cooperative_plan_response", qos_event())
        self.create_subscription(String, "observations", self.on_scene, qos_state())
        self.create_subscription(String, "gate_state", self.on_gate, qos_state())
        self.create_subscription(String, "command_result", self.on_result, qos_event())
        self.create_subscription(String, "/cap/permit", self.on_permit, qos_event())
        self.create_subscription(String, "/cap/assignment", self.on_assignment, qos_event())
        self.create_subscription(String, "/cap/agent_state", self.on_peer, qos_event())
        self.create_subscription(
            String, "cooperative_plan_request", self.on_cooperative, qos_event()
        )
        self.create_subscription(Bool, "/cap/estop", self.on_stop, qos_event())
        self.create_subscription(Bool, "/cap/kill", self.on_stop, qos_event())
        self.create_timer(0.2, self.heartbeat)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def metric(self, event, **fields):
        publish_json(
            self.metrics,
            make_event(
                self.ident,
                event,
                mission_id=self.state["mission_id"],
                task_id=self.state["task_id"],
                **fields,
            ),
        )

    def heartbeat(self):
        with self.lock:
            self.seq += 1
            state = dict(
                schema=1, agent_id=self.ident, boot_id=self.boot, seq=self.seq, **self.state
            )
        publish_json(self.pub, state)

    def set_state(self, status, detail=""):
        with self.lock:
            self.state.update(status=status, detail=detail)
        self.heartbeat()

    def on_stop(self, msg):
        if msg.data:
            with self.lock:
                self.stopped = True

    def on_scene(self, msg):
        try:
            d = validate_observations(decode(msg))
            if d["agent_id"] != self.ident or d["frame_id"] != self.config["base_frame"]:
                return
            with self.lock:
                # Source timestamp also disambiguates a restarted perception sequence.
                if self.scene and d["stamp"] < self.scene["stamp"]:
                    return
                self.scene, self.scene_at = d, time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_gate(self, msg):
        try:
            d = decode(msg)
            if d.get("agent_id") != self.ident:
                return
            with self.lock:
                if self.gate and self.gate["boot_id"] != d.get("boot_id") and self.active:
                    self.stopped = True
                if (
                    self.gate
                    and self.gate["boot_id"] == d.get("boot_id")
                    and d.get("seq", -1) <= self.gate["seq"]
                ):
                    return
                self.gate, self.gate_at = d, time.monotonic()
        except (ValueError, TypeError, KeyError):
            pass

    def on_result(self, msg):
        try:
            d = decode(msg)
            if d.get("token") == self.state["token"]:
                with self.lock:
                    self.results[d["command_id"]] = d
        except (ValueError, KeyError):
            pass

    def on_permit(self, msg):
        try:
            d = decode(msg)
            with self.lock:
                self.permit, self.permit_at = d, time.monotonic()
        except ValueError:
            pass

    def on_peer(self, msg):
        try:
            d = decode(msg)
            if d.get("agent_id") == self.ident:
                return
            with self.lock:
                self.peers[d["agent_id"]] = dict(data=d, received=time.monotonic())
                if (
                    d["agent_id"] == "both"
                    and d.get("token") == self.state["token"]
                    and self.active is None
                    and d.get("status") in ("SUCCEEDED", "FAILED")
                ):
                    self.state.update(status=d["status"], detail=d.get("detail", ""))
        except (ValueError, KeyError):
            pass

    def guard(self, token=None):
        with self.lock:
            p, g = self.permit, self.gate
            if self.quit.is_set() or self.stopped:
                raise PhysicalFailure("agent stopped; operator recovery required")
            if (
                not g
                or time.monotonic() - self.gate_at > 0.7
                or g.get("latched")
                or not g.get("armed")
            ):
                raise PhysicalFailure("local safety gate unavailable")
            if token is not None:
                if (
                    time.monotonic() - self.permit_at > 1.5
                    or not p.get("enabled")
                    or p.get("token") != token
                    or p.get("owner") not in (self.ident, "both")
                ):
                    raise PhysicalFailure("task permit lost")
            return copy.deepcopy(g)

    def observation(self, after_stamp=None, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.guard(self.state["token"])
            with self.lock:
                s, received = copy.deepcopy(self.scene), self.scene_at
            age = float(self.options.get("observation_max_age", 2.0))
            if (
                s
                and s["valid"]
                and time.monotonic() - received <= age
                and -0.1 <= self.now() - s["stamp"] <= age
                and (after_stamp is None or s["stamp"] > after_stamp)
            ):
                return s
            if self.quit.wait(0.02):
                break
        raise ObservationAnomaly("fresh independent observation unavailable")

    def on_assignment(self, msg):
        try:
            d = decode(msg)
            if d.get("task", {}).get("agent") != self.ident:
                return
            token = d.get("token")
            if not isinstance(token, str) or not token:
                return
            self.guard(token)
            if any(d.get(k) != self.permit.get(k) for k in ("session", "mission_id")) or d["task"][
                "id"
            ] != self.permit.get("task_id"):
                return
            with self.lock:
                if token in self.seen or self.active:
                    return
                self.seen.add(token)
                self.active = d
                self.state.update(
                    status="PLANNING",
                    mission_id=d["mission_id"],
                    task_id=d["task"]["id"],
                    token=token,
                    detail="independent local batch planning",
                )
                self.results.clear()
                self.worker = threading.Thread(target=self.run_task, args=(d,), daemon=True)
                self.worker.start()
        except (ValueError, KeyError, TypeError, PhysicalFailure) as exc:
            self.get_logger().warning(str(exc))

    def infer(self, context):
        self.set_state("PLANNING", "creating remaining action batch")
        started = time.monotonic()
        success = False
        detail = ""
        try:
            actions = validate_action_plan(self.llm.generate(self.prompt, context), self.config)
            success = True
            return actions
        except Exception as exc:
            detail = str(exc)
            raise
        finally:
            self.metric(
                "llm_inference",
                latency_s=time.monotonic() - started,
                success=success,
                detail=detail,
                metric_definition="HTTP+JSON+action schema validity proxy",
            )

    def command(self, action):
        self.guard(self.state["token"])
        ident = uuid.uuid4().hex
        c = dict(
            schema=1,
            command_id=ident,
            agent_id=self.ident,
            token=self.state["token"],
            mission_id=self.state["mission_id"],
            task_id=self.state["task_id"],
            stamp=self.now(),
            action=action,
        )
        publish_json(self.commands, c)
        deadline = time.monotonic() + float(self.options.get("command_timeout", 25.0))
        while time.monotonic() < deadline:
            self.guard(self.state["token"])
            with self.lock:
                result = self.results.get(ident)
            if result:
                self.metric(
                    "command_result",
                    success=result.get("status") == "SUCCEEDED",
                    detail=result.get("reason", ""),
                    command_id=ident,
                )
                if result.get("status") != "SUCCEEDED":
                    raise PhysicalFailure(result.get("reason", "gate rejected/failed command"))
                return result
            self.quit.wait(0.02)
        raise PhysicalFailure("command result timeout: physical outcome UNKNOWN, no retry")

    def run_task(self, assignment):
        history = []
        anomaly = ""
        replans = int(self.options.get("max_replans", 2))
        target_baseline = {}
        try:
            for attempt in range(replans + 1):
                scene = self.observation()
                context = dict(
                    goal=assignment["task"]["goal"],
                    agent_id=self.ident,
                    scene=scene,
                    peer_states={a: x["data"] for a, x in self.peers.items()},
                    current_gate=self.guard(assignment["token"]),
                    safety=self.config["safety"],
                    search=self.options,
                    completed_actions=history,
                    anomaly=anomaly,
                    attempt=attempt,
                )
                actions = self.infer(context)
                self.guard(assignment["token"])
                target_baseline = {o["id"]: o["position"] for o in scene["objects"]}
                self.set_state("EXECUTING", f"executing cached batch of {len(actions)} actions")
                try:
                    for index, action in enumerate(actions):
                        self.guard(assignment["token"])
                        kind = action["kind"]
                        self.state["detail"] = f"action {index+1}/{len(actions)}: {kind}"
                        if kind == "move":
                            fresh = self.observation()
                            if "target" in action:
                                targets = matching_objects(fresh, object_id=action["target"])
                                if not targets:
                                    raise ObservationAnomaly(
                                        f'target missing: {action["target"]}; use bounded search'
                                    )
                                baseline = target_baseline.get(action["target"])
                                if baseline and math.dist(baseline, targets[0]["position"]) > 0.03:
                                    raise ObservationAnomaly("target moved since planning")
                            try:
                                resolved = resolve_move(action, fresh)
                            except ValueError as exc:
                                raise ObservationAnomaly(str(exc)) from exc
                            self.command(resolved)
                        elif kind == "gripper":
                            self.command(action)
                        elif kind == "observe":
                            self.observation(after_stamp=self.now())
                        elif kind == "verify":
                            fresh = self.observation(after_stamp=self.now())
                            ok, reason = verify_scene(action, fresh)
                            self.metric("visual_verification", success=ok, detail=reason)
                            if not ok:
                                raise ObservationAnomaly(reason)
                        elif kind == "search":
                            found = False
                            for view in action["viewpoints"]:
                                self.command(
                                    dict(
                                        kind="move",
                                        position=view,
                                        rpy=action["rpy"],
                                        speed=action["speed"],
                                    )
                                )
                                fresh = self.observation(after_stamp=self.now())
                                history.append(
                                    dict(
                                        action=dict(kind="search_view", position=view),
                                        status="SUCCEEDED",
                                    )
                                )
                                if matching_objects(fresh, label=action["label"]):
                                    found = True
                                    break
                            self.metric("search_result", success=found, detail=action["label"])
                            # New evidence invalidates the remaining batch; bounded replan once per discovery.
                            if not found:
                                raise RuntimeError("search budget exhausted; object absent")
                            raise ObservationAnomaly(
                                "search found object; plan from updated observation"
                            )
                        history.append(dict(action=action, status="SUCCEEDED"))
                    self.set_state(
                        "SUCCEEDED",
                        "batch complete; final visual check passed (semantic evaluation separate)",
                    )
                    return
                except ObservationAnomaly as exc:
                    anomaly = str(exc)
                    self.metric("replan_trigger", success=False, detail=anomaly)
                    if attempt == replans:
                        raise
            raise ObservationAnomaly("replan budget exhausted")
        except PhysicalFailure as exc:
            self.stop_pub.publish(Bool(data=True))
            self.set_state("FAILED", str(exc))
        except Exception as exc:
            self.set_state("FAILED", str(exc))
        finally:
            with self.lock:
                self.active = None

    def on_cooperative(self, msg):
        try:
            d = decode(msg)
            if d.get("agent_id") != self.ident:
                return
            self.guard(d["token"])
            if (
                self.permit.get("owner") != "both"
                or self.permit.get("mission_id") != d.get("mission_id")
                or self.permit.get("task_id") != d.get("task_id")
            ):
                return
            request_id = d["request_id"]
            with self.lock:
                if request_id in self.coop_seen or self.active:
                    return
                self.coop_seen.add(request_id)
                self.active = d
                self.state.update(
                    status="PLANNING",
                    token=d["token"],
                    mission_id=d["mission_id"],
                    task_id=d["task_id"],
                    detail="independent shared-object proposal",
                )
                self.worker = threading.Thread(target=self.cooperative_plan, args=(d,), daemon=True)
                self.worker.start()
        except (ValueError, KeyError, TypeError, PhysicalFailure) as exc:
            self.get_logger().warning(str(exc))

    def cooperative_plan(self, request):
        result = dict(
            schema=1,
            request_id=request["request_id"],
            agent_id=self.ident,
            token=request["token"],
            status="FAILED",
            reason="",
        )
        started = time.monotonic()
        try:
            scene = self.observation()
            context = dict(
                request,
                local_scene=scene,
                safety=self.config["safety"],
                cooperative=self.config.get("cooperative", {}),
            )
            plan = self.llm.generate(build_local_prompt(request, scene, self.config), context)
            validate_local_plan(plan, self.ident, self.config)
            self.guard(request["token"])
            result.update(status="SUCCEEDED", plan=plan)
            self.set_state("EXECUTING", "proposal submitted; waiting for shared coordinator")
        except Exception as exc:
            result["reason"] = str(exc)
            self.set_state("FAILED", str(exc))
        finally:
            self.metric(
                "llm_inference",
                latency_s=time.monotonic() - started,
                success=result["status"] == "SUCCEEDED",
                detail=result["reason"],
                mode="cooperative",
            )
            publish_json(self.coop_pub, result)
            with self.lock:
                self.active = None

    def close(self):
        self.quit.set()
        if self.active:
            self.stop_pub.publish(Bool(data=True))
        if self.worker:
            self.worker.join(timeout=2.0)


def main(args=None):
    run_node(AgentNode, args)
