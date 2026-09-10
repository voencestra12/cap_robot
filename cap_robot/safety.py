"""역할: ROS/LLM과 독립된 모든 구동 명령의 최종 안전 게이트.

주요 인터페이스: SafetyGate.submit/update_permit/update_agent/stop/arm/reset,
SafetyLimits.check_move; backend는 cached snapshot, move, gripper, stop을 제공.
좌표 m, RPY rad, 시간 monotonic(감시)/ROS seconds(메시지)로 분리한다.
"""

import copy
import math
import threading
import time


class SafetyError(ValueError):
    """A command cannot be authorized; never repaired by an LLM."""


def number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise SafetyError(f"{name}: finite number required")
    return float(value)


def vector(value, name, count=3):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise SafetyError(f"{name}: {count} numbers required")
    return tuple(number(v, name) for v in value)


def quaternion(rpy):
    r, p, y = (v / 2 for v in vector(rpy, "rpy"))
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def angle_distance(left, right):
    # [변경] Euler 성분 차이 대신 SO(3) 최단 회전을 사용해 ±π 경계를 처리한다.
    dot = abs(sum(a * b for a, b in zip(quaternion(left), quaternion(right))))
    return 2 * math.acos(min(1.0, max(0.0, dot)))


class SafetyLimits:
    def __init__(self, config):
        self.minimum = vector(config.get("min_xyz", [0.10, -0.4, 0.10]), "min_xyz")
        self.maximum = vector(config.get("max_xyz", [0.65, 0.4, 0.65]), "max_xyz")
        self.radius = number(config.get("tcp_radius", 0.01), "tcp_radius")
        if self.radius < 0 or any(
            lo + self.radius >= hi - self.radius for lo, hi in zip(self.minimum, self.maximum)
        ):
            raise SafetyError("invalid fence or TCP/tool envelope radius")
        defaults = dict(
            max_speed=0.08,
            max_accel=0.2,
            max_step=0.30,
            max_rotation_step=0.35,
            max_angular_speed=0.2,
            max_angular_accel=0.4,
            permit_timeout=1.5,
            command_timeout=20.0,
            state_timeout=0.5,
            command_max_age=1.0,
            agent_timeout=2.0,
            position_tolerance=0.002,
            orientation_tolerance=0.015,
            gripper_tolerance=8.0,
            rotation_radius_mm=1.0,
            max_start_skew=0.05,
        )
        for key, default in defaults.items():
            value = number(config.get(key, default), key)
            if value <= 0:
                raise SafetyError(f"{key} must be positive")
            setattr(self, key, value)
        # [변경] 통신 감시를 설정으로 사실상 무력화할 수 없도록 상한 유지.
        if self.permit_timeout > 1.5 or self.state_timeout > 1.0 or self.agent_timeout > 3.0:
            raise SafetyError("watchdog configuration exceeds hard maximum")
        if self.max_speed > 0.25 or self.max_accel > 1.0 or self.max_rotation_step > math.pi / 2:
            raise SafetyError("configured motion limit exceeds hard maximum")
        self.gripper_min = number(config.get("gripper_min", 0.0), "gripper_min")
        self.gripper_max = number(config.get("gripper_max", 850.0), "gripper_max")
        if not 0 <= self.gripper_min < self.gripper_max <= 850:
            raise SafetyError("gripper bounds must be within 0..850")

    def check_position(self, position):
        point = vector(position, "position")
        if any(
            not lo + self.radius <= v <= hi - self.radius
            for v, lo, hi in zip(point, self.minimum, self.maximum)
        ):
            raise SafetyError("TCP/tool envelope outside XYZ fence (including Z floor)")
        return point

    def check_move(self, action, state):
        # [변경] 시작점/끝점 + 볼록 펜스 + 선형 SDK 모드로 TCP 선분 전체를 제한.
        start = self.check_position(state["position"])
        end = self.check_position(action.get("position"))
        if math.dist(start, end) > self.max_step:
            raise SafetyError("move exceeds maximum segment length")
        rpy = vector(action.get("rpy"), "rpy")
        if any(abs(v) > math.pi + 1e-8 for v in rpy):
            raise SafetyError("RPY must be radians within [-pi,pi]")
        angle = angle_distance(state["rpy"], rpy)
        if angle > self.max_rotation_step + 1e-8:
            raise SafetyError("move exceeds maximum rotation step")
        out = dict(action, position=list(end), rpy=list(rpy))
        for field, maximum in [
            ("speed", self.max_speed),
            ("accel", self.max_accel),
            ("angular_speed", self.max_angular_speed),
            ("angular_accel", self.max_angular_accel),
        ]:
            value = number(action.get(field, maximum if field != "speed" else None), field)
            if not 0 < value <= maximum:
                raise SafetyError(f"{field} outside limit")
            out[field] = value
        return out

    def check_action(self, action, state):
        if not isinstance(action, dict):
            raise SafetyError("action must be an object")
        self.check_position(state["position"])
        if action.get("kind") == "move":
            return self.check_move(action, state)
        if action.get("kind") == "gripper":
            position = number(action.get("position"), "gripper position")
            speed = number(action.get("speed", 500), "gripper speed")
            if not self.gripper_min <= position <= self.gripper_max or not 100 <= speed <= 2000:
                raise SafetyError("gripper position/speed outside limit")
            return dict(action, position=position, speed=speed)
        raise SafetyError("only move and gripper are allowed at hardware gate")


class SafetyGate:
    """Single in-flight worker, persistent stop latches, at-most-once command IDs.

    No SDK call takes the gate lock: independent watchdog/stop stays responsive
    even when a vendor call stalls. A stop thread never enables or resets motion.
    """

    def __init__(
        self,
        config,
        backend,
        on_result=lambda result: None,
        clock=time.monotonic,
        ros_clock=time.time,
    ):
        self.config, self.backend, self.on_result = config, backend, on_result
        self.clock, self.ros_clock = clock, ros_clock
        self.agent_id = config.get("agent_id", "agent1")
        self.dry_run = config.get("dry_run", True) is True
        self.limits = SafetyLimits(config.get("safety", {}))
        self.lock = threading.RLock()
        self.armed, self.estopped, self.killed = self.dry_run, False, False
        self.reason = ""
        self.permit, self.permit_received, self.agents = {}, -math.inf, {}
        self.active, self.results = None, {}
        self.cancel = threading.Event()
        self.go = threading.Event()
        self.closed = threading.Event()
        self.previous_sample, self.previous_velocity = None, None
        self.monitor_thread = threading.Thread(
            target=self._monitor, daemon=True, name="safety-watchdog"
        )
        self.monitor_thread.start()

    def _state(self):
        state = self.backend.snapshot()
        if (
            not state.get("connected")
            or self.clock() - state.get("stamp", -math.inf) > self.limits.state_timeout
        ):
            raise SafetyError("controller telemetry disconnected or stale")
        if state.get("error", 0) != 0 or state.get("warn", 0) != 0:
            raise SafetyError("controller error/warning requires operator inspection")
        if state.get("gripper_error", 0) != 0:
            raise SafetyError("gripper fault requires operator inspection")
        if (
            self.active
            and self.clock() - state.get("gripper_stamp", -math.inf) > self.limits.state_timeout
        ):
            raise SafetyError("active command requires fresh gripper telemetry")
        vector(state.get("position"), "telemetry position")
        vector(state.get("rpy"), "telemetry rpy")
        return state

    def arm(self):
        with self.lock:
            if self.estopped or self.killed or self.active:
                raise SafetyError("cannot arm: stop latch or active command")
            flags = self.config.get("commissioning", {})
            if not self.dry_run and not all(
                flags.get(name) is True for name in ("calibration_verified", "workspace_verified")
            ):
                raise SafetyError("calibration/workspace commissioning not verified")
            state = self._state()
            if state["state"] == 1:
                raise SafetyError("cannot arm a controller already moving")
            self.limits.check_position(state["position"])
        # [변경] 오류 clear 없이 명시적 operator arm에서만 controller 활성화.
        try:
            self.backend.arm()
        except Exception:
            self.stop("controller arm failed; partial enable must stop")
            raise
        with self.lock:
            if self.estopped or self.killed:
                self._request_stop()
                raise SafetyError("stop arrived during arm")
            self.armed = True
            self.reason = ""

    def reset(self):
        with self.lock:
            if self.killed:
                raise SafetyError("kill latch requires process restart and hardware inspection")
            if self.active:
                raise SafetyError("cannot reset while worker is active")
            state = self._state()
            if state["state"] == 1:
                raise SafetyError("controller still moving")
            self.estopped, self.armed = False, False
            self.reason = "stop reset; explicit arm required"
            self.cancel.clear()

    def _request_stop(self):
        threading.Thread(target=self._stop_backend, daemon=True, name="hardware-stop").start()

    def _stop_backend(self):
        try:
            self.backend.stop()
        except Exception as error:
            with self.lock:
                self.reason += f"; hardware stop unconfirmed: {error}"

    def stop(self, reason="emergency stop", kill=False):
        with self.lock:
            first = not (self.estopped or self.killed)
            self.estopped = True
            self.killed = self.killed or kill
            self.armed = False
            self.reason = str(reason)
            self.cancel.set()
            self.go.set()
        if first or kill:
            self._request_stop()

    def update_permit(self, permit):
        if not isinstance(permit, dict) or not isinstance(permit.get("enabled"), bool):
            raise SafetyError("malformed permit")
        with self.lock:
            old = self.permit
            changed = any(
                permit.get(k) != old.get(k)
                for k in ("session", "token", "owner", "mission_id", "task_id", "cooperative")
            )
            revoke = self.active and (changed or not permit["enabled"])
            self.permit, self.permit_received = copy.deepcopy(permit), self.clock()
        if revoke:
            self.stop("permit revoked or replaced during command")

    def update_agent(self, state):
        if not isinstance(state, dict) or not state.get("agent_id"):
            return
        ident = state["agent_id"]
        with self.lock:
            previous = self.agents.get(ident)
            if (
                previous
                and previous[1].get("boot_id") == state.get("boot_id")
                and state.get("seq", -1) <= previous[1].get("seq", -1)
            ):
                return
            self.agents[ident] = (self.clock(), dict(state))

    def _authorize(self, command, state):
        if not self.armed or self.estopped or self.killed or self.cancel.is_set():
            raise SafetyError("gate unarmed or stopped")
        if state["state"] not in (0, 1, 2):
            raise SafetyError("controller is paused/stopped")
        p = self.permit
        if self.clock() - self.permit_received > self.limits.permit_timeout or not p.get("enabled"):
            raise SafetyError("permit missing, disabled or expired")
        if p.get("owner") not in (self.agent_id, "both"):
            raise SafetyError("another robot owns motion permit")
        for key in ("token", "mission_id", "task_id"):
            if not command.get(key) or command.get(key) != p.get(key):
                raise SafetyError(f"command {key} does not match permit")
        if command.get("agent_id") != self.agent_id:
            raise SafetyError("wrong command agent")
        required = ("agent1", "agent2") if p.get("owner") == "both" else (self.agent_id,)
        for ident in required:
            received, heartbeat = self.agents.get(ident, (-math.inf, {}))
            if self.clock() - received > self.limits.agent_timeout or heartbeat.get(
                "token"
            ) != p.get("token"):
                raise SafetyError(f"{ident} task heartbeat absent/expired")
            if heartbeat.get("status") == "FAILED":
                raise SafetyError(f"{ident} reported failure")
        if p.get("owner") == "both":
            if p.get("cooperative") is not True or command.get("phase") != "prepare":
                raise SafetyError("paired permit requires cooperative prepare")
            if not self.dry_run and (
                self.config.get("commissioning", {}).get("cooperative_verified") is not True
                or self.config.get("cooperative", {}).get("enabled") is not True
            ):
                raise SafetyError("physical cooperative control not commissioned")
        elif command.get("phase") == "prepare":
            raise SafetyError("prepare requires paired cooperative permit")

    def submit(self, command):
        """Accept at most once. False/failed validation also consumes its ID."""
        ident = command.get("command_id") if isinstance(command, dict) else None
        if not isinstance(ident, str) or not ident or len(ident) > 128:
            raise SafetyError("valid command_id required")
        with self.lock:
            if ident in self.results:
                result = self.results[ident]
                if result is not None:
                    self.on_result(copy.deepcopy(result))
                return False
            if len(self.results) >= 100000:
                raise SafetyError("command ID journal full; controlled restart required")
            self.results[ident] = None
            try:
                if self.active:
                    raise SafetyError("gate busy; command queues are forbidden")
                stamp = number(command.get("stamp"), "stamp")
                age = self.ros_clock() - stamp
                if not -0.1 <= age <= self.limits.command_max_age:
                    raise SafetyError("command timestamp stale or in future")
                state = self._state()
                self._authorize(command, state)
                action = self.limits.check_action(command.get("action"), state)
                if (
                    action["kind"] == "move"
                    and angle_distance(state["rpy"], action["rpy"])
                    > self.limits.orientation_tolerance
                ):
                    if (
                        not self.dry_run
                        and self.config.get("commissioning", {}).get("orientation_verified")
                        is not True
                    ):
                        raise SafetyError("orientation speed mapping not commissioned")
                if command.get("phase") == "prepare":
                    lead = number(command.get("execute_at"), "execute_at") - self.ros_clock()
                    if not 0.25 <= lead <= 3.0 or not command.get("group_id"):
                        raise SafetyError("paired command schedule outside .25..3s horizon")
                    if type(command.get("step_index")) is not int or command["step_index"] < 0:
                        raise SafetyError("paired step_index invalid")
                self.active = copy.deepcopy(command)
                self.active["action"] = action
                self.cancel.clear()
                self.go.clear()
                self.previous_sample, self.previous_velocity = None, None
                worker = threading.Thread(
                    target=self._execute,
                    args=(self.active,),
                    daemon=True,
                    name="single-command-worker",
                )
                worker.start()
                return True
            except Exception as error:
                result = self._result(command, False, str(error))
                self.results[ident] = result
        self.on_result(result)
        return False

    def cooperative_go(self, payload):
        with self.lock:
            c = self.active
            if not c or c.get("phase") != "prepare":
                return False
            for key in ("token", "group_id", "step_index", "execute_at"):
                if payload.get(key) != c.get(key):
                    return False
            ids = payload.get("command_ids", {})
            if set(ids) != {"agent1", "agent2"} or ids.get(self.agent_id) != c["command_id"]:
                return False
            if self.ros_clock() > c["execute_at"] - 0.1:
                self.stop("paired GO arrived too late")
                return False
            self.go.set()
            return True

    def _result(self, command, success, reason, started_at=None):
        return dict(
            schema=1,
            command_id=command.get("command_id", ""),
            token=command.get("token", ""),
            agent_id=self.agent_id,
            mission_id=command.get("mission_id", ""),
            task_id=command.get("task_id", ""),
            status="SUCCEEDED" if success else "FAILED",
            cooperative=command.get("phase") == "prepare",
            reason=reason,
            started_at=started_at,
            completed_at=self.ros_clock(),
        )

    def _execute(self, command):
        started_at = None
        success, reason = False, ""
        try:
            if command.get("phase") == "prepare":
                while not self.go.wait(0.01):
                    if self.ros_clock() >= command["execute_at"] - 0.1:
                        raise SafetyError("paired GO missing before deadline")
                    self._authorize(command, self._state())
                while self.ros_clock() < command["execute_at"]:
                    if self.cancel.wait(0.002):
                        raise SafetyError("paired command cancelled")
                    self._authorize(command, self._state())
                if self.ros_clock() - command["execute_at"] > self.limits.max_start_skew:
                    raise SafetyError("paired dispatch exceeded scheduling skew")
            state = self._state()
            with self.lock:
                self._authorize(command, state)
                action = self.limits.check_action(command["action"], state)
            started_at = self.ros_clock()
            start_mono = self.clock()
            # [변경] wait=False의 반환은 접수뿐이다. 별도 telemetry 완료 확인 전 성공 금지.
            if action["kind"] == "move":
                self.backend.move(action, self.limits)
            else:
                self.backend.gripper(action)
            stable, last_stamp = 0, None
            while True:
                if self.cancel.wait(0.02):
                    raise SafetyError(self.reason or "command cancelled")
                state = self._state()
                with self.lock:
                    self._authorize(command, state)
                if self.clock() - start_mono > self.limits.command_timeout:
                    raise SafetyError("controller completion timeout")
                if action["kind"] == "move":
                    complete = (
                        state["state"] in (0, 2)
                        and math.dist(state["position"], action["position"])
                        <= self.limits.position_tolerance
                        and angle_distance(state["rpy"], action["rpy"])
                        <= self.limits.orientation_tolerance
                    )
                    stamp = state["stamp"]
                else:
                    stamp = state.get("gripper_stamp", -math.inf)
                    if self.clock() - stamp > self.limits.state_timeout:
                        raise SafetyError("gripper feedback stale")
                    # Endpoint OR explicitly reported object contact for closing, never blind SDK success.
                    complete = (
                        state.get("gripper_error", 0) == 0
                        and state.get("gripper_status", 1) in (0, 2)
                        and (
                            abs(state["gripper"] - action["position"])
                            <= self.limits.gripper_tolerance
                            or (
                                state.get("gripper_status") == 2
                                and action["position"] < state["gripper"]
                            )
                        )
                    )
                if stamp != last_stamp and stamp > start_mono:
                    stable = stable + 1 if complete else 0
                    last_stamp = stamp
                if stable >= 3 and self.clock() - start_mono >= 0.10:
                    success, reason = (
                        True,
                        "controller state and measured endpoint/contact confirmed",
                    )
                    break
        except Exception as error:
            reason = str(error)
            self.stop(f"command failed: {reason}")
        finally:
            result = self._result(command, success, reason, started_at)
            with self.lock:
                self.results[command["command_id"]] = result
                self.active = None
            self.on_result(result)

    def _monitor(self):
        while not self.closed.wait(0.02):
            try:
                with self.lock:
                    armed, active = self.armed, copy.deepcopy(self.active)
                if not armed:
                    continue
                state = self._state()
                self.limits.check_position(state["position"])
                if state["state"] not in (0, 1, 2):
                    raise SafetyError("controller entered paused/stopped state")
                if active:
                    with self.lock:
                        self._authorize(active, state)
                    previous = self.previous_sample
                    if previous and state["stamp"] > previous["stamp"]:
                        dt = state["stamp"] - previous["stamp"]
                        linear = math.dist(state["position"], previous["position"]) / dt
                        angular = angle_distance(state["rpy"], previous["rpy"]) / dt
                        # Small absolute measurement allowance; limits stay independent of model output.
                        if (
                            linear > self.limits.max_speed + 0.015
                            or angular > self.limits.max_angular_speed + 0.05
                        ):
                            raise SafetyError("measured TCP linear/angular speed exceeded limit")
                        if self.previous_velocity and dt >= 0.015:
                            acceleration = abs(angular - self.previous_velocity[1]) / dt
                            if acceleration > self.limits.max_angular_accel + 0.5:
                                raise SafetyError("measured angular acceleration exceeded limit")
                        self.previous_velocity = (linear, angular)
                    self.previous_sample = state
            except Exception as error:
                self.stop(f"watchdog: {error}")

    def snapshot(self):
        raw = self.backend.snapshot()
        with self.lock:
            active = self.active or {}
            return dict(
                agent_id=self.agent_id,
                token=self.permit.get("token", ""),
                idle=self.active is None,
                armed=self.armed,
                latched=self.estopped or self.killed,
                estopped=self.estopped,
                killed=self.killed,
                reason=self.reason,
                position=list(raw.get("position", [])),
                rpy=list(raw.get("rpy", [])),
                joints=list(raw.get("joints", [])),
                dry_run=self.dry_run,
                telemetry_fresh=self.clock() - raw.get("stamp", -math.inf)
                <= self.limits.state_timeout,
                controller_state=raw.get("state"),
                gripper=raw.get("gripper"),
                prepared_group_id=active.get("group_id", ""),
                prepared_command_id=(
                    active.get("command_id", "") if active.get("phase") == "prepare" else ""
                ),
                prepared_execute_at=active.get("execute_at", 0.0),
                prepared_step_index=active.get("step_index", -1),
            )

    def close(self):
        self.stop("gate shutdown")
        self.closed.set()
        # [변경] 정지 요청을 보내기 전에 SDK 연결이 먼저 닫히는 shutdown 경합 방지.
        try:
            self.backend.stop()
        except Exception:
            pass  # Snapshot reason already records stop-confirmation failures.
        self.backend.close()
