"""역할: xArm SDK 단위 변환·비동기 구동·독립 telemetry와 무접속 시뮬레이터.

주요 인터페이스: make_backend(config), snapshot/arm/move/gripper/stop/close.
하드웨어 객체는 safety_node만 소유한다. SDK imports는 실제 backend 생성 때만 수행.
"""

import copy
import math
import threading
import time

from .safety import SafetyError, angle_distance, number, vector


def _strict(code, operation):
    # [변경] None/True/비정상 반환값을 성공으로 취급하지 않는다.
    if type(code) is not int or code != 0:
        raise SafetyError(f"{operation}: SDK returned {code!r}")


class DryRunBackend:
    """Time-based fake controller. No xArm import, socket or motor initialization."""

    def __init__(self, config):
        safety = config.get("safety", {})
        self.lock, self.closed = threading.RLock(), threading.Event()
        self.state = dict(
            position=list(
                vector(safety.get("initial_position", [0.3, 0.0, 0.35]), "initial_position")
            ),
            rpy=list(vector(safety.get("initial_rpy", [math.pi, 0.0, 0.0]), "initial_rpy")),
            joints=[0.0] * 6,
            gripper=850.0,
            gripper_status=0,
            gripper_error=0,
            stamp=time.monotonic(),
            gripper_stamp=time.monotonic(),
            connected=True,
            state=0,
            error=0,
            warn=0,
        )
        self.motion = None
        self.calls = []
        threading.Thread(target=self._run, daemon=True, name="dry-run-telemetry").start()

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.state)

    def arm(self):
        with self.lock:
            self.state["state"] = 0

    def move(self, action, limits):
        with self.lock:
            if self.state["state"] not in (0, 2):
                raise SafetyError("dry-run controller not idle")
            distance = math.dist(self.state["position"], action["position"])
            angle = angle_distance(self.state["rpy"], action["rpy"])
            duration = max(
                0.15,
                1.875 * distance / action["speed"],
                math.sqrt(5.78 * distance / action["accel"]),
                1.875 * angle / action["angular_speed"],
                math.sqrt(5.78 * angle / action["angular_accel"]),
            )
            self.motion = (time.monotonic(), duration, copy.deepcopy(self.state), dict(action))
            self.state["state"] = 1
            self.calls.append(("move", copy.deepcopy(action)))

    def gripper(self, action):
        with self.lock:
            duration = max(0.15, abs(self.state["gripper"] - action["position"]) / action["speed"])
            self.motion = (time.monotonic(), duration, copy.deepcopy(self.state), dict(action))
            self.state["gripper_status"] = 1
            self.calls.append(("gripper", dict(action)))

    def stop(self):
        with self.lock:
            self.motion = None
            self.state.update(state=4, gripper_status=0)
            self.calls.append(("stop",))

    def _run(self):
        while not self.closed.wait(0.02):
            with self.lock:
                now = time.monotonic()
                if self.motion:
                    start, duration, before, action = self.motion
                    u = min(1.0, max(0.0, (now - start) / duration))
                    fraction = u**3 * (10 - 15 * u + 6 * u**2)
                    if action["kind"] == "move":
                        self.state["position"] = [
                            a + fraction * (b - a)
                            for a, b in zip(before["position"], action["position"])
                        ]
                        self.state["rpy"] = [
                            math.atan2(
                                math.sin(
                                    a + fraction * math.atan2(math.sin(b - a), math.cos(b - a))
                                ),
                                math.cos(
                                    a + fraction * math.atan2(math.sin(b - a), math.cos(b - a))
                                ),
                            )
                            for a, b in zip(before["rpy"], action["rpy"])
                        ]
                    else:
                        self.state["gripper"] = before["gripper"] + fraction * (
                            action["position"] - before["gripper"]
                        )
                    if u >= 1.0:
                        self.motion = None
                        self.state.update(state=0, gripper_status=0)
                self.state.update(stamp=now, gripper_stamp=now)

    def close(self):
        self.closed.set()


class XArmBackend:
    """SDK 1.17.x adapter: mm at SDK boundary, radians throughout.

    report callback supplies TCP/joints/controller health independently of the
    command worker. A separate feedback worker polls gripper status/position.
    """

    def __init__(self, config, sdk_factory=None):
        if config.get("dry_run", True) is not False:
            raise SafetyError("hardware backend requires explicit dry_run:false")
        if sdk_factory is None:
            from xarm.wrapper import XArmAPI  # [변경] dry-run에서는 SDK import 자체 없음.

            sdk_factory = XArmAPI
        self.config = config
        self.lock, self.closed = threading.RLock(), threading.Event()
        self.state = dict(
            position=[0.0, 0.0, 0.0],
            rpy=[0.0, 0.0, 0.0],
            joints=[0.0] * 6,
            gripper=0.0,
            gripper_status=1,
            gripper_error=0,
            stamp=-math.inf,
            gripper_stamp=-math.inf,
            connected=False,
            state=4,
            error=0,
            warn=0,
        )
        self.arm_api = sdk_factory(
            config["robot_ip"],
            is_radian=True,
            enable_report=True,
            do_not_open=False,
            check_tcp_limit=True,
            check_joint_limit=True,
        )
        # No clean_error, state reset, motor or gripper enable on construction.
        self.arm_api.set_timeout(0.3)
        self.arm_api.register_report_callback(
            self._report,
            report_cartesian=True,
            report_joints=True,
            report_state=True,
            report_error_code=True,
            report_warn_code=True,
        )
        threading.Thread(
            target=self._gripper_feedback, daemon=True, name="gripper-feedback"
        ).start()

    def _report(self, report):
        try:
            cartesian = vector(report["cartesian"], "SDK Cartesian report", 6)
            joints = vector(list(report["joints"])[:6], "SDK joints", 6)
            with self.lock:
                self.state.update(
                    position=[v / 1000.0 for v in cartesian[:3]],
                    rpy=list(cartesian[3:]),
                    joints=list(joints),
                    state=int(report["state"]),
                    error=int(report["error_code"]),
                    warn=int(report["warn_code"]),
                    connected=bool(self.arm_api.connected),
                    stamp=time.monotonic(),
                )
        except (KeyError, TypeError, ValueError):
            # Malformed reports never refresh freshness; watchdog will stop.
            return

    def _gripper_feedback(self):
        while not self.closed.wait(0.05):
            try:
                code, position = self.arm_api.get_gripper_position()
                _strict(code, "get_gripper_position")
                position = number(position, "reported gripper position")
                code, status = self.arm_api.get_gripper_status()
                _strict(code, "get_gripper_status")
                code, error = self.arm_api.get_gripper_err_code()
                _strict(code, "get_gripper_err_code")
                with self.lock:
                    self.state.update(
                        gripper=position,
                        gripper_status=int(status) & 3,
                        gripper_error=int(error),
                        gripper_stamp=time.monotonic(),
                    )
            except Exception:
                continue

    def snapshot(self):
        with self.lock:
            state = copy.deepcopy(self.state)
        state["connected"] = bool(self.arm_api.connected)
        return state

    def arm(self):
        # [변경] operator arm만 실행. 기존 clean_error/clean_gripper_error 자동 해제 제거.
        state = self.snapshot()
        if state["error"] or state["warn"] or state.get("gripper_error"):
            raise SafetyError("controller/gripper fault must be cleared and inspected externally")
        for operation, args in [
            ("motion_enable", (True,)),
            ("set_mode", (0,)),
            ("set_state", (0,)),
            ("set_gripper_enable", (True,)),
        ]:
            _strict(getattr(self.arm_api, operation)(*args), operation)

    def move(self, action, limits):
        state = self.snapshot()
        distance = math.dist(state["position"], action["position"])
        rotation = angle_distance(state["rpy"], action["rpy"])
        speed, accel = action["speed"] * 1000.0, action["accel"] * 1000.0
        if rotation > limits.orientation_tolerance:
            # [변경] 검증되지 않은 회전 반경 변환을 추정하지 않는다. pour는 회전 전용 단계.
            if distance > limits.position_tolerance:
                raise SafetyError("hardware requires separate translation and rotation commands")
            if self.config.get("commissioning", {}).get("orientation_verified") is not True:
                raise SafetyError("rotation behavior must be commissioned for this firmware")
            speed = min(speed, action["angular_speed"])
            accel = min(accel, action["angular_accel"])
        # SDK silently clamps small values; reject rather than exceed requested caps.
        if speed < self.arm_api.tcp_speed_limit[0] or accel < self.arm_api.tcp_acc_limit[0]:
            raise SafetyError(
                "requested speed/acceleration below SDK minimum; commissioning required"
            )
        p, rpy = action["position"], action["rpy"]
        _strict(
            self.arm_api.set_position(
                x=p[0] * 1000.0,
                y=p[1] * 1000.0,
                z=p[2] * 1000.0,
                roll=rpy[0],
                pitch=rpy[1],
                yaw=rpy[2],
                speed=speed,
                mvacc=accel,
                is_radian=True,
                relative=False,
                radius=-1,
                motion_type=0,
                wait=False,
            ),
            "set_position",
        )

    def gripper(self, action):
        _strict(
            self.arm_api.set_gripper_position(
                action["position"],
                speed=action["speed"],
                wait=False,
                wait_motion=False,
                auto_enable=False,
            ),
            "set_gripper_position",
        )

    def stop(self):
        # [변경] SDK 오류 해제/재활성화 없이 정지. 독립 gripper motor도 disable.
        # Gripper de-energization may release a payload; physical support is required.
        errors = []
        for operation, argument in [("set_state", 4), ("set_gripper_enable", False)]:
            try:
                _strict(getattr(self.arm_api, operation)(argument), operation)
            except Exception as error:
                errors.append(str(error))
        if errors:
            raise SafetyError("; ".join(errors))

    def close(self):
        self.closed.set()
        self.arm_api.disconnect()


def make_backend(config):
    # [변경] dry_run 실제 제어 객체와 완전히 분리. False 리터럴만 hardware 선택.
    return XArmBackend(config) if config.get("dry_run", True) is False else DryRunBackend(config)
