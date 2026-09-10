"""역할: 두 독립 LLM의 공동 운반 계획을 검증하고 유한한 동기 단계로 변환.

인터페이스: build_local_prompt, validate_local_plan, pair_plans, transform_point.
ROS/SDK 의존성 없음. 모든 위치는 m, 각도는 rad; T_world_base는 4x4 행렬.
"""

import copy
import math


PHASES = ("approach", "grasp", "carry", "release", "retreat", "verify")


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name}: finite number required")
    return float(value)


def _vector(value, name):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name}: three coordinates required")
    return [_number(v, name) for v in value]


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def transform_point(matrix, point):
    """Apply validated rigid world<-base transform without third-party dependencies."""
    validate_transform(matrix)
    point = _vector(point, "point")
    return [sum(matrix[r][c] * point[c] for c in range(3)) + matrix[r][3] for r in range(3)]


def validate_transform(matrix):
    if (
        not isinstance(matrix, list)
        or len(matrix) != 4
        or any(not isinstance(r, list) or len(r) != 4 for r in matrix)
    ):
        raise ValueError("T_world_base must be a 4x4 rigid matrix")
    for row in matrix:
        for v in row:
            _number(v, "transform")
    if any(abs(matrix[3][i] - (1 if i == 3 else 0)) > 1e-6 for i in range(4)):
        raise ValueError("invalid homogeneous transform")
    for i in range(3):
        for j in range(3):
            dot = sum(matrix[r][i] * matrix[r][j] for r in range(3))
            if abs(dot - (1 if i == j else 0)) > 1e-5:
                raise ValueError("rotation must be orthonormal")
    a, b, c = matrix[:3]
    determinant = (
        a[0] * (b[1] * c[2] - b[2] * c[1])
        - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0])
    )
    if abs(determinant - 1) > 1e-5:
        raise ValueError("reflection is not a calibrated rigid transform")


def quaternion_matrix(translation, quaternion):
    """Convert a TF translation and quaternion (x,y,z,w) to world<-base."""
    t = _vector(list(translation), "translation")
    if len(quaternion) != 4:
        raise ValueError("quaternion requires four values")
    x, y, z, w = [_number(v, "quaternion") for v in quaternion]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError("zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t[0]],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t[1]],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _orientation_same(a, b, tolerance=1e-4):
    return all(abs(math.atan2(math.sin(x - y), math.cos(x - y))) <= tolerance for x, y in zip(a, b))


def build_local_prompt(request, local_scene, config):
    """Prompt contract used INSIDE each robot's independent local LLM node."""
    coop = config.get("cooperative", {})
    # [변경] 고정 샌드위치/커피 레시피 대신 공유 목표와 각 로봇의 관측으로 계획한다.
    return (
        "You are one of two independent robot planners carrying ONE shared object. "
        "Return only JSON: {steps:[{phase,action}],reason:string}. "
        "Both robots independently receive the same goal and scene pair; agree on the "
        "same ordered phase sequence using the shared task description. "
        "Use phases approach, grasp, carry, release, retreat, verify in this order; "
        "exactly one grasp and release, at least one carry, and final verify. "
        "Each approach/carry/retreat action is {kind:'move',position:[x,y,z],rpy:[r,p,y],speed:number}; "
        "positions and rpy are ALWAYS YOUR OWN base frame, metres/radians. "
        "Use provided T_world_base and peer transform for shared-world comparisons. "
        "grasp/release are {kind:'gripper',position:number,speed:number}; obey the configured "
        "gripper open and closed positions. verify is {kind:'verify',object_id:exact_local_id,"
        "expected_position:[x,y,z],tolerance:number} using a visible local object. "
        "At grasp both arms must hold the same rigid object at distinct feasible grasp points. "
        "During carry both TCPs must have IDENTICAL displacement in the shared world frame "
        "and each TCP's rpy MUST stay fixed. Payload rotation/pouring while jointly held "
        "is unsupported; do not plan it. A single-agent task may tilt a vessel separately. "
        "Carry speeds must be identical and <= " + str(coop.get("max_speed", 0.02)) + " m/s. "
        "Keep approach and retreat separated using the commissioned geometry. "
        "Never invent visible objects, calibration, hardware, or grasp/contact evidence. "
        "Return {error:string} if a physically feasible shared plan cannot be produced. "
        "No hardcoded task recipe; plan only the supplied task. Maximum 64 steps."
    )


def validate_local_plan(plan, agent_id, config):
    if not isinstance(plan, dict) or not isinstance(plan.get("steps"), list):
        raise ValueError("cooperative plan.steps list required")
    steps = plan["steps"]
    if not 4 <= len(steps) <= 64:
        raise ValueError("cooperative plan requires 4..64 steps")
    safety, coop = config.get("safety", {}), config.get("cooperative", {})
    speed_limit = min(float(safety.get("max_speed", 0.08)), float(coop.get("max_speed", 0.02)))
    order, counts = -1, {phase: 0 for phase in PHASES}
    for step in steps:
        if (
            not isinstance(step, dict)
            or step.get("phase") not in PHASES
            or not isinstance(step.get("action"), dict)
        ):
            raise ValueError("invalid cooperative phase/action")
        phase, action = step["phase"], step["action"]
        phase_index = PHASES.index(phase)
        if phase_index < order:
            raise ValueError("cooperative phases must preserve grasp/carry/release order")
        order, counts[phase] = phase_index, counts[phase] + 1
        expected_kind = (
            "gripper"
            if phase in ("grasp", "release")
            else "verify" if phase == "verify" else "move"
        )
        if action.get("kind") != expected_kind:
            raise ValueError(f"{phase} requires {expected_kind}")
        if expected_kind == "move":
            pos = _vector(action.get("position"), "position")
            _vector(action.get("rpy"), "rpy")
            speed = _number(action.get("speed"), "speed")
            if not 0 < speed <= speed_limit:
                raise ValueError("cooperative speed outside commissioned bound")
            if "min_xyz" in safety and "max_xyz" in safety:
                radius = float(safety.get("tcp_radius", 0.01))
                if any(
                    pos[i] < safety["min_xyz"][i] + radius or pos[i] > safety["max_xyz"][i] - radius
                    for i in range(3)
                ):
                    raise ValueError("cooperative endpoint outside local software fence")
        elif expected_kind == "gripper":
            position = _number(action.get("position"), "gripper.position")
            if not safety.get("gripper_min", 0) <= position <= safety.get("gripper_max", 850):
                raise ValueError("gripper limit violation")
            if not 100 <= _number(action.get("speed"), "gripper.speed") <= 2000:
                raise ValueError("gripper speed limit violation")
            # [변경] LLM이 grasp 단계에서 여는 동작을 만들면 물체 결합을 가정하지 않는다.
            expected = (
                coop.get("grasp_position", 0.0)
                if phase == "grasp"
                else coop.get("release_position", 850.0)
            )
            if abs(position - float(expected)) > float(coop.get("gripper_position_tolerance", 5.0)):
                raise ValueError(f"{phase} gripper position does not match commissioning")
        else:
            if not isinstance(action.get("object_id"), str) or not action["object_id"]:
                raise ValueError("verify requires an exact local object id")
            _vector(action.get("expected_position"), "verify.expected_position")
            if not 0 < _number(action.get("tolerance"), "verify.tolerance") <= 0.05:
                raise ValueError("verify tolerance must be 0..0.05 m")
    if (
        counts["grasp"] != 1
        or counts["release"] != 1
        or not counts["carry"]
        or counts["verify"] != 1
        or steps[-1]["phase"] != "verify"
    ):
        raise ValueError("require one grasp, carry, one release and final verify")
    return copy.deepcopy(plan)


def pair_plans(plans, transforms, starts, configs, cooperative_config):
    """Validate matching independent proposals; subdivide rigid translation only.

    Returns [{phase,actions:{agent:action},held:bool,relative:[world delta]|None}].
    Different proposals fail closed; no robot is moved to reconcile a disagreement.
    """
    agents = sorted(plans)
    if len(agents) != 2:
        raise ValueError("exactly two robot proposals required")
    validated = {a: validate_local_plan(plans[a], a, configs[a]) for a in agents}
    sequences = [[s["phase"] for s in validated[a]["steps"]] for a in agents]
    if sequences[0] != sequences[1]:
        raise ValueError("independent plans disagree on synchronized phases")
    for a in agents:
        validate_transform(transforms[a])
    positions = {a: _vector(starts[a]["position"], "start.position") for a in agents}
    orientations = {a: _vector(starts[a]["rpy"], "start.rpy") for a in agents}
    max_step = _number(
        cooperative_config.get("max_translation_step", 0.015), "max_translation_step"
    )
    tolerance = _number(cooperative_config.get("relative_tolerance", 0.006), "relative_tolerance")
    if not 0 < max_step <= 0.03 or not 0 < tolerance <= 0.02:
        raise ValueError("invalid commissioned shared-payload bounds")
    result, held, relative = [], False, None
    for index, phase in enumerate(sequences[0]):
        actions = {a: validated[a]["steps"][index]["action"] for a in agents}
        if phase == "grasp":
            held = True
            world = {a: transform_point(transforms[a], positions[a]) for a in agents}
            relative = [world[agents[1]][i] - world[agents[0]][i] for i in range(3)]
            if distance(world[agents[0]], world[agents[1]]) < float(
                cooperative_config.get("min_tcp_separation", 0.08)
            ):
                raise ValueError("shared grasp TCP separation too small")
        count = 1
        if phase == "carry":
            for a in agents:
                if not _orientation_same(orientations[a], actions[a]["rpy"]):
                    raise ValueError("rotation while shared payload is held is unsupported")
            worlds = {a: transform_point(transforms[a], actions[a]["position"]) for a in agents}
            target_relative = [worlds[agents[1]][i] - worlds[agents[0]][i] for i in range(3)]
            # [변경] 서로 다른 base 좌표의 수치를 비교하지 않고 보정된 world에서 강체 변위를 확인.
            if distance(relative, target_relative) > tolerance:
                raise ValueError("independent plans violate rigid-payload relative displacement")
            if abs(actions[agents[0]]["speed"] - actions[agents[1]]["speed"]) > 1e-6:
                raise ValueError("shared carry speeds must match")
            count = max(
                1,
                math.ceil(
                    max(distance(positions[a], actions[a]["position"]) for a in agents) / max_step
                ),
            )
            if count > 128:
                raise ValueError("shared translation exceeds bounded segment budget")
        for part in range(1, count + 1):
            segment = copy.deepcopy(actions)
            if phase == "carry":
                for a in agents:
                    segment[a]["position"] = [
                        positions[a][i]
                        + (actions[a]["position"][i] - positions[a][i]) * part / count
                        for i in range(3)
                    ]
            result.append(
                {
                    "phase": phase,
                    "actions": segment,
                    "held": held,
                    "relative": copy.deepcopy(relative),
                }
            )
        if actions[agents[0]]["kind"] == "move":
            positions = {a: list(actions[a]["position"]) for a in agents}
            orientations = {a: list(actions[a]["rpy"]) for a in agents}
        if phase == "release":
            held, relative = False, None
    if len(result) > 512:
        raise ValueError("shared trajectory exceeds total segment budget")
    return result


def prepared_pair(states, commands):
    """Pure two-gate barrier predicate; never treat command acceptance as completion."""
    return all(
        a in states
        and states[a].get("prepared_group_id") == c["group_id"]
        and states[a].get("prepared_command_id") == c["command_id"]
        and states[a].get("prepared_step_index") == c["step_index"]
        and states[a].get("prepared_execute_at") == c["execute_at"]
        and states[a].get("token") == c["token"]
        for a, c in commands.items()
    )
