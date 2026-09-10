"""역할: 순수 계획 검증·좌표 해석·관측 검증. 인터페이스: validate_* / resolve_move."""

from __future__ import annotations

import copy
import math
from pathlib import Path
from typing import Any


def load_prompt(name: str) -> str:
    from ament_index_python.packages import get_package_share_directory

    return (Path(get_package_share_directory("cap_robot")) / "prompts" / name).read_text(
        encoding="utf-8"
    )


def gate_consensus(
    gates: dict,
    agents: list[str],
    token: str,
    now: float,
    max_age: float,
    *,
    require_token: bool = True,
) -> bool:
    """[변경] 공용 토큰을 양쪽 안전 게이트가 실제 수신했을 때만 배정할 수 있다."""
    for agent in agents:
        record = gates.get(agent)
        if not record or now - record["received"] > max_age:
            return False
        state = record["data"]
        if (
            state.get("idle") is not True
            or state.get("armed") is not True
            or state.get("latched") is not False
        ):
            return False
        if require_token and state.get("token") != token:
            return False
    return True


def number(value: Any, name: str, low: float, high: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or not low <= result <= high:
        raise ValueError(f"{name} must be finite and in [{low}, {high}]")
    return result


def vector(value: Any, name: str, bound: float = 100.0) -> list[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must contain three numbers")
    return [number(item, name, -bound, bound) for item in value]


def text(value: Any, name: str, limit: int = 2048) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise ValueError(f"{name} must be nonempty text <= {limit} characters")
    return value.strip()


def only_keys(data: dict, allowed: set[str], name: str):
    if set(data) - allowed:
        raise ValueError(f"{name} has unsupported fields: {sorted(set(data) - allowed)}")


def validate_task_graph(plan: dict, agents: list[str], max_tasks: int = 64) -> list[dict]:
    """[변경] 레시피·작업 순서를 삽입하지 않고 LLM DAG 자체를 엄격히 검증한다."""
    if not isinstance(plan, dict):
        raise ValueError("mission plan must be an object")
    only_keys(plan, {"tasks", "reason"}, "mission plan")
    tasks = plan.get("tasks")
    if not isinstance(tasks, list) or not 1 <= len(tasks) <= max_tasks:
        raise ValueError(f"tasks must contain 1..{max_tasks} tasks")
    result, known = [], set()
    for task in tasks:
        if not isinstance(task, dict):
            raise ValueError("every task must be an object")
        only_keys(task, {"id", "agent", "goal", "depends_on", "type"}, "task")
        item = copy.deepcopy(task)
        item["id"] = text(task.get("id"), "task id", 128)
        item["goal"] = text(task.get("goal"), "task goal")
        item["type"] = task.get("type", "single")
        if item["id"] in known:
            raise ValueError("duplicate task id")
        known.add(item["id"])
        if item["type"] not in ("single", "cooperative"):
            raise ValueError("task type must be single or cooperative")
        valid_agents = ["both"] if item["type"] == "cooperative" else agents
        if item.get("agent") not in valid_agents:
            raise ValueError("task agent is not a configured participant")
        deps = task.get("depends_on")
        if not isinstance(deps, list) or any(not isinstance(x, str) for x in deps):
            raise ValueError("depends_on must be an array of task IDs")
        if len(set(deps)) != len(deps) or item["id"] in deps:
            raise ValueError("duplicate or self dependency")
        result.append(item)
    if any(set(t["depends_on"]) - known for t in result):
        raise ValueError("unknown dependency")
    ordered, completed = [], set()
    remaining = result[:]
    while remaining:
        ready = [t for t in remaining if set(t["depends_on"]) <= completed]
        if not ready:
            raise ValueError("cyclic dependency graph")
        for task in ready:
            ordered.append(task)
            completed.add(task["id"])
            remaining.remove(task)
    return ordered


def validate_action_plan(plan: dict, config: dict) -> list[dict]:
    if not isinstance(plan, dict):
        raise ValueError("action plan must be an object")
    only_keys(plan, {"actions", "reason"}, "action plan")
    actions = plan.get("actions")
    if not isinstance(actions, list) or not 1 <= len(actions) <= 64:
        raise ValueError("actions must contain 1..64 entries")
    result = []
    planning = config.get("planning", {})
    for action in actions:
        if not isinstance(action, dict):
            raise ValueError("action must be an object")
        action = copy.deepcopy(action)
        kind = action.get("kind")
        if kind == "move":
            only_keys(
                action,
                {
                    "kind",
                    "position",
                    "target",
                    "offset",
                    "rpy",
                    "speed",
                    "angular_speed",
                    "angular_accel",
                },
                "move",
            )
            if ("position" in action) == ("target" in action):
                raise ValueError("move requires exactly position or target")
            if "position" in action:
                action["position"] = vector(action["position"], "position")
                if "offset" in action:
                    raise ValueError("offset requires an exact object target")
            else:
                action["target"] = text(action["target"], "target", 128)
                action["offset"] = vector(action.get("offset", [0, 0, 0]), "offset", 1)
            action["rpy"] = vector(action.get("rpy"), "rpy", 2 * math.pi)
            action["speed"] = number(action.get("speed"), "speed", 0.001, 0.2)
            for key, maximum in [("angular_speed", 0.5), ("angular_accel", 1.0)]:
                if key in action:
                    action[key] = number(action[key], key, 0.001, maximum)
        elif kind == "gripper":
            only_keys(action, {"kind", "position", "speed"}, "gripper")
            action["position"] = number(action.get("position"), "gripper position", 0, 850)
            action["speed"] = number(action.get("speed", 500), "gripper speed", 100, 2000)
        elif kind == "observe":
            only_keys(action, {"kind"}, "observe")
        elif kind == "search":
            only_keys(action, {"kind", "label", "viewpoints", "rpy", "speed"}, "search")
            action["label"] = text(action.get("label"), "search label", 128)
            views = action.get("viewpoints")
            max_views = int(planning.get("search_max_views", 4))
            if not isinstance(views, list) or not 1 <= len(views) <= max_views:
                raise ValueError("search exceeds configured viewpoint budget")
            low = vector(planning.get("search_min", [0.15, -0.3, 0.15]), "search_min")
            high = vector(planning.get("search_max", [0.6, 0.3, 0.6]), "search_max")
            action["viewpoints"] = [vector(v, "viewpoint") for v in views]
            for view in action["viewpoints"]:
                if any(not a <= x <= b for a, x, b in zip(low, view, high)):
                    raise ValueError("search viewpoint outside configured search volume")
            if len({tuple(v) for v in action["viewpoints"]}) != len(views):
                raise ValueError("search viewpoints must be distinct")
            action["rpy"] = vector(action.get("rpy"), "search rpy", 2 * math.pi)
            action["speed"] = number(action.get("speed"), "search speed", 0.001, 0.08)
        elif kind == "verify":
            only_keys(
                action, {"kind", "object_id", "label", "expected_position", "tolerance"}, "verify"
            )
            if ("object_id" in action) == ("label" in action):
                raise ValueError("verify requires exactly object_id or label")
            key = "object_id" if "object_id" in action else "label"
            action[key] = text(action[key], key, 128)
            if "expected_position" in action:
                action["expected_position"] = vector(
                    action["expected_position"], "expected_position"
                )
                action["tolerance"] = number(
                    action.get("tolerance", 0.025), "tolerance", 0.001, 0.1
                )
            elif "tolerance" in action:
                raise ValueError("tolerance requires expected_position")
        else:
            raise ValueError(f"unsupported action kind: {kind}")
        result.append(action)
    if result[-1]["kind"] != "verify":
        raise ValueError("the final action must verify fresh visual evidence")
    return result


def matching_objects(scene: dict, *, object_id=None, label=None) -> list[dict]:
    return [
        o
        for o in scene.get("objects", [])
        if (o.get("id") == object_id if object_id is not None else o.get("label") == label)
    ]


def resolve_move(action: dict, scene: dict) -> dict:
    """[변경] 실행 직전 최신 관측에서만 target을 해석하고 절대 base 좌표로 게이트에 전달."""
    result = copy.deepcopy(action)
    if "target" in action:
        matches = matching_objects(scene, object_id=action["target"])
        if len(matches) != 1:
            raise ValueError(f'exact target missing or ambiguous: {action["target"]}')
        result["position"] = [a + b for a, b in zip(matches[0]["position"], action["offset"])]
        result.pop("target")
        result.pop("offset")
    return result


def verify_scene(action: dict, scene: dict) -> tuple[bool, str]:
    matches = matching_objects(scene, object_id=action.get("object_id"), label=action.get("label"))
    if not matches:
        return False, "requested object is not visible"
    if "expected_position" in action:
        passed = any(
            math.dist(o["position"], action["expected_position"]) <= action["tolerance"]
            for o in matches
        )
        return passed, "visual position check; grasp/contact/liquid success is not measured"
    return True, "object presence proxy only; semantic task success is not measured"
