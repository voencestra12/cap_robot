"""역할: ROS/LLM 경계의 엄격한 JSON 검증. 인터페이스: loads/dumps/validate_*.

# [변경] 임의 dict/NaN/무제한 메시지 대신 버전과 단위를 명시한 공통 프로토콜.
위치는 m, 각도는 rad, gripper position은 xArm 0..850 장치 단위이다.
"""

import json
import math

SCHEMA = 1
MAX_BYTES = 131072


def require_text(value, name="text", max_length=1024):
    if not isinstance(value, str) or not value.strip() or len(value) > max_length:
        raise ValueError(f"{name}: nonempty text <= {max_length} required")
    return value


def number(value, name="number"):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{name}: finite number required")
    return float(value)


def finite_vector(value, n=3, name="vector"):
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise ValueError(f"{name}: {n} numbers required")
    return [number(x, name) for x in value]


def dumps(data):
    try:
        text = json.dumps(data, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    except (TypeError, ValueError, OverflowError, RecursionError) as exc:
        raise ValueError(f"invalid JSON: {exc}") from exc
    if len(text.encode("utf-8")) > MAX_BYTES:
        raise ValueError("message too large")
    return text


def _pairs(pairs):
    out = {}
    for k, v in pairs:
        if k in out:
            raise ValueError(f"duplicate JSON key: {k}")
        out[k] = v
    return out


def loads(text):
    if not isinstance(text, str) or len(text.encode("utf-8")) > MAX_BYTES:
        raise ValueError("invalid message size/type")
    try:
        data = json.loads(
            text,
            object_pairs_hook=_pairs,
            parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)),
        )
    except (ValueError, RecursionError) as exc:
        raise ValueError(f"invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("JSON object required")
    # Overflow exponents (1e999) also need finite validation.
    dumps(data)
    return data


def envelope(data):
    if (
        not isinstance(data, dict)
        or type(data.get("schema")) is not int
        or data["schema"] != SCHEMA
    ):
        raise ValueError("schema=1 required")
    return data


def validate_command(data):
    envelope(data)
    for key in ("command_id", "token", "mission_id", "task_id", "agent_id"):
        require_text(data.get(key), key, 128)
    number(data.get("stamp"), "stamp")
    a = data.get("action")
    if not isinstance(a, dict):
        raise ValueError("action object required")
    if a.get("kind") == "move":
        finite_vector(a.get("position"), 3, "position")
        finite_vector(a.get("rpy"), 3, "rpy")
        if number(a.get("speed"), "speed") <= 0:
            raise ValueError("positive speed required")
        for field in ("angular_speed", "angular_accel"):
            if field in a and number(a[field], field) <= 0:
                raise ValueError(f"positive {field} required")
        allowed = {"kind", "position", "rpy", "speed", "angular_speed", "angular_accel"}
    elif a.get("kind") == "gripper":
        number(a.get("position"), "gripper position")
        if number(a.get("speed"), "gripper speed") <= 0:
            raise ValueError("positive gripper speed required")
        allowed = {"kind", "position", "speed"}
    else:
        raise ValueError("only move/gripper reach the hardware gate")
    if set(a) - allowed:
        raise ValueError(f"unknown action fields: {set(a) - allowed}")
    if "group_id" in data:
        require_text(data["group_id"], "group_id", 128)
    if "execute_at" in data:
        number(data["execute_at"], "execute_at")
    return data


def validate_observations(data):
    envelope(data)
    require_text(data.get("agent_id"), "agent_id", 128)
    require_text(data.get("frame_id"), "frame_id", 128)
    number(data.get("stamp"), "stamp")
    if type(data.get("seq")) is not int or data["seq"] < 0 or type(data.get("valid")) is not bool:
        raise ValueError("invalid observation sequence/valid")
    objects = data.get("objects")
    if not isinstance(objects, list) or len(objects) > 100:
        raise ValueError("objects must be bounded array")
    ids = set()
    for obj in objects:
        if not isinstance(obj, dict):
            raise ValueError("object must be dict")
        oid = require_text(obj.get("id"), "object id", 128)
        require_text(obj.get("label"), "label", 128)
        finite_vector(obj.get("position"), 3, "object position")
        if oid in ids or not 0 <= number(obj.get("confidence"), "confidence") <= 1:
            raise ValueError("duplicate id/invalid confidence")
        ids.add(oid)
    if not data["valid"] and objects:
        raise ValueError("invalid observation must clear objects")
    return data
