"""역할: 입력·비전 핵심 회귀 시험. 인터페이스: pytest, 장치 연결 없음."""

import pytest
import numpy as np
from cap_robot.protocol import loads, validate_command, validate_observations
from cap_robot.perception import deproject, Tracker, transform


@pytest.mark.parametrize(
    "text", ["[]", '{"x":NaN}', '{"x":1e999}', '{"x":1,"x":2}', '{"x":Infinity}']
)
def test_hostile_json(text):
    with pytest.raises(ValueError):
        loads(text)


def test_gate_protocol_rejects_sdk_escape():
    c = dict(
        schema=1,
        command_id="c",
        token="t",
        mission_id="m",
        task_id="s",
        agent_id="agent1",
        stamp=1,
        action=dict(kind="set_servo_angle", position=[0] * 6),
    )
    with pytest.raises(ValueError):
        validate_command(c)


def test_depth_units_match_and_missing_depth_rejected():
    k = [100.0, 0, 2.0, 0, 100.0, 2.0, 0, 0, 1.0]
    assert deproject(np.full((5, 5), 1000, dtype=np.uint16), "16UC1", 2, 2, k) == [0.0, 0.0, 1.0]
    assert deproject(np.ones((5, 5)), "32FC1", 2, 2, k) == [0.0, 0.0, 1.0]
    with pytest.raises(ValueError):
        deproject(np.zeros((5, 5)), "32FC1", 2, 2, k)


def test_two_bread_instances_and_loss():
    tracker = Tracker("a", max_distance=0.1, ttl=1.0)
    a = dict(label="bread", position=[0.1, 0, 0.1], confidence=0.9)
    b = dict(label="bread", position=[0.3, 0, 0.1], confidence=0.9)
    first = tracker.update([a, b], 0.0)
    second = tracker.update([b, a], 0.1)
    assert first[0]["id"] == second[1]["id"]
    assert len({x["id"] for x in first}) == 2
    assert tracker.update([], 2.0) == []
    assert not tracker.items


def test_transform_rejects_bad_quaternion():
    with pytest.raises(ValueError):
        transform([0, 0, 0], [0, 0, 0], [0, 0, 0, 0])
    assert transform([0.1, 0, 0], [0.2, 0, 0], [0, 0, 0, 1]) == pytest.approx([0.3, 0, 0])


def test_original_red_handles_can_be_fused_with_yolo():
    cv2 = pytest.importorskip("cv2")
    from types import SimpleNamespace
    from cap_robot.perception_node import PerceptionNode

    fake = SimpleNamespace(cv2=cv2, backend="yolo", config={"detect_red_handles": True}, models=[])
    frame = np.zeros((100, 180, 3), dtype=np.uint8)
    cv2.circle(frame, (40, 50), 12, (0, 0, 255), -1)
    cv2.circle(frame, (130, 50), 12, (0, 0, 255), -1)
    detections = PerceptionNode.detections(fake, frame)
    assert len(detections) == 2
    assert all(d[0] == "basket_handle" for d in detections)
