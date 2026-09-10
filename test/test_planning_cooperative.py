"""역할: LLM 출력과 협업 합의 검증. 인터페이스: pytest; 외부 LLM 호출 없음."""

import copy
import math
import pytest
from cap_robot.planning import (
    validate_task_graph,
    validate_action_plan,
    resolve_move,
    gate_consensus,
)
from cap_robot.cooperative import pair_plans, prepared_pair, validate_transform


def test_dependency_order_not_input_order():
    tasks = [
        dict(id="top", agent="agent1", goal="place top", depends_on=["middle"]),
        dict(id="base", agent="agent2", goal="place base", depends_on=[]),
        dict(id="middle", agent="agent1", goal="place middle", depends_on=["base"]),
    ]
    assert [x["id"] for x in validate_task_graph(dict(tasks=tasks), ["agent1", "agent2"])] == [
        "base",
        "middle",
        "top",
    ]


@pytest.mark.parametrize("deps", [["a"], ["missing"]])
def test_invalid_dag_rejected(deps):
    with pytest.raises(ValueError):
        validate_task_graph(
            dict(tasks=[dict(id="a", agent="agent1", goal="x", depends_on=deps)]),
            ["agent1", "agent2"],
        )


def test_search_and_final_verification_enforced():
    with pytest.raises(ValueError):
        validate_action_plan(dict(actions=[dict(kind="gripper", position=850, speed=500)]), {})
    search = dict(
        kind="search", label="bread", viewpoints=[[5, 0, 0]], rpy=[math.pi, 0, 0], speed=0.01
    )
    with pytest.raises(ValueError):
        validate_action_plan(dict(actions=[search, dict(kind="verify", label="bread")]), {})


def test_target_requires_exact_id():
    with pytest.raises(ValueError):
        resolve_move(
            dict(target="bread", offset=[0, 0, 0]),
            dict(objects=[dict(id="bread_1", position=[0, 0, 0])]),
        )


def test_both_gate_ack_required():
    s = dict(idle=True, armed=True, latched=False, token="t")
    gates = {a: dict(received=1.0, data=dict(s)) for a in ["agent1", "agent2"]}
    assert gate_consensus(gates, ["agent1", "agent2"], "t", 1.1, 1.0)
    gates["agent2"]["data"]["token"] = "old"
    assert not gate_consensus(gates, ["agent1", "agent2"], "t", 1.1, 1.0)


def shared_inputs():
    agents = ["agent1", "agent2"]
    ident = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    transforms = {a: copy.deepcopy(ident) for a in agents}
    transforms["agent2"][1][3] = 0.2
    starts = {a: dict(position=[0.3, 0.0, 0.35], rpy=[math.pi, 0.0, 0.0]) for a in agents}
    configs = {a: dict(safety={}, cooperative={}) for a in agents}
    plan = dict(
        steps=[
            dict(phase="grasp", action=dict(kind="gripper", position=0.0, speed=500.0)),
            dict(
                phase="carry",
                action=dict(
                    kind="move", position=[0.33, 0.0, 0.35], rpy=[math.pi, 0.0, 0.0], speed=0.02
                ),
            ),
            dict(phase="release", action=dict(kind="gripper", position=850.0, speed=500.0)),
            dict(
                phase="verify",
                action=dict(
                    kind="verify",
                    object_id="handle",
                    expected_position=[0.33, 0.0, 0.35],
                    tolerance=0.02,
                ),
            ),
        ]
    )
    return {a: copy.deepcopy(plan) for a in agents}, transforms, starts, configs, {}


def test_world_relative_translation_subdivided():
    steps = pair_plans(*shared_inputs())
    carries = [s for s in steps if s["phase"] == "carry"]
    assert len(carries) >= 2
    assert carries[-1]["actions"]["agent1"]["position"] == [0.33, 0.0, 0.35]


def test_shared_payload_different_displacement_rejected():
    args = shared_inputs()
    args[0]["agent2"]["steps"][1]["action"]["position"][0] += 0.03
    with pytest.raises(ValueError):
        pair_plans(*args)


def test_shared_payload_rotation_rejected():
    args = shared_inputs()
    args[0]["agent1"]["steps"][1]["action"]["rpy"][1] = 0.1
    with pytest.raises(ValueError):
        pair_plans(*args)


def test_reflected_transform_rejected():
    args = shared_inputs()
    m = args[1]["agent1"]
    m[0][0] = -1.0
    with pytest.raises(ValueError):
        validate_transform(m)


def test_barrier_exact_command_ids():
    c = dict(group_id="g", command_id="c", step_index=0, execute_at=10.0, token="t")
    s = dict(
        prepared_group_id="g",
        prepared_command_id="other",
        prepared_step_index=0,
        prepared_execute_at=10.0,
        token="t",
    )
    assert not prepared_pair({"agent1": s}, {"agent1": c})
