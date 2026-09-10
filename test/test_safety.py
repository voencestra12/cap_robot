"""역할: 실제 하드웨어 무접속 안전 회귀. 인터페이스: pytest, SafetyGate+fake backend.

# [변경] 명령 미접수·중복·정지·시계·통신·양팔 장벽의 실패 경로를 검증.
"""

import copy
import math
import time
import pytest
from cap_robot.hardware import DryRunBackend, make_backend, _strict
from cap_robot.safety import SafetyGate, SafetyLimits, SafetyError


def config():
    return dict(agent_id="agent1", dry_run=True, safety=dict(initial_rpy=[math.pi, 0.0, 0.0]))


@pytest.fixture
def rig():
    c = config()
    b = DryRunBackend(c)
    results = []
    g = SafetyGate(c, b, results.append)
    permit = dict(
        enabled=True,
        token="token",
        session="ws",
        owner="agent1",
        mission_id="m",
        task_id="t",
        cooperative=False,
    )
    g.update_permit(permit)
    g.update_agent(dict(agent_id="agent1", boot_id="a", seq=1, token="token", status="EXECUTING"))
    yield g, b, results, permit
    g.close()


def command(**fields):
    d = dict(
        schema=1,
        agent_id="agent1",
        command_id="c",
        stamp=time.time(),
        token="token",
        mission_id="m",
        task_id="t",
        action=dict(kind="move", position=[0.301, 0.0, 0.35], rpy=[math.pi, 0.0, 0.0], speed=0.04),
    )
    d.update(fields)
    return d


def wait_for(fn, seconds=2.0):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if fn():
            return
        time.sleep(0.01)
    assert fn()


@pytest.mark.parametrize(
    "position", [[0.3, 0, 0.01], [float("nan"), 0, 0.3], [0.9, 0, 0.3], [0.3, -0.5, 0.3]]
)
def test_fence_blocks_without_backend_motion(rig, position):
    g, b, results, _ = rig
    c = command()
    c["action"]["position"] = position
    assert not g.submit(c)
    assert results[-1]["status"] == "FAILED"
    assert not any(x[0] == "move" for x in b.calls)


def test_tool_margin_and_invalid_fence():
    limits = SafetyLimits(dict(min_xyz=[0.1, -0.4, 0.1], max_xyz=[0.6, 0.4, 0.6], tcp_radius=0.02))
    with pytest.raises(SafetyError):
        limits.check_position([0.11, 0, 0.3])
    with pytest.raises(SafetyError):
        SafetyLimits(dict(permit_timeout=999.0))


@pytest.mark.parametrize("field,value", [("token", "old"), ("agent_id", "agent2"), ("stamp", 0.0)])
def test_reject_wrong_identity_or_replayed_time(rig, field, value):
    g, b, results, _ = rig
    assert not g.submit(command(**{field: value}))
    assert not any(x[0] == "move" for x in b.calls)


def test_completion_is_measured_and_duplicate_is_not_executed(rig):
    g, b, results, _ = rig
    c = command()
    assert g.submit(c)
    assert results == []
    assert not g.submit(c)
    wait_for(lambda: bool(results))
    assert results[-1]["status"] == "SUCCEEDED"
    assert not g.submit(c)
    assert len([x for x in b.calls if x[0] == "move"]) == 1


def test_stop_latches_and_kill_cannot_reset(rig):
    g, b, results, _ = rig
    g.stop("operator")
    assert not g.submit(command())
    wait_for(lambda: b.snapshot()["state"] == 4)
    g.reset()
    assert not g.armed
    g.arm()
    g.stop("kill", kill=True)
    with pytest.raises(SafetyError):
        g.reset()


def test_permit_revoked_mid_motion_stops_and_fails(rig):
    g, b, results, p = rig
    c = command()
    c["action"]["position"] = [0.34, 0, 0.35]
    g.submit(c)
    time.sleep(0.05)
    g.update_permit(dict(p, enabled=False))
    wait_for(lambda: bool(results))
    assert results[-1]["status"] == "FAILED"
    assert g.snapshot()["latched"]


def test_expired_heartbeat_stops_inflight(rig):
    g, b, results, p = rig
    c = command()
    c["action"]["position"] = [0.36, 0, 0.35]
    g.submit(c)
    with g.lock:
        g.permit_received -= 3.0
    wait_for(lambda: bool(results))
    assert results[-1]["status"] == "FAILED"


def test_paired_without_go_never_moves(rig):
    g, b, results, p = rig
    g.update_permit(dict(p, owner="both", cooperative=True))
    g.update_agent(dict(agent_id="agent2", boot_id="a2", seq=1, token="token", status="EXECUTING"))
    c = command(phase="prepare", group_id="group", step_index=0, execute_at=time.time() + 0.35)
    assert g.submit(c)
    wait_for(lambda: bool(results))
    assert results[-1]["status"] == "FAILED"
    assert not any(x[0] == "move" for x in b.calls)


def test_sdk_is_not_imported_in_dry_run(monkeypatch):
    import builtins

    original = builtins.__import__

    def no_sdk(name, *args, **kwargs):
        if name.startswith("xarm"):
            raise AssertionError("SDK import in dry-run")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", no_sdk)
    b = make_backend(config())
    assert isinstance(b, DryRunBackend)
    b.close()


@pytest.mark.parametrize("code", [None, True, 1, -1, "0"])
def test_sdk_success_must_be_exact_zero(code):
    with pytest.raises(SafetyError):
        _strict(code, "test")
