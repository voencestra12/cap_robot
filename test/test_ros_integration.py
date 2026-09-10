"""역할: 실제 Humble DDS+HTTP 모의 모델+독립 프로세스 통합 시험.
인터페이스: CAP_ROBOT_ROS_TESTS=1 pytest test/test_ros_integration.py.

# [변경] 시험에서만 HTTP 응답을 주입한다. 런타임의 LLM 실패 대체 경로가 아니다.
모의 장면은 물체/유체 물리 시뮬레이터가 아니며 이 시험은 통신·제어 프로토콜 검증이다.
"""

import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import pytest
import yaml

pytestmark = pytest.mark.skipif(
    os.environ.get("CAP_ROBOT_ROS_TESTS") != "1", reason="opt-in local ROS networking test"
)


def test_two_agents_cooperative_and_estop(tmp_path):
    import rclpy
    from std_msgs.msg import String, Bool
    from cap_robot.ros_support import qos_event, qos_state, publish_json, decode

    package = Path(__file__).resolve().parents[1]
    requests = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            context = json.loads(body["prompt"])
            requests.append(context)
            if "agents" in context:
                if context["goal"] == "paired test":
                    plan = dict(
                        tasks=[
                            dict(
                                id="shared",
                                type="cooperative",
                                agent="both",
                                goal="test paired translation",
                                depends_on=[],
                            )
                        ]
                    )
                else:
                    plan = dict(
                        tasks=[
                            dict(
                                id="first",
                                type="single",
                                agent="agent1",
                                goal="inspect bread from nearby pose",
                                depends_on=[],
                            ),
                            dict(
                                id="second",
                                type="single",
                                agent="agent2",
                                goal="inspect bread after peer",
                                depends_on=["first"],
                            ),
                        ]
                    )
            elif "base_to_world" in context:
                pose = context["local_start"]["position"]
                rpy = context["local_start"]["rpy"]
                scene = context["local_scene"]
                target = next(o for o in scene["objects"] if o["label"] == "handle")
                # world +X maps to opposite local X for the rotated second base.
                rotation = context["base_to_world"]
                final = [pose[i] + rotation[0][i] * 0.003 for i in range(3)]
                plan = dict(
                    steps=[
                        dict(
                            phase="grasp", action=dict(kind="gripper", position=0.0, speed=2000.0)
                        ),
                        dict(
                            phase="carry",
                            action=dict(kind="move", position=final, rpy=rpy, speed=0.02),
                        ),
                        dict(
                            phase="release",
                            action=dict(kind="gripper", position=850.0, speed=2000.0),
                        ),
                        dict(
                            phase="verify",
                            action=dict(
                                kind="verify",
                                object_id=target["id"],
                                expected_position=target["position"],
                                tolerance=0.02,
                            ),
                        ),
                    ],
                    reason="test fixture",
                )
            else:
                g = context["current_gate"]
                p = list(g["position"])
                p[0] += 0.002
                target = next(o for o in context["scene"]["objects"] if o["label"] == "bread")
                plan = dict(
                    actions=[
                        dict(kind="move", position=p, rpy=g["rpy"], speed=0.04),
                        dict(
                            kind="verify",
                            object_id=target["id"],
                            expected_position=target["position"],
                            tolerance=0.02,
                        ),
                    ],
                    reason="test fixture",
                )
            raw = json.dumps(dict(response=json.dumps(plan))).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    env = dict(
        os.environ,
        ROS_DOMAIN_ID="174",
        ROS_LOCALHOST_ONLY="1",
        ROS_LOG_DIR=str(tmp_path / "roslog"),
        PYTHONPATH=str(package) + os.pathsep + os.environ.get("PYTHONPATH", ""),
    )
    processes = []
    streams = []
    configs = {}
    for name in ("agent1", "agent2", "workstation"):
        cfg = yaml.safe_load((package / "config" / f"{name}.yaml").read_text())
        if name != "workstation":
            assert cfg["dry_run"] is True and cfg["perception"]["backend"] == "mock"
        cfg["llm"].update(
            url=f"http://127.0.0.1:{server.server_port}/api/generate",
            model="TEST_ONLY",
            timeout=5.0,
        )
        if name == "workstation":
            cfg["log_dir"] = str(tmp_path / "metrics")
        path = tmp_path / f"{name}.yaml"
        path.write_text(yaml.safe_dump(cfg))
        configs[name] = str(path)

    def launch(module, params, ns=None):
        cmd = [sys.executable, "-c", f"from cap_robot.{module} import main; main()", "--ros-args"]
        if ns:
            cmd += ["-r", f"__ns:=/{ns}"]
        for k, v in params.items():
            cmd += ["-p", f"{k}:={v}"]
        log = (tmp_path / f'{module}_{ns or "ws"}.log').open("w")
        streams.append(log)
        p = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)
        processes.append(p)
        return p

    node = None
    try:
        for agent in ("agent1", "agent2"):
            for module in ("safety_node", "perception_node", "agent_node"):
                launch(module, dict(config_file=configs[agent]), agent)
        launch("workstation_node", dict(config_file=configs["workstation"]))
        launch("metrics", dict(config_file=configs["workstation"]))
        launch(
            "cooperative_node",
            dict(
                config_file=configs["workstation"],
                agent1_config_file=configs["agent1"],
                agent2_config_file=configs["agent2"],
            ),
        )
        for base, x, yaw in [
            ("robot_1_base", "0", "0"),
            ("robot_2_base", ".8", "3.141592653589793"),
        ]:
            log = (tmp_path / (base + ".log")).open("w")
            streams.append(log)
            processes.append(
                subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        "tf2_ros",
                        "static_transform_publisher",
                        "--x",
                        x,
                        "--y",
                        "0",
                        "--z",
                        "0",
                        "--yaw",
                        yaw,
                        "--pitch",
                        "0",
                        "--roll",
                        "0",
                        "--frame-id",
                        "workspace_0",
                        "--child-frame-id",
                        base,
                    ],
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            )
        # Parent DDS domain must match isolated test children.
        os.environ["ROS_DOMAIN_ID"] = "174"
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ["ROS_LOG_DIR"] = str(tmp_path / "parentlogs")
        rclpy.init()
        node = rclpy.create_node("cap_integration_driver")
        snapshots = []
        gates = {}
        results = []
        node.create_subscription(
            String, "/cap/mission_state", lambda m: snapshots.append(decode(m)), qos_state()
        )
        for a in ("agent1", "agent2"):
            node.create_subscription(
                String, f"/{a}/gate_state", lambda m, a=a: gates.update({a: decode(m)}), qos_state()
            )
            node.create_subscription(
                String, f"/{a}/command_result", lambda m: results.append(decode(m)), qos_event()
            )
        pub = node.create_publisher(String, "/cap/mission_input", qos_event())
        stop = node.create_publisher(Bool, "/cap/estop", qos_event())

        def until(predicate, seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                assert all(p.poll() is None for p in processes), f"child exited; inspect {tmp_path}"
                if predicate():
                    return
            state = snapshots[-1] if snapshots else {}
            pytest.fail(f"timeout; state={state}; logs={tmp_path}")

        until(
            lambda: len(gates) == 2 and snapshots and len(snapshots[-1].get("agents", {})) == 3,
            15.0,
        )
        publish_json(pub, dict(schema=1, request_id="seq-test", goal="sequential test"))
        until(
            lambda: any(s.get("status") in ("SUCCEEDED", "BLOCKED", "REJECTED") for s in snapshots),
            20.0,
        )
        assert snapshots[-1]["status"] == "SUCCEEDED", snapshots[-1]
        assert snapshots[-1]["completed"] == ["first", "second"]
        assert len([r for r in results if r["status"] == "SUCCEEDED"]) == 2
        # Repeated request does not execute either robot twice.
        publish_json(pub, dict(schema=1, request_id="seq-test", goal="sequential test"))
        old_count = len(results)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        assert len(results) == old_count
        snapshots.clear()
        publish_json(pub, dict(schema=1, request_id="paired-test", goal="paired test"))
        until(
            lambda: any(
                s.get("request_id") == "paired-test"
                and s.get("status") in ("SUCCEEDED", "BLOCKED", "REJECTED")
                for s in snapshots
            ),
            35.0,
        )
        assert snapshots[-1]["status"] == "SUCCEEDED", snapshots[-1]
        assert snapshots[-1]["completed"] == ["shared"]
        assert len([r for r in results if r["status"] == "SUCCEEDED"]) == 8
        # Interrupt an actual in-flight simulated move via DDS, not just an idle gate.
        snapshots.clear()
        publish_json(pub, dict(schema=1, request_id="interrupt-test", goal="interrupt test"))
        until(lambda: any(not s["idle"] for s in gates.values()), 10.0)
        stop.publish(Bool(data=True))
        until(lambda: all(s["latched"] for s in gates.values()), 5.0)
        until(lambda: snapshots and snapshots[-1].get("status") == "BLOCKED", 5.0)
        # Failed physical commands must be counted even after the agent stops waiting.
        from cap_robot.metrics_report import summarize
        def failure_logged():
            report = summarize(list((tmp_path / "metrics").glob("*.csv")))
            return report["actuator_command_samples"] == 9
        until(failure_logged, 5.0)
        report = summarize(list((tmp_path / "metrics").glob("*.csv")))
        assert report["actuator_command_success_rate"] == pytest.approx(8 / 9)
        assert report["single_agent_command_success_rate"] == pytest.approx(2 / 3)
        assert len(requests) == 8  # third mission stops before its second agent can plan.
    finally:
        for p in processes:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=2.0)
        for stream in streams:
            stream.close()
        server.shutdown()
        server.server_close()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
