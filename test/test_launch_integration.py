"""역할: 설치된 실제 launch 진입점의 기동·설정·피드백 회귀 시험.
인터페이스: 빌드/overlay source 후 CAP_ROBOT_ROS_TESTS=1 pytest test/test_launch_integration.py.

# [변경] 노드 main을 직접 호출하지 않고 문서의 ros2 launch 명령을 그대로 실행한다.
기본 YAML은 dry_run/mock인지 먼저 검사하며 목표·장치 명령은 발행하지 않는다.
"""

import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import yaml

pytestmark = pytest.mark.skipif(
    os.environ.get("CAP_ROBOT_ROS_TESTS") != "1", reason="opt-in installed ROS launch test"
)


@pytest.mark.parametrize("case", ["demo", "workstation_empty", "workstation_custom"])
def test_installed_launch_startup_and_configuration(tmp_path, monkeypatch, case):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from ament_index_python.packages import get_package_share_directory
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String
    from cap_robot.ros_support import decode, qos_state

    share = Path(get_package_share_directory("cap_robot"))
    for agent in ("agent1", "agent2"):
        config = yaml.safe_load((share / "config" / f"{agent}.yaml").read_text())
        assert config["dry_run"] is True
        assert config["perception"]["backend"] == "mock"

    # 별도 DDS domain에서 시험 중 생성한 launch 자식들만 관측한다.
    for key, value in {
        "ROS_DOMAIN_ID": "176",
        "ROS_LOCALHOST_ONLY": "1",
        "ROS_LOG_DIR": str(tmp_path / "roslog"),
    }.items():
        monkeypatch.setenv(key, value)

    ws_config = share / "config" / "workstation.yaml"
    command = ["ros2", "launch", "cap_robot"]
    if case == "demo":
        command.append("demo.launch.py")
    else:
        command.append("workstation.launch.py")
        if case == "workstation_custom":
            config = yaml.safe_load(ws_config.read_text())
            config["log_dir"] = str(tmp_path / "metrics")
            ws_config = tmp_path / "custom-workstation.yaml"
            ws_config.write_text(yaml.safe_dump(config))
            command.append(f"workstation_config_file:={ws_config}")
        else:
            # Humble CLI는 := 뒤 빈 문자열을 거부한다. 실제 include로 빈 값을 전달한다.
            wrapper = tmp_path / "empty-workstation.launch.py"
            wrapper.write_text(
                "from launch import LaunchDescription\n"
                "from launch.actions import IncludeLaunchDescription\n"
                "from launch.launch_description_sources import PythonLaunchDescriptionSource\n"
                "def generate_launch_description():\n"
                "    return LaunchDescription([IncludeLaunchDescription(\n"
                f"        PythonLaunchDescriptionSource({str(share / 'launch' / 'workstation.launch.py')!r}),\n"
                "        launch_arguments={'workstation_config_file': ''}.items())])\n"
            )
            command = ["ros2", "launch", str(wrapper)]

    expected = {
        "/workstation": ws_config,
        "/cooperative_coordinator": ws_config,
        "/metrics": ws_config,
    }
    if case == "demo":
        for agent in ("agent1", "agent2"):
            for name in ("agent", "safety_gate", "perception"):
                expected[f"/{agent}/{name}"] = share / "config" / f"{agent}.yaml"

    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node("launch_regression_driver", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    missions, gates, observations = [], {}, {}
    node.create_subscription(
        String, "/cap/mission_state", lambda m: missions.append(decode(m)), qos_state()
    )
    if case == "demo":
        for agent in ("agent1", "agent2"):
            node.create_subscription(
                String, f"/{agent}/gate_state",
                lambda m, a=agent: gates.update({a: decode(m)}), qos_state(),
            )
            node.create_subscription(
                String, f"/{agent}/observations",
                lambda m, a=agent: observations.update({a: decode(m)}), qos_state(),
            )

    clients = {name: node.create_client(GetParameters, name + "/get_parameters") for name in expected}
    futures = {}
    log_path = tmp_path / "launch.log"
    process = None
    try:
        with log_path.open("w") as output:
            # [변경] SIGINT는 launch 부모에만 보내 자식마다 정지 신호가 중복 전달되지 않게 한다.
            process = subprocess.Popen(
                command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True
            )
            deadline = time.monotonic() + 30.0
            stable_since = None
            initial_seq = {}
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
                log = log_path.read_text()
                assert process.poll() is None, log
                assert "Traceback" not in log and "process has died" not in log, log
                for name, client in clients.items():
                    if name not in futures and client.service_is_ready():
                        request = GetParameters.Request(names=["config_file"])
                        futures[name] = client.call_async(request)
                configured = len(futures) == len(expected) and all(f.done() for f in futures.values())
                ready = configured and bool(missions)
                if case == "demo":
                    ready = ready and all(a in gates and a in observations for a in ("agent1", "agent2"))
                    ready = ready and all(a in missions[-1].get("agents", {}) for a in ("agent1", "agent2"))
                    graph = node.get_node_names_and_namespaces()
                    ready = ready and sum(n.startswith("static_transform_publisher") for n, _ in graph) == 2
                if ready:
                    if stable_since is None:
                        stable_since = time.monotonic()
                        initial_seq = {a: g["seq"] for a, g in gates.items()}
                    if time.monotonic() - stable_since >= 2.0:
                        break
                else:
                    stable_since = None
            else:
                pytest.fail(f"launch did not become ready; nodes={node.get_node_names_and_namespaces()}\n{log_path.read_text()}")

            for name, future in futures.items():
                assert future.result().values[0].string_value == str(expected[name]), name
            if case == "demo":
                for agent in ("agent1", "agent2"):
                    assert gates[agent]["agent_id"] == agent
                    assert gates[agent]["telemetry_fresh"] is True
                    assert gates[agent]["seq"] > initial_seq[agent]
                    assert observations[agent]["agent_id"] == agent
                    assert observations[agent]["source"] == "mock"
                    assert observations[agent]["valid"] is True
                    assert missions[-1]["gates"][agent]["agent_id"] == agent
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
    log = log_path.read_text()
    assert process.returncode == 0, log
    assert "Traceback" not in log and "process has died" not in log, log
    assert "failed to terminate" not in log, log
    assert log.count("process has finished cleanly") == (11 if case == "demo" else 3), log
