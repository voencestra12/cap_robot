"""역할: 운용자 명령·정지·상태·수동 평가. 인터페이스: ros2 run cap_robot cli --help.

# [변경] 자연어 목표와 안전 운용 명령 분리; LLM에 arm/reset 권한 없음.
"""

import argparse
import json
import time
import uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from .ros_support import qos_event, qos_state, publish_json, decode


def main(args=None):
    parser = argparse.ArgumentParser(description="cap_robot operator console")
    sub = parser.add_subparsers(dest="op", required=True)
    mission = sub.add_parser("mission")
    mission.add_argument("goal")
    sub.add_parser("estop")
    sub.add_parser("kill")
    sub.add_parser("monitor")
    for op in ("arm", "reset-stop"):
        p = sub.add_parser(op)
        p.add_argument("agent", choices=["agent1", "agent2"])
    grade = sub.add_parser("grade")
    grade.add_argument("request_id")
    grade.add_argument("--recognized", choices=["yes", "no"], required=True)
    grade.add_argument("--prompt-compliant", choices=["yes", "no"])
    grade.add_argument("--task-completed", choices=["yes", "no"])
    grade.add_argument("--note", default="")
    opt = parser.parse_args(args)
    rclpy.init()
    node = Node("cap_operator")
    try:
        if opt.op == "monitor":

            def show(msg):
                try:
                    print(json.dumps(decode(msg), ensure_ascii=False, indent=2), flush=True)
                except ValueError:
                    pass

            node.create_subscription(String, "/cap/mission_state", show, qos_state())
            rclpy.spin(node)
        elif opt.op in ("arm", "reset-stop"):
            client = node.create_client(
                Trigger, f"/{opt.agent}/" + ("arm" if opt.op == "arm" else "reset_stop")
            )
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError("safety service unavailable")
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if not future.done():
                raise RuntimeError("service timeout; state unknown, inspect gate")
            result = future.result()
            print(result.message)
            if not result.success:
                raise RuntimeError("operator action rejected")
        else:
            if opt.op in ("kill", "estop"):
                pub = node.create_publisher(Bool, f"/cap/{opt.op}", qos_event())
                data = Bool(data=True)
            else:
                topic = "/cap/mission_input" if opt.op == "mission" else "/cap/metrics"
                pub = node.create_publisher(String, topic, qos_event())
                if opt.op == "mission":
                    uid = uuid.uuid4().hex
                    data = dict(schema=1, request_id=uid, goal=opt.goal)
                    print(f"request_id={uid}; submitted after publisher discovery")
                else:
                    data = dict(
                        schema=1,
                        event_id=uuid.uuid4().hex,
                        source="operator",
                        event="human_evaluation",
                        stamp=time.time(),
                        request_id=opt.request_id,
                        mission_id="",
                        task_id="",
                        recognized=opt.recognized == "yes",
                        prompt_compliant=(
                            None if opt.prompt_compliant is None else opt.prompt_compliant == "yes"
                        ),
                        task_completed=(
                            None if opt.task_completed is None else opt.task_completed == "yes"
                        ),
                        detail=opt.note,
                    )
            deadline = time.monotonic() + 3.0
            while not pub.get_subscription_count() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not pub.get_subscription_count():
                raise RuntimeError("no subscriber; command NOT delivered")
            for _ in range(3):
                if isinstance(data, dict):
                    publish_json(pub, data)
                else:
                    pub.publish(data)
                rclpy.spin_once(node, timeout_sec=0.1)
            print("Sent; inspect monitor/gate state for confirmed result.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
