"""역할: Humble ROS 공통 기능. 인터페이스: QoS/JSON/파라미터/노드 실행.

# [변경] 긴 HTTP와 장치 동작은 별도 worker; ROS executor는 상태·정지를 계속 수신.
"""

from pathlib import Path
import signal
import yaml
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from .protocol import dumps, loads, envelope


def qos_state():
    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def qos_event():
    return QoSProfile(
        depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE
    )


def publish_json(pub, data):
    pub.publish(String(data=dumps(data)))


def decode(msg):
    return envelope(loads(msg.data))


def parameter(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


def load_config(path):
    with Path(path).expanduser().open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("configuration must be YAML mapping")
    return data


def run_node(factory, args=None):
    # [변경] rclpy가 SIGINT에서 context를 먼저 닫으면 마지막 정지 메시지를 못 보낸다.
    # 종료 신호를 main으로 전달하고 노드 close/정지 후 ROS context를 종료한다.
    previous = {}

    def interrupt(signum, frame):
        raise KeyboardInterrupt()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.signal(signum, interrupt)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = factory()
        executor.add_node(node)
        # [변경] 타이머/이벤트가 없는 metrics에서도 Python 종료 신호를 처리하도록
        # native DDS 무한 대기 대신 유한 시간씩 기다린다.
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if hasattr(node, "close"):
                node.close()
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown(timeout_sec=2.0)
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
