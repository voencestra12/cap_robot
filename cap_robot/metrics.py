"""역할: 정량 이벤트 및 CSV 영속화. 인터페이스: make_event(), MetricsNode."""

from __future__ import annotations

import csv
import json
import threading
import time
import uuid
from collections import deque
from pathlib import Path


FIELDS = [
    "schema",
    "event_id",
    "source",
    "event",
    "stamp",
    "mission_id",
    "task_id",
    "latency_s",
    "success",
    "detail",
    "extra_json",
]


def make_event(source: str, event: str, **fields) -> dict:
    return {
        "schema": 1,
        "event_id": uuid.uuid4().hex,
        "source": source,
        "event": event,
        "stamp": time.time(),
        **fields,
    }


class CsvMetrics:
    """[변경] 분모가 다른 JSON 준수·제어 성공·사람 의미평가를 별도 event로 저장."""

    def __init__(self, path):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = path.open("a", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDS)
        self._lock = threading.Lock()
        self._seen, self._order = set(), deque()
        if self._file.tell() == 0:
            self._writer.writeheader()
            self._file.flush()

    def write(self, event: dict):
        if event.get("schema") != 1 or not all(
            isinstance(event.get(k), str) and event[k] for k in ("event_id", "source", "event")
        ):
            raise ValueError("invalid metric event")
        with self._lock:
            if event["event_id"] in self._seen:
                return
            self._seen.add(event["event_id"])
            self._order.append(event["event_id"])
            if len(self._order) > 100000:
                self._seen.discard(self._order.popleft())
            row = {key: event.get(key, "") for key in FIELDS[:-1]}
            row["extra_json"] = json.dumps(
                {k: v for k, v in event.items() if k not in FIELDS},
                ensure_ascii=False,
                allow_nan=False,
            )
            self._writer.writerow(row)
            self._file.flush()

    def close(self):
        with self._lock:
            self._file.close()


def main(args=None):
    from rclpy.node import Node
    from std_msgs.msg import String
    from .ros_support import decode, load_config, parameter, qos_event, run_node

    class MetricsNode(Node):
        def __init__(self):
            super().__init__("metrics")
            config = load_config(parameter(self, "config_file", ""))
            directory = Path(config.get("log_dir", "~/.ros/cap_robot")).expanduser()
            self.csv = CsvMetrics(directory / f"metrics_{time.time_ns()}.csv")
            self.create_subscription(String, "/cap/metrics", self.receive, qos_event())

        def receive(self, msg):
            try:
                self.csv.write(decode(msg))
            except (ValueError, OSError) as error:
                self.get_logger().error(f"metrics write failed: {error}")

        def close(self):
            self.csv.close()

    run_node(MetricsNode, args)
