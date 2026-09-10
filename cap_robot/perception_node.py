"""역할: 에이전트별 독립 RGB-D 인식. 인터페이스: Image/CameraInfo -> observations.

# [변경] 이동·LLM 추론 중에도 인식 지속, 영상시각 TF만 허용, 만료 객체 제거.
"""

import threading
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from .ros_support import parameter, load_config, qos_state, publish_json, run_node
from .perception import deproject, transform, Tracker


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception")
        config = load_config(parameter(self, "config_file", ""))
        self.agent_id, self.base = config["agent_id"], config["base_frame"]
        self.config = config["perception"]
        self.backend = self.config.get("backend", "yolo")
        if not config["dry_run"] and self.backend == "mock":
            raise ValueError("mock perception forbidden for hardware")
        self.pub = self.create_publisher(String, "observations", qos_state())
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.frames, self.info, self.seq, self.last_stamp = {}, None, 0, -1.0
        self.tracker = Tracker(self.agent_id)
        self.models = []
        if self.backend == "mock":
            self.create_timer(0.2, self.mock)
            return
        import cv2
        from cv_bridge import CvBridge

        self.cv2, self.bridge = cv2, CvBridge()
        if self.backend == "yolo":
            from ultralytics import YOLO

            for raw in self.config["model_paths"]:
                p = Path(raw).expanduser()
                if not p.is_absolute():
                    p = Path(get_package_share_directory("cap_robot")) / p
                if not p.is_file():
                    raise ValueError(f"model missing: {p}")
                self.models.append(YOLO(str(p)))
        elif self.backend != "red":
            raise ValueError("backend must be mock, yolo or red")
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(
            Image,
            self.config["color_topic"],
            lambda m: self.frame("color", m),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.config["depth_topic"],
            lambda m: self.frame("depth", m),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, self.config["camera_info_topic"], self.camera_info, qos_profile_sensor_data
        )
        self.worker = threading.Thread(target=self.loop, daemon=True)
        self.worker.start()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def camera_info(self, msg):
        with self.lock:
            self.info = msg

    def frame(self, key, msg):
        with self.lock:
            self.frames[key] = msg

    def emit(self, objects, stamp, valid=True, reason=""):
        self.seq += 1
        publish_json(
            self.pub,
            dict(
                schema=1,
                agent_id=self.agent_id,
                seq=self.seq,
                stamp=stamp,
                frame_id=self.base,
                valid=valid,
                objects=objects,
                reason=reason,
                source=self.backend,
            ),
        )

    def mock(self):
        objects = [dict(x) for x in self.config.get("mock_objects", [])]
        self.emit(objects, self.now(), reason="SIMULATED SCENE; no physical outcome evidence")

    def detections(self, color):
        cv2 = self.cv2
        out = []
        if self.backend == "red" or self.config.get("detect_red_handles", False):
            hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255])) | cv2.inRange(
                hsv, np.array([170, 120, 70]), np.array([180, 255, 255])
            )
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) < self.config.get("minimum_area_px", 50):
                    continue
                m = cv2.moments(c)
                if m["m00"] > 0:
                    out.append(("basket_handle", m["m10"] / m["m00"], m["m01"] / m["m00"], 0.8))
        if self.backend == "yolo":
            aliases = self.config.get("label_aliases", {})
            for model in self.models:
                for r in model.predict(
                    color, conf=self.config.get("confidence", 0.45), verbose=False
                ):
                    for b in r.boxes:
                        x1, y1, x2, y2 = b.xyxy[0].cpu().tolist()
                        name = r.names[int(b.cls[0])]
                        out.append(
                            (
                                aliases.get(name, name),
                                (x1 + x2) / 2,
                                (y1 + y2) / 2,
                                float(b.conf[0]),
                            )
                        )
        return out

    def process(self, color_msg, depth_msg, info):
        stamp = Time.from_msg(color_msg.header.stamp)
        stamp_s = stamp.nanoseconds * 1e-9
        dtime = Time.from_msg(depth_msg.header.stamp).nanoseconds * 1e-9
        age = self.now() - stamp_s
        if not 0 <= age <= self.config.get("max_age", 0.5):
            raise ValueError("stale/future RGB image")
        if abs(stamp_s - dtime) > self.config.get("sync_slop", 0.08):
            raise ValueError("RGB/depth skew")
        if color_msg.header.frame_id != info.header.frame_id:
            raise ValueError("RGB/CameraInfo optical frame mismatch")
        if depth_msg.width != color_msg.width or depth_msg.height != color_msg.height:
            raise ValueError("depth must be aligned to color")
        if info.width != color_msg.width or info.height != color_msg.height:
            raise ValueError("CameraInfo resolution mismatch")
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        # [변경] 과거/최신 TF로 조용히 대체하지 않는다.
        t = self.buffer.lookup_transform(
            self.base, color_msg.header.frame_id, stamp, timeout=Duration(seconds=0.15)
        ).transform
        tr = [t.translation.x, t.translation.y, t.translation.z]
        q = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        objects = []
        for label, u, v, confidence in self.detections(color):
            try:
                xyz = transform(deproject(depth, depth_msg.encoding, u, v, list(info.k)), tr, q)
            except ValueError:
                continue
            if any(
                o["label"] == label and np.linalg.norm(np.array(o["position"]) - xyz) < 0.025
                for o in objects
            ):
                continue
            objects.append(dict(label=label, position=xyz, confidence=confidence))
        # Inference latency is included in freshness, not hidden by restamping.
        if self.now() - stamp_s > self.config.get("max_age", 0.5):
            raise ValueError("perception inference too old")
        return self.tracker.update(objects, stamp_s), stamp_s

    def loop(self):
        while not self.stop.wait(0.1):
            with self.lock:
                frames, info = dict(self.frames), self.info
            try:
                if len(frames) != 2 or info is None:
                    raise ValueError("awaiting RGB-D/CameraInfo")
                objects, stamp = self.process(frames["color"], frames["depth"], info)
                if stamp == self.last_stamp:
                    continue
                self.last_stamp = stamp
                self.emit(objects, stamp)
            except Exception as exc:
                self.emit([], self.now(), False, str(exc))

    def close(self):
        self.stop.set()
        if hasattr(self, "worker"):
            self.worker.join(timeout=2.0)


def main(args=None):
    run_node(PerceptionNode, args)
