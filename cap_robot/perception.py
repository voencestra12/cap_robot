"""역할: 깊이 역투영·좌표 변환·객체 ID 유지. 인터페이스: deproject/transform/Tracker.

# [변경] 클래스 이름 하나당 객체 하나로 덮어쓰던 인식을 인스턴스 추적으로 교체.
"""

import math
import numpy as np
from .protocol import finite_vector


def deproject(depth, encoding, u, v, k, radius=2):
    if depth.ndim != 2 or len(k) != 9 or k[0] <= 0 or k[4] <= 0:
        raise ValueError("invalid depth/intrinsics")
    if encoding not in ("16UC1", "32FC1"):
        raise ValueError("depth encoding must be 16UC1 or 32FC1")
    h, w = depth.shape
    u, v = int(u), int(v)
    if not (0 <= u < w and 0 <= v < h):
        raise ValueError("pixel outside aligned depth")
    patch = depth[
        max(0, v - radius) : min(h, v + radius + 1), max(0, u - radius) : min(w, u + radius + 1)
    ].astype(float)
    values = patch[np.isfinite(patch) & (patch > 0)]
    if not len(values):
        raise ValueError("no valid depth")
    z = float(np.median(values)) * (0.001 if encoding == "16UC1" else 1.0)
    if not 0.05 <= z <= 3.0:
        raise ValueError("depth outside sensing range")
    return [(u - k[2]) * z / k[0], (v - k[5]) * z / k[4], z]


def rotation(q):
    q = np.array(finite_vector(q, 4, "quaternion"))
    n = float(np.linalg.norm(q))
    if n < 1e-8 or abs(n - 1.0) > 0.02:
        raise ValueError("non-unit quaternion")
    x, y, z, w = q / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def transform(xyz, translation, q):
    return (
        rotation(q) @ np.array(finite_vector(xyz)) + np.array(finite_vector(translation))
    ).tolist()


class Tracker:
    """Nearest-neighbour gated IDs, per source; lost objects are never republished."""

    def __init__(self, prefix, max_distance=0.08, ttl=2.0):
        self.prefix, self.max_distance, self.ttl = prefix, max_distance, ttl
        self.items, self.counter = {}, 0

    def update(self, detections, now):
        available = {key: x for key, x in self.items.items() if 0 <= now - x[1] <= self.ttl}
        result = []
        for d in sorted(detections, key=lambda x: -x["confidence"]):
            candidates = [
                (math.dist(x[0]["position"], d["position"]), key)
                for key, x in available.items()
                if x[0]["label"] == d["label"]
            ]
            distance, key = min(candidates, default=(float("inf"), None))
            if distance > self.max_distance:
                self.counter += 1
                key = f"{self.prefix}_{self.counter}"
            else:
                available.pop(key)
            obj = dict(d, id=key)
            self.items[key] = (obj, now)
            result.append(obj)
        self.items = {k: v for k, v in self.items.items() if 0 <= now - v[1] <= self.ttl}
        return result
