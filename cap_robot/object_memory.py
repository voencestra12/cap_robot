"""Workspace 위치로 같은 종류의 물체를 개별 추적합니다 (좌표 단위: mm)."""

import math
import re
import time

import numpy as np
from scipy.optimize import linear_sum_assignment


def object_class(name):
    """개체 ID의 양의 정수 접미사만 제거합니다. 예: 빵_2 → 빵."""
    return re.sub(r'_[1-9][0-9]*$', '', str(name).strip())


def resolve_object_target(raw_target, poses):
    """정확한 ID 또는 유일한 클래스만 선택하며 모호한 선택은 거부합니다."""
    target = str(raw_target).strip()
    if target in poses:
        return target
    if not target or object_class(target) != target:
        return None  # 사라진 빵_1을 빵_10 등으로 바꾸지 않습니다.
    matches = [key for key in poses if object_class(key) == target]
    return matches[0] if len(matches) == 1 else None


class ObjectMemory:
    """클래스별 일대일 거리 매칭. 잠시 가려진 개체의 ID만 제한 시간 보관.

    update 결과는 이번 관측에 실제로 있는 물체만 포함합니다. 미검출 개체의
    옛 좌표를 실행 대상으로 제공하지 않습니다. ID는 프로세스 내에서 재사용하지
    않으며 Agent 간 공유 ID나 큰 이동/교차 후의 시각적 재식별은 지원하지 않습니다.
    """

    def __init__(self, match_distance_mm=60.0, retention_sec=3.0):
        for value in (match_distance_mm, retention_sec):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('물체 매칭 거리와 기억 시간은 양의 유한수여야 합니다.')
        self.match_distance_mm = float(match_distance_mm)
        self.retention_sec = float(retention_sec)
        self._tracks = {}
        self._counts = {}

    def update(self, detections, now=None):
        """[(class_name, (x, y, z, yaw)), ...] → 입력 순서의 {ID: pose}."""
        now = time.monotonic() if now is None else float(now)
        observations = [(str(name), tuple(float(v) for v in pose))
                        for name, pose in detections]
        if not math.isfinite(now) or any(
            not name or len(pose) != 4 or not all(map(math.isfinite, pose))
            for name, pose in observations
        ):
            raise ValueError('물체 이름과 유한한 (x, y, z, yaw), 시각이 필요합니다.')
        self._tracks = {
            key: track for key, track in self._tracks.items()
            if now - track['last_seen'] <= self.retention_sec
        }
        assigned = {}
        for name in sorted({name for name, _ in observations}):
            indices = [i for i, (label, _) in enumerate(observations) if label == name]
            keys = sorted(key for key, track in self._tracks.items()
                          if track['class_name'] == name)
            if keys:
                distances = np.array([
                    [math.dist(observations[i][1][:3], self._tracks[key]['pose'][:3])
                     for key in keys] for i in indices
                ])
                # 더미 열은 신규 개체입니다. 최대 매칭 수를 우선하고 총 거리를 최소화.
                unmatched = (len(indices) + 1) * self.match_distance_mm
                costs = np.full((len(indices), len(keys) + len(indices)), unmatched)
                costs[:, :len(keys)] = np.where(
                    distances <= self.match_distance_mm, distances, unmatched * 2,
                )
                rows, columns = linear_sum_assignment(costs)
                for row, col in zip(rows, columns):
                    if col < len(keys):
                        assigned[indices[row]] = keys[col]
            # 처음 본 물체는 위치 순서로 번호를 부여합니다.
            for i in sorted(indices, key=lambda i: observations[i][1][:3]):
                if i not in assigned:
                    self._counts[name] = self._counts.get(name, 0) + 1
                    assigned[i] = f'{name}_{self._counts[name]}'
        visible = {}
        for i, (name, pose) in enumerate(observations):
            key = assigned[i]
            self._tracks[key] = dict(class_name=name, pose=pose, last_seen=now)
            visible[key] = pose
        return visible
