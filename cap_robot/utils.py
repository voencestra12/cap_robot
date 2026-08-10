from __future__ import annotations

import math
from typing import Any, Iterable, Optional

import numpy as np


def normalize_quaternion_xyzw(q: Iterable[float]) -> np.ndarray:
    arr = np.asarray(list(q), dtype=float).reshape(4)
    n = np.linalg.norm(arr)
    if n < 1e-12:
        raise ValueError('Quaternion norm is near zero')
    return arr / n


def quaternion_xyzw_to_matrix(q_xyzw: Iterable[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(q_xyzw)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=float)


def matrix_to_quaternion_xyzw(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    tr = np.trace(R)
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return normalize_quaternion_xyzw([x, y, z, w])


def parse_matrix_4x4(value: Any, key_name: str = 'matrix') -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.shape != (4, 4):
        raise ValueError(f'{key_name} must be a 4x4 matrix, got shape {arr.shape}')
    return arr


def transform_msg_to_matrix(transform_stamped_msg: Any) -> np.ndarray:
    tr = transform_stamped_msg.transform.translation
    rot = transform_stamped_msg.transform.rotation
    T = np.eye(4, dtype=float)
    T[:3, :3] = quaternion_xyzw_to_matrix([rot.x, rot.y, rot.z, rot.w])
    T[:3, 3] = [tr.x, tr.y, tr.z]
    return T


def matrix_to_transform_stamped(T: np.ndarray, parent: str, child: str, stamp) -> Any:
    from geometry_msgs.msg import TransformStamped
    qx, qy, qz, qw = matrix_to_quaternion_xyzw(T[:3, :3])
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(T[0, 3])
    msg.transform.translation.y = float(T[1, 3])
    msg.transform.translation.z = float(T[2, 3])
    msg.transform.rotation.x = float(qx)
    msg.transform.rotation.y = float(qy)
    msg.transform.rotation.z = float(qz)
    msg.transform.rotation.w = float(qw)
    return msg


def transform_xyz(T_parent_child: np.ndarray, xyz_child: Iterable[float]) -> np.ndarray:
    p = np.ones(4, dtype=float)
    p[:3] = np.asarray(list(xyz_child), dtype=float).reshape(3)
    out = T_parent_child @ p
    return out[:3]


def get_optional_float(node, name: str) -> Optional[float]:
    value = node.get_parameter(name).value
    if value is None:
        return None
    try:
        return float(value)
    except TypeError:
        return None
