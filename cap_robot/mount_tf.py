"""역할: 검증된 hand-eye rigid transform 게시. 인터페이스: config_file -> /tf_static.

# [변경] 원본 link6 기준 변환 유지; TCP 기준으로 임의 재해석하지 않는다.
"""

import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from .ros_support import parameter, load_config, run_node
from .calibration_node import CalibrationNode


class MountTF(Node):
    def __init__(self):
        super().__init__("mount_tf")
        c = load_config(parameter(self, "config_file", ""))
        matrix = np.array(c["T_parent_child"]["matrix"], dtype=float)
        if (
            matrix.shape != (4, 4)
            or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1.0, atol=1e-6)
        ):
            raise ValueError("hand-eye matrix must be finite rigid transform in metres")
        self.tf = StaticTransformBroadcaster(self)
        self.tf.sendTransform(
            CalibrationNode.msg(
                c["parent_frame"],
                c["child_frame"],
                matrix[:3, 3],
                Rotation.from_matrix(matrix[:3, :3]).as_quat(),
                self.get_clock().now().to_msg(),
            )
        )


def main(args=None):
    run_node(MountTF, args)
