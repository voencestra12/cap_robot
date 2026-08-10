#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .utils import matrix_to_transform_stamped, parse_matrix_4x4


class StaticTfFromYaml(Node):
    def __init__(self) -> None:
        super().__init__('mount_tf')
        self.declare_parameter('result_yaml', 'handeye_camera_link_result.yaml')
        path = Path(str(self.get_parameter('result_yaml').value)).expanduser()
        with path.open('r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        parent = str(data['parent_frame'])
        child = str(data['child_frame'])
        T = parse_matrix_4x4(data['T_parent_child']['matrix'], 'T_parent_child.matrix')
        msg = matrix_to_transform_stamped(T, parent, child, self.get_clock().now().to_msg())
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(msg)
        self.get_logger().info(f'Published static TF {parent} -> {child} from {path}')
        self.get_logger().info('Keep this node alive. If it stops, robot and camera TF trees may disconnect.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticTfFromYaml()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
