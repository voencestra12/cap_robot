from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory('cap_robot'))
    config_file = share_dir / 'config' / 'calibration.yaml'

    return LaunchDescription([
        Node(
            package='cap_robot',
            executable='aruco_calib',
            name='aruco_calib',
            output='screen',
            parameters=[str(config_file)],
        )
    ])
