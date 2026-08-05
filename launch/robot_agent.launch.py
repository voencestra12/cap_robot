from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory('cap_robot'))
    default_config = str(share_dir / 'config' / 'agent1.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='robot_agent.py에 사용할 ROS2 파라미터 YAML 경로',
    )

    agent_node = Node(
        package='cap_robot',
        executable='robot_agent',
        name='robot_agent_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([config_arg, agent_node])
