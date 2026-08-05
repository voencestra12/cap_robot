import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    cap_share = get_package_share_directory("cap_robot")
    realsense_share = get_package_share_directory("realsense2_camera")

    realsense_launch = os.path.join(
        realsense_share,
        "launch",
        "rs_launch.py",
    )

    agent_config = os.path.join(
        cap_share,
        "config",
        "agent2.yaml",
    )

    mount_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            "camera_namespace": "agent2",
            "camera_name": "camera",
            "tf_prefix": "agent2/",
            "align_depth.enable": "true",
            "enable_sync": "true",
        }.items(),
    )

    robot_agent = Node(
        package="cap_robot",
        executable="robot_agent",
        namespace="agent2",
        name="robot_agent_node",
        output="screen",
        parameters=[agent_config],
    )

    return LaunchDescription([
        mount_camera,
        robot_agent,
    ])