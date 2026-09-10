"""역할: SDK 무접속 전체 스택. 인터페이스: ros2 launch cap_robot demo.launch.py.

# [변경] 모의 TF는 이 launch에서만 사용하며 실측 좌표로 주장하지 않는다.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("cap_robot")
    nodes = [
        # [변경] include마다 인자 범위를 분리해 agent/config/camera 값의 전파를 차단한다.
        GroupAction(
            scoped=True,
            forwarding=False,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "agent.launch.py")),
                launch_arguments={
                    "agent": a,
                    "config_file": os.path.join(share, "config", a + ".yaml"),
                }.items(),
            )],
        )
        for a in ("agent1", "agent2")
    ]
    nodes.append(
        GroupAction(
            scoped=True,
            forwarding=False,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "workstation.launch.py")),
                launch_arguments={
                    "workstation_config_file": os.path.join(share, "config", "workstation.yaml"),
                    "agent1_config_file": os.path.join(share, "config", "agent1.yaml"),
                    "agent2_config_file": os.path.join(share, "config", "agent2.yaml"),
                }.items(),
            )],
        )
    )
    for base, x, yaw in [("robot_1_base", "0", "0"), ("robot_2_base", ".8", "3.141592653589793")]:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    x,
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--yaw",
                    yaw,
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "workspace_0",
                    "--child-frame-id",
                    base,
                ],
            )
        )
    return LaunchDescription(nodes)
