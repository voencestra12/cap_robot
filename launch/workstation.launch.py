"""역할: 워크스테이션 LLM·피드백·CSV·공동 운반.
인터페이스: workstation_config_file/agent1_config_file/agent2_config_file/camera.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = get_package_share_directory("cap_robot")
    # [변경] agent의 config_file과 이름을 분리하고 명시적 빈 값도 기본 파일로 해석한다.
    cfg = LaunchConfiguration("workstation_config_file").perform(context) or os.path.join(
        share, "config", "workstation.yaml"
    )
    a1 = LaunchConfiguration("agent1_config_file").perform(context)
    a2 = LaunchConfiguration("agent2_config_file").perform(context)
    result = [
        Node(package="cap_robot", executable=m, output="screen", parameters=[{"config_file": cfg}])
        for m in ("workstation_node", "metrics")
    ]
    result.append(
        Node(
            package="cap_robot",
            executable="cooperative_node",
            output="screen",
            parameters=[{"config_file": cfg, "agent1_config_file": a1, "agent2_config_file": a2}],
        )
    )
    if LaunchConfiguration("camera").perform(context) == "true":
        rs = os.path.join(
            get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
        )
        result.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs),
                launch_arguments={
                    "camera_namespace": "fixed_camera",
                    "camera_name": "camera",
                    "tf_prefix": "fixed_camera/",
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                }.items(),
            )
        )
        result.append(
            Node(
                package="cap_robot",
                executable="calibration_node",
                parameters=[{"config_file": os.path.join(share, "config", "calibration.yaml")}],
            )
        )
    if LaunchConfiguration("record").perform(context) == "true":
        result.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    LaunchConfiguration("bag_path").perform(context),
                    "/cap/mission_input",
                    "/cap/mission_state",
                    "/cap/agent_state",
                    "/cap/permit",
                    "/cap/assignment",
                    "/cap/cooperative_assignment",
                    "/cap/cooperative_go",
                    "/cap/estop",
                    "/cap/kill",
                    "/cap/metrics",
                    "/agent1/observations",
                    "/agent2/observations",
                    "/agent1/command",
                    "/agent2/command",
                    "/agent1/command_result",
                    "/agent2/command_result",
                    "/agent1/gate_state",
                    "/agent2/gate_state",
                    "/tf",
                    "/tf_static",
                ],
                output="screen",
            )
        )
    return result


def generate_launch_description():
    share = get_package_share_directory("cap_robot")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "workstation_config_file",
                default_value=os.path.join(share, "config", "workstation.yaml"),
            ),
            DeclareLaunchArgument(
                "agent1_config_file", default_value=os.path.join(share, "config", "agent1.yaml")
            ),
            DeclareLaunchArgument(
                "agent2_config_file", default_value=os.path.join(share, "config", "agent2.yaml")
            ),
            DeclareLaunchArgument("camera", default_value="false"),
            DeclareLaunchArgument("record", default_value="false"),
            DeclareLaunchArgument(
                "bag_path", default_value=os.path.expanduser("~/.ros/cap_robot/bag")
            ),
            OpaqueFunction(function=start),
        ]
    )
