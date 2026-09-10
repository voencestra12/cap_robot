"""역할: 노트북 한 대의 독립 에이전트 스택. 인터페이스: agent/config_file/camera/handeye."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    share = get_package_share_directory("cap_robot")
    agent = LaunchConfiguration("agent").perform(context)
    if agent not in ("agent1", "agent2"):
        raise ValueError("agent1 or agent2 required")
    path = LaunchConfiguration("config_file").perform(context) or os.path.join(
        share, "config", agent + ".yaml"
    )
    nodes = [
        Node(
            package="cap_robot",
            executable=m,
            namespace=agent,
            output="screen",
            parameters=[{"config_file": path}],
        )
        for m in ("safety_node", "perception_node", "agent_node")
    ]
    if LaunchConfiguration("camera").perform(context) == "true":
        rs = os.path.join(
            get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
        )
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs),
                launch_arguments={
                    "camera_namespace": agent,
                    "camera_name": "camera",
                    "tf_prefix": agent + "/",
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                    "serial_no": LaunchConfiguration("camera_serial").perform(context),
                }.items(),
            )
        )
    if LaunchConfiguration("handeye").perform(context) == "true":
        # [변경] 별도 xarm_driver 제어 연결 없이 안전 노드의 joint_states만 사용한다.
        description = ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    os.path.join(share, "urdf", "xarm_tf_only.urdf.xacro"),
                    " agent_base:=robot_",
                    agent[-1],
                    "_base prefix:=",
                    agent,
                    "_",
                ]
            ),
            value_type=str,
        )
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=agent,
                parameters=[{"robot_description": description}],
                remappings=[("joint_states", f"/{agent}/joint_states")],
            )
        )
        nodes.append(
            Node(
                package="cap_robot",
                executable="mount_tf",
                namespace=agent,
                parameters=[
                    {
                        "config_file": LaunchConfiguration("handeye_file").perform(context)
                        or os.path.join(share, "config", "handeye_" + agent + ".yaml")
                    }
                ],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("agent", default_value="agent1"),
            DeclareLaunchArgument("config_file", default_value=""),
            DeclareLaunchArgument("camera", default_value="false"),
            DeclareLaunchArgument("camera_serial", default_value=""),
            DeclareLaunchArgument("handeye", default_value="false"),
            DeclareLaunchArgument("handeye_file", default_value=""),
            OpaqueFunction(function=start),
        ]
    )
