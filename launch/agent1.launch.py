import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from uf_ros_lib.uf_robot_utils import generate_robot_api_params


def _load_agent_parameters(config_path: str, node_fqn: str) -> dict:
    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    try:
        return data[node_fqn]["ros__parameters"]
    except KeyError as exc:
        raise RuntimeError(
            f"Missing {node_fqn}.ros__parameters in {config_path}"
        ) from exc


def generate_launch_description():
    agent_ns = "agent1"
    node_fqn = "/agent1/robot_agent_node"
    xarm_prefix = "agent1_"

    cap_share = get_package_share_directory("cap_robot")
    realsense_share = get_package_share_directory("realsense2_camera")
    xarm_api_share = get_package_share_directory("xarm_api")

    realsense_launch = os.path.join(
        realsense_share,
        "launch",
        "rs_launch.py",
    )
    agent_config = os.path.join(cap_share, "config", "agent1.yaml")
    xarm_tf_xacro = os.path.join(cap_share, "urdf", "xarm_tf_only.urdf.xacro")
    default_handeye_yaml = os.path.join(cap_share, "config", "handeye_agent1.yaml")

    agent_params = _load_agent_parameters(agent_config, node_fqn)
    robot_ip = str(agent_params["robot_ip"])
    robot_base_frame = str(agent_params["robot_base_frame"])

    # xarm_api의 기본/사용자 파라미터를 그대로 사용하되,
    # driver 노드는 Agent별 namespace 아래에 둔다.
    xarm_driver_params = generate_robot_api_params(
        os.path.join(xarm_api_share, "config", "xarm_params.yaml"),
        os.path.join(xarm_api_share, "config", "xarm_user_params.yaml"),
        ros_namespace=agent_ns,
        node_name="ufactory_driver",
    )

    # 실제 제어는 robot_agent.py의 XArmAPI가 담당한다.
    # 이 driver는 joint state / robot state 공급용으로만 사용한다.
    xarm_state_driver = Node(
        package="xarm_api",
        executable="xarm_driver_node",
        namespace=agent_ns,
        name="ufactory_driver",
        output="screen",
        emulate_tty=True,
        parameters=[
            xarm_driver_params,
            {
                "robot_ip": robot_ip,
                "report_type": "normal",
                "dof": 6,
                "add_gripper": False,
                "add_bio_gripper": False,
                "hw_ns": "xarm",
                "prefix": xarm_prefix,
                "joint_states_rate": 10,
            },
        ],
    )

    # cap_robot의 wrapper xacro가 robot_1_base를 실제 URDF root로 선언한다.
    # 따라서 world -> link_base를 만들지 않고
    # robot_1_base -> agent1_link_base -> ... -> agent1_link6를 만든다.
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            xarm_tf_xacro,
            " agent_base:=", robot_base_frame,
            " prefix:=", xarm_prefix,
            " hw_ns:=xarm",
            " dof:=6",
            " robot_type:=xarm",
            " add_gripper:=false",
        ]),
        value_type=str,
    )

    xarm_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="agent1_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        remappings=[
            ("joint_states", "/agent1/xarm/joint_states"),
        ],
    )

    mount_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            "camera_namespace": "agent1",
            "camera_name": "camera",
            "tf_prefix": "agent1/",
            "align_depth.enable": "true",
            "enable_sync": "true",
        }.items(),
    )

    enable_handeye_tf = LaunchConfiguration("enable_handeye_tf")
    handeye_yaml = LaunchConfiguration("handeye_yaml")

    handeye_static_tf = Node(
        package="cap_robot",
        executable="mount_tf",
        namespace=agent_ns,
        name="handeye_static_tf",
        output="screen",
        parameters=[{"result_yaml": handeye_yaml}],
        condition=IfCondition(enable_handeye_tf),
    )

    enable_robot_agent = LaunchConfiguration("enable_robot_agent")

    robot_agent = Node(
        package="cap_robot",
        executable="robot_agent",
        namespace="agent1",
        name="robot_agent_node",
        output="screen",
        parameters=[agent_config],
        condition=IfCondition(enable_robot_agent),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_robot_agent",
            default_value="true",
            description="Start the existing XArmAPI-based robot_agent node. Set false for TF-only integration tests.",
        ),
        DeclareLaunchArgument(
            "enable_handeye_tf",
            default_value="true",
            description="Publish Agent 1 link6 -> camera_link hand-eye static TF.",
        ),
        DeclareLaunchArgument(
            "handeye_yaml",
            default_value=default_handeye_yaml,
            description="Agent 1 hand-eye YAML path.",
        ),
        xarm_state_driver,
        xarm_robot_state_publisher,
        mount_camera,
        handeye_static_tf,
        robot_agent,
    ])
