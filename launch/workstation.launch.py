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

    calibration_config = os.path.join(
        cap_share,
        "config",
        "calibration.yaml",
    )
    extra_perception_config = os.path.join(
        cap_share,
        "config",
        "yolo_extra_perception.yaml",
    )

    fixed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            "camera_namespace": "fixed_camera",
            "camera_name": "camera",
            "tf_prefix": "fixed_camera/",
            "align_depth.enable": "true",
            "enable_sync": "true",
        }.items(),
    )

    aruco_calib = Node(
        package="cap_robot",
        executable="aruco_calib",
        name="aruco_calib",
        output="screen",
        parameters=[calibration_config],
    )

    # [MERGED] ArUco와 분리된 바구니 손잡이 인식 노드입니다.
    yolo_extra_perception = Node(
        package="cap_robot",
        executable="yolo_extra_perception",
        name="yolo_extra_perception",
        output="screen",
        parameters=[extra_perception_config],
    )

    # A+B 공용 구역 토큰 매니저. 시스템 전체에서 정확히 1개만 실행합니다.
    # lease_sec 은 각 agent yaml 의 zone_token_lease_sec 과 일치시켜야 합니다.
    zone_token_manager = Node(
        package="cap_robot",
        executable="zone_token_manager",
        name="zone_token_manager",
        output="screen",
        parameters=[{
            "request_topic": "/zone_token/request",
            "state_topic": "/zone_token/state",
            "lease_sec": 45.0,
            "tick_period_sec": 1.0,
        }],
    )

    return LaunchDescription([
        fixed_camera,
        aruco_calib,
        yolo_extra_perception,
        zone_token_manager,
    ])
