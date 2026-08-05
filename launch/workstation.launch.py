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

    return LaunchDescription([
        fixed_camera,
        aruco_calib,
    ])