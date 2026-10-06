"""Launch CogProj camera input and detection nodes with safe default configuration."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("cogproj_bringup")
    config_file = os.path.join(bringup_dir, "config", "cogproj_config.yaml")

    camera_node = Node(
        package="cogproj_camera",
        executable="camera_input_node",
        name="camera_input_node",
        output="screen",
        parameters=[config_file],
    )

    detection_node = Node(
        package="cogproj_detection",
        executable="detection_node",
        name="detection_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        camera_node,
        detection_node,
    ])
