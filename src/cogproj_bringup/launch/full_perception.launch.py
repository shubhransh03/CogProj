"""Launch full CogProj perception pipeline: camera, detection, tracking, counting, and visualizer."""

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

    tracking_node = Node(
        package="cogproj_tracking",
        executable="tracking_node",
        name="tracking_node",
        output="screen",
        parameters=[config_file],
    )

    counting_node = Node(
        package="cogproj_counting",
        executable="counting_node",
        name="counting_node",
        output="screen",
        parameters=[config_file],
    )

    visualizer_node = Node(
        package="cogproj_visualization",
        executable="visualizer_node",
        name="visualizer_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        camera_node,
        detection_node,
        tracking_node,
        counting_node,
        visualizer_node,
    ])

