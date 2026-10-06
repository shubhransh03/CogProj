"""Launch only CogProj perception visualizer node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("cogproj_bringup")
    config_file = os.path.join(bringup_dir, "config", "cogproj_config.yaml")

    visualizer_node = Node(
        package="cogproj_visualization",
        executable="visualizer_node",
        name="visualizer_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        visualizer_node,
    ])

