"""Launch complete CogProj perception pipeline in TEST/MOCK mode.

Launches all 5 nodes:
- camera_input_node
- detection_node (with enabled=True, test_mode=True using MockDetector)
- tracking_node
- counting_node
- visualizer_node

Allows end-to-end testing of data flow, tracking, counting, and HUD visualization
without requiring real ML runtimes or trained model weights.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("cogproj_bringup")
    config_file = os.path.join(bringup_dir, "config", "cogproj_config.yaml")

    camera_input_topic_arg = DeclareLaunchArgument(
        "camera_input_topic",
        default_value="/pi_camera/image_raw",
        description="Source camera topic from Beetle Bot driver (e.g. /pi_camera/image_raw or /camera/image_raw)",
    )

    camera_node = Node(
        package="cogproj_camera",
        executable="camera_input_node",
        name="camera_input_node",
        output="screen",
        parameters=[
            config_file,
            {
                "input_topic": LaunchConfiguration("camera_input_topic"),
            },
        ],
    )

    detection_node = Node(
        package="cogproj_detection",
        executable="detection_node",
        name="detection_node",
        output="screen",
        parameters=[
            config_file,
            {
                "enabled": True,
                "test_mode": True,
            },
        ],
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
        camera_input_topic_arg,
        camera_node,
        detection_node,
        tracking_node,
        counting_node,
        visualizer_node,
    ])

