"""Launch CogProj perception pipeline with the REAL ONNX ore detector.

Launches all 5 nodes using the trained YOLOv8s model via OpenCV DNN:
- camera_input_node
- detection_node (with enabled=True, test_mode=False, model_directory set)
- tracking_node
- counting_node
- visualizer_node

IMPORTANT: This launch file enables real-model inference.
The model must be available at the configured path.
Inference latency is approximately 1.25 seconds per frame on the Pi 5 CPU.

This is a PERCEPTION-ONLY launch file. No movement, navigation, SLAM,
actuation, or vehicle-control components are launched.
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
        default_value="/camera/image_raw",
        description="Source camera topic (e.g. /camera/image_raw or /pi_camera/image_raw)",
    )

    model_directory_arg = DeclareLaunchArgument(
        "model_directory",
        default_value="/home/veerobot/CogProj/models/ore_yolo",
        description="Path to directory containing adapter.py and config.yaml for the ONNX detector",
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.50",
        description="Minimum confidence threshold for detection results",
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
                "test_mode": False,
                "model_directory": LaunchConfiguration("model_directory"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
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
        model_directory_arg,
        confidence_threshold_arg,
        camera_node,
        detection_node,
        tracking_node,
        counting_node,
        visualizer_node,
    ])

