"""Launch CogProj camera input and detection nodes in TEST/MOCK mode.

This launch file activates MockDetector to generate synthetic test detections
from incoming camera frames without loading any real machine learning framework.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = Node(
        package="cogproj_camera",
        executable="camera_input_node",
        name="camera_input_node",
        output="screen",
        parameters=[{
            "input_topic": "/pi_camera/image_raw",
            "output_topic": "/cogproj/image_raw",
        }],
    )

    detection_node = Node(
        package="cogproj_detection",
        executable="detection_node",
        name="detection_node",
        output="screen",
        parameters=[{
            "enabled": True,
            "test_mode": True,
            "image_topic": "/cogproj/image_raw",
            "detection_topic": "/cogproj/detections",
            "confidence_threshold": 0.50,
        }],
    )

    return LaunchDescription([
        camera_node,
        detection_node,
    ])

