"""Unified SLAM-Integrated Ore Detection and Counting Bringup Launch File.

Launches the complete software perception and spatial mapping pipeline:
1. Camera Input Node (with BGRA->BGR conversion)
2. Ore Detection Node (Mock or Real ONNX via OpenCV DNN)
3. Tracking Node (Multi-object tracking with hysteresis confirmation)
4. Counting Node (Unique session-level ore counter)
5. Visualizer Node (Headless 2D annotated and compressed video streams)
6. Spatial Association Node (LiDAR-Camera geometric projection & SLAM map localization)
7. SLAM Toolbox (Optional, off by default: starts 2D LiDAR mapping when requested)
8. RViz2 (Optional, off by default: map, TF, laser scan, tracks, and 3D ore markers)

SAFETY & DESIGN GUARANTEES:
- Safe by default: detector_mode='mock', start_slam='false', start_rviz='false'.
- Zero motor drivers, bridges, joystick nodes, or movement controllers.
- Never publishes to command velocity or robot actuation topics.
- Does not start duplicate camera drivers (subscribes to existing camera stream).
- Never fabricates 3D map coordinates if calibration, scan, or TF is unavailable.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("cogproj_bringup")
    config_file = os.path.join(bringup_dir, "config", "cogproj_config.yaml")

    # Locate default rviz configuration
    rviz_config_path = os.path.join(bringup_dir, "rviz", "cogproj_slam_ore.rviz")
    if not os.path.exists(rviz_config_path):
        # Fallback to source directory during development if not yet installed
        rviz_config_path = "/home/veerobot/CogProj/src/cogproj_bringup/rviz/cogproj_slam_ore.rviz"

    # Locate SLAM Toolbox configuration
    # Prefer existing lyra_slam config if available, fallback to cogproj config
    lyra_slam_config = "/home/veerobot/lyra_ws/src/lyra_slam/config/slam_toolbox.yaml"
    if not os.path.exists(lyra_slam_config):
        lyra_slam_config = config_file

    # =========================================================================
    # Launch Arguments
    # =========================================================================
    camera_input_topic_arg = DeclareLaunchArgument(
        "camera_input_topic",
        default_value="/camera/image_raw",
        description="Input camera topic (e.g. /camera/image_raw from camera_ros)",
    )

    detector_mode_arg = DeclareLaunchArgument(
        "detector_mode",
        default_value="mock",
        description="Detector operational mode: 'mock' (synthetic) or 'real' (ONNX)",
    )

    model_directory_arg = DeclareLaunchArgument(
        "model_directory",
        default_value="/home/veerobot/CogProj/models/ore_yolo",
        description="Directory containing adapter.py and config.yaml for ONNX detector",
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.50",
        description="Confidence threshold for ore detections",
    )

    enable_spatial_association_arg = DeclareLaunchArgument(
        "enable_spatial_association",
        default_value="true",
        description="Enable LiDAR-to-camera spatial association and map-frame localization",
    )

    start_slam_arg = DeclareLaunchArgument(
        "start_slam",
        default_value="false",
        description="Whether to start SLAM Toolbox mapping node (default: false, safe)",
    )

    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=lyra_slam_config,
        description="Path to SLAM Toolbox configuration YAML",
    )

    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="Whether to launch RViz2 for visualization (default: false, headless)",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config_path,
        description="Path to RViz configuration file",
    )

    # Launch configurations
    camera_input_topic = LaunchConfiguration("camera_input_topic")
    detector_mode = LaunchConfiguration("detector_mode")
    model_directory = LaunchConfiguration("model_directory")
    confidence_threshold = LaunchConfiguration("confidence_threshold")
    enable_spatial = LaunchConfiguration("enable_spatial_association")
    start_slam = LaunchConfiguration("start_slam")
    slam_params_file = LaunchConfiguration("slam_params_file")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # Mode conditions
    is_real_detector = PythonExpression(["'", detector_mode, "' == 'real'"])
    is_mock_detector = PythonExpression(["'", detector_mode, "' == 'mock'"])

    # =========================================================================
    # Core Perception Nodes
    # =========================================================================

    # 1. Camera Input Node
    camera_input_node = Node(
        package="cogproj_camera",
        executable="camera_input_node",
        name="camera_input_node",
        output="screen",
        parameters=[
            config_file,
            {"input_topic": camera_input_topic},
        ],
    )

    # 2a. Detection Node - Mock Mode
    detection_node_mock = Node(
        package="cogproj_detection",
        executable="detection_node",
        name="detection_node",
        output="screen",
        condition=IfCondition(is_mock_detector),
        parameters=[
            config_file,
            {
                "enabled": True,
                "test_mode": True,
                "confidence_threshold": confidence_threshold,
            },
        ],
    )

    # 2b. Detection Node - Real Mode
    detection_node_real = Node(
        package="cogproj_detection",
        executable="detection_node",
        name="detection_node",
        output="screen",
        condition=IfCondition(is_real_detector),
        parameters=[
            config_file,
            {
                "enabled": True,
                "test_mode": False,
                "model_directory": model_directory,
                "confidence_threshold": confidence_threshold,
            },
        ],
    )

    # 3. Tracking Node
    tracking_node = Node(
        package="cogproj_tracking",
        executable="tracking_node",
        name="tracking_node",
        output="screen",
        parameters=[config_file],
    )

    # 4. Counting Node
    counting_node = Node(
        package="cogproj_counting",
        executable="counting_node",
        name="counting_node",
        output="screen",
        parameters=[config_file],
    )

    # 5. Visualizer Node
    visualizer_node = Node(
        package="cogproj_visualization",
        executable="visualizer_node",
        name="visualizer_node",
        output="screen",
        parameters=[config_file],
    )

    # =========================================================================
    # Spatial Association Node
    # =========================================================================
    spatial_association_node = Node(
        package="cogproj_spatial",
        executable="spatial_association_node",
        name="spatial_association_node",
        output="screen",
        condition=IfCondition(enable_spatial),
        parameters=[config_file],
    )

    # =========================================================================
    # SLAM Toolbox Integration (Optional, safe mapping only)
    # =========================================================================
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(start_slam),
        parameters=[slam_params_file],
        remappings=[
            ("scan", "/scan"),
            ("odom", "/odometry/filtered"),
        ],
    )

    slam_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="slam_lifecycle_manager",
        output="screen",
        condition=IfCondition(start_slam),
        parameters=[{
            "autostart": True,
            "node_names": ["slam_toolbox"],
            "bond_timeout": 0.0,
        }],
    )

    # =========================================================================
    # RViz2 Visualization (Optional)
    # =========================================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        # Arguments
        camera_input_topic_arg,
        detector_mode_arg,
        model_directory_arg,
        confidence_threshold_arg,
        enable_spatial_association_arg,
        start_slam_arg,
        slam_params_file_arg,
        start_rviz_arg,
        rviz_config_arg,

        # Status Banner
        LogInfo(msg="[CogProj] Launching SLAM-Integrated Ore Pipeline..."),

        # Nodes
        camera_input_node,
        detection_node_mock,
        detection_node_real,
        tracking_node,
        counting_node,
        visualizer_node,
        spatial_association_node,
        slam_toolbox_node,
        slam_lifecycle_manager,
        rviz_node,
    ])
