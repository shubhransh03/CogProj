"""CogProj Coordinated Demonstration Showcase Launch File.

Launches the complete demo-ready perception, tracking, counting, visualization,
and spatial association pipeline in either MOCK mode or REAL ONNX detector mode.

SAFETY GUARANTEES:
- Defaults to safe headless operation: detector_mode='mock', start_slam='false', start_rviz='false'.
- Zero motor drivers, bridges, or joystick navigation movement nodes.
- Strictly never publishes to command velocity or robot actuation topics.
- Does not start duplicate camera drivers (consumes existing /camera/image_raw).
- Explicitly labels mock detections and simulated maps so they cannot be conflated with real ores or physical SLAM.
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

    # Locate default RViz configuration
    rviz_config_path = os.path.join(bringup_dir, "rviz", "cogproj_slam_ore.rviz")
    if not os.path.exists(rviz_config_path):
        rviz_config_path = "/home/veerobot/CogProj/src/cogproj_bringup/rviz/cogproj_slam_ore.rviz"

    # Locate SLAM Toolbox configuration
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
        description="Demonstration operational mode: 'mock' (synthetic) or 'real' (ONNX)",
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

    publish_simulated_demo_map_arg = DeclareLaunchArgument(
        "publish_simulated_demo_map",
        default_value="false",
        description="Publish isolated mock map on /cogproj/demo_simulated_map for RViz layout demo",
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
    publish_demo_map = LaunchConfiguration("publish_simulated_demo_map")

    # Mode conditions
    is_real_detector = PythonExpression(["'", detector_mode, "' == 'real'"])
    is_mock_detector = PythonExpression(["'", detector_mode, "' == 'mock'"])
    should_publish_demo_map = PythonExpression([
        "'", publish_demo_map, "' == 'true' and '", detector_mode, "' == 'mock'"
    ])

    # =========================================================================
    # Nodes
    # =========================================================================

    # 1. Camera Input Node (BGRA8 -> BGR8 conversion)
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

    # 5. Visualizer Node (passes detector_mode so overlay displays explicit mode badge)
    visualizer_node = Node(
        package="cogproj_visualization",
        executable="visualizer_node",
        name="visualizer_node",
        output="screen",
        parameters=[
            config_file,
            {"detector_mode": detector_mode},
        ],
    )

    # 6. Spatial Association Node
    spatial_association_node = Node(
        package="cogproj_spatial",
        executable="spatial_association_node",
        name="spatial_association_node",
        output="screen",
        condition=IfCondition(enable_spatial),
        parameters=[config_file],
    )

    # 7. Simulated Demo Map Node (Mock mode RViz layout demo only)
    demo_map_node = Node(
        package="cogproj_bringup",
        executable="demo_map_node",
        name="demo_simulated_map_node",
        output="screen",
        condition=IfCondition(should_publish_demo_map),
        parameters=[{
            "map_topic": "/cogproj/demo_simulated_map",
            "marker_topic": "/cogproj/demo_simulated_map_markers",
            "frame_id": "map",
        }],
    )

    # 8. SLAM Toolbox Integration (Optional mapping)
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

    # 9. RViz2 Visualization (Optional GUI)
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
        publish_simulated_demo_map_arg,

        # Startup Banner
        LogInfo(msg="============================================================"),
        LogInfo(msg="[CogProj] Launching Demo Showcase Pipeline..."),
        LogInfo(msg="[CogProj] Safety: Zero command velocity or actuation publishers."),
        LogInfo(msg="============================================================"),

        # Perception & Spatial Nodes
        camera_input_node,
        detection_node_mock,
        detection_node_real,
        tracking_node,
        counting_node,
        visualizer_node,
        spatial_association_node,
        demo_map_node,
        slam_toolbox_node,
        slam_lifecycle_manager,
        rviz_node,
    ])
