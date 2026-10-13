"""Launch file for CogProj Operator Console and Dashboard Gateway Node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    web_host_arg = DeclareLaunchArgument(
        "web_host",
        default_value="127.0.0.1",
        description="Host address to bind dashboard web server (default 127.0.0.1 for SSH tunneling)",
    )

    web_port_arg = DeclareLaunchArgument(
        "web_port",
        default_value="8080",
        description="HTTP port for dashboard web server (default: 8080)",
    )

    max_linear_x_arg = DeclareLaunchArgument(
        "max_linear_x",
        default_value="0.10",
        description="Ceiling linear velocity in m/s (default: 0.10)",
    )

    max_angular_z_arg = DeclareLaunchArgument(
        "max_angular_z",
        default_value="0.25",
        description="Ceiling angular velocity in rad/s (default: 0.25)",
    )

    target_topic_arg = DeclareLaunchArgument(
        "target_cmd_vel_topic",
        default_value="/cmd_vel_joy",
        description="Documented pre-gate input topic for safety gate (default: /cmd_vel_joy)",
    )

    gateway_node = Node(
        package="cogproj_dashboard",
        executable="dashboard_node",
        name="cogproj_dashboard_node",
        output="screen",
        parameters=[
            {
                "web_host": LaunchConfiguration("web_host"),
                "web_port": LaunchConfiguration("web_port"),
                "max_linear_x": LaunchConfiguration("max_linear_x"),
                "max_angular_z": LaunchConfiguration("max_angular_z"),
                "target_cmd_vel_topic": LaunchConfiguration("target_cmd_vel_topic"),
            }
        ],
    )

    return LaunchDescription(
        [
            web_host_arg,
            web_port_arg,
            max_linear_x_arg,
            max_angular_z_arg,
            target_topic_arg,
            LogInfo(
                msg=[
                    "[CogProj Dashboard] Starting Operator Console on http://",
                    LaunchConfiguration("web_host"),
                    ":",
                    LaunchConfiguration("web_port"),
                ]
            ),
            gateway_node,
        ]
    )
