"""Launch file for Foxglove Bridge WebSocket server in CogProj.

Enables secure remote telemetry, video streams, pointclouds, and diagnostics
streaming to Foxglove Studio or Web visualization clients over WebSockets.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8765",
        description="Port for Foxglove WebSocket server (default: 8765)",
    )

    address_arg = DeclareLaunchArgument(
        "address",
        default_value="127.0.0.1",
        description="Listening address (default 127.0.0.1 for SSH tunneling; use 0.0.0.0 for LAN)",
    )

    send_buffer_limit_arg = DeclareLaunchArgument(
        "send_buffer_limit",
        default_value="10000000",
        description="WebSocket send buffer limit in bytes (default: 10MB)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (default: false)",
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "address": LaunchConfiguration("address"),
                "send_buffer_limit": LaunchConfiguration("send_buffer_limit"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            port_arg,
            address_arg,
            send_buffer_limit_arg,
            use_sim_time_arg,
            LogInfo(
                msg=[
                    "[Foxglove Bridge] Starting WebSocket bridge on ",
                    LaunchConfiguration("address"),
                    ":",
                    LaunchConfiguration("port"),
                ]
            ),
            foxglove_node,
        ]
    )

