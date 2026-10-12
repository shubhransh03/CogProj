import importlib.util
import os
import pytest
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

launch_path = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "launch",
    "integrated_slam_ore_pipeline.launch.py"
)
spec = importlib.util.spec_from_file_location("integrated_slam_ore_pipeline", launch_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
generate_launch_description = module.generate_launch_description


def get_default_text(arg: DeclareLaunchArgument) -> str:
    """Extract string value from DeclareLaunchArgument default_value substitutions."""
    if arg.default_value is None:
        return ""
    if isinstance(arg.default_value, (list, tuple)):
        return "".join(getattr(sub, "text", str(sub)) for sub in arg.default_value)
    return str(arg.default_value)


def test_integrated_launch_description_generation():
    """Verify launch description generates cleanly without exceptions."""
    ld = generate_launch_description()
    assert ld is not None
    assert len(ld.entities) > 0


def test_integrated_launch_declared_arguments():
    """Verify all required arguments exist with safe defaults."""
    ld = generate_launch_description()

    arg_dict = {}
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            arg_dict[entity.name] = get_default_text(entity)

    # Required arguments check
    assert "camera_input_topic" in arg_dict
    assert "detector_mode" in arg_dict
    assert "enable_spatial_association" in arg_dict
    assert "start_slam" in arg_dict
    assert "start_rviz" in arg_dict

    # Safety defaults check
    assert "mock" in arg_dict["detector_mode"]
    assert "false" in arg_dict["start_slam"].lower()
    assert "false" in arg_dict["start_rviz"].lower()


def test_integrated_launch_safety_no_movement_commands():
    """SAFETY AUDIT: Confirm zero motor, teleop, or navigation movement nodes exist."""
    ld = generate_launch_description()

    forbidden_packages = {
        "lyra_bridge",
        "lyra_control",
        "teleop_twist_joy",
        "joy",
        "nav2_bringup",
        "nav2_controller",
        "nav2_planner",
        "nav2_bt_navigator",
    }

    forbidden_topics = {
        "/cmd_vel",
        "/cmd_vel_nav",
        "/cmd_vel_joy",
        "/cmd_vel_teleop",
    }

    for entity in ld.entities:
        if isinstance(entity, Node):
            # Check package safety
            pkg = entity.node_package
            assert pkg not in forbidden_packages, f"FORBIDDEN PACKAGE DETECTED IN INTEGRATED LAUNCH: {pkg}"

            # Check remapping safety
            remappings = getattr(entity, "_Node__remappings", None)
            if remappings:
                for remap_pair in remappings:
                    if isinstance(remap_pair, (list, tuple)) and len(remap_pair) >= 2:
                        src_str = str(remap_pair[0])
                        dst_str = str(remap_pair[1])
                        for forb in forbidden_topics:
                            assert forb not in src_str, f"FORBIDDEN MOVEMENT TOPIC REMAPPING: {src_str}"
                            assert forb not in dst_str, f"FORBIDDEN MOVEMENT TOPIC REMAPPING: {dst_str}"


def test_integrated_launch_node_inventory():
    """Verify all 6 core pipeline nodes and optional mapping nodes are configured."""
    ld = generate_launch_description()

    node_packages = []

    for entity in ld.entities:
        if isinstance(entity, Node):
            node_packages.append(entity.node_package)

    # Core perception nodes
    assert "cogproj_camera" in node_packages
    assert "cogproj_detection" in node_packages
    assert "cogproj_tracking" in node_packages
    assert "cogproj_counting" in node_packages
    assert "cogproj_visualization" in node_packages
    assert "cogproj_spatial" in node_packages

    # Mapping and visualizer
    assert "slam_toolbox" in node_packages
    assert "rviz2" in node_packages
