"""Integration test for SpatialAssociationNode."""

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, LaserScan
from cogproj_interfaces.msg import TrackedOreArray, SpatialOreArray

from cogproj_spatial.spatial_node import SpatialAssociationNode


def create_test_camera_info(width=640, height=480, fx=500.0, fy=500.0, cx=320.0, cy=240.0):
    info = CameraInfo()
    info.width = width
    info.height = height
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    return info


def create_test_laser_scan():
    scan = LaserScan()
    scan.header.frame_id = "lidar_link"
    scan.angle_min = -3.14
    scan.angle_max = 3.14
    scan.angle_increment = 0.017
    scan.range_min = 0.1
    scan.range_max = 12.0
    scan.ranges = [2.0] * 360
    return scan


def create_test_tracked_ore(track_id=1):
    from cogproj_interfaces.msg import TrackedOre
    ore = TrackedOre()
    ore.track_id = track_id
    ore.class_id = 0
    ore.class_name = "resource"
    ore.confidence = 0.85
    ore.x_min = 300.0
    ore.y_min = 200.0
    ore.x_max = 340.0
    ore.y_max = 260.0
    ore.state = "CONFIRMED"
    return ore


@pytest.fixture
def rclpy_init():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_spatial_node_initialization(rclpy_init):
    """Test that SpatialAssociationNode initializes parameters and topics without crashing."""
    node = SpatialAssociationNode()
    assert node.get_name() == 'spatial_association_node'

    # Check parameter declarations
    assert node.get_parameter('target_frame').value == 'map'
    assert node.get_parameter('base_frame').value == 'base_footprint'
    assert node.get_parameter('camera_optical_frame').value == 'camera_optical_link'
    assert node.get_parameter('lidar_frame').value == 'lidar_link'

    node.destroy_node()


def test_spatial_node_uncalibrated_processing(rclpy_init):
    """Test that when an uncalibrated camera message and tracks are received, the node emits unlocalized tracks safely."""
    node = SpatialAssociationNode()

    received_arrays = []

    def spatial_cb(msg: SpatialOreArray):
        received_arrays.append(msg)

    test_sub = node.create_subscription(
        SpatialOreArray,
        '/cogproj/spatial_ores',
        spatial_cb,
        qos_profile_sensor_data
    )

    # Send uncalibrated CameraInfo
    uncal_cam = create_test_camera_info(640, 480, 0.0, 0.0, 0.0, 0.0)
    node._cam_info_callback(uncal_cam)

    # Send scan
    scan = create_test_laser_scan()
    node._scan_callback(scan)

    # Send tracks
    track_msg = TrackedOreArray()
    track_msg.header.frame_id = "camera_optical_link"
    track_msg.tracks.append(create_test_tracked_ore(track_id=1))

    node._tracks_callback(track_msg)

    # Dispatch to local subscription
    rclpy.spin_once(node, timeout_sec=0.1)

    # Verify message was processed
    assert len(received_arrays) == 1
    out = received_arrays[0]
    assert out.localized_count == 0
    assert out.unlocalized_count == 1
    assert len(out.ores) == 1
    assert out.ores[0].is_localized is False
    assert out.ores[0].localization_status == "UNCALIBRATED_CAMERA"
    assert out.ores[0].position_map.x == 0.0

    node.destroy_node()
