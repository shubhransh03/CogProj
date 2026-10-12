from typing import Any
import pytest
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, LaserScan
from visualization_msgs.msg import Marker

from cogproj_interfaces.msg import TrackedOre
from cogproj_spatial.spatial_associator import SpatialAssociator


def create_test_camera_info(
    width: int = 640,
    height: int = 480,
    fx: float = 500.0,
    fy: float = 500.0,
    cx: float = 320.0,
    cy: float = 240.0
) -> CameraInfo:
    """Helper to create a CameraInfo message with specified intrinsics."""
    info = CameraInfo()
    info.width = width
    info.height = height
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    return info


class MockTFBuffer:
    """Mock TF2 buffer for deterministic testing."""

    def __init__(self, should_fail: bool = False) -> None:
        self.should_fail = should_fail
        self.transforms = {}

    def add_transform(self, target_frame: str, source_frame: str, tx: float, ty: float, tz: float, qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0) -> None:
        tf = TransformStamped()
        tf.header.frame_id = target_frame
        tf.child_frame_id = source_frame
        tf.transform.translation.x = tx
        tf.transform.translation.y = ty
        tf.transform.translation.z = tz
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.transforms[(target_frame, source_frame)] = tf

    def lookup_transform(self, target_frame: str, source_frame: str, time: Any = None) -> TransformStamped:
        if self.should_fail:
            raise RuntimeError(f"TF lookup failed: transform from {source_frame} to {target_frame} does not exist.")
        key = (target_frame, source_frame)
        if key in self.transforms:
            return self.transforms[key]
        # Return identity if not explicitly specified
        tf = TransformStamped()
        tf.header.frame_id = target_frame
        tf.child_frame_id = source_frame
        tf.transform.rotation.w = 1.0
        return tf

    def get_latest_time(self):
        return None


def create_test_laser_scan(distance: float = 2.0) -> LaserScan:
    """Create a LaserScan with a forward obstacle at specified distance."""
    scan = LaserScan()
    scan.header.frame_id = "lidar_link"
    scan.angle_min = -np.pi
    scan.angle_max = np.pi
    scan.angle_increment = np.pi / 180.0  # 360 points
    scan.range_min = 0.1
    scan.range_max = 12.0
    scan.ranges = [distance] * 360
    return scan


def create_test_tracked_ore(
    track_id: int = 1,
    class_name: str = "resource",
    x_min: float = 300.0,
    y_min: float = 200.0,
    x_max: float = 340.0,
    y_max: float = 260.0
) -> TrackedOre:
    """Create a TrackedOre message centered around (320, 240)."""
    ore = TrackedOre()
    ore.track_id = track_id
    ore.class_id = 0 if class_name == "resource" else 1
    ore.class_name = class_name
    ore.confidence = 0.88
    ore.x_min = x_min
    ore.y_min = y_min
    ore.x_max = x_max
    ore.y_max = y_max
    ore.center_x = (x_min + x_max) / 2.0
    ore.center_y = (y_min + y_max) / 2.0
    ore.state = "CONFIRMED"
    return ore


def test_spatial_associator_full_success():
    """Test end-to-end spatial association when all geometry and sensors are valid."""
    associator = SpatialAssociator(
        target_frame="map",
        base_frame="base_footprint",
        camera_optical_frame="camera_optical_link",
        lidar_frame="lidar_link"
    )

    cam_info = create_test_camera_info(640, 480, 500.0, 500.0, 320.0, 240.0)
    cam_info.header.frame_id = "camera_optical_link"

    # Laser scan with 2.0m forward reading
    scan = create_test_laser_scan(distance=2.0)

    # Robot TF setup
    tf_buffer = MockTFBuffer()
    # lidar_link -> camera_optical_link (rotates X_lidar forward to Z_cam forward)
    tf_buffer.add_transform("camera_optical_link", "lidar_link", 0.0, 0.0, 0.0, qx=0.5, qy=-0.5, qz=0.5, qw=0.5)
    # camera_optical_link -> map: robot is at (5.0, 3.0, 0.0) in map
    tf_buffer.add_transform("map", "camera_optical_link", 5.0, 3.0, 0.0)
    tf_buffer.add_transform("base_footprint", "camera_optical_link", 0.15, 0.0, 0.1)

    ore = create_test_tracked_ore(track_id=1, class_name="resource")

    header = Header()
    header.stamp.sec = 100
    header.stamp.nanosec = 0

    spatial_array, markers, diag = associator.process_observation(
        tracked_ores=[ore],
        scan=scan,
        camera_info=cam_info,
        tf_buffer=tf_buffer,
        observation_header=header
    )

    assert spatial_array.localized_count == 1
    assert spatial_array.unlocalized_count == 0
    assert len(spatial_array.ores) == 1

    sp = spatial_array.ores[0]
    assert sp.is_localized is True
    assert sp.localization_status.startswith("LOCALIZED")
    assert sp.track_id == 1
    assert sp.class_name == "resource"
    assert abs(sp.estimated_distance - 2.0) < 0.1

    # Map coordinates should reflect robot position + object offset
    assert abs(sp.position_map.x - (5.0 + 0.0)) < 0.5
    assert abs(sp.position_map.y - (3.0 + 0.0)) < 0.5

    # Markers should be generated for localized ore
    assert len(markers.markers) == 2  # SPHERE and TEXT
    sphere_marker = [m for m in markers.markers if m.type == Marker.SPHERE][0]
    assert sphere_marker.header.frame_id == "map"
    assert sphere_marker.id == 2  # track_id * 2
    assert sphere_marker.color.g == 1.0  # Green for resource


def test_spatial_associator_uncalibrated_camera():
    """Test safety: Uncalibrated camera reports UNCALIBRATED_CAMERA and fabricates NO coordinates."""
    associator = SpatialAssociator()
    # K matrix all zeros
    cam_info = create_test_camera_info(640, 480, 0.0, 0.0, 0.0, 0.0)
    scan = create_test_laser_scan()
    tf_buffer = MockTFBuffer()
    ore = create_test_tracked_ore(track_id=2)

    spatial_array, markers, diag = associator.process_observation(
        tracked_ores=[ore],
        scan=scan,
        camera_info=cam_info,
        tf_buffer=tf_buffer
    )

    assert spatial_array.localized_count == 0
    assert spatial_array.unlocalized_count == 1
    sp = spatial_array.ores[0]
    assert sp.is_localized is False
    assert sp.localization_status == "UNCALIBRATED_CAMERA"
    # Guaranteed zero coordinates
    assert sp.position_map.x == 0.0
    assert sp.position_map.y == 0.0
    assert sp.position_map.z == 0.0
    # ZERO markers emitted
    assert len(markers.markers) == 0


def test_spatial_associator_missing_tf():
    """Test safety: Missing TF lookup reports TF_LOOKUP_FAILED and fabricates NO coordinates."""
    associator = SpatialAssociator()
    cam_info = create_test_camera_info()
    scan = create_test_laser_scan()
    tf_buffer = MockTFBuffer(should_fail=True)  # Simulates missing TF
    ore = create_test_tracked_ore(track_id=3)

    spatial_array, markers, diag = associator.process_observation(
        tracked_ores=[ore],
        scan=scan,
        camera_info=cam_info,
        tf_buffer=tf_buffer
    )

    assert spatial_array.localized_count == 0
    assert spatial_array.unlocalized_count == 1
    sp = spatial_array.ores[0]
    assert sp.is_localized is False
    assert sp.localization_status == "TF_LOOKUP_FAILED"
    assert sp.position_map.x == 0.0
    assert len(markers.markers) == 0


def test_spatial_associator_missing_scan():
    """Test safety: Missing LaserScan reports NO_LIDAR_SCAN and fabricates NO coordinates."""
    associator = SpatialAssociator()
    cam_info = create_test_camera_info()
    tf_buffer = MockTFBuffer()
    ore = create_test_tracked_ore(track_id=4)

    spatial_array, markers, diag = associator.process_observation(
        tracked_ores=[ore],
        scan=None,  # No LiDAR scan
        camera_info=cam_info,
        tf_buffer=tf_buffer
    )

    assert spatial_array.localized_count == 0
    assert spatial_array.unlocalized_count == 1
    sp = spatial_array.ores[0]
    assert sp.is_localized is False
    assert sp.localization_status == "NO_LIDAR_SCAN"
    assert sp.position_map.x == 0.0
    assert len(markers.markers) == 0


def test_spatial_associator_box_no_lidar_hits():
    """Test safety: Bounding box with no intersecting LiDAR points reports NO_LIDAR_POINTS and fabricates NO coordinates."""
    associator = SpatialAssociator()
    cam_info = create_test_camera_info()
    # Laser scan with points only at 10m away
    scan = create_test_laser_scan(distance=10.0)
    tf_buffer = MockTFBuffer()

    # Bbox in corner [10, 10, 50, 50] where forward 0-deg laser ray does not hit
    ore = create_test_tracked_ore(track_id=5, x_min=10.0, y_min=10.0, x_max=50.0, y_max=50.0)

    spatial_array, markers, diag = associator.process_observation(
        tracked_ores=[ore],
        scan=scan,
        camera_info=cam_info,
        tf_buffer=tf_buffer
    )

    assert spatial_array.localized_count == 0
    assert spatial_array.unlocalized_count == 1
    sp = spatial_array.ores[0]
    assert sp.is_localized is False
    assert sp.localization_status == "NO_LIDAR_POINTS"
    assert sp.position_map.x == 0.0
    assert len(markers.markers) == 0


def test_marker_cleanup_on_track_loss():
    """Test that when a localized track is lost, a DELETE marker is sent to clean up RViz."""
    associator = SpatialAssociator()
    cam_info = create_test_camera_info()
    scan = create_test_laser_scan()
    tf_buffer = MockTFBuffer()
    tf_buffer.add_transform("camera_optical_link", "lidar_link", 0.0, 0.0, 0.0, qx=0.5, qy=-0.5, qz=0.5, qw=0.5)

    ore1 = create_test_tracked_ore(track_id=10)

    # Frame 1: track 10 localized
    associator.process_observation([ore1], scan, cam_info, tf_buffer)

    # Frame 2: track 10 lost (empty tracks list)
    _, markers2, _ = associator.process_observation([], scan, cam_info, tf_buffer)

    # Must contain DELETE markers for track 10
    delete_markers = [m for m in markers2.markers if m.action == Marker.DELETE]
    assert len(delete_markers) == 2  # sphere + text
    assert delete_markers[0].id == 20
    assert delete_markers[1].id == 21
