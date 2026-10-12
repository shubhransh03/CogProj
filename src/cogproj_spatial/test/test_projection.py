"""Unit tests for LiDAR-Camera projection and calibration validation."""

import pytest
import numpy as np
from sensor_msgs.msg import CameraInfo, LaserScan
from geometry_msgs.msg import TransformStamped

from cogproj_spatial.projection import (
    validate_camera_info,
    scan_to_cartesian_points,
    quaternion_to_rotation_matrix,
    transform_points,
    project_points_to_image,
    associate_box_with_points,
)


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
    # K is 3x3 row-major: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    return info


def test_validate_camera_info_valid():
    """Test that properly calibrated CameraInfo passes validation."""
    info = create_test_camera_info(640, 480, 525.0, 525.0, 319.5, 239.5)
    is_valid, msg, intrinsics = validate_camera_info(info)
    assert is_valid is True
    assert intrinsics['fx'] == 525.0
    assert intrinsics['fy'] == 525.0
    assert intrinsics['cx'] == 319.5
    assert intrinsics['cy'] == 239.5
    assert intrinsics['width'] == 640.0
    assert intrinsics['height'] == 480.0


def test_validate_camera_info_uncalibrated_zeros():
    """Test that uncalibrated camera with all zeros is rejected."""
    info = create_test_camera_info(640, 480, 0.0, 0.0, 0.0, 0.0)
    is_valid, msg, intrinsics = validate_camera_info(info)
    assert is_valid is False
    assert "uncalibrated" in msg.lower()
    assert intrinsics == {}


def test_validate_camera_info_none():
    """Test that None CameraInfo is rejected gracefully."""
    is_valid, msg, intrinsics = validate_camera_info(None)
    assert is_valid is False
    assert "None" in msg
    assert intrinsics == {}


def test_validate_camera_info_zero_dimensions():
    """Test that zero dimensions are rejected."""
    info = create_test_camera_info(0, 0, 500.0, 500.0, 320.0, 240.0)
    is_valid, msg, intrinsics = validate_camera_info(info)
    assert is_valid is False
    assert "resolution" in msg.lower()


def test_scan_to_cartesian_points_filtering():
    """Test that scan conversion properly filters NaNs, Infs, and out-of-range values."""
    scan = LaserScan()
    scan.angle_min = -np.pi / 2.0
    scan.angle_max = np.pi / 2.0
    scan.angle_increment = np.pi / 4.0  # 5 angles: -90, -45, 0, 45, 90 deg
    scan.range_min = 0.2
    scan.range_max = 5.0

    # 5 readings: [valid, nan, inf, too_close, valid]
    scan.ranges = [1.0, float('nan'), float('inf'), 0.05, 2.0]

    points, ranges = scan_to_cartesian_points(scan, min_range=0.2, max_range=5.0)

    # Only readings 0 (-90 deg, r=1.0) and 4 (+90 deg, r=2.0) should be valid
    assert len(points) == 2
    assert len(ranges) == 2

    # Angle -90 deg: cos(-pi/2)=0, sin(-pi/2)=-1 -> (0, -1, 0)
    np.testing.assert_allclose(points[0], [0.0, -1.0, 0.0], atol=1e-6)

    # Angle +90 deg: cos(pi/2)=0, sin(pi/2)=+1 -> (0, +2, 0)
    np.testing.assert_allclose(points[1], [0.0, 2.0, 0.0], atol=1e-6)


def test_transform_points():
    """Test rigid body transformation of 3D points."""
    points = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ], dtype=np.float64)

    tf = TransformStamped()
    tf.transform.translation.x = 2.0
    tf.transform.translation.y = 3.0
    tf.transform.translation.z = -1.0
    # Identity rotation (w=1)
    tf.transform.rotation.w = 1.0

    transformed = transform_points(points, tf)
    expected = np.array([
        [3.0, 3.0, -1.0],
        [2.0, 4.0, -1.0],
    ])
    np.testing.assert_allclose(transformed, expected, atol=1e-6)


def test_project_points_to_image():
    """Test pinhole projection into pixel coordinates."""
    # Camera optical frame: X right, Y down, Z forward
    points_cam = np.array([
        [0.0, 0.0, 2.0],       # Exactly center at depth 2m -> u=320, v=240
        [1.0, 0.5, 2.0],       # Right 1m, down 0.5m -> u = 500*(1/2)+320 = 570, v = 500*(0.5/2)+240 = 365
        [0.0, 0.0, -1.0],      # Behind camera -> must be rejected
        [10.0, 0.0, 2.0],      # Way off to the right -> projects outside 640 width -> rejected
    ], dtype=np.float64)

    indices, uv, depths = project_points_to_image(
        points_cam,
        fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        width=640, height=480, min_depth=0.1
    )

    # Only points 0 and 1 should project within FOV
    assert len(indices) == 2
    np.testing.assert_array_equal(indices, [0, 1])
    np.testing.assert_allclose(uv[0], [320.0, 240.0], atol=1e-6)
    np.testing.assert_allclose(uv[1], [570.0, 365.0], atol=1e-6)
    np.testing.assert_allclose(depths, [2.0, 2.0], atol=1e-6)


def test_associate_box_with_points_success():
    """Test valid association of bounding box containing LiDAR points."""
    projected_uv = np.array([
        [320.0, 240.0],  # Inside box [300, 200, 350, 260]
        [325.0, 245.0],  # Inside box
        [100.0, 100.0],  # Outside box
    ])
    points_cam = np.array([
        [0.0, 0.0, 2.0],
        [0.05, 0.05, 2.05],
        [-1.0, -1.0, 3.0],
    ])
    depths = np.array([2.0, 2.05, 3.0])

    bbox = (300.0, 200.0, 350.0, 260.0)
    result = associate_box_with_points(bbox, projected_uv, points_cam, depths)

    assert result is not None
    pt_cam, est_depth, count = result
    assert count == 2
    assert 2.0 <= est_depth <= 2.05
    assert abs(pt_cam[2] - est_depth) < 0.1


def test_associate_box_with_points_no_points_returns_none():
    """CRITICAL SAFETY TEST: Verify that box with NO LiDAR points returns None and NEVER fabricates depth."""
    projected_uv = np.array([
        [100.0, 100.0],
        [150.0, 150.0],
    ])
    points_cam = np.array([
        [-1.0, -1.0, 3.0],
        [-0.8, -0.8, 3.0],
    ])
    depths = np.array([3.0, 3.0])

    # Box is at bottom right [500, 400, 600, 470] - zero points fall inside!
    bbox = (500.0, 400.0, 600.0, 470.0)
    result = associate_box_with_points(bbox, projected_uv, points_cam, depths)

    # Must return None!
    assert result is None
