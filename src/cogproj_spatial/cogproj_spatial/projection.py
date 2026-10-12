"""LiDAR-to-Camera geometric projection and calibration verification engine.

Provides mathematically rigorous, timestamp-aware projection of 2D planar
LaserScan points into calibrated pinhole camera image space, with outlier
rejection and zero-fabrication safety guarantees.
"""

from typing import Dict, List, Optional, Tuple, Any
import numpy as np
from sensor_msgs.msg import CameraInfo, LaserScan
from geometry_msgs.msg import TransformStamped


def validate_camera_info(camera_info: Optional[CameraInfo]) -> Tuple[bool, str, Dict[str, float]]:
    """Verify whether camera intrinsics are valid for 3D-to-2D projection.

    Args:
        camera_info: CameraInfo ROS 2 message.

    Returns:
        Tuple of (is_valid, reason, intrinsics_dict).
        If is_valid is False, intrinsics_dict will be empty.
    """
    if camera_info is None:
        return False, "CameraInfo message is None (not received)", {}

    if camera_info.width == 0 or camera_info.height == 0:
        return False, f"Invalid camera resolution ({camera_info.width}x{camera_info.height})", {}

    if len(camera_info.k) < 9:
        return False, "Camera intrinsic matrix K has fewer than 9 elements", {}

    fx = float(camera_info.k[0])
    cx = float(camera_info.k[2])
    fy = float(camera_info.k[4])
    cy = float(camera_info.k[5])

    # In uncalibrated setups (such as standard uncalibrated OV5647 CSI camera),
    # K contains all zeros.
    if fx <= 0.0 or fy <= 0.0 or cx <= 0.0 or cy <= 0.0:
        return False, (
            f"Camera intrinsics are uncalibrated or non-positive: "
            f"fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}"
        ), {}

    # Check for sane values (focal length shouldn't be astronomically large or small)
    if fx > 10000.0 or fy > 10000.0 or cx > camera_info.width * 2 or cy > camera_info.height * 2:
        return False, f"Camera intrinsics appear corrupt or out of bounds: fx={fx}, fy={fy}", {}

    intrinsics = {
        'fx': fx,
        'fy': fy,
        'cx': cx,
        'cy': cy,
        'width': float(camera_info.width),
        'height': float(camera_info.height),
    }
    return True, "Valid camera calibration", intrinsics


def scan_to_cartesian_points(
    scan: LaserScan,
    min_range: float = 0.10,
    max_range: float = 12.0
) -> Tuple[np.ndarray, np.ndarray]:
    """Convert valid 2D LaserScan ranges to 3D Cartesian coordinates in LiDAR frame.

    Args:
        scan: LaserScan message.
        min_range: Minimum acceptable range threshold in meters.
        max_range: Maximum acceptable range threshold in meters.

    Returns:
        Tuple of (points_lidar, valid_ranges), where points_lidar is Nx3 numpy array
        [x, y, 0.0] and valid_ranges is 1D array of ranges.
    """
    if not scan.ranges:
        return np.empty((0, 3), dtype=np.float64), np.empty((0,), dtype=np.float64)

    ranges = np.array(scan.ranges, dtype=np.float64)
    n = len(ranges)

    angles = scan.angle_min + np.arange(n, dtype=np.float64) * scan.angle_increment

    # Filter out NaNs, Infs, and ranges outside sensor bounds
    effective_min = max(min_range, float(scan.range_min))
    effective_max = min(max_range, float(scan.range_max))

    valid_mask = (
        np.isfinite(ranges) &
        (ranges >= effective_min) &
        (ranges <= effective_max)
    )

    if not np.any(valid_mask):
        return np.empty((0, 3), dtype=np.float64), np.empty((0,), dtype=np.float64)

    valid_ranges = ranges[valid_mask]
    valid_angles = angles[valid_mask]

    x = valid_ranges * np.cos(valid_angles)
    y = valid_ranges * np.sin(valid_angles)
    z = np.zeros_like(x)

    points_lidar = np.column_stack((x, y, z))
    return points_lidar, valid_ranges


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert unit quaternion to 3x3 rotation matrix."""
    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        return np.eye(3, dtype=np.float64)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)

    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)

    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

    return np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22]
    ], dtype=np.float64)


def transform_points(points: np.ndarray, transform: TransformStamped) -> np.ndarray:
    """Transform an Nx3 array of 3D points using a TransformStamped.

    Args:
        points: Nx3 array of points.
        transform: TransformStamped from source frame to target frame.

    Returns:
        Nx3 array of points in target frame.
    """
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float64)

    tx = transform.transform.translation.x
    ty = transform.transform.translation.y
    tz = transform.transform.translation.z

    qx = transform.transform.rotation.x
    qy = transform.transform.rotation.y
    qz = transform.transform.rotation.z
    qw = transform.transform.rotation.w

    r_matrix = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    translation = np.array([tx, ty, tz], dtype=np.float64)

    # P_target = (R @ P_source.T).T + T
    transformed = (r_matrix @ points.T).T + translation
    return transformed


def project_points_to_image(
    points_cam: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    width: float,
    height: float,
    min_depth: float = 0.05
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project 3D points in camera optical frame into 2D image coordinates.

    Optical frame convention: X right, Y down, Z forward.

    Args:
        points_cam: Nx3 array of points in camera optical frame.
        fx: Focal length in X.
        fy: Focal length in Y.
        cx: Principal point X.
        cy: Principal point Y.
        width: Image width in pixels.
        height: Image height in pixels.
        min_depth: Minimum forward distance in meters.

    Returns:
        Tuple of:
        - in_fov_indices: Indices of points_cam that project inside image bounds.
        - projected_uv: Mx2 array of (u, v) pixel coordinates.
        - depths: Mx1 array of Z depths.
    """
    if len(points_cam) == 0:
        return np.empty(0, dtype=int), np.empty((0, 2)), np.empty(0)

    z = points_cam[:, 2]
    forward_mask = z > min_depth

    if not np.any(forward_mask):
        return np.empty(0, dtype=int), np.empty((0, 2)), np.empty(0)

    forward_indices = np.where(forward_mask)[0]
    pts = points_cam[forward_indices]
    z_fwd = z[forward_indices]

    u = fx * (pts[:, 0] / z_fwd) + cx
    v = fy * (pts[:, 1] / z_fwd) + cy

    # Bounds check
    in_bounds_mask = (u >= 0.0) & (u < width) & (v >= 0.0) & (v < height)
    if not np.any(in_bounds_mask):
        return np.empty(0, dtype=int), np.empty((0, 2)), np.empty(0)

    final_indices = forward_indices[in_bounds_mask]
    final_uv = np.column_stack((u[in_bounds_mask], v[in_bounds_mask]))
    final_depths = z_fwd[in_bounds_mask]

    return final_indices, final_uv, final_depths


def associate_box_with_points(
    bbox: Tuple[float, float, float, float],
    projected_uv: np.ndarray,
    points_cam: np.ndarray,
    depths: np.ndarray,
    cluster_tolerance_m: float = 0.35
) -> Optional[Tuple[np.ndarray, float, int]]:
    """Associate a 2D bounding box with projected LiDAR points.

    Args:
        bbox: (x_min, y_min, x_max, y_max) in pixel coordinates.
        projected_uv: Mx2 pixel coordinates of in-FOV LiDAR points.
        points_cam: Mx3 3D coordinates in camera frame.
        depths: Array of M forward depth values.
        cluster_tolerance_m: Maximum depth spread for inlier clustering.

    Returns:
        If valid LiDAR points fall inside the box:
            (point_3d_cam, median_depth, inlier_count)
        If no LiDAR points fall inside the box:
            None (Guarantees zero coordinate fabrication!)
    """
    if len(projected_uv) == 0 or len(points_cam) == 0:
        return None

    x_min, y_min, x_max, y_max = bbox

    # Select points inside bounding box
    inside_mask = (
        (projected_uv[:, 0] >= x_min) &
        (projected_uv[:, 0] <= x_max) &
        (projected_uv[:, 1] >= y_min) &
        (projected_uv[:, 1] <= y_max)
    )

    if not np.any(inside_mask):
        return None

    box_points_cam = points_cam[inside_mask]
    box_depths = depths[inside_mask]

    # Use median depth to reject background clutter / wall bleed behind the ore
    median_depth = float(np.median(box_depths))

    # Inliers are points within cluster_tolerance_m of the median
    inlier_mask = np.abs(box_depths - median_depth) <= cluster_tolerance_m
    inlier_points = box_points_cam[inlier_mask]

    if len(inlier_points) == 0:
        return None

    # Compute centroid of inliers in camera optical frame
    point_cam = np.median(inlier_points, axis=0)
    inlier_count = len(inlier_points)

    return point_cam, median_depth, inlier_count
