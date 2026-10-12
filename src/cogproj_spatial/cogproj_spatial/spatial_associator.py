"""Spatial Associator engine for CogProj.

Associates 2D tracked ore detections with 3D physical coordinates in the
SLAM map frame using timestamp-aware TF2 transforms and LiDAR-camera fusion.
"""

from typing import Dict, List, Optional, Tuple, Any, Set
import numpy as np

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Vector3, TransformStamped
from sensor_msgs.msg import CameraInfo, LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from cogproj_interfaces.msg import TrackedOre, SpatialOre, SpatialOreArray
from cogproj_spatial.projection import (
    validate_camera_info,
    scan_to_cartesian_points,
    transform_points,
    project_points_to_image,
    associate_box_with_points,
)


class SpatialAssociator:
    """Core spatial association engine."""

    # Color definitions (RGBA)
    COLOR_RESOURCE = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.85)     # Green
    COLOR_HOST_ROCK = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.85)   # Amber / Orange
    COLOR_DEFAULT = ColorRGBA(r=0.5, g=0.5, b=1.0, a=0.85)     # Light Blue
    COLOR_WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)

    def __init__(
        self,
        target_frame: str = "map",
        base_frame: str = "base_footprint",
        camera_optical_frame: str = "camera_optical_link",
        lidar_frame: str = "lidar_link",
        max_timestamp_skew_sec: float = 0.5,
        min_range: float = 0.1,
        max_range: float = 12.0,
        cluster_tolerance_m: float = 0.35,
        marker_lifetime_sec: float = 2.0,
        position_smoothing_alpha: float = 0.7,
    ) -> None:
        """Initialize SpatialAssociator with verified geometric parameters."""
        self.target_frame = target_frame
        self.base_frame = base_frame
        self.camera_optical_frame = camera_optical_frame
        self.lidar_frame = lidar_frame
        self.max_timestamp_skew_sec = max_timestamp_skew_sec
        self.min_range = min_range
        self.max_range = max_range
        self.cluster_tolerance_m = cluster_tolerance_m
        self.marker_lifetime_sec = marker_lifetime_sec
        self.position_smoothing_alpha = position_smoothing_alpha

        # Track history: track_id -> smoothed position in map frame
        self._track_spatial_cache: Dict[int, Dict[str, Any]] = {}
        # Active marker IDs for cleanup
        self._active_marker_ids: Set[int] = set()

    def reset(self) -> None:
        """Reset internal caches and tracking histories."""
        self._track_spatial_cache.clear()
        self._active_marker_ids.clear()

    def process_observation(
        self,
        tracked_ores: List[TrackedOre],
        scan: Optional[LaserScan],
        camera_info: Optional[CameraInfo],
        tf_buffer: Any,
        observation_header: Optional[Header] = None,
    ) -> Tuple[SpatialOreArray, MarkerArray, Dict[str, Any]]:
        """Process an observation frame and associate tracks with map coordinates.

        Args:
            tracked_ores: List of TrackedOre messages from the tracker.
            scan: Latest LaserScan message (if available).
            camera_info: Latest CameraInfo message (if available).
            tf_buffer: TF2 Buffer with lookup_transform method.
            observation_header: Optional Header for timestamp alignment.

        Returns:
            Tuple of:
            - SpatialOreArray: Structured ROS 2 message.
            - MarkerArray: RViz visualization markers.
            - Diagnostics dictionary: Health and status summary.
        """
        output_array = SpatialOreArray()
        if observation_header is not None:
            output_array.header.stamp = observation_header.stamp
            output_array.header.frame_id = self.target_frame
        elif tracked_ores and len(tracked_ores) > 0:
            output_array.header.frame_id = self.target_frame

        diagnostics: Dict[str, Any] = {
            "total_tracks": len(tracked_ores),
            "localized_count": 0,
            "unlocalized_count": 0,
            "camera_calibration_status": "UNKNOWN",
            "lidar_scan_status": "UNKNOWN",
            "tf_status": "UNKNOWN",
            "reasons": {},
        }

        # 1. Verify Camera Calibration
        calib_ok, calib_msg, intrinsics = validate_camera_info(camera_info)
        diagnostics["camera_calibration_status"] = "VALID" if calib_ok else "UNAVAILABLE"
        diagnostics["camera_calibration_detail"] = calib_msg

        # 2. Verify LiDAR Scan Availability & Freshness
        scan_ok = False
        scan_msg = ""
        points_lidar = np.empty((0, 3))
        if scan is None:
            scan_msg = "No LaserScan message received"
        else:
            # Check timestamp skew if observation header exists
            if observation_header is not None and scan.header.stamp.sec > 0:
                t_obs = observation_header.stamp.sec + observation_header.stamp.nanosec * 1e-9
                t_scan = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
                skew = abs(t_obs - t_scan)
                if skew > self.max_timestamp_skew_sec:
                    scan_msg = f"LiDAR scan is stale (skew: {skew:.3f}s > {self.max_timestamp_skew_sec}s)"
                else:
                    points_lidar, _ = scan_to_cartesian_points(scan, self.min_range, self.max_range)
                    scan_ok = len(points_lidar) > 0
                    scan_msg = f"Valid scan with {len(points_lidar)} points" if scan_ok else "Scan has 0 valid points"
            else:
                points_lidar, _ = scan_to_cartesian_points(scan, self.min_range, self.max_range)
                scan_ok = len(points_lidar) > 0
                scan_msg = f"Valid scan with {len(points_lidar)} points" if scan_ok else "Scan has 0 valid points"

        diagnostics["lidar_scan_status"] = "VALID" if scan_ok else "UNAVAILABLE"
        diagnostics["lidar_scan_detail"] = scan_msg

        # 3. Lookup TF Transforms
        tf_lidar_to_cam = None
        tf_cam_to_map = None
        tf_cam_to_base = None
        tf_ok = False
        tf_msg = ""

        if calib_ok and scan_ok and tf_buffer is not None:
            try:
                # Lookup LiDAR -> Camera Optical transform
                cam_frame = camera_info.header.frame_id if (camera_info and camera_info.header.frame_id) else self.camera_optical_frame
                lidar_frame = scan.header.frame_id if (scan and scan.header.frame_id) else self.lidar_frame

                tf_lidar_to_cam = tf_buffer.lookup_transform(
                    cam_frame,
                    lidar_frame,
                    observation_header.stamp if observation_header else tf_buffer.get_latest_time()
                )

                # Lookup Camera Optical -> Map transform
                tf_cam_to_map = tf_buffer.lookup_transform(
                    self.target_frame,
                    cam_frame,
                    observation_header.stamp if observation_header else tf_buffer.get_latest_time()
                )

                # Lookup Camera Optical -> Base transform
                tf_cam_to_base = tf_buffer.lookup_transform(
                    self.base_frame,
                    cam_frame,
                    observation_header.stamp if observation_header else tf_buffer.get_latest_time()
                )

                tf_ok = True
                tf_msg = f"Transforms resolved: {lidar_frame} -> {cam_frame} -> {self.target_frame}"
            except Exception as e:
                tf_ok = False
                tf_msg = f"TF lookup failed: {str(e)}"

        diagnostics["tf_status"] = "VALID" if tf_ok else "UNAVAILABLE"
        diagnostics["tf_detail"] = tf_msg

        # 4. Project LiDAR points into Image if prerequisites pass
        projected_uv = np.empty((0, 2))
        points_cam_in_fov = np.empty((0, 3))
        depths_in_fov = np.empty(0)

        if calib_ok and scan_ok and tf_ok and tf_lidar_to_cam is not None:
            points_cam_all = transform_points(points_lidar, tf_lidar_to_cam)
            fov_idx, projected_uv, depths_in_fov = project_points_to_image(
                points_cam_all,
                intrinsics['fx'],
                intrinsics['fy'],
                intrinsics['cx'],
                intrinsics['cy'],
                intrinsics['width'],
                intrinsics['height'],
                min_depth=self.min_range
            )
            if len(fov_idx) > 0:
                points_cam_in_fov = points_cam_all[fov_idx]

        # 5. Process each tracked ore
        spatial_ores: List[SpatialOre] = []
        current_frame_track_ids: Set[int] = set()

        for track in tracked_ores:
            current_frame_track_ids.add(track.track_id)
            sp_ore = SpatialOre()
            sp_ore.track_id = track.track_id
            sp_ore.class_id = track.class_id
            sp_ore.class_name = track.class_name
            sp_ore.confidence = track.confidence
            sp_ore.is_localized = False

            # Check prerequisite failures
            if not calib_ok:
                sp_ore.localization_status = "UNCALIBRATED_CAMERA"
                diagnostics["reasons"][track.track_id] = "UNCALIBRATED_CAMERA"
                spatial_ores.append(sp_ore)
                continue

            if not scan_ok:
                sp_ore.localization_status = "NO_LIDAR_SCAN"
                diagnostics["reasons"][track.track_id] = "NO_LIDAR_SCAN"
                spatial_ores.append(sp_ore)
                continue

            if not tf_ok:
                sp_ore.localization_status = "TF_LOOKUP_FAILED"
                diagnostics["reasons"][track.track_id] = "TF_LOOKUP_FAILED"
                spatial_ores.append(sp_ore)
                continue

            # Associate bounding box with projected LiDAR points
            bbox = (track.x_min, track.y_min, track.x_max, track.y_max)
            assoc_result = associate_box_with_points(
                bbox,
                projected_uv,
                points_cam_in_fov,
                depths_in_fov,
                cluster_tolerance_m=self.cluster_tolerance_m
            )

            if assoc_result is None:
                # No LiDAR points hit this bounding box
                # NEVER fabricate a distance or coordinate!
                sp_ore.localization_status = "NO_LIDAR_POINTS"
                diagnostics["reasons"][track.track_id] = "NO_LIDAR_POINTS"
                spatial_ores.append(sp_ore)
                continue

            pt_cam, est_dist, inlier_count = assoc_result

            # Transform pt_cam into Map and Base frames
            pt_cam_arr = np.array([pt_cam], dtype=np.float64)
            pt_map_arr = transform_points(pt_cam_arr, tf_cam_to_map)
            pt_base_arr = transform_points(pt_cam_arr, tf_cam_to_base)

            map_x = float(pt_map_arr[0, 0])
            map_y = float(pt_map_arr[0, 1])
            map_z = float(pt_map_arr[0, 2])

            base_x = float(pt_base_arr[0, 0])
            base_y = float(pt_base_arr[0, 1])
            base_z = float(pt_base_arr[0, 2])

            # Position smoothing across temporal track observations
            if track.track_id in self._track_spatial_cache:
                prev = self._track_spatial_cache[track.track_id]
                alpha = self.position_smoothing_alpha
                map_x = alpha * map_x + (1.0 - alpha) * prev['map_x']
                map_y = alpha * map_y + (1.0 - alpha) * prev['map_y']
                map_z = alpha * map_z + (1.0 - alpha) * prev['map_z']

            self._track_spatial_cache[track.track_id] = {
                'map_x': map_x,
                'map_y': map_y,
                'map_z': map_z,
                'class_name': track.class_name,
                'track_id': track.track_id,
            }

            sp_ore.position_map.x = map_x
            sp_ore.position_map.y = map_y
            sp_ore.position_map.z = map_z

            sp_ore.position_base.x = base_x
            sp_ore.position_base.y = base_y
            sp_ore.position_base.z = base_z

            sp_ore.estimated_distance = float(est_dist)
            sp_ore.is_localized = True
            sp_ore.localization_status = "LOCALIZED"

            diagnostics["reasons"][track.track_id] = f"LOCALIZED ({inlier_count} pts, dist {est_dist:.2f}m)"
            spatial_ores.append(sp_ore)

        # 6. Tally output counts
        loc_count = sum(1 for o in spatial_ores if o.is_localized)
        unloc_count = len(spatial_ores) - loc_count

        output_array.ores = spatial_ores
        output_array.localized_count = loc_count
        output_array.unlocalized_count = unloc_count

        diagnostics["localized_count"] = loc_count
        diagnostics["unlocalized_count"] = unloc_count

        # 7. Generate RViz Visualization Markers
        marker_array = self._generate_markers(spatial_ores, output_array.header)

        # Cleanup stale track cache entries
        stale_tracks = set(self._track_spatial_cache.keys()) - current_frame_track_ids
        for tid in stale_tracks:
            # We retain map locations of previous confirmed counts in cache
            pass

        return output_array, marker_array, diagnostics

    def _generate_markers(
        self,
        spatial_ores: List[SpatialOre],
        header: Header
    ) -> MarkerArray:
        """Create RViz visualization markers for localized ores."""
        marker_array = MarkerArray()
        current_marker_ids: Set[int] = set()

        for ore in spatial_ores:
            if not ore.is_localized:
                continue

            sphere_id = ore.track_id * 2
            text_id = ore.track_id * 2 + 1
            current_marker_ids.add(sphere_id)
            current_marker_ids.add(text_id)

            color = self.COLOR_RESOURCE if ore.class_name == "resource" else (
                self.COLOR_HOST_ROCK if ore.class_name == "host_rock" else self.COLOR_DEFAULT
            )

            # 1. 3D Sphere Marker at mapped coordinates
            sphere = Marker()
            sphere.header.frame_id = self.target_frame
            sphere.header.stamp = header.stamp
            sphere.ns = "cogproj_ores_3d"
            sphere.id = sphere_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = ore.position_map.x
            sphere.pose.position.y = ore.position_map.y
            sphere.pose.position.z = ore.position_map.z
            sphere.pose.orientation.w = 1.0
            sphere.scale = Vector3(x=0.15, y=0.15, z=0.15)
            sphere.color = color
            sphere.lifetime.sec = int(self.marker_lifetime_sec)
            sphere.lifetime.nanosec = int((self.marker_lifetime_sec % 1) * 1e9)
            marker_array.markers.append(sphere)

            # 2. Text Label Marker
            label = Marker()
            label.header.frame_id = self.target_frame
            label.header.stamp = header.stamp
            label.ns = "cogproj_ores_text"
            label.id = text_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = ore.position_map.x
            label.pose.position.y = ore.position_map.y
            label.pose.position.z = ore.position_map.z + 0.20  # Float above sphere
            label.pose.orientation.w = 1.0
            label.scale = Vector3(x=0.0, y=0.0, z=0.12)  # Text height
            label.color = self.COLOR_WHITE
            label.text = f"#{ore.track_id} {ore.class_name} ({ore.estimated_distance:.2f}m)"
            label.lifetime.sec = int(self.marker_lifetime_sec)
            label.lifetime.nanosec = int((self.marker_lifetime_sec % 1) * 1e9)
            marker_array.markers.append(label)

        # Remove markers that are no longer active
        obsolete_ids = self._active_marker_ids - current_marker_ids
        for obs_id in obsolete_ids:
            del_marker = Marker()
            del_marker.header.frame_id = self.target_frame
            del_marker.header.stamp = header.stamp
            del_marker.ns = "cogproj_ores_3d" if (obs_id % 2 == 0) else "cogproj_ores_text"
            del_marker.id = obs_id
            del_marker.action = Marker.DELETE
            marker_array.markers.append(del_marker)

        self._active_marker_ids = current_marker_ids
        return marker_array
