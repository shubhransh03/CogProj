"""Spatial Association ROS 2 Node for CogProj.

Subscribes to tracked ores, camera calibration, and LiDAR scans. Associates
detections with physical coordinates in the SLAM map frame using TF2.
"""

from typing import Optional, List, Dict, Any
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener

from sensor_msgs.msg import CameraInfo, LaserScan
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header

from cogproj_interfaces.msg import TrackedOreArray, DetectionArray, TrackedOre, SpatialOreArray
from cogproj_spatial.spatial_associator import SpatialAssociator


class SpatialAssociationNode(Node):
    """ROS 2 Node for associating 2D ore tracks with 3D map coordinates."""

    def __init__(self) -> None:
        super().__init__('spatial_association_node')

        # Declare parameters with robust defaults
        self.declare_parameter('tracked_topic', '/cogproj/tracked_ores')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_spatial_topic', '/cogproj/spatial_ores')
        self.declare_parameter('output_marker_topic', '/cogproj/spatial_markers')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_optical_frame', 'camera_optical_link')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('max_timestamp_skew_sec', 0.5)
        self.declare_parameter('min_range', 0.10)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('cluster_tolerance_m', 0.35)
        self.declare_parameter('marker_lifetime_sec', 2.0)

        # Retrieve parameter values
        tracked_topic = self.get_parameter('tracked_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        output_spatial_topic = self.get_parameter('output_spatial_topic').get_parameter_value().string_value
        output_marker_topic = self.get_parameter('output_marker_topic').get_parameter_value().string_value
        target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        camera_optical_frame = self.get_parameter('camera_optical_frame').get_parameter_value().string_value
        lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        max_timestamp_skew = self.get_parameter('max_timestamp_skew_sec').get_parameter_value().double_value
        min_range = self.get_parameter('min_range').get_parameter_value().double_value
        max_range = self.get_parameter('max_range').get_parameter_value().double_value
        cluster_tol = self.get_parameter('cluster_tolerance_m').get_parameter_value().double_value
        marker_lifetime = self.get_parameter('marker_lifetime_sec').get_parameter_value().double_value

        self.get_logger().info("Initializing SpatialAssociationNode...")
        self.get_logger().info(f"  Tracked topic:      {tracked_topic}")
        self.get_logger().info(f"  CameraInfo topic:   {camera_info_topic}")
        self.get_logger().info(f"  LiDAR scan topic:   {scan_topic}")
        self.get_logger().info(f"  Spatial output:     {output_spatial_topic}")
        self.get_logger().info(f"  Markers output:     {output_marker_topic}")
        self.get_logger().info(f"  Frames: target='{target_frame}', base='{base_frame}', cam='{camera_optical_frame}', lidar='{lidar_frame}'")

        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize core associator
        self.associator = SpatialAssociator(
            target_frame=target_frame,
            base_frame=base_frame,
            camera_optical_frame=camera_optical_frame,
            lidar_frame=lidar_frame,
            max_timestamp_skew_sec=max_timestamp_skew,
            min_range=min_range,
            max_range=max_range,
            cluster_tolerance_m=cluster_tol,
            marker_lifetime_sec=marker_lifetime,
        )

        # Cached sensor messages
        self._latest_camera_info: Optional[CameraInfo] = None
        self._latest_scan: Optional[LaserScan] = None

        # Diagnostic state
        self._last_diag_time = self.get_clock().now()
        self._diag_interval_sec = 5.0
        self._processed_frames = 0
        self._first_frame_logged = False

        # QoS Profiles
        marker_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriptions
        self._tracks_sub = self.create_subscription(
            TrackedOreArray,
            tracked_topic,
            self._tracks_callback,
            qos_profile_sensor_data
        )

        self._cam_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._cam_info_callback,
            qos_profile_sensor_data
        )

        self._scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data
        )

        # Publishers
        self._spatial_pub = self.create_publisher(
            SpatialOreArray,
            output_spatial_topic,
            qos_profile_sensor_data
        )

        self._marker_pub = self.create_publisher(
            MarkerArray,
            output_marker_topic,
            marker_qos
        )

        self.get_logger().info("SpatialAssociationNode initialized successfully.")

    def _cam_info_callback(self, msg: CameraInfo) -> None:
        """Cache incoming CameraInfo."""
        self._latest_camera_info = msg

    def _scan_callback(self, msg: LaserScan) -> None:
        """Cache incoming LaserScan."""
        self._latest_scan = msg

    def _tracks_callback(self, msg: TrackedOreArray) -> None:
        """Process incoming tracked ores and associate with physical map coordinates."""
        self._processed_frames += 1

        spatial_array, marker_array, diagnostics = self.associator.process_observation(
            tracked_ores=msg.tracks,
            scan=self._latest_scan,
            camera_info=self._latest_camera_info,
            tf_buffer=self.tf_buffer,
            observation_header=msg.header
        )

        # Publish results
        self._spatial_pub.publish(spatial_array)
        if len(marker_array.markers) > 0:
            self._marker_pub.publish(marker_array)

        # Periodic / first-frame status logging
        now = self.get_clock().now()
        dt = (now - self._last_diag_time).nanoseconds * 1e-9

        if not self._first_frame_logged or dt >= self._diag_interval_sec:
            self._last_diag_time = now
            self._first_frame_logged = True

            cal_st = diagnostics["camera_calibration_status"]
            scan_st = diagnostics["lidar_scan_status"]
            tf_st = diagnostics["tf_status"]
            tot = diagnostics["total_tracks"]
            loc = diagnostics["localized_count"]
            unloc = diagnostics["unlocalized_count"]

            self.get_logger().info(
                f"[Spatial Status] Frames: {self._processed_frames} | "
                f"Tracks: {tot} (Loc: {loc}, Unloc: {unloc}) | "
                f"Calib: {cal_st} | Scan: {scan_st} | TF: {tf_st}"
            )

            # Issue actionable warnings if prerequisites are failing
            if cal_st == "UNAVAILABLE" and self._latest_camera_info is not None:
                self.get_logger().warn(
                    f"[Spatial Warning] {diagnostics.get('camera_calibration_detail')}. "
                    "Spatial localization disabled; emitting unlocalized tracks."
                )
            if scan_st == "UNAVAILABLE" and tot > 0:
                self.get_logger().warn(
                    f"[Spatial Warning] LiDAR scan unavailable: {diagnostics.get('lidar_scan_detail')}"
                )


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SpatialAssociationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
