"""Tracking Node for CogProj.

Subscribes to /cogproj/detections, runs MultiObjectTracker, and publishes
persistent TrackedOreArray messages to /cogproj/tracked_ores.
"""

from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry

from cogproj_interfaces.msg import DetectionArray, TrackedOreArray
from .tracker import MultiObjectTracker


class TrackingNode(Node):
    """ROS 2 node for multi-object tracking in CogProj."""

    def __init__(self) -> None:
        super().__init__("tracking_node")

        # Declare parameters
        self.declare_parameter("input_topic", "/cogproj/detections")
        self.declare_parameter("output_topic", "/cogproj/tracked_ores")
        self.declare_parameter("max_centroid_distance_px", 50.0)
        self.declare_parameter("min_iou", 0.10)
        self.declare_parameter("max_missed_frames", 5)
        self.declare_parameter("min_confirmed_hits", 3)
        self.declare_parameter("use_odom", False)
        self.declare_parameter("odom_topic", "/odometry/filtered")

        # Retrieve parameter values
        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.max_dist = self.get_parameter("max_centroid_distance_px").get_parameter_value().double_value
        self.min_iou = self.get_parameter("min_iou").get_parameter_value().double_value
        self.max_missed = self.get_parameter("max_missed_frames").get_parameter_value().integer_value
        self.min_confirmed = self.get_parameter("min_confirmed_hits").get_parameter_value().integer_value
        self.use_odom = self.get_parameter("use_odom").get_parameter_value().bool_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        # Instantiate core tracker
        self.tracker = MultiObjectTracker(
            max_centroid_distance_px=self.max_dist,
            min_iou=self.min_iou,
            max_missed_frames=self.max_missed,
            min_confirmed_hits=self.min_confirmed,
        )

        self.last_odom: Optional[Odometry] = None

        # Streaming QoS Profile for high-rate perception (matches detection publisher)
        streaming_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Publisher for tracked ores
        self.tracks_pub = self.create_publisher(
            TrackedOreArray,
            self.output_topic,
            streaming_qos,
        )

        # Subscriber to detections
        self.detection_sub = self.create_subscription(
            DetectionArray,
            self.input_topic,
            self._detections_callback,
            streaming_qos,
        )

        # Odometry extension point (passive subscription only if enabled)
        if self.use_odom:
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self._odom_callback,
                streaming_qos,
            )
            self.get_logger().info(f"Odometry integration active on {self.odom_topic}")
        else:
            self.odom_sub = None

        self.get_logger().info("TrackingNode initialized.")
        self.get_logger().info(f"  input_topic:              {self.input_topic}")
        self.get_logger().info(f"  output_topic:             {self.output_topic}")
        self.get_logger().info(f"  max_centroid_distance_px: {self.max_dist}")
        self.get_logger().info(f"  min_iou:                  {self.min_iou}")
        self.get_logger().info(f"  max_missed_frames:        {self.max_missed}")
        self.get_logger().info(f"  min_confirmed_hits:       {self.min_confirmed}")
        self.get_logger().info(f"  use_odom:                 {self.use_odom}")

    def _odom_callback(self, msg: Odometry) -> None:
        """Store latest robot odometry for future ego-motion compensation."""
        self.last_odom = msg

    def _detections_callback(self, msg: DetectionArray) -> None:
        """Process incoming detection array and publish updated active tracks."""
        # Optional ego-motion extension hook:
        # In Phase 3, image-space association is maintained without ungrounded 3D projections.
        if self.use_odom and self.last_odom is not None:
            self._compensate_ego_motion()

        # Update tracker with current detections
        active_tracks = self.tracker.update(msg.detections)

        # Create output message preserving detection header (stamp and frame_id)
        out_msg = TrackedOreArray()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id
        out_msg.tracks = [track.to_msg() for track in active_tracks]

        self.tracks_pub.publish(out_msg)

    def _compensate_ego_motion(self) -> None:
        """Extension point for camera ego-motion compensation.

        Kept as an architectural hook for future phases when camera intrinsics
        and 3D ground-plane transforms are integrated.
        """
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

