"""Counting Node for CogProj.

Subscribes to /cogproj/tracked_ores, evaluates track confirmation states,
and publishes cumulative session count summaries to /cogproj/counts_summary.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from cogproj_interfaces.msg import OreCountSummary, TrackedOreArray
from .counter import OreCounter


class CountingNode(Node):
    """ROS 2 node that computes and publishes cumulative ore count summaries."""

    def __init__(self) -> None:
        super().__init__("counting_node")

        # Declare parameters
        self.declare_parameter("input_topic", "/cogproj/tracked_ores")
        self.declare_parameter("output_topic", "/cogproj/counts_summary")
        self.declare_parameter("counting_enabled", True)

        # Retrieve parameter values
        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.counting_enabled = self.get_parameter("counting_enabled").get_parameter_value().bool_value

        self.counter = OreCounter(counting_enabled=self.counting_enabled)

        # Streaming QoS Profile for incoming tracks subscription (matches tracker publisher)
        streaming_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Summary/State QoS Profile for counts summary (Reliable, Transient Local latching)
        summary_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publisher for count summary
        self.summary_pub = self.create_publisher(
            OreCountSummary,
            self.output_topic,
            summary_qos,
        )

        # Subscriber to tracked ores
        self.tracks_sub = self.create_subscription(
            TrackedOreArray,
            self.input_topic,
            self._tracks_callback,
            streaming_qos,
        )

        self.get_logger().info("CountingNode initialized.")
        self.get_logger().info(f"  input_topic:      {self.input_topic}")
        self.get_logger().info(f"  output_topic:     {self.output_topic}")
        self.get_logger().info(f"  counting_enabled: {self.counting_enabled}")

    def _tracks_callback(self, msg: TrackedOreArray) -> None:
        """Process incoming tracked ores and publish updated count summary."""
        summary_msg = self.counter.process_tracks(
            tracks=msg.tracks,
            header=msg.header,
        )
        self.summary_pub.publish(summary_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CountingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

