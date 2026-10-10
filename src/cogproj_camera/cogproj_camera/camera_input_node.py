"""Camera Input Node for CogProj.

Subscribes to the existing Beetle Bot camera topic, validates and converts
frames using cv_bridge, and safely publishes to the CogProj camera topic
while strictly preserving the original timestamp and frame_id.
"""

from typing import Optional

import cv_bridge
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraInputNode(Node):
    """ROS 2 node that safely ingests robot camera frames into CogProj."""

    def __init__(self) -> None:
        super().__init__("camera_input_node")

        # Declare parameters
        self.declare_parameter("input_topic", "/pi_camera/image_raw")
        self.declare_parameter("output_topic", "/cogproj/image_raw")

        # Retrieve parameter values
        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self.bridge = CvBridge()
        self.frame_count = 0

        # QoS profile appropriate for live streaming (low latency, drop stale frames)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Publisher for CogProj-owned camera topic
        self.image_pub = self.create_publisher(
            Image,
            self.output_topic,
            qos_profile,
        )

        # Subscriber to existing robot camera topic
        self.image_sub = self.create_subscription(
            Image,
            self.input_topic,
            self._image_callback,
            qos_profile,
        )

        self.get_logger().info("CameraInputNode initialized.")
        self.get_logger().info(f"  Subscribing to: {self.input_topic}")
        self.get_logger().info(f"  Publishing to:  {self.output_topic}")

    def _image_callback(self, msg: Image) -> None:
        """Process incoming camera frame from Beetle Bot."""
        self.process_image(msg)

    def process_image(self, msg: Image) -> Optional[Image]:
        """Validate, convert to BGR, preserve headers, and publish.

        Args:
            msg: Incoming ROS 2 Image message.

        Returns:
            The published Image message if valid, or None if conversion failed.
        """
        try:
            # Validate conversion to OpenCV BGR format without altering pixels
            cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if cv_frame is None or cv_frame.size == 0:
                self.get_logger().warn("Received empty or invalid image frame.", throttle_duration_sec=2.0)
                return None
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f"cv_bridge conversion error: {e}", throttle_duration_sec=2.0)
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected image processing error: {e}", throttle_duration_sec=2.0)
            return None

        # Build output message preserving exact original header (stamp & frame_id)
        out_msg = Image()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id
        out_msg.height = int(cv_frame.shape[0])
        out_msg.width = int(cv_frame.shape[1])
        out_msg.encoding = "bgr8"
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.step = int(cv_frame.shape[1] * 3)
        out_msg.data = cv_frame.tobytes()

        self.image_pub.publish(out_msg)
        self.frame_count += 1

        if self.frame_count == 1:
            self.get_logger().info(
                f"First frame processed successfully: {msg.width}x{msg.height}, "
                f"frame_id='{msg.header.frame_id}', encoding='{msg.encoding}'"
            )

        return out_msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

