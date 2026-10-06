"""Laptop-side Viewer for CogProj perception pipeline.

Subscribes to /cogproj/image_annotated/compressed (or uncompressed),
decodes JPEG frames, and displays them via OpenCV GUI with headless fallback.
"""

import sys
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class CogprojViewer(Node):
    """Client viewer node for monitoring annotated rover feeds on a remote laptop."""

    def __init__(self) -> None:
        super().__init__("cogproj_viewer")

        # Declare parameters
        self.declare_parameter("input_topic", "/cogproj/image_annotated/compressed")
        self.declare_parameter("window_name", "CogProj Perception Viewer")

        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.window_name = self.get_parameter("window_name").get_parameter_value().string_value

        self.frame_count = 0
        self.headless_logged = False

        streaming_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            CompressedImage,
            self.input_topic,
            self._compressed_callback,
            streaming_qos,
        )

        self.get_logger().info("CogprojViewer started.")
        self.get_logger().info(f"  Subscribing to: {self.input_topic}")
        self.get_logger().info("  Press 'q' in GUI window to quit.")

    def _compressed_callback(self, msg: CompressedImage) -> None:
        """Decode JPEG frame and render to OpenCV window."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None or frame.size == 0:
            self.get_logger().warn("Failed to decode compressed image frame.", throttle_duration_sec=2.0)
            return

        self.frame_count += 1

        # Attempt to display in GUI window
        try:
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.get_logger().info("Exit key 'q' pressed. Shutting down viewer.")
                raise KeyboardInterrupt
        except cv2.error:
            if not self.headless_logged:
                self.get_logger().info(
                    "Headless environment detected (no GUI display). "
                    "Frames decoded successfully; running headlessly without GUI popup."
                )
                self.headless_logged = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CogprojViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

