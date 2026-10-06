"""Visualizer Node for CogProj.

Subscribes to camera frames, detections, tracked ores, and count summaries,
renders a comprehensive perception HUD overlay, and publishes both raw and
JPEG-compressed annotated image streams for laptop viewing.
"""

import time
from typing import Dict, List, Optional
import cv2
import cv_bridge
from cv_bridge import CvBridge
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from cogproj_interfaces.msg import DetectionArray, OreCountSummary, TrackedOre, TrackedOreArray
from .annotator import PerceptionAnnotator


class VisualizerNode(Node):
    """ROS 2 node for generating perception HUD annotations on camera feeds."""

    def __init__(self) -> None:
        super().__init__("visualizer_node")

        # Declare parameters
        self.declare_parameter("image_topic", "/cogproj/image_raw")
        self.declare_parameter("detection_topic", "/cogproj/detections")
        self.declare_parameter("tracking_topic", "/cogproj/tracked_ores")
        self.declare_parameter("count_topic", "/cogproj/counts_summary")
        self.declare_parameter("annotated_topic", "/cogproj/image_annotated")
        self.declare_parameter("compressed_topic", "/cogproj/image_annotated/compressed")
        self.declare_parameter("camera_timeout_sec", 2.0)
        self.declare_parameter("detection_timeout_sec", 2.0)
        self.declare_parameter("tracking_timeout_sec", 2.0)
        self.declare_parameter("count_timeout_sec", 5.0)
        self.declare_parameter("jpeg_quality", 80)

        # Retrieve parameter values
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.detection_topic = self.get_parameter("detection_topic").get_parameter_value().string_value
        self.tracking_topic = self.get_parameter("tracking_topic").get_parameter_value().string_value
        self.count_topic = self.get_parameter("count_topic").get_parameter_value().string_value
        self.annotated_topic = self.get_parameter("annotated_topic").get_parameter_value().string_value
        self.compressed_topic = self.get_parameter("compressed_topic").get_parameter_value().string_value
        self.cam_timeout = self.get_parameter("camera_timeout_sec").get_parameter_value().double_value
        self.det_timeout = self.get_parameter("detection_timeout_sec").get_parameter_value().double_value
        self.trk_timeout = self.get_parameter("tracking_timeout_sec").get_parameter_value().double_value
        self.cnt_timeout = self.get_parameter("count_timeout_sec").get_parameter_value().double_value
        self.jpeg_quality = self.get_parameter("jpeg_quality").get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.annotator = PerceptionAnnotator()

        # Cached latest perception messages
        self.latest_tracks: List[TrackedOre] = []
        self.latest_counts: Optional[OreCountSummary] = None

        # Message arrival tracking
        self.last_camera_time: float = 0.0
        self.last_detection_time: float = 0.0
        self.last_tracking_time: float = 0.0
        self.last_count_time: float = 0.0
        self.frame_count: int = 0

        # Streaming QoS Profile (BEST_EFFORT, VOLATILE, depth 5)
        streaming_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # State / Summary QoS Profile (RELIABLE, TRANSIENT_LOCAL, depth 10)
        summary_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, streaming_qos)
        self.compressed_pub = self.create_publisher(CompressedImage, self.compressed_topic, streaming_qos)

        # Subscribers
        self.image_sub = self.create_subscription(Image, self.image_topic, self._image_callback, streaming_qos)
        self.det_sub = self.create_subscription(DetectionArray, self.detection_topic, self._det_callback, streaming_qos)
        self.trk_sub = self.create_subscription(TrackedOreArray, self.tracking_topic, self._trk_callback, streaming_qos)
        self.cnt_sub = self.create_subscription(OreCountSummary, self.count_topic, self._cnt_callback, summary_qos)

        self.get_logger().info("VisualizerNode initialized.")
        self.get_logger().info(f"  Input image topic:      {self.image_topic}")
        self.get_logger().info(f"  Annotated topic:        {self.annotated_topic}")
        self.get_logger().info(f"  Compressed topic:       {self.compressed_topic}")
        self.get_logger().info(f"  JPEG Quality:           {self.jpeg_quality}")

    def _det_callback(self, msg: DetectionArray) -> None:
        """Record detection message arrival."""
        self.last_detection_time = time.monotonic()

    def _trk_callback(self, msg: TrackedOreArray) -> None:
        """Update cached active tracks."""
        self.last_tracking_time = time.monotonic()
        self.latest_tracks = list(msg.tracks)

    def _cnt_callback(self, msg: OreCountSummary) -> None:
        """Update cached cumulative count summary."""
        self.last_count_time = time.monotonic()
        self.latest_counts = msg

    def get_status_dict(self) -> Dict[str, str]:
        """Compute current perception status strings based on timeouts."""
        now = time.monotonic()

        cam_st = "OK" if (now - self.last_camera_time) <= self.cam_timeout and self.last_camera_time > 0 else "WAITING"
        det_st = "ACTIVE" if (now - self.last_detection_time) <= self.det_timeout and self.last_detection_time > 0 else "WAITING"
        trk_st = "ACTIVE" if (now - self.last_tracking_time) <= self.trk_timeout and self.last_tracking_time > 0 else "WAITING"
        cnt_st = "ACTIVE" if (now - self.last_count_time) <= self.cnt_timeout and self.last_count_time > 0 else "WAITING"

        return {
            "camera": cam_st,
            "detection": det_st,
            "tracking": trk_st,
            "counting": cnt_st,
        }

    def _image_callback(self, msg: Image) -> None:
        """Process incoming camera frame and publish annotated feeds."""
        self.last_camera_time = time.monotonic()

        try:
            cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if cv_frame is None or cv_frame.size == 0:
                self.get_logger().warn("Empty frame received in visualizer.", throttle_duration_sec=2.0)
                return
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f"cv_bridge error in visualizer: {e}", throttle_duration_sec=2.0)
            return
        except Exception as e:
            self.get_logger().error(f"Unexpected image conversion error: {e}", throttle_duration_sec=2.0)
            return

        # Render annotations and HUD
        status = self.get_status_dict()
        annotated_bgr = self.annotator.annotate_frame(
            frame_bgr=cv_frame,
            tracked_ores=self.latest_tracks,
            counts_summary=self.latest_counts,
            status=status,
        )

        h, w = annotated_bgr.shape[:2]

        # 1. Build and publish raw annotated Image message (preserving header)
        raw_msg = Image()
        raw_msg.header.stamp = msg.header.stamp
        raw_msg.header.frame_id = msg.header.frame_id
        raw_msg.height = h
        raw_msg.width = w
        raw_msg.encoding = "bgr8"
        raw_msg.is_bigendian = 0
        raw_msg.step = w * 3
        raw_msg.data = annotated_bgr.tobytes()

        self.annotated_pub.publish(raw_msg)

        # 2. Build and publish JPEG-compressed image message (preserving header)
        success, encoded_jpg = cv2.imencode(
            ".jpg",
            annotated_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )

        if success:
            comp_msg = CompressedImage()
            comp_msg.header.stamp = msg.header.stamp
            comp_msg.header.frame_id = msg.header.frame_id
            comp_msg.format = "jpeg"
            comp_msg.data = encoded_jpg.tobytes()
            self.compressed_pub.publish(comp_msg)

        self.frame_count += 1
        if self.frame_count == 1:
            self.get_logger().info("First annotated frame published successfully.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

