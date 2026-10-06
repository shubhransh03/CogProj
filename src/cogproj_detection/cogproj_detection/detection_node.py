"""CogProj Detection Node.

Wraps the pluggable BaseOreDetector interface into a ROS 2 lifecycle/execution node,
converts standardized DetectionResult outputs into DetectionArray messages,
and measures inference performance.
"""

import time
from typing import Any, List, Optional

import cv_bridge
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from cogproj_detector_base import (
    BaseOreDetector,
    DetectionResult,
    MockDetector,
    PluginError,
    load_detector_plugin,
)
from cogproj_interfaces.msg import DetectionArray

from .converters import detection_results_to_array_msg


class DetectionNode(Node):
    """ROS 2 node responsible for object detection inference."""

    def __init__(self) -> None:
        super().__init__("detection_node")

        # Declare ROS parameters with safe defaults
        self.declare_parameter("enabled", False)
        self.declare_parameter("test_mode", False)
        self.declare_parameter("image_topic", "/cogproj/image_raw")
        self.declare_parameter("detection_topic", "/cogproj/detections")
        self.declare_parameter("model_directory", "")
        self.declare_parameter("model_plugin_path", "")
        self.declare_parameter("model_config_path", "")
        self.declare_parameter("confidence_threshold", 0.60)

        # Retrieve parameter values
        self.enabled = self.get_parameter("enabled").get_parameter_value().bool_value
        self.test_mode = self.get_parameter("test_mode").get_parameter_value().bool_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.detection_topic = self.get_parameter("detection_topic").get_parameter_value().string_value
        self.model_dir = self.get_parameter("model_directory").get_parameter_value().string_value
        self.model_plugin_path = self.get_parameter("model_plugin_path").get_parameter_value().string_value
        self.model_config_path = self.get_parameter("model_config_path").get_parameter_value().string_value
        self.conf_threshold = self.get_parameter("confidence_threshold").get_parameter_value().double_value

        self.bridge = CvBridge()
        self.detector: Optional[BaseOreDetector] = None

        # Setup QoS for detection topic (sensor data profile)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Publisher for detection results
        self.detection_pub = self.create_publisher(
            DetectionArray,
            self.detection_topic,
            qos_profile,
        )

        # Subscriber to CogProj camera stream
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile,
        )

        self.get_logger().info("DetectionNode initialized.")
        self.get_logger().info(f"  enabled:              {self.enabled}")
        self.get_logger().info(f"  test_mode:            {self.test_mode}")
        self.get_logger().info(f"  image_topic:          {self.image_topic}")
        self.get_logger().info(f"  detection_topic:      {self.detection_topic}")
        self.get_logger().info(f"  confidence_threshold: {self.conf_threshold}")

        # State A: Disabled by default (Safe state)
        if not self.enabled:
            self.get_logger().info(
                "Detection is DISABLED by default (safe state). "
                "Set parameter 'enabled: true' to activate inference."
            )
            return

        # If enabled, initialize detector plugin or test mock
        self._initialize_detector()

    def _initialize_detector(self) -> bool:
        """Attempt to load the detector plugin or test mock."""
        # State C: Test / Mock Mode
        if self.test_mode:
            self.detector = MockDetector()
            self.detector.load("/tmp/mock_model")
            self.get_logger().info(
                "============================================================"
            )
            self.get_logger().info(
                "[TEST/MOCK MODE ACTIVE] Using MockDetector with synthetic test detections."
            )
            self.get_logger().info(
                "No machine learning framework or real model is loaded."
            )
            self.get_logger().info(
                "============================================================"
            )
            return True

        # State B: Enabled but no model configured (Fail safely without crash)
        if not self.model_dir:
            self.get_logger().error(
                "SAFE FAILURE: Detection is enabled but 'model_directory' is empty. "
                "No detector loaded. Detection will not execute on incoming frames."
            )
            self.detector = None
            return False

        # State D: Real plugin mode (Dynamic loading from model directory)
        self.get_logger().info(f"Attempting to load detector from: '{self.model_dir}'")
        try:
            self.detector = load_detector_plugin(
                model_dir=self.model_dir,
                config_path=self.model_config_path or None,
                auto_init=True,
            )
            metadata = self.detector.get_metadata()
            self.get_logger().info(
                f"Successfully loaded detector plugin: {metadata.get('model_name', 'Unknown')}"
            )
            return True
        except PluginError as e:
            self.get_logger().error(f"Failed to load detector plugin: {e}")
            self.detector = None
            return False
        except Exception as e:
            self.get_logger().error(f"Unexpected error loading detector plugin: {e}")
            self.detector = None
            return False

    def _image_callback(self, msg: Image) -> None:
        """Callback for incoming camera frames."""
        # If disabled: do not run detection, do not publish
        if not self.enabled:
            return

        # If enabled but no detector available: safe skip with throttled warning
        if self.detector is None:
            self.get_logger().warn(
                "Frame received but no detector is loaded. Skipping inference.",
                throttle_duration_sec=5.0,
            )
            return

        # Convert ROS Image to OpenCV BGR numpy array
        try:
            cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f"cv_bridge conversion error: {e}", throttle_duration_sec=2.0)
            return
        except Exception as e:
            self.get_logger().error(f"Unexpected image conversion error: {e}", throttle_duration_sec=2.0)
            return

        # Process frame and publish detection results
        self.process_frame(cv_frame, header=msg.header)

    def process_frame(self, frame_bgr: Any, header: Optional[Header] = None) -> DetectionArray:
        """Process an image frame through the loaded detector and publish results.

        This method is callable by image subscriber callbacks or synthetic test harnesses.
        """
        if header is None:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "camera_optical_link"

        if self.detector is None:
            empty_msg = DetectionArray()
            empty_msg.header = header
            empty_msg.inference_time_ms = 0.0
            return empty_msg

        # Measure inference execution time
        start_time = time.perf_counter()
        try:
            raw_detections: List[DetectionResult] = self.detector.predict(frame_bgr)
            inference_time_ms = (time.perf_counter() - start_time) * 1000.0
        except Exception as e:
            self.get_logger().error(f"Exception during detector inference: {e}")
            raw_detections = []
            inference_time_ms = (time.perf_counter() - start_time) * 1000.0

        # Convert to ROS 2 message
        msg = detection_results_to_array_msg(
            results=raw_detections,
            header=header,
            inference_time_ms=inference_time_ms,
            confidence_threshold=self.conf_threshold,
        )

        # Publish detections
        self.detection_pub.publish(msg)
        return msg

    def destroy_node(self) -> None:
        """Clean shutdown releasing detector resources."""
        if self.detector is not None:
            try:
                self.detector.unload()
            except Exception as e:
                self.get_logger().warn(f"Exception during detector unload: {e}")
            self.detector = None
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
