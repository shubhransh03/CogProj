"""Unit tests for cogproj_camera: construction, parameters, and header preservation."""

import unittest
import numpy as np

import rclpy
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from cogproj_camera import CameraInputNode


class TestCameraInputNode(unittest.TestCase):
    """Test CameraInputNode initialization and frame processing logic."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def test_camera_node_construction_and_defaults(self):
        """1. Camera node can be constructed and uses default parameters."""
        node = CameraInputNode()
        self.assertEqual(node.input_topic, "/pi_camera/image_raw")
        self.assertEqual(node.output_topic, "/cogproj/image_raw")
        self.assertEqual(node.frame_count, 0)
        node.destroy_node()

    def test_camera_node_custom_parameter(self):
        """2. Correct input topic parameter is used when overridden."""
        node = CameraInputNode()
        node.set_parameters([
            Parameter("input_topic", Parameter.Type.STRING, "/custom_cam/image_raw"),
            Parameter("output_topic", Parameter.Type.STRING, "/custom_out/image_raw"),
        ])
        # Re-read parameter values
        input_topic = node.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = node.get_parameter("output_topic").get_parameter_value().string_value

        self.assertEqual(input_topic, "/custom_cam/image_raw")
        self.assertEqual(output_topic, "/custom_out/image_raw")
        node.destroy_node()

    def test_image_conversion_and_header_preservation(self):
        """3, 4, 5. Image conversion path accepts BGR and preserves stamp and frame_id."""
        node = CameraInputNode()
        bridge = CvBridge()

        # Create a synthetic 320x240 BGR image
        synthetic_bgr = np.zeros((240, 320, 3), dtype=np.uint8)
        synthetic_bgr[50:100, 50:100] = [0, 255, 0]  # Green square

        # Convert to ROS Image message with specific timestamp and frame_id
        input_msg = Image()
        input_msg.header.stamp = Time(sec=1791283200, nanosec=543210)
        input_msg.header.frame_id = "camera_optical_link"
        input_msg.height = 240
        input_msg.width = 320
        input_msg.encoding = "bgr8"
        input_msg.step = 320 * 3
        input_msg.data = synthetic_bgr.tobytes()

        # Process through node
        output_msg = node.process_image(input_msg)

        # Assertions
        self.assertIsNotNone(output_msg)
        self.assertEqual(output_msg.header.stamp.sec, 1791283200)
        self.assertEqual(output_msg.header.stamp.nanosec, 543210)
        self.assertEqual(output_msg.header.frame_id, "camera_optical_link")
        self.assertEqual(output_msg.width, 320)
        self.assertEqual(output_msg.height, 240)
        self.assertEqual(output_msg.encoding, "bgr8")

        # Convert output back to verify image integrity
        recovered_bgr = bridge.imgmsg_to_cv2(output_msg, desired_encoding="bgr8")
        self.assertEqual(recovered_bgr.shape, (240, 320, 3))
        self.assertTrue(np.all(recovered_bgr[50:100, 50:100] == [0, 255, 0]))

        node.destroy_node()

    def test_invalid_image_fails_safely(self):
        """Verify invalid or corrupted messages are handled gracefully without raising."""
        node = CameraInputNode()
        empty_msg = Image()
        empty_msg.encoding = "invalid_encoding_xyz"

        result = node.process_image(empty_msg)
        self.assertIsNone(result)
        node.destroy_node()


if __name__ == "__main__":
    unittest.main()
