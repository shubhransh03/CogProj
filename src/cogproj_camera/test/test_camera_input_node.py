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

    def test_bgra8_conversion_pixel_values(self):
        """Verify bgra8 input is correctly converted to bgr8 with correct pixel values."""
        node = CameraInputNode()
        bridge = CvBridge()

        # Build a small 2x2 bgra8 image with distinct, verifiable pixel values.
        # bgra8 layout: [B, G, R, A] per pixel
        h, w = 2, 2
        bgra_arr = np.zeros((h, w, 4), dtype=np.uint8)
        bgra_arr[0, 0] = [10, 20, 30, 255]   # B=10 G=20 R=30 A=255
        bgra_arr[0, 1] = [40, 50, 60, 128]   # B=40 G=50 R=60 A=128
        bgra_arr[1, 0] = [70, 80, 90, 0]     # B=70 G=80 R=90 A=0
        bgra_arr[1, 1] = [100, 110, 120, 255] # B=100 G=110 R=120 A=255

        in_msg = Image()
        in_msg.header.stamp = Time(sec=42, nanosec=123)
        in_msg.header.frame_id = "test_bgra8"
        in_msg.height = h
        in_msg.width = w
        in_msg.encoding = "bgra8"
        in_msg.step = w * 4
        in_msg.data = bgra_arr.tobytes()

        out_msg = node.process_image(in_msg)

        # Must produce a valid output
        self.assertIsNotNone(out_msg)

        # Encoding must be bgr8
        self.assertEqual(out_msg.encoding, "bgr8")

        # Step must be width * 3 (not width * 4)
        self.assertEqual(out_msg.step, w * 3)

        # Data length must be h * w * 3
        self.assertEqual(len(out_msg.data), h * w * 3)

        # Dimensions preserved
        self.assertEqual(out_msg.height, h)
        self.assertEqual(out_msg.width, w)

        # Header preserved
        self.assertEqual(out_msg.header.stamp.sec, 42)
        self.assertEqual(out_msg.header.stamp.nanosec, 123)
        self.assertEqual(out_msg.header.frame_id, "test_bgra8")

        # Decode output and verify pixel values match expected BGR (alpha stripped)
        recovered = bridge.imgmsg_to_cv2(out_msg, desired_encoding="bgr8")
        self.assertEqual(recovered.shape, (h, w, 3))
        np.testing.assert_array_equal(recovered[0, 0], [10, 20, 30])
        np.testing.assert_array_equal(recovered[0, 1], [40, 50, 60])
        np.testing.assert_array_equal(recovered[1, 0], [70, 80, 90])
        np.testing.assert_array_equal(recovered[1, 1], [100, 110, 120])

        node.destroy_node()

    def test_bgra8_640x480_step_and_data_consistency(self):
        """Verify bgra8 640x480 produces correct step, data length, and decodable output."""
        node = CameraInputNode()
        bridge = CvBridge()

        h, w = 480, 640
        bgra_arr = np.random.randint(0, 256, (h, w, 4), dtype=np.uint8)

        in_msg = Image()
        in_msg.header.stamp = Time(sec=100, nanosec=0)
        in_msg.header.frame_id = "camera_frame"
        in_msg.height = h
        in_msg.width = w
        in_msg.encoding = "bgra8"
        in_msg.step = w * 4
        in_msg.data = bgra_arr.tobytes()

        out_msg = node.process_image(in_msg)

        self.assertIsNotNone(out_msg)
        self.assertEqual(out_msg.encoding, "bgr8")
        self.assertEqual(out_msg.step, w * 3)
        self.assertEqual(len(out_msg.data), h * w * 3)
        self.assertEqual(out_msg.height, h)
        self.assertEqual(out_msg.width, w)

        # Downstream decode must succeed without error
        recovered = bridge.imgmsg_to_cv2(out_msg, desired_encoding="bgr8")
        self.assertEqual(recovered.shape, (h, w, 3))

        # BGR channels must match original (alpha stripped)
        np.testing.assert_array_equal(recovered, bgra_arr[:, :, :3])

        node.destroy_node()

    def test_bgr8_passthrough_step_and_data_consistency(self):
        """Verify bgr8 input passes through with correct step, data length, and pixels."""
        node = CameraInputNode()
        bridge = CvBridge()

        h, w = 240, 320
        bgr_arr = np.random.randint(0, 256, (h, w, 3), dtype=np.uint8)

        in_msg = Image()
        in_msg.header.stamp = Time(sec=200, nanosec=999)
        in_msg.header.frame_id = "bgr_frame"
        in_msg.height = h
        in_msg.width = w
        in_msg.encoding = "bgr8"
        in_msg.step = w * 3
        in_msg.data = bgr_arr.tobytes()

        out_msg = node.process_image(in_msg)

        self.assertIsNotNone(out_msg)
        self.assertEqual(out_msg.encoding, "bgr8")
        self.assertEqual(out_msg.step, w * 3)
        self.assertEqual(len(out_msg.data), h * w * 3)

        recovered = bridge.imgmsg_to_cv2(out_msg, desired_encoding="bgr8")
        np.testing.assert_array_equal(recovered, bgr_arr)

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
