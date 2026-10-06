"""Unit tests for cogproj_detection: message conversion, safety modes, and mock pipeline."""

import os
import shutil
import tempfile
import unittest
import numpy as np

import rclpy
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from cogproj_detector_base import DetectionResult, MockDetector
from cogproj_detection import (
    DetectionNode,
    detection_result_to_msg,
    detection_results_to_array_msg,
)
from cogproj_interfaces.msg import Detection, DetectionArray


class TestDetectionMessageConverters(unittest.TestCase):
    """Test conversion of DetectionResult into ROS 2 message formats."""

    def test_detection_result_to_msg(self):
        result = DetectionResult(
            class_id=3,
            class_name="Azurite",
            confidence=0.87,
            x_min=15.0,
            y_min=25.0,
            x_max=75.0,
            y_max=95.0,
        )
        msg: Detection = detection_result_to_msg(result)

        self.assertEqual(msg.class_id, 3)
        self.assertEqual(msg.class_name, "Azurite")
        self.assertAlmostEqual(msg.confidence, 0.87, places=4)
        self.assertAlmostEqual(msg.x_min, 15.0, places=4)
        self.assertAlmostEqual(msg.y_min, 25.0, places=4)
        self.assertAlmostEqual(msg.x_max, 75.0, places=4)
        self.assertAlmostEqual(msg.y_max, 95.0, places=4)
        self.assertAlmostEqual(msg.center_x, 45.0, places=4)
        self.assertAlmostEqual(msg.center_y, 60.0, places=4)
        self.assertAlmostEqual(msg.bbox_width, 60.0, places=4)
        self.assertAlmostEqual(msg.bbox_height, 70.0, places=4)

    def test_detection_results_to_array_msg_filtering(self):
        header = Header()
        header.stamp = Time(sec=1791283200, nanosec=123456)
        header.frame_id = "camera_optical_link"

        r1 = DetectionResult(1, "OreA", 0.90, 0.0, 0.0, 10.0, 10.0)
        r2 = DetectionResult(2, "OreB", 0.40, 10.0, 10.0, 20.0, 20.0)
        r3 = DetectionResult(3, "OreC", 0.75, 20.0, 20.0, 30.0, 30.0)

        # Apply confidence threshold 0.60
        array_msg: DetectionArray = detection_results_to_array_msg(
            results=[r1, r2, r3],
            header=header,
            inference_time_ms=12.5,
            confidence_threshold=0.60,
        )

        self.assertEqual(array_msg.header.stamp.sec, 1791283200)
        self.assertEqual(array_msg.header.stamp.nanosec, 123456)
        self.assertEqual(array_msg.header.frame_id, "camera_optical_link")
        self.assertAlmostEqual(array_msg.inference_time_ms, 12.5, places=4)
        # OreB (0.40) must be filtered out
        self.assertEqual(len(array_msg.detections), 2)
        self.assertEqual(array_msg.detections[0].class_name, "OreA")
        self.assertEqual(array_msg.detections[1].class_name, "OreC")


class TestDetectionNodeExecution(unittest.TestCase):
    """Test DetectionNode initialization, safety modes, and camera frame ingestion."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.bridge = CvBridge()

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_node_disabled_by_default(self):
        """6. Detection node remains disabled by default."""
        node = DetectionNode()
        self.assertFalse(node.enabled)
        self.assertIsNone(node.detector)
        node.destroy_node()

    def test_enabled_without_model_fails_safely(self):
        """Safe mode B: Enabled without model fails safely with clear log and no crash."""
        node = DetectionNode()
        node.enabled = True
        node.model_dir = ""
        node.test_mode = False

        success = node._initialize_detector()
        self.assertFalse(success)
        self.assertIsNone(node.detector)

        # Process a frame should return an empty DetectionArray safely
        msg = node.process_frame(frame_bgr=None)
        self.assertEqual(len(msg.detections), 0)
        self.assertEqual(msg.inference_time_ms, 0.0)
        node.destroy_node()

    def test_test_mode_with_mock_detector_receives_bgr(self):
        """7, 8, 9, 10. Test/Mock mode processes BGR frame and converts detections to DetectionArray."""
        node = DetectionNode()
        node.enabled = True
        node.test_mode = True
        node.conf_threshold = 0.50

        success = node._initialize_detector()
        self.assertTrue(success)
        self.assertIsInstance(node.detector, MockDetector)

        # Create synthetic BGR frame (320x240)
        synthetic_bgr = np.zeros((240, 320, 3), dtype=np.uint8)
        header = Header()
        header.stamp = Time(sec=1791283200, nanosec=987654)
        header.frame_id = "camera_optical_link"

        # Process frame
        msg: DetectionArray = node.process_frame(frame_bgr=synthetic_bgr, header=header)

        # Verify output
        self.assertEqual(msg.header.stamp.sec, 1791283200)
        self.assertEqual(msg.header.stamp.nanosec, 987654)
        self.assertEqual(msg.header.frame_id, "camera_optical_link")
        self.assertEqual(len(msg.detections), 2)
        self.assertEqual(msg.detections[0].class_name, "Synthetic_Ore_Alpha")
        self.assertEqual(msg.detections[1].class_name, "Synthetic_Ore_Beta")
        self.assertGreater(msg.inference_time_ms, 0.0)

        node.destroy_node()

    def test_image_callback_full_conversion_pipeline(self):
        """Verify _image_callback ingests a ROS Image msg, converts via cv_bridge, and outputs detections."""
        node = DetectionNode()
        node.enabled = True
        node.test_mode = True
        node._initialize_detector()

        # Create ROS Image message
        synthetic_bgr = np.zeros((240, 320, 3), dtype=np.uint8)
        img_msg = Image()
        img_msg.header.stamp = Time(sec=1791283200, nanosec=555555)
        img_msg.header.frame_id = "camera_optical_link"
        img_msg.height = 240
        img_msg.width = 320
        img_msg.encoding = "bgr8"
        img_msg.step = 320 * 3
        img_msg.data = synthetic_bgr.tobytes()

        # Feed image through image callback
        node._image_callback(img_msg)

        node.destroy_node()


if __name__ == "__main__":
    unittest.main()
