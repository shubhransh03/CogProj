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

        try:
            # Capture published messages
            published_msgs = []
            node.detection_pub.publish = lambda msg: published_msgs.append(msg)

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

            # Exercise image callback
            node._image_callback(img_msg)

            # Assert published message received and correctly populated
            self.assertEqual(len(published_msgs), 1)
            out_msg = published_msgs[0]
            self.assertEqual(out_msg.header.stamp.sec, 1791283200)
            self.assertEqual(out_msg.header.stamp.nanosec, 555555)
            self.assertEqual(out_msg.header.frame_id, "camera_optical_link")
            self.assertEqual(len(out_msg.detections), 2)
            self.assertEqual(out_msg.detections[0].class_name, "Synthetic_Ore_Alpha")
            self.assertEqual(out_msg.detections[1].class_name, "Synthetic_Ore_Beta")
            self.assertGreater(out_msg.inference_time_ms, 0.0)
        finally:
            node.destroy_node()

    def test_detector_status_reporting(self):
        """Verify detector status reporting across disabled, mock, error, and unloaded states."""
        # 1. Disabled state
        node = DetectionNode()
        try:
            self.assertEqual(node.status, "DISABLED")
            self.assertEqual(node.get_parameter("detector_status").get_parameter_value().string_value, "DISABLED")

            # 2. Enabled without model
            node.enabled = True
            node.model_dir = ""
            node.test_mode = False
            self.assertFalse(node._initialize_detector())
            self.assertEqual(node.status, "ERROR_NO_MODEL")
            self.assertEqual(node.get_parameter("detector_status").get_parameter_value().string_value, "ERROR_NO_MODEL")

            # 3. Enabled with invalid model dir (load failure)
            node.model_dir = "/nonexistent/invalid/model/path"
            self.assertFalse(node._initialize_detector())
            self.assertEqual(node.status, "ERROR_LOAD_FAILED")
            self.assertEqual(node.get_parameter("detector_status").get_parameter_value().string_value, "ERROR_LOAD_FAILED")

            # 4. Mock mode
            node.test_mode = True
            self.assertTrue(node._initialize_detector())
            self.assertEqual(node.status, "MOCK")
            self.assertEqual(node.get_parameter("detector_status").get_parameter_value().string_value, "MOCK")
        finally:
            node.destroy_node()

        # 5. Destroyed state
        self.assertEqual(node.status, "UNLOADED")

    def test_invalid_detection_result_filtered_safely(self):
        """Verify detection_results_to_array_msg handles malformed detection results safely."""
        header = Header()
        valid = DetectionResult(1, "ValidOre", 0.90, 10.0, 10.0, 50.0, 50.0)

        # Create duck-typed detections with various corruption modes
        class CorruptDetection:
            def __init__(self, **kwargs):
                self.class_id = kwargs.get("class_id", 1)
                self.class_name = kwargs.get("class_name", "Corrupt")
                self.confidence = kwargs.get("confidence", 0.8)
                self.x_min = kwargs.get("x_min", 0.0)
                self.y_min = kwargs.get("y_min", 0.0)
                self.x_max = kwargs.get("x_max", 10.0)
                self.y_max = kwargs.get("y_max", 10.0)

        c_nan_conf = CorruptDetection(confidence=float("nan"))
        c_inf_coord = CorruptDetection(x_min=float("inf"))
        c_neg_coord = CorruptDetection(x_min=-5.0)
        c_inverted = CorruptDetection(x_min=20.0, x_max=10.0)
        c_bad_type = CorruptDetection(class_name=12345)
        c_missing = object()

        results = [
            valid,
            c_nan_conf,
            c_inf_coord,
            c_neg_coord,
            c_inverted,
            c_bad_type,
            c_missing,
            None,
        ]

        msg = detection_results_to_array_msg(
            results=results,
            header=header,
            confidence_threshold=0.50,
        )
        self.assertEqual(len(msg.detections), 1)
        self.assertEqual(msg.detections[0].class_name, "ValidOre")
        self.assertEqual(msg.detections[0].class_id, 1)


if __name__ == "__main__":
    unittest.main()
