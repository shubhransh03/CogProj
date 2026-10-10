"""Integration tests for real ONNX detector within the CogProj detection pipeline.

Tests the integration path from detection_node's plugin-loading mechanism
through the ONNX adapter, without starting ROS nodes or the camera driver.
"""

import os
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Header

from cogproj_detector_base import (
    BaseOreDetector,
    DetectionResult,
    PluginError,
    PluginNotFoundError,
    load_detector_plugin,
)
from cogproj_detection import DetectionNode, detection_results_to_array_msg

# Path to the adapter directory
ADAPTER_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))
    )))),
    "models", "ore_yolo",
)

# Fallback: resolve relative to CogProj workspace
if not os.path.isdir(ADAPTER_DIR):
    ADAPTER_DIR = "/home/veerobot/CogProj/models/ore_yolo"

MODEL_FILE = "/home/veerobot/CogProj/Rover (2)/best.onnx"
MODEL_AVAILABLE = os.path.isfile(MODEL_FILE)


class TestDefaultConfiguration(unittest.TestCase):
    """Verify default configuration does not enable the real model."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def test_default_detection_node_is_disabled(self):
        """Default DetectionNode must start in DISABLED state."""
        node = DetectionNode()
        try:
            self.assertFalse(node.enabled)
            self.assertFalse(node.test_mode)
            self.assertEqual(node.status, "DISABLED")
            self.assertIsNone(node.detector)
        finally:
            node.destroy_node()

    def test_mock_mode_still_works(self):
        """Mock detector selection must still function."""
        node = DetectionNode()
        try:
            node.enabled = True
            node.test_mode = True
            result = node._initialize_detector()
            self.assertTrue(result)
            self.assertEqual(node.status, "MOCK")
            self.assertIsNotNone(node.detector)
            metadata = node.detector.get_metadata()
            self.assertTrue(metadata.get("is_mock", False))
        finally:
            node.destroy_node()

    def test_enabled_without_model_dir_fails_safely(self):
        """Enabled real mode without model_directory must fail safely."""
        node = DetectionNode()
        try:
            node.enabled = True
            node.test_mode = False
            node.model_dir = ""
            result = node._initialize_detector()
            self.assertFalse(result)
            self.assertEqual(node.status, "ERROR_NO_MODEL")
            self.assertIsNone(node.detector)
        finally:
            node.destroy_node()

    def test_invalid_model_dir_fails_safely(self):
        """Pointing to a nonexistent directory must fail safely."""
        node = DetectionNode()
        try:
            node.enabled = True
            node.test_mode = False
            node.model_dir = "/nonexistent/path/to/model"
            result = node._initialize_detector()
            self.assertFalse(result)
            self.assertEqual(node.status, "ERROR_LOAD_FAILED")
            self.assertIsNone(node.detector)
        finally:
            node.destroy_node()


class TestPluginLoaderIntegration(unittest.TestCase):
    """Verify the plugin loader discovers and validates the ONNX adapter."""

    def test_plugin_loader_finds_adapter(self):
        """Plugin loader must find adapter.py in models/ore_yolo."""
        if not os.path.isdir(ADAPTER_DIR):
            self.skipTest(f"Adapter directory not found: {ADAPTER_DIR}")

        # Load with auto_init=False to avoid needing the model file
        plugin = load_detector_plugin(
            model_dir=ADAPTER_DIR,
            auto_init=False,
        )
        self.assertIsInstance(plugin, BaseOreDetector)
        self.assertFalse(plugin.is_loaded)

    def test_plugin_loader_rejects_empty_dir(self):
        """Plugin loader must reject a directory without adapter.py."""
        import tempfile
        with tempfile.TemporaryDirectory() as tmpdir:
            with self.assertRaises(PluginNotFoundError):
                load_detector_plugin(model_dir=tmpdir)

    @unittest.skipUnless(MODEL_AVAILABLE, "ONNX model file not available")
    def test_plugin_loader_with_auto_init(self):
        """Plugin loader auto_init must successfully load the ONNX model."""
        plugin = load_detector_plugin(
            model_dir=ADAPTER_DIR,
            auto_init=True,
        )
        self.assertIsInstance(plugin, BaseOreDetector)
        self.assertTrue(plugin.is_loaded)
        metadata = plugin.get_metadata()
        self.assertFalse(metadata.get("is_mock", True))
        self.assertEqual(metadata.get("framework"), "opencv_dnn_onnx")
        plugin.unload()


class TestDetectionNodeRealModelIntegration(unittest.TestCase):
    """Test DetectionNode integration with the real ONNX adapter."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    @unittest.skipUnless(MODEL_AVAILABLE, "ONNX model file not available")
    def test_detection_node_loads_real_model(self):
        """DetectionNode must load real model when model_directory is set."""
        node = DetectionNode()
        try:
            node.enabled = True
            node.test_mode = False
            node.model_dir = ADAPTER_DIR
            result = node._initialize_detector()
            self.assertTrue(result)
            self.assertTrue(node.status.startswith("REAL:"))
            self.assertIsNotNone(node.detector)
            self.assertFalse(node.detector.get_metadata().get("is_mock", True))
        finally:
            node.destroy_node()

    @unittest.skipUnless(MODEL_AVAILABLE, "ONNX model file not available")
    def test_real_model_synthetic_frame(self):
        """Real model must process a synthetic frame and return valid DetectionResults."""
        node = DetectionNode()
        try:
            node.enabled = True
            node.test_mode = False
            node.model_dir = ADAPTER_DIR
            result = node._initialize_detector()
            self.assertTrue(result)

            # Create a synthetic 640x480 BGR frame
            synthetic = np.zeros((480, 640, 3), dtype=np.uint8)

            header = Header()
            header.frame_id = "test_real_detector"

            det_msg = node.process_frame(synthetic, header=header)
            self.assertIsNotNone(det_msg)
            self.assertEqual(det_msg.header.frame_id, "test_real_detector")
            self.assertGreaterEqual(det_msg.inference_time_ms, 0.0)

            # Detections on a blank image may be empty — that's valid
            for det in det_msg.detections:
                self.assertIsInstance(det.class_name, str)
                self.assertIn(det.class_name, ["resource", "host_rock"])
                self.assertGreaterEqual(det.confidence, 0.0)
                self.assertLessEqual(det.confidence, 1.0)
                self.assertGreaterEqual(det.x_min, 0.0)
                self.assertGreaterEqual(det.y_min, 0.0)
        finally:
            node.destroy_node()

    @unittest.skipUnless(MODEL_AVAILABLE, "ONNX model file not available")
    def test_real_model_unload_on_destroy(self):
        """Destroying DetectionNode must safely unload the real model."""
        node = DetectionNode()
        try:
            node.enabled = True
            node.test_mode = False
            node.model_dir = ADAPTER_DIR
            node._initialize_detector()
            self.assertIsNotNone(node.detector)
        finally:
            node.destroy_node()
        self.assertEqual(node.status, "UNLOADED")


class TestClassAndResultConsistency(unittest.TestCase):
    """Verify class names and DetectionResult conversion consistency."""

    @unittest.skipUnless(MODEL_AVAILABLE, "ONNX model file not available")
    def test_class_names_match(self):
        """Adapter class names must be 'resource' and 'host_rock'."""
        plugin = load_detector_plugin(model_dir=ADAPTER_DIR, auto_init=True)
        try:
            metadata = plugin.get_metadata()
            classes = metadata.get("classes", {})
            self.assertEqual(classes.get(0), "resource")
            self.assertEqual(classes.get(1), "host_rock")
        finally:
            plugin.unload()

    def test_detection_result_converts_to_ros_msg(self):
        """DetectionResult with real-model class names must convert cleanly."""
        det = DetectionResult(
            class_id=0,
            class_name="resource",
            confidence=0.85,
            x_min=10.0, y_min=20.0,
            x_max=100.0, y_max=200.0,
        )

        header = Header()
        msg = detection_results_to_array_msg(
            results=[det],
            header=header,
            confidence_threshold=0.50,
        )
        self.assertEqual(len(msg.detections), 1)
        self.assertEqual(msg.detections[0].class_name, "resource")
        self.assertEqual(msg.detections[0].class_id, 0)
        self.assertAlmostEqual(msg.detections[0].confidence, 0.85, places=4)

    def test_host_rock_detection_converts(self):
        """DetectionResult for host_rock class must convert cleanly."""
        det = DetectionResult(
            class_id=1,
            class_name="host_rock",
            confidence=0.72,
            x_min=50.0, y_min=60.0,
            x_max=200.0, y_max=300.0,
        )

        header = Header()
        msg = detection_results_to_array_msg(
            results=[det],
            header=header,
            confidence_threshold=0.50,
        )
        self.assertEqual(len(msg.detections), 1)
        self.assertEqual(msg.detections[0].class_name, "host_rock")
        self.assertEqual(msg.detections[0].class_id, 1)


class TestLaunchFileIntegrity(unittest.TestCase):
    """Verify real-model launch file integrity and safety."""

    def test_real_detector_launch_file_exists(self):
        """The real detector launch file must exist."""
        launch_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "launch", "real_detector_pipeline.launch.py",
        )
        if not os.path.isfile(launch_path):
            launch_path = "/home/veerobot/CogProj/src/cogproj_bringup/launch/real_detector_pipeline.launch.py"
        self.assertTrue(os.path.isfile(launch_path), f"Launch file not found: {launch_path}")

    def test_real_detector_launch_file_has_no_actuation(self):
        """The real detector launch file must not reference actuation topics."""
        launch_path = "/home/veerobot/CogProj/src/cogproj_bringup/launch/real_detector_pipeline.launch.py"
        if not os.path.isfile(launch_path):
            self.skipTest("Launch file not found")

        with open(launch_path, "r") as f:
            content = f.read()

        forbidden = ["/cmd_vel", "cmd_vel_nav", "cmd_vel_joy", "lyra_control",
                     "nav2", "slam_toolbox"]
        for term in forbidden:
            self.assertNotIn(term, content,
                             f"Launch file contains forbidden term: '{term}'")

    def test_real_detector_launch_file_is_perception_only(self):
        """Launch file must only reference perception packages."""
        launch_path = "/home/veerobot/CogProj/src/cogproj_bringup/launch/real_detector_pipeline.launch.py"
        if not os.path.isfile(launch_path):
            self.skipTest("Launch file not found")

        with open(launch_path, "r") as f:
            content = f.read()

        expected_packages = [
            "cogproj_camera", "cogproj_detection", "cogproj_tracking",
            "cogproj_counting", "cogproj_visualization",
        ]
        for pkg in expected_packages:
            self.assertIn(pkg, content, f"Expected package '{pkg}' not in launch file")

    def test_mock_launch_unchanged(self):
        """The mock pipeline launch file must still use test_mode=True."""
        mock_path = "/home/veerobot/CogProj/src/cogproj_bringup/launch/mock_full_pipeline.launch.py"
        if not os.path.isfile(mock_path):
            self.skipTest("Mock launch file not found")

        with open(mock_path, "r") as f:
            content = f.read()

        self.assertIn('"test_mode": True', content)
        self.assertIn('"enabled": True', content)


if __name__ == "__main__":
    unittest.main()

