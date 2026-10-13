"""Unit tests for CogProj PreflightChecker diagnostic tool."""

import unittest
import rclpy
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from cogproj_bringup.preflight_check import PreflightChecker


class TestPreflightChecker(unittest.TestCase):
    """Test suite for preflight environment and topic checking logic."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.checker = PreflightChecker(mode="mock", timeout_sec=0.1)

    def tearDown(self):
        self.checker.destroy_node()

    def test_environment_audit_keys(self):
        """Verify environment audit exposes expected platform metrics."""
        env = self.checker.audit_environment()
        assert "Host" in env
        assert "Platform" in env
        assert "ROS_DISTRO" in env
        assert "ROS_DOMAIN_ID" in env
        assert "Selected Mode" in env
        assert env["Selected Mode"] == "MOCK"

    def test_model_artifacts_audit(self):
        """Verify ONNX model weights and adapter are detected as READY."""
        models = self.checker.audit_model_artifacts()
        assert "ONNX Weights" in models
        assert "Model Adapter" in models
        assert models["ONNX Weights"][0] == "READY"
        assert models["Model Adapter"][0] == "READY"

    def test_camera_info_uncalibrated_detection(self):
        """Verify uncalibrated CameraInfo (K=0) is detected and reported."""
        info = CameraInfo()
        info.k = [0.0] * 9
        desc = self.checker._format_sample_details("/camera/camera_info", info)
        assert "UNCALIBRATED" in desc

    def test_camera_info_calibrated_detection(self):
        """Verify calibrated CameraInfo (fx>0, fy>0) reports focal lengths."""
        info = CameraInfo()
        info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        desc = self.checker._format_sample_details("/camera/camera_info", info)
        assert "CALIBRATED" in desc
        assert "fx=500.0" in desc

    def test_image_details_formatting(self):
        """Verify image details formatting reports resolution and encoding."""
        img = Image()
        img.width = 640
        img.height = 480
        img.encoding = "bgra8"
        img.step = 2560
        desc = self.checker._format_sample_details("/camera/image_raw", img)
        assert "640x480" in desc
        assert "bgra8" in desc

    def test_checker_has_zero_actuation_publishers(self):
        """SAFETY AUDIT: Verify PreflightChecker registers zero actuation publishers."""
        for pub in self.checker.publishers:
            topic = pub.topic_name
            assert "cmd_vel" not in topic

