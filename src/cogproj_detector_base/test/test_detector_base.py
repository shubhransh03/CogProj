"""Unit tests for cogproj_detector_base: data structures, validation, mock detector, and plugin loader."""

import os
import shutil
import tempfile
import unittest

from cogproj_detector_base import (
    BaseOreDetector,
    BoundingBoxValidationError,
    DetectionResult,
    MockDetector,
    PluginError,
    PluginInterfaceError,
    PluginNotFoundError,
    load_detector_plugin,
)


class TestDetectionResultAndValidation(unittest.TestCase):
    """Test DetectionResult creation and coordinate validation logic."""

    def test_detection_result_valid(self):
        det = DetectionResult(
            class_id=1,
            class_name="Malachite",
            confidence=0.92,
            x_min=10.0,
            y_min=20.0,
            x_max=50.0,
            y_max=80.0,
        )
        self.assertEqual(det.class_id, 1)
        self.assertEqual(det.class_name, "Malachite")
        self.assertAlmostEqual(det.confidence, 0.92)
        self.assertAlmostEqual(det.x_min, 10.0)
        self.assertAlmostEqual(det.y_min, 20.0)
        self.assertAlmostEqual(det.x_max, 50.0)
        self.assertAlmostEqual(det.y_max, 80.0)
        self.assertAlmostEqual(det.width, 40.0)
        self.assertAlmostEqual(det.height, 60.0)
        self.assertAlmostEqual(det.center_x, 30.0)
        self.assertAlmostEqual(det.center_y, 50.0)

    def test_bounding_box_invalid_confidence_high(self):
        with self.assertRaises(BoundingBoxValidationError):
            DetectionResult(1, "Ore", 1.5, 0.0, 0.0, 10.0, 10.0)

    def test_bounding_box_invalid_confidence_negative(self):
        with self.assertRaises(BoundingBoxValidationError):
            DetectionResult(1, "Ore", -0.1, 0.0, 0.0, 10.0, 10.0)

    def test_bounding_box_invalid_xmin_greater_xmax(self):
        with self.assertRaises(BoundingBoxValidationError):
            DetectionResult(1, "Ore", 0.8, 50.0, 10.0, 20.0, 40.0)

    def test_bounding_box_invalid_ymin_greater_ymax(self):
        with self.assertRaises(BoundingBoxValidationError):
            DetectionResult(1, "Ore", 0.8, 10.0, 60.0, 40.0, 20.0)

    def test_bounding_box_invalid_negative_coordinates(self):
        with self.assertRaises(BoundingBoxValidationError):
            DetectionResult(1, "Ore", 0.8, -5.0, 10.0, 20.0, 40.0)


class TestMockDetector(unittest.TestCase):
    """Test MockDetector functionality without any ML runtime."""

    def test_mock_detector_lifecycle(self):
        detector = MockDetector()
        self.assertFalse(detector.is_loaded)

        success = detector.load(model_dir="/tmp/test_mock")
        self.assertTrue(success)
        self.assertTrue(detector.is_loaded)

        metadata = detector.get_metadata()
        self.assertTrue(metadata.get("is_mock"))
        self.assertEqual(metadata.get("model_name"), "MockDetector_TestOnly")

        detections = detector.predict(frame_bgr=None)
        self.assertEqual(len(detections), 2)
        self.assertEqual(detections[0].class_name, "Synthetic_Ore_Alpha")
        self.assertEqual(detections[1].class_name, "Synthetic_Ore_Beta")

        detector.unload()
        self.assertFalse(detector.is_loaded)

    def test_predict_before_load_raises_runtime_error(self):
        detector = MockDetector()
        with self.assertRaises(RuntimeError):
            detector.predict(frame_bgr=None)


class TestPluginLoader(unittest.TestCase):
    """Test safe dynamic loading, contract validation, and error reporting."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_missing_model_directory(self):
        non_existent = os.path.join(self.test_dir, "non_existent_dir")
        with self.assertRaises(PluginNotFoundError):
            load_detector_plugin(model_dir=non_existent)

    def test_missing_adapter_file(self):
        empty_dir = os.path.join(self.test_dir, "empty_model")
        os.makedirs(empty_dir, exist_ok=True)
        with self.assertRaises(PluginNotFoundError):
            load_detector_plugin(model_dir=empty_dir)

    def test_adapter_missing_ore_detector_plugin_class(self):
        invalid_plugin_dir = os.path.join(self.test_dir, "invalid_plugin")
        os.makedirs(invalid_plugin_dir, exist_ok=True)
        adapter_path = os.path.join(invalid_plugin_dir, "adapter.py")
        with open(adapter_path, "w") as f:
            f.write("# Does not export OreDetectorPlugin\nSOME_VAR = 42\n")

        with self.assertRaises(PluginInterfaceError):
            load_detector_plugin(model_dir=invalid_plugin_dir)

    def test_adapter_class_not_subclass_of_base(self):
        invalid_plugin_dir = os.path.join(self.test_dir, "not_a_subclass")
        os.makedirs(invalid_plugin_dir, exist_ok=True)
        adapter_path = os.path.join(invalid_plugin_dir, "adapter.py")
        with open(adapter_path, "w") as f:
            f.write("class OreDetectorPlugin:\n    pass\n")

        with self.assertRaises(PluginInterfaceError):
            load_detector_plugin(model_dir=invalid_plugin_dir)

    def test_valid_plugin_loading(self):
        valid_dir = os.path.join(self.test_dir, "valid_model")
        os.makedirs(valid_dir, exist_ok=True)
        adapter_path = os.path.join(valid_dir, "adapter.py")
        with open(adapter_path, "w") as f:
            f.write("""
from cogproj_detector_base import BaseOreDetector, DetectionResult

class OreDetectorPlugin(BaseOreDetector):
    def load(self, model_dir, config=None):
        self.loaded = True
        return True

    def predict(self, frame_bgr):
        return [DetectionResult(1, "TestOre", 0.95, 0.0, 0.0, 10.0, 10.0)]

    def get_metadata(self):
        return {"model_name": "DynamicTestDetector"}

    def unload(self):
        self.loaded = False
""")

        detector = load_detector_plugin(model_dir=valid_dir)
        self.assertIsInstance(detector, BaseOreDetector)
        metadata = detector.get_metadata()
        self.assertEqual(metadata["model_name"], "DynamicTestDetector")

        results = detector.predict(None)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].class_name, "TestOre")
        detector.unload()


if __name__ == "__main__":
    unittest.main()

