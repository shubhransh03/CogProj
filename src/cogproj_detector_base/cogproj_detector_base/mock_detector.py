"""Lightweight mock detector implementation for CogProj testing.

IMPORTANT NOTICE:
This is a synthetic test implementation intended strictly for unit testing,
CI/CD verification, and software pipeline validation.
It does NOT perform real ore detection, does NOT contain machine learning weights,
and does NOT pretend to be the user's trained detection model.
"""

from typing import Any, Dict, List, Optional
from .base_detector import BaseOreDetector, DetectionResult


class MockDetector(BaseOreDetector):
    """Synthetic detector for verifying CogProj architecture without ML runtimes."""

    def __init__(self) -> None:
        self.is_loaded = False
        self.model_dir: Optional[str] = None
        self.config: Dict[str, Any] = {}
        self.frame_count = 0
        self.deterministic_classes = ["Synthetic_Ore_Alpha", "Synthetic_Ore_Beta"]

    def load(self, model_dir: str, config: Optional[Dict[str, Any]] = None) -> bool:
        """Initialize mock detector state."""
        self.model_dir = model_dir
        self.config = config or {}
        self.is_loaded = True
        self.frame_count = 0
        return True

    def predict(self, frame_bgr: Any) -> List[DetectionResult]:
        """Generate deterministic synthetic detections for testing.

        Returns two predictable bounding boxes so tests can assert exact geometries.
        """
        if not self.is_loaded:
            raise RuntimeError("MockDetector predict() called before load() was initialized.")

        self.frame_count += 1

        # Check if frame size can be determined
        img_h, img_w = 240, 320
        if hasattr(frame_bgr, "shape") and len(frame_bgr.shape) >= 2:
            img_h, img_w = int(frame_bgr.shape[0]), int(frame_bgr.shape[1])

        # Deterministic synthetic detection #1: Synthetic_Ore_Alpha in top-left quadrant
        det1 = DetectionResult(
            class_id=1,
            class_name=self.deterministic_classes[0],
            confidence=0.88,
            x_min=float(img_w * 0.1),
            y_min=float(img_h * 0.1),
            x_max=float(img_w * 0.3),
            y_max=float(img_h * 0.4),
        )

        # Deterministic synthetic detection #2: Synthetic_Ore_Beta in bottom-right quadrant
        det2 = DetectionResult(
            class_id=2,
            class_name=self.deterministic_classes[1],
            confidence=0.75,
            x_min=float(img_w * 0.6),
            y_min=float(img_h * 0.5),
            x_max=float(img_w * 0.85),
            y_max=float(img_h * 0.85),
        )

        return [det1, det2]

    def get_metadata(self) -> Dict[str, Any]:
        """Return mock detector metadata."""
        return {
            "model_name": "MockDetector_TestOnly",
            "version": "0.1.0-test",
            "is_mock": True,
            "classes": self.deterministic_classes,
            "framework": "pure_python_test_harness",
            "description": "Synthetic mock detector for CogProj interface and pipeline verification",
        }

    def unload(self) -> None:
        """Reset mock detector state."""
        self.is_loaded = False
        self.frame_count = 0


# Export required plugin class name for testing plugin loader directly
OreDetectorPlugin = MockDetector

