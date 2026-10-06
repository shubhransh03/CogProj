"""Framework-independent base detector interface and data structures for CogProj.

This module defines the abstract contract that any ore detection model must implement,
ensuring complete decoupling from specific deep learning runtimes (PyTorch, TensorFlow,
ONNX Runtime, TensorRT, Ultralytics, etc.).
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


class BoundingBoxValidationError(ValueError):
    """Raised when bounding box coordinates or confidence violate validity constraints."""
    pass


@dataclass
class DetectionResult:
    """Standardized representation of a single detected object."""

    class_id: int
    class_name: str
    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    center_x: float = 0.0
    center_y: float = 0.0
    width: float = 0.0
    height: float = 0.0

    def __post_init__(self) -> None:
        """Validate detection coordinates and compute derived spatial attributes."""
        self.validate()
        self.width = float(self.x_max - self.x_min)
        self.height = float(self.y_max - self.y_min)
        self.center_x = float(self.x_min + self.x_max) / 2.0
        self.center_y = float(self.y_min + self.y_max) / 2.0

    def validate(self) -> None:
        """Check coordinate geometry and confidence bounds."""
        if not (0.0 <= self.confidence <= 1.0):
            raise BoundingBoxValidationError(
                f"Confidence {self.confidence} must be in range [0.0, 1.0]"
            )
        if self.x_min > self.x_max:
            raise BoundingBoxValidationError(
                f"x_min ({self.x_min}) cannot be greater than x_max ({self.x_max})"
            )
        if self.y_min > self.y_max:
            raise BoundingBoxValidationError(
                f"y_min ({self.y_min}) cannot be greater than y_max ({self.y_max})"
            )
        if self.x_min < 0.0 or self.y_min < 0.0:
            raise BoundingBoxValidationError(
                f"Coordinates cannot be negative: x_min={self.x_min}, y_min={self.y_min}"
            )

    def to_dict(self) -> Dict[str, Any]:
        """Serialize detection to a dictionary."""
        return {
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": self.confidence,
            "x_min": self.x_min,
            "y_min": self.y_min,
            "x_max": self.x_max,
            "y_max": self.y_max,
            "center_x": self.center_x,
            "center_y": self.center_y,
            "width": self.width,
            "height": self.height,
        }


class BaseOreDetector(ABC):
    """Abstract Base Class for all CogProj ore detector plugins.

    Every model adapter must inherit from this class and implement the required methods.
    """

    @abstractmethod
    def load(self, model_dir: str, config: Optional[Dict[str, Any]] = None) -> bool:
        """Initialize the model, weights, and inference runtime.

        Args:
            model_dir: Absolute path to the directory containing model weights and assets.
            config: Optional configuration dictionary loaded from config.yaml or ROS parameters.

        Returns:
            True if model loading succeeded, False otherwise.
        """
        pass

    @abstractmethod
    def predict(self, frame_bgr: Any) -> List[DetectionResult]:
        """Perform object detection inference on a single image frame.

        Args:
            frame_bgr: Input image in BGR format (typically numpy.ndarray of shape [H, W, 3]).

        Returns:
            List of validated DetectionResult objects.
        """
        pass

    @abstractmethod
    def get_metadata(self) -> Dict[str, Any]:
        """Return metadata describing the detector.

        Returns:
            Dictionary containing model information (e.g., model_name, version,
            class_labels, input_resolution, framework).
        """
        pass

    @abstractmethod
    def unload(self) -> None:
        """Release all model resources, accelerators, and memory."""
        pass

