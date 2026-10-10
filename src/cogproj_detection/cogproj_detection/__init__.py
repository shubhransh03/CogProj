"""cogproj_detection package."""

from .converters import (
    detection_result_to_msg,
    detection_results_to_array_msg,
    is_valid_detection_result,
)
from .detection_node import DetectionNode

__all__ = [
    "DetectionNode",
    "detection_result_to_msg",
    "detection_results_to_array_msg",
    "is_valid_detection_result",
]

