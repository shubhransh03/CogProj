"""cogproj_detection package."""

from .converters import detection_result_to_msg, detection_results_to_array_msg
from .detection_node import DetectionNode

__all__ = [
    "DetectionNode",
    "detection_result_to_msg",
    "detection_results_to_array_msg",
]

