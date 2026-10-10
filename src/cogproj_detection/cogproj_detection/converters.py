"""Message conversion utilities for CogProj detection pipeline."""

import logging
import math
from typing import Any, List
from std_msgs.msg import Header

from cogproj_detector_base import BoundingBoxValidationError, DetectionResult
from cogproj_interfaces.msg import Detection, DetectionArray

logger = logging.getLogger("cogproj_detection.converters")


def is_valid_detection_result(result: Any, log_rejections: bool = True) -> bool:
    """Validate that a detection result object is finite, bounded, and geometrically consistent.

    Args:
        result: An instance of DetectionResult or object with detection attributes.
        log_rejections: If True, logs a warning explaining why a result was rejected.

    Returns:
        True if the detection is valid, False otherwise.
    """
    if result is None:
        if log_rejections:
            logger.warning("Rejected detection: result is None")
        return False

    # Check for required attributes
    required_fields = ("class_id", "class_name", "confidence", "x_min", "y_min", "x_max", "y_max")
    for f in required_fields:
        if not hasattr(result, f):
            if log_rejections:
                logger.warning(f"Rejected detection: missing required field '{f}'")
            return False

    # Check that class_name is a string
    if not isinstance(result.class_name, str):
        if log_rejections:
            logger.warning(f"Rejected detection: class_name is not a string ({type(result.class_name)})")
        return False

    # Convert numeric fields and check finiteness
    try:
        _ = int(result.class_id)
        conf = float(result.confidence)
        x_min = float(result.x_min)
        y_min = float(result.y_min)
        x_max = float(result.x_max)
        y_max = float(result.y_max)
    except (TypeError, ValueError) as e:
        if log_rejections:
            logger.warning(f"Rejected detection: numeric conversion failed: {e}")
        return False

    if not math.isfinite(conf):
        if log_rejections:
            logger.warning(f"Rejected detection: confidence {conf} is not finite")
        return False

    if not (0.0 <= conf <= 1.0):
        if log_rejections:
            logger.warning(f"Rejected detection: confidence {conf} out of range [0.0, 1.0]")
        return False

    for name, val in [("x_min", x_min), ("y_min", y_min), ("x_max", x_max), ("y_max", y_max)]:
        if not math.isfinite(val):
            if log_rejections:
                logger.warning(f"Rejected detection: coordinate '{name}'={val} is not finite")
            return False

    if x_min < 0.0 or y_min < 0.0:
        if log_rejections:
            logger.warning(f"Rejected detection: negative coordinates (x_min={x_min}, y_min={y_min})")
        return False

    if x_min > x_max:
        if log_rejections:
            logger.warning(f"Rejected detection: inverted x bounds (x_min={x_min} > x_max={x_max})")
        return False

    if y_min > y_max:
        if log_rejections:
            logger.warning(f"Rejected detection: inverted y bounds (y_min={y_min} > y_max={y_max})")
        return False

    # If result has validate() method (e.g. DetectionResult), run it
    if hasattr(result, "validate") and callable(result.validate):
        try:
            result.validate()
        except BoundingBoxValidationError as e:
            if log_rejections:
                logger.warning(f"Rejected detection: BoundingBoxValidationError: {e}")
            return False
        except Exception as e:
            if log_rejections:
                logger.warning(f"Rejected detection: validation error: {e}")
            return False

    return True


def detection_result_to_msg(result: DetectionResult) -> Detection:
    """Convert a framework-independent DetectionResult into a ROS 2 Detection message."""
    msg = Detection()
    msg.class_id = int(result.class_id)
    msg.class_name = str(result.class_name)
    msg.confidence = float(result.confidence)
    msg.x_min = float(result.x_min)
    msg.y_min = float(result.y_min)
    msg.x_max = float(result.x_max)
    msg.y_max = float(result.y_max)
    msg.center_x = float(result.center_x)
    msg.center_y = float(result.center_y)
    msg.bbox_width = float(result.width)
    msg.bbox_height = float(result.height)
    return msg


def detection_results_to_array_msg(
    results: List[DetectionResult],
    header: Header,
    inference_time_ms: float = 0.0,
    confidence_threshold: float = 0.0,
) -> DetectionArray:
    """Filter and convert a list of DetectionResult objects into a DetectionArray message."""
    array_msg = DetectionArray()
    array_msg.header = header
    array_msg.inference_time_ms = float(inference_time_ms)

    filtered_detections: List[Detection] = []
    for r in results:
        # Validate geometry, finiteness, and bounds before converting
        if not is_valid_detection_result(r):
            continue
        if r.confidence >= confidence_threshold:
            filtered_detections.append(detection_result_to_msg(r))

    array_msg.detections = filtered_detections
    return array_msg
