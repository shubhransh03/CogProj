"""Message conversion utilities for CogProj detection pipeline."""

from typing import List
from std_msgs.msg import Header

from cogproj_detector_base import DetectionResult
from cogproj_interfaces.msg import Detection, DetectionArray


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
        if r.confidence >= confidence_threshold:
            filtered_detections.append(detection_result_to_msg(r))

    array_msg.detections = filtered_detections
    return array_msg

