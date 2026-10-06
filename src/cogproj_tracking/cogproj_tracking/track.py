"""Track class representing a single tracked object across temporal observations."""

from typing import Tuple
from cogproj_interfaces.msg import Detection, TrackedOre


class Track:
    """Represents a temporal object track with lifecycle and confirmation states."""

    STATE_TENTATIVE = "TENTATIVE"
    STATE_CONFIRMED = "CONFIRMED"
    STATE_LOST = "LOST"

    def __init__(
        self,
        track_id: int,
        detection: Detection,
        min_confirmed_hits: int = 3,
        max_missed_frames: int = 5,
    ) -> None:
        self.track_id: int = track_id
        self.class_id: int = detection.class_id
        self.class_name: str = detection.class_name
        self.confidence: float = float(detection.confidence)

        self.x_min: float = float(detection.x_min)
        self.y_min: float = float(detection.y_min)
        self.x_max: float = float(detection.x_max)
        self.y_max: float = float(detection.y_max)
        self.center_x: float = float(detection.center_x)
        self.center_y: float = float(detection.center_y)
        self.bbox_width: float = float(detection.bbox_width)
        self.bbox_height: float = float(detection.bbox_height)

        self.min_confirmed_hits: int = min_confirmed_hits
        self.max_missed_frames: int = max_missed_frames

        self.age: int = 1
        self.hits: int = 1
        self.missed_frames: int = 0
        self.state: str = self.STATE_TENTATIVE

        # Transition immediately if min_confirmed_hits is 1
        if self.hits >= self.min_confirmed_hits:
            self.state = self.STATE_CONFIRMED

    @property
    def center(self) -> Tuple[float, float]:
        """Return (center_x, center_y)."""
        return self.center_x, self.center_y

    @property
    def bbox(self) -> Tuple[float, float, float, float]:
        """Return (x_min, y_min, x_max, y_max)."""
        return self.x_min, self.y_min, self.x_max, self.y_max

    def update(self, detection: Detection) -> None:
        """Update track with matched detection."""
        self.class_id = detection.class_id
        self.class_name = detection.class_name
        self.confidence = float(detection.confidence)

        self.x_min = float(detection.x_min)
        self.y_min = float(detection.y_min)
        self.x_max = float(detection.x_max)
        self.y_max = float(detection.y_max)
        self.center_x = float(detection.center_x)
        self.center_y = float(detection.center_y)
        self.bbox_width = float(detection.bbox_width)
        self.bbox_height = float(detection.bbox_height)

        self.hits += 1
        self.missed_frames = 0
        self.age += 1

        if self.state == self.STATE_TENTATIVE and self.hits >= self.min_confirmed_hits:
            self.state = self.STATE_CONFIRMED

    def mark_missed(self) -> None:
        """Mark track as missed in the current frame."""
        self.missed_frames += 1
        self.age += 1

        if self.missed_frames > self.max_missed_frames:
            self.state = self.STATE_LOST

    def to_msg(self) -> TrackedOre:
        """Convert internal track state to TrackedOre ROS 2 message."""
        msg = TrackedOre()
        msg.track_id = self.track_id
        msg.class_id = self.class_id
        msg.class_name = self.class_name
        msg.confidence = self.confidence
        msg.x_min = self.x_min
        msg.y_min = self.y_min
        msg.x_max = self.x_max
        msg.y_max = self.y_max
        msg.center_x = self.center_x
        msg.center_y = self.center_y
        msg.bbox_width = self.bbox_width
        msg.bbox_height = self.bbox_height
        msg.age = self.age
        msg.hits = self.hits
        msg.missed_frames = self.missed_frames
        msg.state = self.state
        return msg

