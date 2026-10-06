"""Multi-Object Tracker for CogProj.

Deterministic image-space multi-object tracker using centroid distance,
bounding-box IoU, and class consistency with Hungarian association.
"""

import math
from typing import List, Tuple
import numpy as np
from scipy.optimize import linear_sum_assignment

from cogproj_interfaces.msg import Detection
from .track import Track


def compute_iou(bbox1: Tuple[float, float, float, float], bbox2: Tuple[float, float, float, float]) -> float:
    """Compute Intersection-over-Union (IoU) between two bounding boxes.

    Args:
        bbox1: (x_min, y_min, x_max, y_max)
        bbox2: (x_min, y_min, x_max, y_max)

    Returns:
        IoU value in range [0.0, 1.0].
    """
    x_min1, y_min1, x_max1, y_max1 = bbox1
    x_min2, y_min2, x_max2, y_max2 = bbox2

    inter_x_min = max(x_min1, x_min2)
    inter_y_min = max(y_min1, y_min2)
    inter_x_max = min(x_max1, x_max2)
    inter_y_max = min(y_max1, y_max2)

    inter_w = max(0.0, inter_x_max - inter_x_min)
    inter_h = max(0.0, inter_y_max - inter_y_min)
    inter_area = inter_w * inter_h

    area1 = max(0.0, x_max1 - x_min1) * max(0.0, y_max1 - y_min1)
    area2 = max(0.0, x_max2 - x_min2) * max(0.0, y_max2 - y_min2)

    union_area = area1 + area2 - inter_area
    if union_area <= 0.0:
        return 0.0
    return inter_area / union_area


def compute_centroid_distance(c1: Tuple[float, float], c2: Tuple[float, float]) -> float:
    """Compute Euclidean distance between two 2D centroids in pixel coordinates."""
    return math.hypot(c1[0] - c2[0], c1[1] - c2[1])


class MultiObjectTracker:
    """Deterministic multi-object tracker for camera-space detections."""

    GATE_INFINITY = 1e6

    def __init__(
        self,
        max_centroid_distance_px: float = 50.0,
        min_iou: float = 0.10,
        max_missed_frames: int = 5,
        min_confirmed_hits: int = 3,
    ) -> None:
        """Initialize MultiObjectTracker with configurable parameters.

        Args:
            max_centroid_distance_px: Maximum allowed Euclidean distance in pixels
                for associating a detection to an existing track (default: 50.0 px,
                chosen conservatively for 320x240 image resolution).
            min_iou: Minimum IoU overlap fallback gate (default: 0.10).
            max_missed_frames: Maximum consecutive frames a track may be missed before
                being declared LOST and removed (default: 5 frames).
            min_confirmed_hits: Number of detections required before a TENTATIVE track
                transitions to CONFIRMED (default: 3 frames hysteresis).
        """
        self.max_centroid_distance_px = float(max_centroid_distance_px)
        self.min_iou = float(min_iou)
        self.max_missed_frames = int(max_missed_frames)
        self.min_confirmed_hits = int(min_confirmed_hits)

        self._next_track_id: int = 1
        self.tracks: List[Track] = []

    @property
    def active_tracks(self) -> List[Track]:
        """Return all active tracks (TENTATIVE and CONFIRMED)."""
        return [t for t in self.tracks if t.state != Track.STATE_LOST]

    @property
    def confirmed_tracks(self) -> List[Track]:
        """Return only CONFIRMED tracks."""
        return [t for t in self.tracks if t.state == Track.STATE_CONFIRMED]

    def update(self, detections: List[Detection]) -> List[Track]:
        """Update tracker with incoming detections from a camera frame.

        Args:
            detections: List of Detection messages for the current frame.

        Returns:
            List of active tracks after update and pruning.
        """
        # Case 1: No existing active tracks
        if not self.tracks:
            for det in detections:
                new_track = Track(
                    track_id=self._next_track_id,
                    detection=det,
                    min_confirmed_hits=self.min_confirmed_hits,
                    max_missed_frames=self.max_missed_frames,
                )
                self._next_track_id += 1
                self.tracks.append(new_track)
            return self.active_tracks

        # Case 2: No incoming detections this frame
        if not detections:
            for track in self.tracks:
                track.mark_missed()
            # Prune lost tracks
            self.tracks = [t for t in self.tracks if t.state != Track.STATE_LOST]
            return self.active_tracks

        # Case 3: Pairwise association via Hungarian matching
        num_tracks = len(self.tracks)
        num_dets = len(detections)
        cost_matrix = np.full((num_tracks, num_dets), self.GATE_INFINITY, dtype=np.float64)

        for i, track in enumerate(self.tracks):
            for j, det in enumerate(detections):
                # Hard gate: class mismatch
                if track.class_id != det.class_id:
                    continue

                det_center = (float(det.center_x), float(det.center_y))
                dist = compute_centroid_distance(track.center, det_center)

                det_bbox = (float(det.x_min), float(det.y_min), float(det.x_max), float(det.y_max))
                iou = compute_iou(track.bbox, det_bbox)

                # Spatial gate: must be within max_centroid_distance_px OR overlap with min_iou
                is_within_dist = dist <= self.max_centroid_distance_px
                is_within_iou = iou >= self.min_iou and self.min_iou > 0.0

                if not (is_within_dist or is_within_iou):
                    continue

                # Association cost combining normalized distance and IoU penalty
                norm_dist = min(dist / max(self.max_centroid_distance_px, 1.0), 1.0)
                iou_penalty = 1.0 - iou
                cost_matrix[i, j] = norm_dist + iou_penalty

        # Solve optimal bipartite matching
        row_indices, col_indices = linear_sum_assignment(cost_matrix)

        matched_tracks = set()
        matched_dets = set()

        for r, c in zip(row_indices, col_indices):
            if cost_matrix[r, c] < (self.GATE_INFINITY / 2.0):
                self.tracks[r].update(detections[c])
                matched_tracks.add(r)
                matched_dets.add(c)

        # Handle unmatched active tracks (missed detection in current frame)
        for i in range(num_tracks):
            if i not in matched_tracks:
                self.tracks[i].mark_missed()

        # Handle unmatched detections (new tentative objects)
        for j in range(num_dets):
            if j not in matched_dets:
                new_track = Track(
                    track_id=self._next_track_id,
                    detection=detections[j],
                    min_confirmed_hits=self.min_confirmed_hits,
                    max_missed_frames=self.max_missed_frames,
                )
                self._next_track_id += 1
                self.tracks.append(new_track)

        # Prune LOST tracks
        self.tracks = [t for t in self.tracks if t.state != Track.STATE_LOST]

        return self.active_tracks

    def reset(self) -> None:
        """Reset tracker state and active tracks."""
        self.tracks.clear()
        self._next_track_id = 1

