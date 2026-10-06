"""Ore Counter core logic for unique ore counting across confirmed tracks.

Maintains session cumulative count and per-class counts, preventing
duplicate counting of the same object across temporal observations.
"""

from typing import Dict, List, Optional, Set
from std_msgs.msg import Header
from cogproj_interfaces.msg import OreCountSummary, TrackedOre


class OreCounter:
    """Session-level unique ore counter based on confirmed temporal tracks.

    NOTE: This records cumulative session counts based on unique confirmed
    track IDs. It represents unique observations within the current tracking
    session, not an absolute ground-truth physical-world survey.
    """

    def __init__(self, counting_enabled: bool = True) -> None:
        """Initialize OreCounter.

        Args:
            counting_enabled: Master enable switch for counting logic (default: True).
        """
        self.counting_enabled: bool = counting_enabled
        self.counted_track_ids: Set[int] = set()
        self.total_count: int = 0
        self.class_counts: Dict[str, int] = {}

    def process_tracks(
        self,
        tracks: List[TrackedOre],
        header: Optional[Header] = None,
    ) -> OreCountSummary:
        """Process incoming tracked ores and return updated OreCountSummary.

        Args:
            tracks: List of TrackedOre messages from the current frame.
            header: Header preserving timestamp and frame_id from perception pipeline.

        Returns:
            OreCountSummary message with current cumulative and per-class counts.
        """
        summary = OreCountSummary()
        if header is not None:
            summary.header.stamp = header.stamp
            summary.header.frame_id = header.frame_id

        summary.active_track_count = len(tracks)

        if not self.counting_enabled:
            summary.total_count = self.total_count
            sorted_classes = sorted(self.class_counts.keys())
            summary.class_names = sorted_classes
            summary.class_counts = [self.class_counts[cls] for cls in sorted_classes]
            return summary

        # Process each track in the current frame
        for track in tracks:
            # Rule 1: Ignore TENTATIVE or non-CONFIRMED tracks (hysteresis confirmation)
            if track.state != "CONFIRMED":
                continue

            # Rule 2: Count each unique track ID exactly once
            if track.track_id in self.counted_track_ids:
                continue

            # New confirmed track!
            self.counted_track_ids.add(track.track_id)
            self.total_count += 1
            self.class_counts[track.class_name] = self.class_counts.get(track.class_name, 0) + 1

        # Populate summary output
        summary.total_count = self.total_count
        sorted_classes = sorted(self.class_counts.keys())
        summary.class_names = sorted_classes
        summary.class_counts = [self.class_counts[cls] for cls in sorted_classes]

        return summary

    def reset(self) -> None:
        """Reset all cumulative counts and session track IDs."""
        self.counted_track_ids.clear()
        self.total_count = 0
        self.class_counts.clear()

