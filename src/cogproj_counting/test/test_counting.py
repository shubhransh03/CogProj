"""Unit tests for cogproj_counting: confirmation hysteresis, deduplication, and synthetic scenarios."""

import unittest
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from cogproj_interfaces.msg import Detection, OreCountSummary, TrackedOre
from cogproj_tracking import MultiObjectTracker
from cogproj_counting import OreCounter


def make_tracked_ore(
    track_id: int,
    class_id: int = 1,
    class_name: str = "Hematite",
    state: str = "CONFIRMED",
    confidence: float = 0.85,
) -> TrackedOre:
    """Helper to instantiate TrackedOre messages for tests."""
    msg = TrackedOre()
    msg.track_id = track_id
    msg.class_id = class_id
    msg.class_name = class_name
    msg.confidence = confidence
    msg.state = state
    msg.x_min = 100.0
    msg.y_min = 100.0
    msg.x_max = 140.0
    msg.y_max = 140.0
    msg.center_x = 120.0
    msg.center_y = 120.0
    msg.bbox_width = 40.0
    msg.bbox_height = 40.0
    msg.age = 5
    msg.hits = 5
    msg.missed_frames = 0
    return msg


def make_detection(
    class_id: int,
    class_name: str,
    center_x: float,
    center_y: float,
    size: float = 30.0,
    confidence: float = 0.85,
) -> Detection:
    """Helper to instantiate Detection messages."""
    det = Detection()
    det.class_id = class_id
    det.class_name = class_name
    det.confidence = confidence
    det.x_min = center_x - size / 2.0
    det.y_min = center_y - size / 2.0
    det.x_max = center_x + size / 2.0
    det.y_max = center_y + size / 2.0
    det.center_x = center_x
    det.center_y = center_y
    det.bbox_width = size
    det.bbox_height = size
    return det


class TestOreCounter(unittest.TestCase):
    """Test suite for unique OreCounter logic."""

    def setUp(self):
        self.counter = OreCounter(counting_enabled=True)

    def test_16_tentative_track_is_not_counted(self):
        """16. TENTATIVE track is not counted."""
        t1 = make_tracked_ore(track_id=1, state="TENTATIVE")
        summary = self.counter.process_tracks([t1])
        self.assertEqual(summary.total_count, 0)
        self.assertEqual(len(summary.class_names), 0)
        self.assertEqual(len(summary.class_counts), 0)
        self.assertEqual(summary.active_track_count, 1)

    def test_17_confirmed_track_is_counted_once(self):
        """17. CONFIRMED track is counted once."""
        t1 = make_tracked_ore(track_id=1, state="CONFIRMED")
        summary = self.counter.process_tracks([t1])
        self.assertEqual(summary.total_count, 1)
        self.assertEqual(list(summary.class_names), ["Hematite"])
        self.assertEqual(list(summary.class_counts), [1])

    def test_18_same_confirmed_track_across_100_frames_counted_once(self):
        """18. Same confirmed track across 100 frames is counted only once."""
        t1 = make_tracked_ore(track_id=1, state="CONFIRMED")
        for _ in range(100):
            summary = self.counter.process_tracks([t1])
        self.assertEqual(summary.total_count, 1)
        self.assertEqual(list(summary.class_counts), [1])

    def test_19_two_confirmed_tracks_produce_count_two(self):
        """19. Two distinct confirmed tracks produce total_count = 2."""
        t1 = make_tracked_ore(track_id=1, class_name="Hematite", state="CONFIRMED")
        t2 = make_tracked_ore(track_id=2, class_name="Hematite", state="CONFIRMED")
        summary = self.counter.process_tracks([t1, t2])
        self.assertEqual(summary.total_count, 2)
        self.assertEqual(list(summary.class_counts), [2])

    def test_20_different_classes_maintain_correct_per_class_counts(self):
        """20. Different classes maintain accurate separate counts."""
        t1 = make_tracked_ore(track_id=1, class_name="Azurite", state="CONFIRMED")
        t2 = make_tracked_ore(track_id=2, class_name="Malachite", state="CONFIRMED")
        t3 = make_tracked_ore(track_id=3, class_name="Azurite", state="CONFIRMED")
        summary = self.counter.process_tracks([t1, t2, t3])
        self.assertEqual(summary.total_count, 3)
        self.assertEqual(list(summary.class_names), ["Azurite", "Malachite"])
        self.assertEqual(list(summary.class_counts), [2, 1])

    def test_21_lost_track_does_not_create_duplicate_count(self):
        """21. Track that becomes LOST does not increment count again."""
        t1_confirmed = make_tracked_ore(track_id=1, state="CONFIRMED")
        self.counter.process_tracks([t1_confirmed])
        self.assertEqual(self.counter.total_count, 1)

        # Later frame where track 1 is absent / lost
        summary = self.counter.process_tracks([])
        self.assertEqual(summary.total_count, 1)
        self.assertEqual(summary.active_track_count, 0)

    def test_22_genuinely_new_track_increments_count(self):
        """22. Genuinely new confirmed track increments cumulative count."""
        t1 = make_tracked_ore(track_id=1, state="CONFIRMED")
        self.counter.process_tracks([t1])
        self.assertEqual(self.counter.total_count, 1)

        t2 = make_tracked_ore(track_id=2, state="CONFIRMED")
        summary = self.counter.process_tracks([t2])
        self.assertEqual(summary.total_count, 2)

    def test_23_empty_frames_do_not_change_count(self):
        """23. Empty frames do not alter cumulative session counts."""
        t1 = make_tracked_ore(track_id=1, state="CONFIRMED")
        self.counter.process_tracks([t1])

        # 10 empty frames
        for _ in range(10):
            summary = self.counter.process_tracks([])
            self.assertEqual(summary.total_count, 1)

    def test_24_summary_arrays_remain_consistent(self):
        """24. Class names and class counts arrays always have identical length."""
        summary = self.counter.process_tracks([])
        self.assertEqual(len(summary.class_names), len(summary.class_counts))

        t1 = make_tracked_ore(track_id=1, class_name="Pyrite", state="CONFIRMED")
        summary = self.counter.process_tracks([t1])
        self.assertEqual(len(summary.class_names), len(summary.class_counts))
        self.assertEqual(len(summary.class_names), 1)

    def test_25_header_timestamp_preserved(self):
        """25. Header timestamp and frame_id are preserved in summary output."""
        header = Header()
        header.stamp = Time(sec=1791283200, nanosec=456789)
        header.frame_id = "camera_optical_link"

        t1 = make_tracked_ore(track_id=1, state="CONFIRMED")
        summary = self.counter.process_tracks([t1], header=header)
        self.assertEqual(summary.header.stamp.sec, 1791283200)
        self.assertEqual(summary.header.stamp.nanosec, 456789)
        self.assertEqual(summary.header.frame_id, "camera_optical_link")


class TestSyntheticScenarios(unittest.TestCase):
    """End-to-end multi-frame integration scenarios connecting Tracker and Counter."""

    def setUp(self):
        self.tracker = MultiObjectTracker(
            max_centroid_distance_px=50.0,
            min_iou=0.10,
            max_missed_frames=5,
            min_confirmed_hits=3,
        )
        self.counter = OreCounter(counting_enabled=True)

    def test_scenario_1_missed_detection_and_reconnection(self):
        """Scenario 1: Ore A moves slightly, misses 1 frame, reappears -> same track, count=1."""
        # Frame 1: Ore A at (100, 100) -> TENTATIVE
        f1 = [make_detection(1, "Hematite", 100.0, 100.0)]
        tracks = self.tracker.update(f1)
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].track_id, 1)
        self.assertEqual(tracks[0].state, "TENTATIVE")
        summary = self.counter.process_tracks([t.to_msg() for t in tracks])
        self.assertEqual(summary.total_count, 0)

        # Frame 2: Ore A at (104, 102) -> TENTATIVE (hits=2)
        f2 = [make_detection(1, "Hematite", 104.0, 102.0)]
        tracks = self.tracker.update(f2)
        self.assertEqual(tracks[0].track_id, 1)
        self.assertEqual(tracks[0].state, "TENTATIVE")
        summary = self.counter.process_tracks([t.to_msg() for t in tracks])
        self.assertEqual(summary.total_count, 0)

        # Frame 3: Ore A at (108, 104) -> CONFIRMED (hits=3) -> Counted!
        f3 = [make_detection(1, "Hematite", 108.0, 104.0)]
        tracks = self.tracker.update(f3)
        self.assertEqual(tracks[0].track_id, 1)
        self.assertEqual(tracks[0].state, "CONFIRMED")
        summary = self.counter.process_tracks([t.to_msg() for t in tracks])
        self.assertEqual(summary.total_count, 1)

        # Frame 4: Ore A missing (detector dropped frame)
        tracks = self.tracker.update([])
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].track_id, 1)
        self.assertEqual(tracks[0].missed_frames, 1)
        summary = self.counter.process_tracks([t.to_msg() for t in tracks])
        self.assertEqual(summary.total_count, 1)

        # Frame 5: Ore A at (112, 106) -> Reconnected! Same track_id
        f5 = [make_detection(1, "Hematite", 112.0, 106.0)]
        tracks = self.tracker.update(f5)
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].track_id, 1)
        self.assertEqual(tracks[0].state, "CONFIRMED")
        summary = self.counter.process_tracks([t.to_msg() for t in tracks])
        self.assertEqual(summary.total_count, 1)  # NOT counted again!

    def test_scenario_2_two_distinct_ores(self):
        """Scenario 2: Ore A and Ore B at distinct positions get separate IDs and counts."""
        # Frame 1: Ore A at (100, 100)
        tracks = self.tracker.update([make_detection(1, "Hematite", 100.0, 100.0)])
        self.assertEqual(len(tracks), 1)

        # Frame 2: Ore A at (104, 102), Ore B at (250, 180)
        tracks = self.tracker.update([
            make_detection(1, "Hematite", 104.0, 102.0),
            make_detection(2, "Malachite", 250.0, 180.0),
        ])
        self.assertEqual(len(tracks), 2)
        t_ids = {t.track_id for t in tracks}
        self.assertEqual(len(t_ids), 2)

        # Frame 3: Both ores present
        tracks = self.tracker.update([
            make_detection(1, "Hematite", 108.0, 104.0),
            make_detection(2, "Malachite", 252.0, 182.0),
        ])
        self.counter.process_tracks([t.to_msg() for t in tracks])

        # Frame 4: Both ores present (Ore B hits 3 hits)
        tracks = self.tracker.update([
            make_detection(1, "Hematite", 110.0, 105.0),
            make_detection(2, "Malachite", 254.0, 184.0),
        ])
        summary = self.counter.process_tracks([t.to_msg() for t in tracks])
        self.assertEqual(summary.total_count, 2)
        self.assertEqual(list(summary.class_names), ["Hematite", "Malachite"])
        self.assertEqual(list(summary.class_counts), [1, 1])

    def test_scenario_3_missed_reappear_second_ore_and_false_detection(self):
        """Scenario 3: Temporary miss + reappearance + second ore + 1-frame false detection noise."""
        # Step 1: Ore 1 confirmed over 3 frames
        for c in [(50.0, 50.0), (52.0, 52.0), (54.0, 54.0)]:
            tracks = self.tracker.update([make_detection(1, "OreAlpha", c[0], c[1])])
            self.counter.process_tracks([t.to_msg() for t in tracks])
        self.assertEqual(self.counter.total_count, 1)

        # Step 2: Ore 1 missing for 2 frames
        self.tracker.update([])
        self.tracker.update([])
        self.assertEqual(self.counter.total_count, 1)

        # Step 3: Frame with Ore 1 reappearing, plus a 1-frame false detection at (300, 200)
        tracks = self.tracker.update([
            make_detection(1, "OreAlpha", 56.0, 56.0),
            make_detection(2, "NoiseGhost", 300.0, 200.0, confidence=0.61),
        ])
        # Two tracks active: Ore 1 (CONFIRMED) and NoiseGhost (TENTATIVE)
        self.assertEqual(len(tracks), 2)
        summary = self.counter.process_tracks([t.to_msg() for t in tracks])
        # Total count must STILL be 1 because NoiseGhost is TENTATIVE
        self.assertEqual(summary.total_count, 1)

        # Step 4: Next frame: NoiseGhost is GONE. Ore 2 (OreBeta) appears at (200, 150)
        # Ore 1 continues
        for c1, c2 in [((58.0, 58.0), (200.0, 150.0)),
                       ((60.0, 60.0), (202.0, 152.0)),
                       ((62.0, 62.0), (204.0, 154.0))]:
            tracks = self.tracker.update([
                make_detection(1, "OreAlpha", c1[0], c1[1]),
                make_detection(3, "OreBeta", c2[0], c2[1]),
            ])
            summary = self.counter.process_tracks([t.to_msg() for t in tracks])

        # Expected outcome:
        # OreAlpha: count 1
        # OreBeta: count 1
        # NoiseGhost: count 0 (never confirmed)
        self.assertEqual(summary.total_count, 2)
        self.assertEqual(list(summary.class_names), ["OreAlpha", "OreBeta"])
        self.assertEqual(list(summary.class_counts), [1, 1])


if __name__ == "__main__":
    unittest.main()
