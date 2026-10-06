"""Unit tests for cogproj_tracking: track lifecycle, Hungarian association, and thresholds."""

import unittest
from cogproj_interfaces.msg import Detection, DetectionArray
from cogproj_tracking import MultiObjectTracker, Track, compute_centroid_distance, compute_iou


def make_detection(
    class_id: int = 1,
    class_name: str = "Hematite",
    confidence: float = 0.85,
    x_min: float = 100.0,
    y_min: float = 100.0,
    x_max: float = 140.0,
    y_max: float = 140.0,
) -> Detection:
    """Helper to instantiate Detection messages."""
    det = Detection()
    det.class_id = class_id
    det.class_name = class_name
    det.confidence = confidence
    det.x_min = x_min
    det.y_min = y_min
    det.x_max = x_max
    det.y_max = y_max
    det.bbox_width = x_max - x_min
    det.bbox_height = y_max - y_min
    det.center_x = (x_min + x_max) / 2.0
    det.center_y = (y_min + y_max) / 2.0
    return det


class TestMultiObjectTracking(unittest.TestCase):
    """Test suite for deterministic MultiObjectTracker."""

    def setUp(self):
        self.tracker = MultiObjectTracker(
            max_centroid_distance_px=50.0,
            min_iou=0.10,
            max_missed_frames=5,
            min_confirmed_hits=3,
        )

    def test_01_empty_detection_frame(self):
        """1. Empty detection frame produces zero active tracks."""
        tracks = self.tracker.update([])
        self.assertEqual(len(tracks), 0)

    def test_02_one_detection_creates_one_track(self):
        """2. One detection creates one track with TENTATIVE state."""
        det = make_detection()
        tracks = self.tracker.update([det])
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].track_id, 1)
        self.assertEqual(tracks[0].state, Track.STATE_TENTATIVE)
        self.assertEqual(tracks[0].hits, 1)
        self.assertEqual(tracks[0].age, 1)
        self.assertEqual(tracks[0].missed_frames, 0)

    def test_03_same_detection_across_frames_keeps_same_track_id(self):
        """3. Same detection across multiple frames keeps same track_id."""
        det1 = make_detection(x_min=100.0, y_min=100.0, x_max=140.0, y_max=140.0)
        tracks1 = self.tracker.update([det1])
        t_id1 = tracks1[0].track_id

        det2 = make_detection(x_min=102.0, y_min=101.0, x_max=142.0, y_max=141.0)
        tracks2 = self.tracker.update([det2])
        self.assertEqual(len(tracks2), 1)
        self.assertEqual(tracks2[0].track_id, t_id1)

        det3 = make_detection(x_min=104.0, y_min=102.0, x_max=144.0, y_max=142.0)
        tracks3 = self.tracker.update([det3])
        self.assertEqual(len(tracks3), 1)
        self.assertEqual(tracks3[0].track_id, t_id1)

    def test_04_track_becomes_confirmed_after_required_hits(self):
        """4. Track becomes CONFIRMED after required hits (min_confirmed_hits=3)."""
        det1 = make_detection(x_min=100.0, y_min=100.0, x_max=140.0, y_max=140.0)
        tracks = self.tracker.update([det1])
        self.assertEqual(tracks[0].state, Track.STATE_TENTATIVE)

        det2 = make_detection(x_min=102.0, y_min=102.0, x_max=142.0, y_max=142.0)
        tracks = self.tracker.update([det2])
        self.assertEqual(tracks[0].state, Track.STATE_TENTATIVE)

        det3 = make_detection(x_min=104.0, y_min=104.0, x_max=144.0, y_max=144.0)
        tracks = self.tracker.update([det3])
        self.assertEqual(tracks[0].state, Track.STATE_CONFIRMED)
        self.assertEqual(tracks[0].hits, 3)

    def test_05_temporary_missed_detection_does_not_immediately_delete_track(self):
        """5. Temporary missed detection increments missed_frames but keeps track active."""
        det = make_detection()
        self.tracker.update([det])
        self.assertEqual(len(self.tracker.active_tracks), 1)

        # Frame with missing detection
        tracks = self.tracker.update([])
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].missed_frames, 1)
        self.assertEqual(tracks[0].age, 2)

    def test_06_track_removed_after_max_missed_frames(self):
        """6. Track is removed after max_missed_frames (5 frames)."""
        det = make_detection()
        self.tracker.update([det])

        # Miss 5 frames -> still active with missed_frames=5
        for i in range(1, 6):
            tracks = self.tracker.update([])
            self.assertEqual(len(tracks), 1)
            self.assertEqual(tracks[0].missed_frames, i)

        # 6th miss -> exceeds max_missed_frames (5) -> marked LOST and pruned
        tracks = self.tracker.update([])
        self.assertEqual(len(tracks), 0)

    def test_07_detection_reappearing_within_distance_reconnects(self):
        """7. Detection reappearing within association distance reconnects to existing track."""
        det1 = make_detection(x_min=100.0, y_min=100.0, x_max=140.0, y_max=140.0)
        tracks = self.tracker.update([det1])
        orig_id = tracks[0].track_id

        # Miss 2 frames
        self.tracker.update([])
        self.tracker.update([])

        # Reappears nearby (shift 10 pixels, within 50px max_dist)
        det_reappear = make_detection(x_min=110.0, y_min=110.0, x_max=150.0, y_max=150.0)
        tracks_reconnect = self.tracker.update([det_reappear])
        self.assertEqual(len(tracks_reconnect), 1)
        self.assertEqual(tracks_reconnect[0].track_id, orig_id)
        self.assertEqual(tracks_reconnect[0].missed_frames, 0)
        self.assertEqual(tracks_reconnect[0].hits, 2)

    def test_08_far_away_detection_creates_new_track(self):
        """8. Detection farther than max_centroid_distance_px creates a new track."""
        det1 = make_detection(x_min=10.0, y_min=10.0, x_max=40.0, y_max=40.0)
        tracks1 = self.tracker.update([det1])
        id1 = tracks1[0].track_id

        # Det2 is far away (center around 250, 200 vs 25, 25: distance > 200px)
        det2 = make_detection(x_min=230.0, y_min=180.0, x_max=270.0, y_max=220.0)
        tracks2 = self.tracker.update([det2])
        # Old track missed, new track created -> 2 active tracks
        self.assertEqual(len(tracks2), 2)
        track_ids = [t.track_id for t in tracks2]
        self.assertIn(id1, track_ids)
        new_id = [tid for tid in track_ids if tid != id1][0]
        self.assertNotEqual(new_id, id1)

    def test_09_two_simultaneous_detections_produce_two_different_track_ids(self):
        """9. Two simultaneous detections produce two different track IDs."""
        det1 = make_detection(x_min=50.0, y_min=50.0, x_max=90.0, y_max=90.0)
        det2 = make_detection(x_min=200.0, y_min=150.0, x_max=240.0, y_max=190.0)
        tracks = self.tracker.update([det1, det2])
        self.assertEqual(len(tracks), 2)
        self.assertNotEqual(tracks[0].track_id, tracks[1].track_id)

    def test_10_class_mismatch_does_not_associate(self):
        """10. Class mismatch prevents association even at identical bounding box coordinates."""
        det1 = make_detection(class_id=1, class_name="Hematite", x_min=100.0, y_min=100.0, x_max=140.0, y_max=140.0)
        tracks1 = self.tracker.update([det1])
        id1 = tracks1[0].track_id

        # Exact same box, different class
        det2 = make_detection(class_id=2, class_name="Malachite", x_min=100.0, y_min=100.0, x_max=140.0, y_max=140.0)
        tracks2 = self.tracker.update([det2])
        self.assertEqual(len(tracks2), 2)
        # Track 1 is missed, Track 2 is new
        t1 = [t for t in tracks2 if t.track_id == id1][0]
        t2 = [t for t in tracks2 if t.track_id != id1][0]
        self.assertEqual(t1.missed_frames, 1)
        self.assertEqual(t2.class_name, "Malachite")

    def test_11_iou_and_distance_helpers(self):
        """11. IoU and centroid distance calculation verification."""
        bbox_a = (0.0, 0.0, 10.0, 10.0)
        bbox_b = (5.0, 0.0, 15.0, 10.0)
        # Inter: 5 * 10 = 50. Union: 100 + 100 - 50 = 150. IoU = 50 / 150 = 0.3333
        iou = compute_iou(bbox_a, bbox_b)
        self.assertAlmostEqual(iou, 1.0 / 3.0, places=4)

        # Disjoint boxes
        bbox_c = (20.0, 20.0, 30.0, 30.0)
        self.assertEqual(compute_iou(bbox_a, bbox_c), 0.0)

        dist = compute_centroid_distance((0.0, 0.0), (3.0, 4.0))
        self.assertEqual(dist, 5.0)

    def test_12_track_age_increments_correctly(self):
        """12. Track age increments on both hits and misses."""
        det = make_detection()
        t = self.tracker.update([det])[0]
        self.assertEqual(t.age, 1)

        # Hit frame
        t = self.tracker.update([det])[0]
        self.assertEqual(t.age, 2)

        # Miss frame
        t = self.tracker.update([])[0]
        self.assertEqual(t.age, 3)

    def test_13_hit_count_increments_correctly(self):
        """13. Hit count increments on hit, does not increment on miss."""
        det = make_detection()
        t = self.tracker.update([det])[0]
        self.assertEqual(t.hits, 1)

        t = self.tracker.update([det])[0]
        self.assertEqual(t.hits, 2)

        t = self.tracker.update([])[0]
        self.assertEqual(t.hits, 2)

    def test_14_missed_frame_count_behaves_correctly(self):
        """14. Missed frame count increments on miss and resets on match."""
        det = make_detection()
        t = self.tracker.update([det])[0]
        self.assertEqual(t.missed_frames, 0)

        t = self.tracker.update([])[0]
        self.assertEqual(t.missed_frames, 1)

        t = self.tracker.update([])[0]
        self.assertEqual(t.missed_frames, 2)

        # Match resets missed_frames to 0
        t = self.tracker.update([det])[0]
        self.assertEqual(t.missed_frames, 0)

    def test_15_track_state_transitions(self):
        """15. Track state transitions from TENTATIVE to CONFIRMED, and eventually LOST."""
        det = make_detection()
        self.tracker.update([det])
        self.assertEqual(self.tracker.tracks[0].state, Track.STATE_TENTATIVE)

        self.tracker.update([det])
        self.assertEqual(self.tracker.tracks[0].state, Track.STATE_TENTATIVE)

        self.tracker.update([det])
        self.assertEqual(self.tracker.tracks[0].state, Track.STATE_CONFIRMED)

        # Let track expire
        for _ in range(6):
            self.tracker.update([])

        self.assertEqual(len(self.tracker.active_tracks), 0)


if __name__ == "__main__":
    unittest.main()

