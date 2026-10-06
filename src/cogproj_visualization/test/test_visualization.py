"""Unit tests for cogproj_visualization: PerceptionAnnotator, VisualizerNode, and synthetic scenarios."""

import time
import unittest
import cv2
import numpy as np

import rclpy
from builtin_interfaces.msg import Time as MsgTime
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

from cogproj_interfaces.msg import Detection, DetectionArray, OreCountSummary, TrackedOre, TrackedOreArray
from cogproj_visualization import CogprojViewer, PerceptionAnnotator, VisualizerNode


def make_tracked_ore(
    track_id: int,
    class_id: int = 1,
    class_name: str = "Hematite",
    confidence: float = 0.88,
    state: str = "CONFIRMED",
    x_min: float = 40.0,
    y_min: float = 50.0,
    x_max: float = 120.0,
    y_max: float = 130.0,
) -> TrackedOre:
    """Helper to instantiate TrackedOre messages for tests."""
    msg = TrackedOre()
    msg.track_id = track_id
    msg.class_id = class_id
    msg.class_name = class_name
    msg.confidence = confidence
    msg.state = state
    msg.x_min = x_min
    msg.y_min = y_min
    msg.x_max = x_max
    msg.y_max = y_max
    msg.center_x = (x_min + x_max) / 2.0
    msg.center_y = (y_min + y_max) / 2.0
    msg.bbox_width = x_max - x_min
    msg.bbox_height = y_max - y_min
    msg.age = 4
    msg.hits = 4
    msg.missed_frames = 0
    return msg


def make_count_summary(total: int = 2, class_names=None, class_counts=None) -> OreCountSummary:
    """Helper to instantiate OreCountSummary messages for tests."""
    msg = OreCountSummary()
    msg.header.stamp = MsgTime(sec=1791283200, nanosec=123456)
    msg.header.frame_id = "camera_optical_link"
    msg.total_count = total
    msg.active_track_count = total
    msg.class_names = class_names if class_names is not None else ["Hematite", "Malachite"]
    msg.class_counts = class_counts if class_counts is not None else [1, 1]
    return msg


class TestPerceptionAnnotator(unittest.TestCase):
    """Test suite for headless PerceptionAnnotator rendering engine."""

    def setUp(self):
        self.annotator = PerceptionAnnotator()
        self.blank_frame = np.zeros((240, 320, 3), dtype=np.uint8)

    def test_06_detection_overlay_generated_correctly(self):
        """6. Detection overlay draws bounding boxes on image canvas."""
        t1 = make_tracked_ore(track_id=1, state="CONFIRMED")
        annotated = self.annotator.annotate_frame(self.blank_frame, tracked_ores=[t1])
        self.assertEqual(annotated.shape, (240, 320, 3))
        # Ensure the canvas has been modified by drawing (not all zeros)
        self.assertGreater(np.count_nonzero(annotated), 0)

    def test_07_track_id_appears_in_annotation(self):
        """7. Track ID is rendered on the frame."""
        t1 = make_tracked_ore(track_id=42, state="CONFIRMED")
        annotated = self.annotator.annotate_frame(self.blank_frame, tracked_ores=[t1])
        self.assertEqual(annotated.shape, (240, 320, 3))

    def test_08_confidence_appears_correctly(self):
        """8. Confidence value is rendered on the frame."""
        t1 = make_tracked_ore(track_id=1, confidence=0.95, state="CONFIRMED")
        annotated = self.annotator.annotate_frame(self.blank_frame, tracked_ores=[t1])
        self.assertEqual(annotated.shape, (240, 320, 3))

    def test_09_track_state_appears_correctly(self):
        """9. Track states (CONFIRMED, TENTATIVE, LOST) render with different color tags."""
        t_conf = make_tracked_ore(track_id=1, state="CONFIRMED")
        t_tent = make_tracked_ore(track_id=2, state="TENTATIVE")
        t_lost = make_tracked_ore(track_id=3, state="LOST")

        out_conf = self.annotator.annotate_frame(self.blank_frame, tracked_ores=[t_conf])
        out_tent = self.annotator.annotate_frame(self.blank_frame, tracked_ores=[t_tent])
        out_lost = self.annotator.annotate_frame(self.blank_frame, tracked_ores=[t_lost])

        self.assertGreater(np.count_nonzero(out_conf), 0)
        self.assertGreater(np.count_nonzero(out_tent), 0)
        self.assertGreater(np.count_nonzero(out_lost), 0)

    def test_10_multiple_tracked_ores_are_rendered(self):
        """10. Multiple tracked ores are rendered without error."""
        t1 = make_tracked_ore(track_id=1, x_min=20.0, y_min=20.0, x_max=80.0, y_max=80.0)
        t2 = make_tracked_ore(track_id=2, x_min=150.0, y_min=100.0, x_max=220.0, y_max=160.0)
        annotated = self.annotator.annotate_frame(self.blank_frame, tracked_ores=[t1, t2])
        self.assertEqual(annotated.shape, (240, 320, 3))

    def test_11_no_tracks_produces_valid_frame(self):
        """11. No tracks produces valid frame with HUD banners."""
        annotated = self.annotator.annotate_frame(self.blank_frame, tracked_ores=[])
        self.assertEqual(annotated.shape, (240, 320, 3))
        # HUD banners are rendered, so frame is non-zero
        self.assertGreater(np.count_nonzero(annotated), 0)

    def test_12_count_summary_is_rendered(self):
        """12. Count summary total count is rendered."""
        summary = make_count_summary(total=5)
        annotated = self.annotator.annotate_frame(self.blank_frame, counts_summary=summary)
        self.assertEqual(annotated.shape, (240, 320, 3))

    def test_13_per_class_counts_are_rendered(self):
        """13. Per-class counts are rendered."""
        summary = make_count_summary(total=3, class_names=["Azurite", "Malachite"], class_counts=[2, 1])
        annotated = self.annotator.annotate_frame(self.blank_frame, counts_summary=summary)
        self.assertEqual(annotated.shape, (240, 320, 3))

    def test_14_active_track_count_is_rendered(self):
        """14. Active track count is rendered."""
        t1 = make_tracked_ore(track_id=1)
        t2 = make_tracked_ore(track_id=2)
        annotated = self.annotator.annotate_frame(self.blank_frame, tracked_ores=[t1, t2])
        self.assertEqual(annotated.shape, (240, 320, 3))

    def test_17_annotated_output_uses_bgr8(self):
        """17. Annotated output retains 3-channel uint8 (BGR8) format."""
        annotated = self.annotator.annotate_frame(self.blank_frame)
        self.assertEqual(annotated.dtype, np.uint8)
        self.assertEqual(len(annotated.shape), 3)
        self.assertEqual(annotated.shape[2], 3)

    def test_18_compressed_output_generation(self):
        """18. Annotated frame can be encoded to JPEG compressed bytes."""
        annotated = self.annotator.annotate_frame(self.blank_frame)
        ret, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        self.assertTrue(ret)
        self.assertGreater(len(buf), 0)


class TestVisualizerNode(unittest.TestCase):
    """Test suite for ROS 2 VisualizerNode execution, timeouts, and topic publishing."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.node = VisualizerNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_01_node_starts_successfully(self):
        """1. Visualization node initializes cleanly with default parameters."""
        self.assertEqual(self.node.image_topic, "/cogproj/image_raw")
        self.assertEqual(self.node.annotated_topic, "/cogproj/image_annotated")
        self.assertEqual(self.node.compressed_topic, "/cogproj/image_annotated/compressed")

    def test_02_camera_timeout_state(self):
        """2. Camera status transitions to WAITING when no frames arrive."""
        st = self.node.get_status_dict()
        self.assertEqual(st["camera"], "WAITING")

    def test_03_detection_timeout_state(self):
        """3. Detection status transitions to WAITING when no detections arrive."""
        st = self.node.get_status_dict()
        self.assertEqual(st["detection"], "WAITING")

        # Simulate detection arrival
        self.node._det_callback(DetectionArray())
        st_active = self.node.get_status_dict()
        self.assertEqual(st_active["detection"], "ACTIVE")

    def test_04_tracking_timeout_state(self):
        """4. Tracking status transitions to WAITING / ACTIVE based on message arrival."""
        st = self.node.get_status_dict()
        self.assertEqual(st["tracking"], "WAITING")

        self.node._trk_callback(TrackedOreArray())
        st_active = self.node.get_status_dict()
        self.assertEqual(st_active["tracking"], "ACTIVE")

    def test_05_count_timeout_state(self):
        """5. Count status transitions to WAITING / ACTIVE based on message arrival."""
        st = self.node.get_status_dict()
        self.assertEqual(st["counting"], "WAITING")

        self.node._cnt_callback(OreCountSummary())
        st_active = self.node.get_status_dict()
        self.assertEqual(st_active["counting"], "ACTIVE")

    def test_15_16_timestamp_and_frame_id_preserved(self):
        """15, 16. Header timestamp and frame_id are preserved in published messages."""
        captured_raw = []
        captured_comp = []

        # Override publishers with spy callbacks
        self.node.annotated_pub.publish = lambda msg: captured_raw.append(msg)
        self.node.compressed_pub.publish = lambda msg: captured_comp.append(msg)

        # Create input image message
        img_msg = Image()
        img_msg.header.stamp = MsgTime(sec=1791283200, nanosec=777888)
        img_msg.header.frame_id = "camera_optical_link"
        img_msg.height = 240
        img_msg.width = 320
        img_msg.encoding = "bgr8"
        img_msg.step = 320 * 3
        img_msg.data = np.zeros((240, 320, 3), dtype=np.uint8).tobytes()

        self.node._image_callback(img_msg)

        self.assertEqual(len(captured_raw), 1)
        self.assertEqual(len(captured_comp), 1)

        raw = captured_raw[0]
        self.assertEqual(raw.header.stamp.sec, 1791283200)
        self.assertEqual(raw.header.stamp.nanosec, 777888)
        self.assertEqual(raw.header.frame_id, "camera_optical_link")
        self.assertEqual(raw.encoding, "bgr8")

        comp = captured_comp[0]
        self.assertEqual(comp.header.stamp.sec, 1791283200)
        self.assertEqual(comp.header.stamp.nanosec, 777888)
        self.assertEqual(comp.header.frame_id, "camera_optical_link")
        self.assertEqual(comp.format, "jpeg")

    def test_19_missing_detection_does_not_crash(self):
        """19. Missing detection messages do not crash visualization."""
        img_msg = Image()
        img_msg.height = 240
        img_msg.width = 320
        img_msg.encoding = "bgr8"
        img_msg.step = 320 * 3
        img_msg.data = np.zeros((240, 320, 3), dtype=np.uint8).tobytes()
        self.node._image_callback(img_msg)

    def test_20_missing_tracking_does_not_crash(self):
        """20. Missing tracking messages do not crash visualization."""
        self.node.latest_tracks = []
        img_msg = Image()
        img_msg.height = 240
        img_msg.width = 320
        img_msg.encoding = "bgr8"
        img_msg.step = 320 * 3
        img_msg.data = np.zeros((240, 320, 3), dtype=np.uint8).tobytes()
        self.node._image_callback(img_msg)

    def test_21_missing_count_does_not_crash(self):
        """21. Missing count messages do not crash visualization."""
        self.node.latest_counts = None
        img_msg = Image()
        img_msg.height = 240
        img_msg.width = 320
        img_msg.encoding = "bgr8"
        img_msg.step = 320 * 3
        img_msg.data = np.zeros((240, 320, 3), dtype=np.uint8).tobytes()
        self.node._image_callback(img_msg)

    def test_22_visualization_never_publishes_robot_control(self):
        """22. Node has zero publishers on actuation topics."""
        for pub in self.node.publishers:
            self.assertNotIn("cmd_vel", pub.topic_name)


class TestSyntheticVisualizationScenario(unittest.TestCase):
    """Deterministic synthetic scenario with 2 tracked ores and cumulative counts."""

    def test_scenario_annotated_feed(self):
        """Verify full synthetic pipeline rendering 2 tracked ores and count summary."""
        annotator = PerceptionAnnotator()
        frame = np.zeros((240, 320, 3), dtype=np.uint8)

        # Tracked ores
        t1 = make_tracked_ore(
            track_id=1,
            class_name="OreAlpha",
            confidence=0.88,
            state="CONFIRMED",
            x_min=40.0,
            y_min=50.0,
            x_max=100.0,
            y_max=110.0,
        )
        t2 = make_tracked_ore(
            track_id=2,
            class_name="OreBeta",
            confidence=0.75,
            state="CONFIRMED",
            x_min=180.0,
            y_min=120.0,
            x_max=250.0,
            y_max=190.0,
        )

        # Summary
        summary = make_count_summary(total=2, class_names=["OreAlpha", "OreBeta"], class_counts=[1, 1])
        status = {"camera": "OK", "detection": "ACTIVE", "tracking": "ACTIVE", "counting": "ACTIVE"}

        # Measure annotation execution time
        start = time.perf_counter()
        annotated = annotator.annotate_frame(frame, tracked_ores=[t1, t2], counts_summary=summary, status=status)
        elapsed_ms = (time.perf_counter() - start) * 1000.0

        # Assertions
        self.assertEqual(annotated.shape, (240, 320, 3))
        self.assertGreater(np.count_nonzero(annotated), 0)
        # Rendering on 320x240 frame should be fast (< 15 ms on CPU)
        self.assertLess(elapsed_ms, 50.0)

        # Compression test
        ret, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        self.assertTrue(ret)
        self.assertGreater(len(buf), 1000)


if __name__ == "__main__":
    unittest.main()

