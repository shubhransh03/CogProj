"""CogProj Live Terminal Demonstration Monitor.

Provides a real-time, zero-GUI text dashboard for headless SSH demonstration sessions.
Visibly exposes:
- Detected class, bounding box, confidence, and track ID
- Current tracked-object count vs confirmed count
- Distinct session count vs per-class breakdown
- Localized count on map vs unlocalized status
Distinguishes all four quantities without conflating them.
Displays zero detections honestly when real model finds no targets.
"""

import argparse
import os
import sys
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from nav_msgs.msg import OccupancyGrid

from cogproj_interfaces.msg import (
    DetectionArray,
    OreCountSummary,
    SpatialOre,
    SpatialOreArray,
    TrackedOre,
    TrackedOreArray,
)


class DemoStatusMonitor(Node):
    """Subscribes to perception topics and displays a clear terminal dashboard."""

    def __init__(self, detector_mode: str = "mock") -> None:
        super().__init__("cogproj_demo_status_monitor")
        self.detector_mode = detector_mode.lower()

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        summary_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # State storage
        self.last_img_time = 0.0
        self.img_count = 0
        self.img_info = "Waiting..."

        self.last_det_time = 0.0
        self.det_count = 0
        self.raw_detections: List[Any] = []
        self.last_inference_ms = 0.0

        self.last_trk_time = 0.0
        self.trk_count = 0
        self.tracked_ores: List[TrackedOre] = []

        self.last_cnt_time = 0.0
        self.counts_summary: Optional[OreCountSummary] = None

        self.last_spatial_time = 0.0
        self.spatial_ores: Optional[SpatialOreArray] = None

        # Subscribers
        self.img_sub = self.create_subscription(
            Image, "/cogproj/image_raw", self._on_image, sensor_qos
        )
        self.det_sub = self.create_subscription(
            DetectionArray, "/cogproj/detections", self._on_detections, sensor_qos
        )
        self.trk_sub = self.create_subscription(
            TrackedOreArray, "/cogproj/tracked_ores", self._on_tracking, sensor_qos
        )
        self.cnt_sub = self.create_subscription(
            OreCountSummary, "/cogproj/counts_summary", self._on_counting, summary_qos
        )
        self.spatial_sub = self.create_subscription(
            SpatialOreArray, "/cogproj/spatial_ores", self._on_spatial, sensor_qos
        )

    def _on_image(self, msg: Image) -> None:
        self.last_img_time = time.monotonic()
        self.img_count += 1
        self.img_info = f"{msg.width}x{msg.height} {msg.encoding}"

    def _on_detections(self, msg: DetectionArray) -> None:
        self.last_det_time = time.monotonic()
        self.det_count += 1
        self.raw_detections = list(msg.detections)
        self.last_inference_ms = msg.inference_time_ms

    def _on_tracking(self, msg: TrackedOreArray) -> None:
        self.last_trk_time = time.monotonic()
        self.trk_count += 1
        self.tracked_ores = list(msg.tracks)

    def _on_counting(self, msg: OreCountSummary) -> None:
        self.last_cnt_time = time.monotonic()
        self.counts_summary = msg

    def _on_spatial(self, msg: SpatialOreArray) -> None:
        self.last_spatial_time = time.monotonic()
        self.spatial_ores = msg

    def render_dashboard(self) -> str:
        """Construct the textual dashboard string."""
        now = time.monotonic()
        domain_id = os.environ.get("ROS_DOMAIN_ID", "155")

        mode_badge = (
            "[MOCK / SIMULATED PIPELINE]"
            if self.detector_mode == "mock"
            else "[REAL DETECTOR: YOLOv8s ONNX]"
        )

        # Status timeouts
        cam_active = (now - self.last_img_time) < 2.0 and self.last_img_time > 0
        det_active = (now - self.last_det_time) < 3.0 and self.last_det_time > 0
        trk_active = (now - self.last_trk_time) < 3.0 and self.last_trk_time > 0
        cnt_active = (now - self.last_cnt_time) < 10.0 and self.last_cnt_time > 0
        spatial_active = (now - self.last_spatial_time) < 3.0 and self.last_spatial_time > 0

        # Distinct quantities
        raw_det_count = len(self.raw_detections)
        total_active_tracks = len(self.tracked_ores)
        confirmed_tracks = sum(1 for t in self.tracked_ores if getattr(t, "state", "") == "CONFIRMED")
        tentative_tracks = sum(1 for t in self.tracked_ores if getattr(t, "state", "") == "TENTATIVE")
        lost_tracks = sum(1 for t in self.tracked_ores if getattr(t, "state", "") == "LOST")

        session_total_count = self.counts_summary.total_count if self.counts_summary else 0
        localized_map_count = self.spatial_ores.localized_count if self.spatial_ores else 0

        # Class breakdown
        class_summary = []
        if self.counts_summary and self.counts_summary.class_names:
            for name, c in zip(self.counts_summary.class_names, self.counts_summary.class_counts):
                class_summary.append(f"{name}: {c}")
        class_str = ", ".join(class_summary) if class_summary else "none"

        lines = [
            "=" * 78,
            f"  COGPROJ DEMONSTRATION DASHBOARD  |  ROS_DOMAIN_ID: {domain_id}",
            f"  Operational Mode: {mode_badge}",
            "=" * 78,
            "",
            "--- [1] FOUR DISTINCT PERCEPTION QUANTITIES (STRICTLY SEPARATE) ---",
            f"  • Current 2D Detections : {raw_det_count:3d}  (Raw bounding boxes in latest frame)",
            f"  • Active Tracks         : {total_active_tracks:3d}  (CONFIRMED: {confirmed_tracks}, TENTATIVE: {tentative_tracks}, LOST: {lost_tracks})",
            f"  • Distinct Session Count: {session_total_count:3d}  (Unique objects verified this session)",
            f"  • Mapped Localized Count: {localized_map_count:3d}  (Objects successfully localized in SLAM map)",
            f"  • Per-Class Breakdown   : {class_str}",
            "",
            "--- [2] PIPELINE STATUS & TIMINGS ---",
            f"  • Camera Feed  : {'ACTIVE (' + self.img_info + ')' if cam_active else 'WAITING [CAMERA UNAVAILABLE]'}",
            f"  • Ore Detector : {'ACTIVE (' + f'{self.last_inference_ms:.1f}ms' + ')' if det_active else 'WAITING [DETECTOR INITIALIZATION PENDING]'}",
            f"  • Track Engine : {'ACTIVE' if trk_active else 'WAITING'}",
            f"  • Session Count: {'ACTIVE' if cnt_active else 'WAITING'}",
            f"  • Spatial Map  : {'ACTIVE' if spatial_active else 'OFF / UNCALIBRATED'}",
            "",
            "--- [3] ACTIVE TRACKS TABLE ---",
            f"  {'ID':<5} {'Class':<12} {'Conf':<6} {'State':<11} {'BBox [x1, y1, x2, y2]':<26} {'Map Pos [X, Y, Z]':<18}",
            f"  {'-'*4} {'-'*11} {'-'*5} {'-'*10} {'-'*25} {'-'*18}",
        ]

        if not self.tracked_ores:
            lines.append("  (No active tracks currently tracked)")
        else:
            # Build map pos lookup from spatial ores if available
            spatial_lookup = {}
            if self.spatial_ores and self.spatial_ores.ores:
                for o in self.spatial_ores.ores:
                    if o.is_localized:
                        spatial_lookup[o.track_id] = f"({o.position_map.x:.2f}, {o.position_map.y:.2f})"
                    else:
                        spatial_lookup[o.track_id] = f"({o.localization_status[:12]})"

            for t in self.tracked_ores:
                bbox_str = f"[{int(t.x_min):3d}, {int(t.y_min):3d}, {int(t.x_max):3d}, {int(t.y_max):3d}]"
                map_pos_str = spatial_lookup.get(t.track_id, "(Unlocalized)")
                lines.append(
                    f"  #{t.track_id:<4} {t.class_name[:11]:<12} {t.confidence:<6.2f} {t.state:<11} {bbox_str:<26} {map_pos_str:<18}"
                )

        lines.extend([
            "",
            "--- [4] VIEWING WORKFLOW INSTRUCTIONS ---",
            "  • Desktop GUI Image Viewer (Run on workstation with ROS_DOMAIN_ID=155):",
            "      ros2 run rqt_image_view rqt_image_view /cogproj/image_annotated",
            "  • RViz Multi-Layer Display (Run on workstation):",
            "      rviz2 -d /home/veerobot/CogProj/src/cogproj_bringup/rviz/cogproj_slam_ore.rviz",
            "=" * 78,
        ])

        return "\n".join(lines)


def run_monitor(detector_mode: str = "mock", once: bool = False) -> None:
    """Run monitor loop or single-shot output."""
    rclpy.init()
    monitor = DemoStatusMonitor(detector_mode=detector_mode)

    try:
        if once:
            # Spin briefly and render
            start = time.monotonic()
            while time.monotonic() - start < 0.5:
                rclpy.spin_once(monitor, timeout_sec=0.05)
            print(monitor.render_dashboard())
            return

        print("\033[2J\033[H", end="")  # Clear terminal
        while rclpy.ok():
            rclpy.spin_once(monitor, timeout_sec=0.25)
            # Render every 0.5s
            dashboard_text = monitor.render_dashboard()
            print("\033[H" + dashboard_text, end="", flush=True)
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\n[CogProj Demo Monitor] Exiting gracefully...")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="CogProj Live Demonstration Status Monitor")
    parser.add_argument(
        "--mode",
        choices=["mock", "real"],
        default="mock",
        help="Detector operational mode being demonstrated (default: mock)",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Print single snapshot and exit (for headless test scripts)",
    )
    args = parser.parse_args()
    run_monitor(detector_mode=args.mode, once=args.once)


if __name__ == "__main__":
    main()

