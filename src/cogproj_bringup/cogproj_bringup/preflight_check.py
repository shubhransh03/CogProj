"""CogProj Pre-Flight Environment and Topic Checker.

Audits hardware sensor streams, ROS 2 environment, CogProj perception topics,
SLAM prerequisites, and camera calibration state.
Distinguishes missing topics from topics that exist but have no active publishers.
Does NOT publish commands, does NOT alter running nodes, and does NOT command robot actuation.
"""

import argparse
import os
import platform
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros

from cogproj_interfaces.msg import (
    DetectionArray,
    OreCountSummary,
    SpatialOreArray,
    TrackedOreArray,
)


class PreflightChecker(Node):
    """Audits running topics, message payloads, TF frames, and calibration."""

    def __init__(self, mode: str = "mock", timeout_sec: float = 0.8) -> None:
        super().__init__("cogproj_preflight_checker")
        self.mode = mode.lower()
        self.timeout_sec = timeout_sec

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage for sampled messages
        self.samples: Dict[str, Any] = {}

    def audit_environment(self) -> Dict[str, str]:
        """Audit system OS, ROS 2, Domain ID, and RMW settings."""
        return {
            "Host": platform.node(),
            "Platform": f"{platform.system()} {platform.machine()} ({platform.release()})",
            "Python": platform.python_version(),
            "ROS_DISTRO": os.environ.get("ROS_DISTRO", "UNKNOWN"),
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0 (default)"),
            "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp (assumed)"),
            "Selected Mode": self.mode.upper(),
        }

    def audit_model_artifacts(self) -> Dict[str, Tuple[str, str]]:
        """Check availability of trained ONNX model and adapter."""
        results = {}
        onnx_path = "/home/veerobot/CogProj/Rover (2)/best.onnx"
        adapter_path = "/home/veerobot/CogProj/models/ore_yolo/adapter.py"
        config_path = "/home/veerobot/CogProj/models/ore_yolo/config.yaml"

        if os.path.exists(onnx_path):
            size_mb = os.path.getsize(onnx_path) / (1024 * 1024)
            results["ONNX Weights"] = ("READY", f"{size_mb:.1f} MB ({onnx_path})")
        else:
            results["ONNX Weights"] = ("MISSING", f"File not found: {onnx_path}")

        if os.path.exists(adapter_path) and os.path.exists(config_path):
            results["Model Adapter"] = ("READY", "adapter.py and config.yaml verified")
        else:
            results["Model Adapter"] = ("MISSING", "Plugin files missing in models/ore_yolo")

        return results

    def sample_active_topics(self) -> Dict[str, Dict[str, Any]]:
        """Query graph topics, count publishers, and sample payloads."""
        # Sensor data QoS profile
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        latch_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        topic_specs = [
            ("/camera/image_raw", Image, sensor_qos),
            ("/camera/camera_info", CameraInfo, sensor_qos),
            ("/scan", LaserScan, sensor_qos),
            ("/map", OccupancyGrid, latch_qos),
            ("/cogproj/image_raw", Image, sensor_qos),
            ("/cogproj/detections", DetectionArray, sensor_qos),
            ("/cogproj/tracked_ores", TrackedOreArray, sensor_qos),
            ("/cogproj/counts_summary", OreCountSummary, latch_qos),
            ("/cogproj/spatial_ores", SpatialOreArray, sensor_qos),
            ("/cogproj/image_annotated", Image, sensor_qos),
        ]

        # Register temporary subscribers
        subs = []
        for name, msg_type, qos in topic_specs:
            def make_cb(topic_name):
                return lambda msg: self.samples.setdefault(topic_name, msg)
            subs.append(self.create_subscription(msg_type, name, make_cb(name), qos))

        # Spin briefly to collect messages
        start = time.monotonic()
        while time.monotonic() - start < self.timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

        # Destroy temporary subs
        for s in subs:
            self.destroy_subscription(s)

        # Audit publishers count from ROS 2 graph
        topic_info_dict = {}
        for name, msg_type, _ in topic_specs:
            pub_info = self.get_publishers_info_by_topic(name)
            pub_count = len(pub_info)
            sample = self.samples.get(name)

            if pub_count == 0:
                status = "NOT_PUBLISHING"
                details = "0 publishers registered"
            elif sample is None:
                status = "IDLE"
                details = f"{pub_count} publisher(s), but no message received during timeout"
            else:
                status = "ACTIVE"
                details = self._format_sample_details(name, sample)

            topic_info_dict[name] = {
                "status": status,
                "pub_count": pub_count,
                "details": details,
                "sample": sample,
            }

        return topic_info_dict

    def _format_sample_details(self, topic: str, msg: Any) -> str:
        """Format detailed payload metrics for sampled messages."""
        if isinstance(msg, Image):
            return f"{msg.width}x{msg.height} {msg.encoding} (step={msg.step})"
        elif isinstance(msg, CameraInfo):
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]
            if fx > 0 and fy > 0:
                return f"CALIBRATED (fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f})"
            else:
                return "UNCALIBRATED (K matrix contains zero focal length)"
        elif isinstance(msg, LaserScan):
            valid_pts = sum(1 for r in msg.ranges if msg.range_min <= r <= msg.range_max)
            return f"{len(msg.ranges)} beams ({valid_pts} in-range, max={msg.range_max:.1f}m)"
        elif isinstance(msg, OccupancyGrid):
            return f"{msg.info.width}x{msg.info.height} map (res={msg.info.resolution:.2f}m)"
        elif isinstance(msg, DetectionArray):
            return f"{len(msg.detections)} detections (infer={msg.inference_time_ms:.1f}ms)"
        elif isinstance(msg, TrackedOreArray):
            return f"{len(msg.tracks)} active tracks"
        elif isinstance(msg, OreCountSummary):
            return f"Total count: {msg.total_count} (active={msg.active_track_count})"
        elif isinstance(msg, SpatialOreArray):
            return f"{msg.localized_count} localized, {msg.unlocalized_count} unlocalized"
        return "Message received"

    def audit_tf_transforms(self) -> Dict[str, Tuple[str, str]]:
        """Audit key transform linkages in the Beetle Bot kinematic tree."""
        pairs = [
            ("base_footprint", "base_link"),
            ("base_link", "lidar_link"),
            ("base_link", "camera_optical_link"),
            ("odom", "base_footprint"),
            ("map", "odom"),
        ]
        results = {}
        for target, source in pairs:
            link_name = f"{target} <- {source}"
            try:
                can_transform = self.tf_buffer.can_transform(
                    target, source, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
                )
                if can_transform:
                    results[link_name] = ("CONNECTED", "Valid TF frame transform exists")
                else:
                    results[link_name] = ("UNAVAILABLE", "TF lookup timed out")
            except Exception as e:
                results[link_name] = ("UNAVAILABLE", str(e))
        return results


def run_preflight(mode: str = "mock") -> Dict[str, Any]:
    """Execute preflight check and print formatted diagnostic summary."""
    rclpy.init()
    checker = PreflightChecker(mode=mode)

    try:
        env = checker.audit_environment()
        models = checker.audit_model_artifacts()
        topics = checker.sample_active_topics()
        tfs = checker.audit_tf_transforms()

        # Print Report
        print("\n" + "=" * 78)
        print("          COGPROJ DEMONSTRATION PRE-FLIGHT DIAGNOSTIC REPORT")
        print("=" * 78)

        print("\n[1] SYSTEM & ROS ENVIRONMENT:")
        for k, v in env.items():
            print(f"  • {k:20s}: {v}")

        print("\n[2] DETECTION MODEL & PLUGIN READINESS:")
        for k, (st, desc) in models.items():
            badge = f"[{st}]"
            print(f"  • {k:20s}: {badge:12s} {desc}")

        print("\n[3] SENSOR INPUT TOPICS:")
        sensor_topics = ["/camera/image_raw", "/camera/camera_info", "/scan"]
        for st_name in sensor_topics:
            info = topics.get(st_name, {})
            badge = f"[{info.get('status', 'UNKNOWN')}]"
            print(f"  • {st_name:20s}: {badge:16s} {info.get('details', '')}")

        print("\n[4] SLAM & SPATIAL KINEMATICS:")
        map_info = topics.get("/map", {})
        map_badge = f"[{map_info.get('status', 'UNKNOWN')}]"
        print(f"  • {'/map (Occupancy)':20s}: {map_badge:16s} {map_info.get('details', '')}")
        for link, (st, desc) in tfs.items():
            badge = f"[{st}]"
            print(f"  • {link:20s}: {badge:16s} {desc}")

        print("\n[5] COGPROJ PIPELINE TOPICS:")
        cogproj_topics = [
            "/cogproj/image_raw",
            "/cogproj/detections",
            "/cogproj/tracked_ores",
            "/cogproj/counts_summary",
            "/cogproj/spatial_ores",
            "/cogproj/image_annotated",
        ]
        for cp_name in cogproj_topics:
            info = topics.get(cp_name, {})
            badge = f"[{info.get('status', 'UNKNOWN')}]"
            print(f"  • {cp_name:25s}: {badge:16s} {info.get('details', '')}")

        print("\n" + "-" * 78)
        print("DIAGNOSTIC SUMMARY & ACTIONABLE RECOMMENDATIONS:")

        # Actionable rules
        cam_info = topics.get("/camera/image_raw", {})
        cam_calib = topics.get("/camera/camera_info", {})
        scan_info = topics.get("/scan", {})
        map_topic_info = topics.get("/map", {})

        if cam_info.get("status") == "NOT_PUBLISHING":
            print("  [!] CAMERA UNAVAILABLE: /camera/image_raw is not publishing.")
            print("      Action: If running live hardware, verify external camera node.")
        elif cam_info.get("status") == "ACTIVE":
            print("  [✓] Camera stream is active and communicating.")

        if "UNCALIBRATED" in cam_calib.get("details", ""):
            print("  [!] CAMERA NOT CALIBRATED: K matrix focal lengths are zero.")
            print("      Action: 3D spatial localization will output (0,0,0) and status UNCALIBRATED_CAMERA.")
            print("              Run checkerboard camera calibration for physical 3D localization.")
        elif cam_calib.get("status") == "ACTIVE":
            print("  [✓] Camera calibration matrix is valid.")

        if map_topic_info.get("status") == "NOT_PUBLISHING":
            print("  [i] SLAM MAP UNAVAILABLE: /map is not currently published.")
            print("      Action: Use start_slam:=true or run lyra_slam if SLAM mapping is needed.")

        if scan_info.get("status") == "NOT_PUBLISHING":
            print("  [i] LIDAR ASSOCIATION UNAVAILABLE: /scan is not currently published.")
            print("      Action: Start lidar.launch.py from lyra_bringup if LiDAR projection is needed.")

        if mode == "mock":
            print("  [✓] MODE A (MOCK DEMONSTRATION): Ready to launch via demo_showcase.launch.py.")
            print("      Annotated feed will display explicit [MOCK / SIMULATED] badges.")
        else:
            if models.get("ONNX Weights", ("MISSING", ""))[0] == "READY":
                print("  [✓] MODE B (REAL DETECTOR): Ready to launch via demo_showcase.launch.py detector_mode:=real.")
            else:
                print("  [!] MODE B BLOCKED: ONNX model weights missing or inaccessible.")

        print("=" * 78 + "\n")

        return {
            "environment": env,
            "models": models,
            "topics": topics,
            "tfs": tfs,
        }

    finally:
        checker.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="CogProj Demonstration Pre-Flight Diagnostic Checker")
    parser.add_argument(
        "--mode",
        choices=["mock", "real"],
        default="mock",
        help="Demonstration mode to validate prerequisites for (default: mock)",
    )
    args = parser.parse_args()
    run_preflight(mode=args.mode)


if __name__ == "__main__":
    main()

