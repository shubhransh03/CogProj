"""Dashboard Gateway Node for CogProj Operator Console.

Coordinates ROS 2 sensor subscriptions, camera streaming, map updates,
TF kinematics, Nav2 autonomous action dispatching, and safe teleoperation
supervised by ControlSupervisor.
"""

import base64
import math
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, CompressedImage, Joy, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import tf2_ros

from nav2_msgs.action import NavigateToPose
from cogproj_interfaces.msg import (
    DetectionArray,
    OreCountSummary,
    SpatialOreArray,
    TrackedOreArray,
)

from .supervisor import ControlMode, ControlSupervisor
from .web_server import DashboardWebSocketHandler, WebServerThread


class DashboardGatewayNode(Node):
    """Bridge between ROS 2 perception/control graphs and browser dashboard."""

    def __init__(self, **kwargs) -> None:
        super().__init__("cogproj_dashboard_node", **kwargs)

        # Declare parameters
        self.declare_parameter("web_host", "127.0.0.1")
        self.declare_parameter("web_port", 8080)
        self.declare_parameter("max_linear_x", 0.35)
        self.declare_parameter("max_angular_z", 0.80)
        self.declare_parameter("command_timeout_sec", 0.30)
        self.declare_parameter("target_cmd_vel_topic", "/cmd_vel_joy")
        self.declare_parameter("enable_web_server", True)

        web_host = self.get_parameter("web_host").value
        web_port = self.get_parameter("web_port").value
        max_linear = self.get_parameter("max_linear_x").value
        max_angular = self.get_parameter("max_angular_z").value
        cmd_timeout = self.get_parameter("command_timeout_sec").value
        self.target_topic = self.get_parameter("target_cmd_vel_topic").value
        self.enable_web_server = self.get_parameter("enable_web_server").value

        # Motion Safety Supervisor
        self.supervisor = ControlSupervisor(
            max_linear_x=max_linear,
            max_angular_z=max_angular,
            command_timeout_sec=cmd_timeout,
            on_velocity_output=self._publish_velocity,
            on_mode_changed=self._handle_mode_changed,
            on_stop_requested=self._handle_stop_requested,
        )

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS Profiles
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

        # Teleop publisher to documented pre-gate topic
        self.cmd_vel_pub = self.create_publisher(Twist, self.target_topic, 10)
        # Joystick deadman publisher for lyra_cmd_vel_gate (deadman = button index 4)
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)

        # Hardware arm/disarm/estop service clients
        self.arm_client = self.create_client(Trigger, "/lyra/arm")
        self.disarm_client = self.create_client(Trigger, "/lyra/disarm")
        self.estop_client = self.create_client(Trigger, "/lyra/emergency_stop")

        # Nav2 Action Client
        self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._current_nav_goal_handle: Any = None
        self._nav_state: str = "IDLE"

        # Cached telemetry state
        self._camera_fps = 0.0
        self._camera_frame_count = 0
        self._last_fps_calc = time.monotonic()
        self._last_image_time = 0.0
        self._latest_image_b64: Optional[str] = None
        self._is_annotated = False

        self._detector_mode = "REAL"
        self._detector_fps = 0.0
        self._detector_latency_ms = 0.0
        self._latest_detections: List[Dict[str, Any]] = []

        self._counts_summary: Dict[str, Any] = {
            "total": 0,
            "resource": 0,
            "host_rock": 0,
            "active_tracks": 0,
            "localized": 0,
        }
        self._active_tracks: List[Dict[str, Any]] = []
        self._spatial_ores: List[Dict[str, Any]] = []

        self._lidar_rate_hz = 0.0
        self._lidar_scan_count = 0
        self._last_lidar_time = 0.0
        self._latest_scan_points: List[List[float]] = []

        self._calibration_state = "UNCALIBRATED"
        self._slam_status = "NO_MAP"
        self._latest_map_data: Optional[Dict[str, Any]] = None
        self._robot_armed = False

        # Subscriptions
        self.create_subscription(
            CompressedImage, "/cogproj/image_annotated/compressed", self._annotated_image_cb, sensor_qos
        )
        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed", self._raw_image_cb, sensor_qos
        )
        self.create_subscription(CameraInfo, "/camera/camera_info", self._camera_info_cb, sensor_qos)
        self.create_subscription(DetectionArray, "/cogproj/detections", self._detections_cb, sensor_qos)
        self.create_subscription(TrackedOreArray, "/cogproj/tracked_ores", self._tracks_cb, sensor_qos)
        self.create_subscription(OreCountSummary, "/cogproj/counts_summary", self._counts_cb, latch_qos)
        self.create_subscription(SpatialOreArray, "/cogproj/spatial_ores", self._spatial_cb, sensor_qos)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, sensor_qos)
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, latch_qos)
        self.create_subscription(Bool, "/lyra/armed", self._armed_cb, 10)

        # Timers
        self.create_timer(0.05, self._watchdog_timer_cb)  # 20 Hz watchdog
        self.create_timer(0.10, self._broadcast_telemetry_cb)  # 10 Hz telemetry

        # Start background WebServer if enabled
        if self.enable_web_server:
            self.web_server = WebServerThread(self, host=web_host, port=web_port)
            self.web_server.start()
            self.get_logger().info(
                f"CogProj Dashboard Gateway initialized on http://{web_host}:{web_port} "
                f"(target={self.target_topic}, limits: lin<={max_linear}m/s, ang<={max_angular}rad/s)"
            )
        else:
            self.get_logger().info("DashboardGatewayNode running with web_server disabled (test mode).")

    # =========================================================================
    # ROS Callbacks
    # =========================================================================
    def _annotated_image_cb(self, msg: CompressedImage) -> None:
        self._last_image_time = time.monotonic()
        self._camera_frame_count += 1
        self._is_annotated = True
        self._latest_image_b64 = base64.b64encode(bytes(msg.data)).decode("utf-8")

        # Stream frame immediately to clients
        DashboardWebSocketHandler.broadcast_json(
            {"type": "image", "data": self._latest_image_b64, "annotated": True}
        )

    def _raw_image_cb(self, msg: CompressedImage) -> None:
        # Fallback if no annotated frames received recently (>1.0s)
        if (time.monotonic() - self._last_image_time) > 1.0 or not self._is_annotated:
            self._last_image_time = time.monotonic()
            self._camera_frame_count += 1
            self._is_annotated = False
            self._latest_image_b64 = base64.b64encode(bytes(msg.data)).decode("utf-8")
            DashboardWebSocketHandler.broadcast_json(
                {"type": "image", "data": self._latest_image_b64, "annotated": False}
            )

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if all(abs(val) < 1e-6 for val in msg.k):
            self._calibration_state = "UNCALIBRATED"
        else:
            self._calibration_state = "CALIBRATED"

    def _detections_cb(self, msg: DetectionArray) -> None:
        dets = []
        for d in msg.detections:
            dets.append(
                {
                    "class": d.class_name,
                    "confidence": round(float(d.confidence), 2),
                    "bbox": [int(d.x_min), int(d.y_min), int(d.x_max), int(d.y_max)],
                }
            )
        self._latest_detections = dets

    def _tracks_cb(self, msg: TrackedOreArray) -> None:
        tracks = []
        for t in msg.tracks:
            tracks.append(
                {
                    "id": int(t.track_id),
                    "class": t.class_name,
                    "confidence": round(float(t.confidence), 2),
                    "state": t.state,
                    "bbox": [
                        int(t.x_min),
                        int(t.y_min),
                        int(t.x_max),
                        int(t.y_max),
                    ],
                    "pos": "UNCALIBRATED" if self._calibration_state == "UNCALIBRATED" else "LOCALIZED",
                }
            )
        self._active_tracks = tracks

    def _counts_cb(self, msg: OreCountSummary) -> None:
        res_count = 0
        rock_count = 0
        for name, count in zip(msg.class_names, msg.class_counts):
            if name.lower() == "resource":
                res_count = int(count)
            elif name.lower() == "host_rock":
                rock_count = int(count)

        self._counts_summary = {
            "total": int(msg.total_count),
            "resource": res_count,
            "host_rock": rock_count,
            "active_tracks": int(msg.active_track_count),
            "localized": sum(1 for o in self._spatial_ores if o.get("localized", False)),
        }

    def _spatial_cb(self, msg: SpatialOreArray) -> None:
        ores = []
        for ore in msg.ores:
            ores.append(
                {
                    "track_id": int(ore.track_id),
                    "class": ore.class_name,
                    "confidence": round(float(ore.confidence), 2),
                    "x": round(float(ore.position_map.x), 2),
                    "y": round(float(ore.position_map.y), 2),
                    "z": round(float(ore.position_map.z), 2),
                    "localized": bool(ore.is_localized),
                    "status": ore.localization_status,
                }
            )
        self._spatial_ores = ores
        self._counts_summary["localized"] = sum(1 for o in ores if o["localized"])

    def _scan_cb(self, msg: LaserScan) -> None:
        self._last_lidar_time = time.monotonic()
        self._lidar_scan_count += 1

        # Subsample scan points for lightweight web canvas rendering
        points = []
        angle = msg.angle_min
        step = max(1, len(msg.ranges) // 72)  # ~72 points per revolution
        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            cur_angle = angle + i * msg.angle_increment
            if msg.range_min < r < msg.range_max:
                px = r * math.cos(cur_angle)
                py = r * math.sin(cur_angle)
                points.append([round(px, 3), round(py, 3)])
        self._latest_scan_points = points

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._slam_status = "MAPPING"
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Downsample or encode map cells for efficient web transmission
        # If map is large (>400x400), downsample by factor of 2
        stride = 1
        if width > 400 or height > 400:
            stride = 2

        out_w = width // stride
        out_h = height // stride
        cells = []
        raw_data = msg.data

        for y in range(0, height, stride):
            row_start = y * width
            for x in range(0, width, stride):
                val = raw_data[row_start + x]
                cells.append(val)

        self._latest_map_data = {
            "width": out_w,
            "height": out_h,
            "resolution": resolution * stride,
            "origin_x": origin_x,
            "origin_y": origin_y,
            "cells": cells,
        }

        # Broadcast map immediately
        DashboardWebSocketHandler.broadcast_json({"type": "map_update", "map": self._latest_map_data})

    def _armed_cb(self, msg: Bool) -> None:
        self._robot_armed = msg.data

    # =========================================================================
    # Watchdog & Telemetry Broadcast
    # =========================================================================
    def _watchdog_timer_cb(self) -> None:
        self.supervisor.check_watchdog()

    def _broadcast_telemetry_cb(self) -> None:
        now = time.monotonic()
        elapsed = now - self._last_fps_calc
        if elapsed >= 1.0:
            self._camera_fps = round(self._camera_frame_count / elapsed, 1)
            self._camera_frame_count = 0
            self._lidar_rate_hz = round(self._lidar_scan_count / elapsed, 1)
            self._lidar_scan_count = 0
            self._last_fps_calc = now

        state = self.get_full_state()
        DashboardWebSocketHandler.broadcast_json({"type": "telemetry", "state": state})

    def get_full_state(self) -> Dict[str, Any]:
        """Construct full JSON snapshot of system state."""
        now = time.monotonic()
        camera_online = (now - self._last_image_time) < 1.5
        lidar_online = (now - self._last_lidar_time) < 1.5

        # Query TF robot pose
        pose = self._lookup_robot_pose()

        return {
            "mode": self.supervisor.mode.value,
            "is_moving": self.supervisor.is_moving,
            "camera_fps": self._camera_fps if camera_online else 0.0,
            "detector_mode": self._detector_mode,
            "detector_latency_ms": self._detector_latency_ms,
            "counts": self._counts_summary,
            "tracks": self._active_tracks,
            "detections": self._latest_detections,
            "spatial_ores": self._spatial_ores,
            "scan_points": self._latest_scan_points if lidar_online else [],
            "robot_pose": pose,
            "sensors": {
                "camera_online": camera_online,
                "lidar_online": lidar_online,
                "lidar_rate_hz": self._lidar_rate_hz if lidar_online else 0.0,
                "lidar_health": "OK" if lidar_online else "OFFLINE",
                "slam_status": self._slam_status,
                "calibration": self._calibration_state,
                "robot_armed": self._robot_armed,
            },
            "nav_state": self._nav_state,
        }

    def _lookup_robot_pose(self) -> Dict[str, Any]:
        """Query TF tree for current base_footprint pose in map or odom frame."""
        for target_frame in ("map", "odom"):
            try:
                t = self.tf_buffer.lookup_transform(target_frame, "base_footprint", rclpy.time.Time())
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                qx = t.transform.rotation.x
                qy = t.transform.rotation.y
                qz = t.transform.rotation.z
                qw = t.transform.rotation.w
                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                return {"x": round(tx, 3), "y": round(ty, 3), "yaw": round(yaw, 3), "frame": target_frame, "valid": True}
            except Exception:
                continue

        return {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame": "none", "valid": False}

    # =========================================================================
    # Supervisor Callbacks & Motion Output
    # =========================================================================
    def _publish_velocity(self, linear_x: float, angular_z: float, deadman: bool = False) -> None:
        """Publish velocity command to /cmd_vel_joy and deadman assert to /joy."""
        if not rclpy.ok():
            return
        try:
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.angular.z = float(angular_z)
            self.cmd_vel_pub.publish(msg)

            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            joy_msg.header.frame_id = "cogproj_dashboard"
            joy_msg.axes = [0.0, 0.0, 0.0, 0.0]
            # Buttons: [0, 1, 2, 3, 4 (deadman), 5 (turbo)]
            joy_msg.buttons = [0, 0, 0, 0, 1 if deadman else 0, 0]
            self.joy_pub.publish(joy_msg)
        except Exception:
            pass

    def _handle_mode_changed(self, old_mode: ControlMode, new_mode: ControlMode) -> None:
        self.get_logger().info(f"Operator control mode transition: {old_mode.value} -> {new_mode.value}")

        if new_mode == ControlMode.STOPPED:
            self._cancel_nav_internal()
            self._call_disarm()

        elif new_mode == ControlMode.MANUAL:
            self._cancel_nav_internal()
            self._call_arm()

        elif new_mode == ControlMode.AUTONOMOUS:
            self._publish_velocity(0.0, 0.0, False)
            self._call_arm()

    def _handle_stop_requested(self) -> None:
        self._publish_velocity(0.0, 0.0, False)
        self._cancel_nav_internal()
        self._call_disarm()
        self._call_estop()

    def _call_arm(self) -> None:
        if self.arm_client.service_is_ready():
            req = Trigger.Request()
            self.arm_client.call_async(req)

    def _call_disarm(self) -> None:
        if self.disarm_client.service_is_ready():
            req = Trigger.Request()
            self.disarm_client.call_async(req)

    def _call_estop(self) -> None:
        if hasattr(self, "estop_client") and self.estop_client.service_is_ready():
            req = Trigger.Request()
            self.estop_client.call_async(req)

    # =========================================================================
    # Autonomous Navigation (Nav2 Action Integration)
    # =========================================================================
    def dispatch_nav_goal(self, x: float, y: float, yaw: float) -> None:
        if self.supervisor.mode != ControlMode.AUTONOMOUS:
            self.get_logger().warn("Rejected nav goal: Mode is not AUTONOMOUS")
            return

        if not self.nav2_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("Nav2 navigate_to_pose action server unavailable")
            self._nav_state = "NAV2_UNAVAILABLE"
            return

        # Cancel any previous goal
        self._cancel_nav_internal()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        half_yaw = yaw * 0.5
        goal_msg.pose.pose.orientation.z = math.sin(half_yaw)
        goal_msg.pose.pose.orientation.w = math.cos(half_yaw)

        self._nav_state = "GOAL_PENDING"
        send_future = self.nav2_client.send_goal_async(goal_msg, feedback_callback=self._nav_feedback_cb)
        send_future.add_done_callback(self._nav_goal_response_cb)
        self.get_logger().info(f"Dispatched Nav2 goal to (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f})")

    def cancel_nav_goal(self) -> None:
        self._cancel_nav_internal()
        self._nav_state = "GOAL_CANCELED"

    def _cancel_nav_internal(self) -> None:
        if self._current_nav_goal_handle:
            try:
                self._current_nav_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._current_nav_goal_handle = None

    def _nav_goal_response_cb(self, future: Any) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected navigation goal")
            self._nav_state = "GOAL_REJECTED"
            return

        self._current_nav_goal_handle = goal_handle
        self._nav_state = "NAVIGATING"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg: Any) -> None:
        self._nav_state = "NAVIGATING"

    def _nav_result_cb(self, future: Any) -> None:
        result = future.result()
        status = result.status
        self._current_nav_goal_handle = None

        if status == 4:  # STATUS_SUCCEEDED
            self._nav_state = "GOAL_REACHED"
            self.get_logger().info("Nav2 Goal Reached Successfully!")
        elif status == 5:  # STATUS_CANCELED
            self._nav_state = "GOAL_CANCELED"
        else:
            self._nav_state = "GOAL_FAILED"
            self.get_logger().warn(f"Nav2 Goal terminated with status code {status}")

    def destroy_node(self) -> None:
        try:
            self.supervisor.emergency_stop()
        except Exception:
            pass
        if hasattr(self, "web_server"):
            self.web_server.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardGatewayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
