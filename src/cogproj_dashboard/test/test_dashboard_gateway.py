"""Comprehensive integration and unit tests for DashboardGatewayNode (Phase 5M)."""

import time
import unittest

import rclpy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from cogproj_dashboard.dashboard_gateway_node import DashboardGatewayNode
from cogproj_dashboard.supervisor import ControlMode, NavState
from cogproj_interfaces.msg import (
    Detection,
    DetectionArray,
    SpatialOre,
    SpatialOreArray,
    TrackedOre,
    TrackedOreArray,
)


class TestDashboardGatewayNode(unittest.TestCase):
    """Test suite verifying DashboardGatewayNode publishers, safety routing, and Nav2 validation."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        # Disable background web server during unit testing to avoid port binding conflicts
        param = rclpy.parameter.Parameter("enable_web_server", rclpy.parameter.Parameter.Type.BOOL, False)
        self.node = DashboardGatewayNode(parameter_overrides=[param])

    def tearDown(self):
        self.node.destroy_node()

    def test_target_topic_is_pre_gate_and_deadman_publisher_exists(self):
        """Gateway must strictly publish to /cmd_vel_joy (pre-gate) and /joy (deadman), NEVER /cmd_vel."""
        self.assertEqual(self.node.cmd_vel_pub.topic_name, "/cmd_vel_joy")
        self.assertNotEqual(self.node.cmd_vel_pub.topic_name, "/cmd_vel")
        self.assertEqual(self.node.joy_pub.topic_name, "/joy")

    def test_service_client_targets(self):
        """Service clients must connect to the documented Lyra hardware services."""
        self.assertEqual(self.node.arm_client.srv_name, "/lyra/arm")
        self.assertEqual(self.node.disarm_client.srv_name, "/lyra/disarm")
        self.assertEqual(self.node.estop_client.srv_name, "/lyra/emergency_stop")

    def test_initial_state_is_stopped_and_safe(self):
        """Initial state must be STOPPED with zero motion and motors disarmed."""
        state = self.node.get_full_state()
        self.assertEqual(state["mode"], "STOPPED")
        self.assertFalse(state["motion_enabled"])
        self.assertFalse(state["is_moving"])
        self.assertEqual(state["nav_state"], "STOPPED")

    def test_state_dictionary_structure(self):
        """Full state dictionary must include all Phase 5M required telemetry fields."""
        state = self.node.get_full_state()
        required_keys = [
            "mode",
            "motion_enabled",
            "is_moving",
            "linear_velocity",
            "angular_velocity",
            "deadman_active",
            "watchdog_healthy",
            "watchdog_age_ms",
            "active_session",
            "nav_state",
            "nav_blockers",
            "camera_fps",
            "counts",
            "tracks",
            "sensors",
            "robot_pose",
        ]
        for key in required_keys:
            self.assertIn(key, state, f"Missing required state key: {key}")

    def test_publish_velocity_sends_joy_deadman(self):
        """_publish_velocity must populate /joy button 4 when deadman is True, and 0 when False."""
        published_twists = []
        published_joys = []

        orig_twist_pub = self.node.cmd_vel_pub.publish
        orig_joy_pub = self.node.joy_pub.publish

        self.node.cmd_vel_pub.publish = lambda msg: published_twists.append(msg)
        self.node.joy_pub.publish = lambda msg: published_joys.append(msg)

        try:
            # Send moving command with deadman=True
            self.node._publish_velocity(0.08, 0.15, deadman=True)
            self.assertEqual(len(published_twists), 1)
            self.assertEqual(len(published_joys), 1)
            self.assertAlmostEqual(published_twists[0].linear.x, 0.08)
            self.assertAlmostEqual(published_twists[0].angular.z, 0.15)
            self.assertGreaterEqual(len(published_joys[0].buttons), 6)
            self.assertEqual(published_joys[0].buttons[4], 1)
            self.assertEqual(published_joys[0].buttons[5], 0)

            # Send stop command with deadman=False
            self.node._publish_velocity(0.0, 0.0, deadman=False)
            self.assertEqual(len(published_twists), 2)
            self.assertEqual(len(published_joys), 2)
            self.assertAlmostEqual(published_twists[1].linear.x, 0.0)
            self.assertAlmostEqual(published_twists[1].angular.z, 0.0)
            self.assertEqual(published_joys[1].buttons[4], 0)
        finally:
            self.node.cmd_vel_pub.publish = orig_twist_pub
            self.node.joy_pub.publish = orig_joy_pub

    def test_mode_switch_arms_and_disarms(self):
        """Mode transitions must invoke hardware arming and disarming service calls."""
        arm_calls = 0
        disarm_calls = 0

        orig_arm = self.node._call_arm
        orig_disarm = self.node._call_disarm

        def mock_arm():
            nonlocal arm_calls
            arm_calls += 1

        def mock_disarm():
            nonlocal disarm_calls
            disarm_calls += 1

        self.node._call_arm = mock_arm
        self.node._call_disarm = mock_disarm

        try:
            # STOPPED -> MANUAL: arms
            self.node.supervisor.set_mode("MANUAL")
            self.assertEqual(arm_calls, 1)
            self.assertEqual(disarm_calls, 0)

            # MANUAL -> STOPPED: disarms
            self.node.supervisor.set_mode("STOPPED")
            self.assertEqual(arm_calls, 1)
            self.assertEqual(disarm_calls, 1)
        finally:
            self.node._call_arm = orig_arm
            self.node._call_disarm = orig_disarm

    def test_nav_prerequisites_blocking(self):
        """Nav2 goal dispatch must be blocked if preconditions are not satisfied."""
        # 1. Blocked in STOPPED mode
        ready, blockers = self.node.check_nav_prerequisites()
        self.assertFalse(ready)
        self.assertTrue(any("Operating mode is not AUTONOMOUS" in b for b in blockers))
        self.assertTrue(any("Motion is disabled" in b for b in blockers))

        # 2. Blocked if mode is AUTONOMOUS but operator explicitly disables motion
        self.node.supervisor.set_mode("AUTONOMOUS")
        self.node.supervisor.enable_motion(False)
        ready, blockers = self.node.check_nav_prerequisites()
        self.assertFalse(ready)
        self.assertTrue(any("Motion is disabled" in b for b in blockers))

        # 3. Blocked if map is missing
        self.node.supervisor.enable_motion(True)
        ready, blockers = self.node.check_nav_prerequisites()
        self.assertFalse(ready)
        self.assertTrue(any("No occupancy grid map" in b for b in blockers))

    def test_occupancy_grid_goal_validation(self):
        """Goal coordinates must be validated against map bounds and obstacle cost."""
        grid = OccupancyGrid()
        grid.info = MapMetaData()
        grid.info.resolution = 0.1  # 10 cm per cell
        grid.info.width = 10
        grid.info.height = 10
        grid.info.origin = Pose()
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0

        # Create 10x10 map: cell (5,5) is an obstacle (cost=100), others are free (cost=0)
        data = [0] * 100
        data[5 * 10 + 5] = 100  # Obstacle at (x=0.55, y=0.55)
        grid.data = data

        self.node._latest_occupancy_grid = grid

        # Free space goal (x=0.25, y=0.25) -> valid
        valid, reason = self.node._validate_goal_against_map(0.25, 0.25)
        self.assertTrue(valid)

        # Obstacle goal (x=0.55, y=0.55) -> invalid
        valid, reason = self.node._validate_goal_against_map(0.55, 0.55)
        self.assertFalse(valid)
        self.assertIn("obstacle", reason.lower())

        # Out-of-bounds goal (x=1.5, y=0.5) -> invalid
        valid, reason = self.node._validate_goal_against_map(1.5, 0.5)
        self.assertFalse(valid)
        self.assertIn("outside map bounds", reason.lower())

        # Negative coordinate out-of-bounds goal (x=-0.2, y=0.5) -> invalid
        valid, reason = self.node._validate_goal_against_map(-0.2, 0.5)
        self.assertFalse(valid)
        self.assertIn("outside map bounds", reason.lower())

    def test_dispatch_nav_goal_blocked_without_prerequisites(self):
        """dispatch_nav_goal must safely reject goals if prerequisites are unfulfilled."""
        ok, msg = self.node.dispatch_nav_goal(1.0, 1.0, 0.0)
        self.assertFalse(ok)
        self.assertIn("Cannot dispatch navigation goal", msg)

    def test_cancel_nav_goal(self):
        """cancel_nav_goal must update supervisor nav_state to GOAL_CANCELED."""
        self.node.cancel_nav_goal()
        self.assertEqual(self.node.supervisor.nav_state, NavState.GOAL_CANCELED)

    def test_detections_cb(self):
        """Verify _detections_cb properly parses DetectionArray."""
        msg = DetectionArray()
        d = Detection()
        d.class_name = "resource"
        d.confidence = 0.92
        d.x_min = 100.0
        d.y_min = 150.0
        d.x_max = 200.0
        d.y_max = 250.0
        msg.detections.append(d)

        self.node._detections_cb(msg)
        state = self.node.get_full_state()
        self.assertEqual(len(state["detections"]), 1)
        self.assertEqual(state["detections"][0]["class"], "resource")
        self.assertEqual(state["detections"][0]["bbox"], [100, 150, 200, 250])

    def test_tracks_cb(self):
        """Verify _tracks_cb properly parses TrackedOreArray."""
        msg = TrackedOreArray()
        t = TrackedOre()
        t.track_id = 7
        t.class_name = "host_rock"
        t.confidence = 0.88
        t.state = "CONFIRMED"
        t.x_min = 50.0
        t.y_min = 60.0
        t.x_max = 120.0
        t.y_max = 140.0
        msg.tracks.append(t)

        self.node._tracks_cb(msg)
        state = self.node.get_full_state()
        self.assertEqual(len(state["tracks"]), 1)
        self.assertEqual(state["tracks"][0]["id"], 7)
        self.assertEqual(state["tracks"][0]["class"], "host_rock")
        self.assertEqual(state["tracks"][0]["bbox"], [50, 60, 120, 140])

    def test_spatial_cb(self):
        """Verify _spatial_cb properly parses SpatialOreArray."""
        msg = SpatialOreArray()
        ore = SpatialOre()
        ore.track_id = 3
        ore.class_name = "resource"
        ore.confidence = 0.95
        ore.position_map = Point(x=1.5, y=-0.8, z=0.1)
        ore.is_localized = True
        ore.localization_status = "LOCALIZED"
        msg.ores.append(ore)

        self.node._spatial_cb(msg)
        state = self.node.get_full_state()
        self.assertEqual(len(state["spatial_ores"]), 1)
        self.assertEqual(state["spatial_ores"][0]["track_id"], 3)
        self.assertEqual(state["spatial_ores"][0]["x"], 1.5)
        self.assertTrue(state["spatial_ores"][0]["localized"])


if __name__ == "__main__":
    unittest.main()
