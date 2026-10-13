"""Integration and unit tests for DashboardGatewayNode."""

import unittest
import rclpy
from cogproj_dashboard.dashboard_gateway_node import DashboardGatewayNode
from cogproj_dashboard.supervisor import ControlMode


class TestDashboardGatewayNode(unittest.TestCase):
    """Test suite verifying DashboardGatewayNode publisher targets and safety."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        # Disable web server during unit testing to avoid port binding conflicts
        param = rclpy.parameter.Parameter("enable_web_server", rclpy.parameter.Parameter.Type.BOOL, False)
        self.node = DashboardGatewayNode(parameter_overrides=[param])

    def tearDown(self):
        self.node.destroy_node()

    def test_target_topic_is_pre_gate(self):
        """Gateway must strictly publish to /cmd_vel_joy (pre-gate) and NEVER /cmd_vel."""
        self.assertEqual(self.node.cmd_vel_pub.topic_name, "/cmd_vel_joy")
        self.assertNotEqual(self.node.cmd_vel_pub.topic_name, "/cmd_vel")

    def test_initial_state_is_stopped(self):
        """Initial state must be STOPPED with zero motion."""
        state = self.node.get_full_state()
        self.assertEqual(state["mode"], "STOPPED")
        self.assertFalse(state["is_moving"])

    def test_state_dictionary_structure(self):
        """Full state dictionary must include all required telemetry fields."""
        state = self.node.get_full_state()
        required_keys = ["mode", "is_moving", "camera_fps", "counts", "tracks", "sensors", "robot_pose", "nav_state"]
        for key in required_keys:
            self.assertIn(key, state, f"Missing required state key: {key}")

    def test_detections_cb(self):
        """Verify _detections_cb properly parses DetectionArray without attribute errors."""
        from cogproj_interfaces.msg import Detection, DetectionArray
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
        """Verify _tracks_cb properly parses TrackedOreArray without attribute errors."""
        from cogproj_interfaces.msg import TrackedOre, TrackedOreArray
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
        """Verify _spatial_cb properly parses SpatialOreArray without attribute errors."""
        from cogproj_interfaces.msg import SpatialOre, SpatialOreArray
        from geometry_msgs.msg import Point
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
