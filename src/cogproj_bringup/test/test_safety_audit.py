"""System-wide safety audit tests for CogProj perception nodes and launch files.

These tests statically audit the ROS 2 node definitions and launch files
within CogProj to verify that:
1. No CogProj perception node registers publishers on robot actuation topics
   (/cmd_vel, /cmd_vel_nav, /cmd_vel_joy).
2. All advertised topics across instantiated CogProj node classes remain confined to
   perception, tracking, counting, and visualization namespaces.
3. All launch files in cogproj_bringup do not configure nodes or parameters that
   command robot actuation.

Note: These checks verify the static configuration and programmatic publisher
registration of CogProj components; they do not audit running external processes.
"""

import glob
import os
import unittest

import rclpy
from cogproj_camera import CameraInputNode
from cogproj_counting import CountingNode
from cogproj_detection import DetectionNode
from cogproj_tracking import TrackingNode
from cogproj_visualization import CogprojViewer, VisualizerNode


class TestSystemSafetyAudit(unittest.TestCase):
    """Safety audit verifying workspace isolation and zero robot movement capability."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def _create_node_safely(self, node_factory):
        """Instantiate a node and register its cleanup immediately."""
        node = node_factory()
        self.addCleanup(node.destroy_node)
        return node

    def test_nodes_have_zero_actuation_publishers(self):
        """Verify every CogProj node has ZERO publishers on /cmd_vel* topics."""
        node_factories = [
            ("CameraInputNode", CameraInputNode),
            ("DetectionNode", DetectionNode),
            ("TrackingNode", TrackingNode),
            ("CountingNode", CountingNode),
            ("VisualizerNode", VisualizerNode),
            ("CogprojViewer", CogprojViewer),
        ]

        forbidden_patterns = ["cmd_vel", "cmd_vel_nav", "cmd_vel_joy", "lyra/armed"]

        for name, factory in node_factories:
            node = self._create_node_safely(factory)
            for pub in node.publishers:
                topic = pub.topic_name
                for forbidden in forbidden_patterns:
                    self.assertNotIn(
                        forbidden,
                        topic,
                        f"SAFETY VIOLATION: {name} publishes to forbidden topic '{topic}' containing '{forbidden}'!",
                    )

    def test_advertised_topics_are_safe_perception_only(self):
        """Verify that all topics published by CogProj belong strictly to perception namespaces."""
        node_factories = [
            CameraInputNode,
            DetectionNode,
            TrackingNode,
            CountingNode,
            VisualizerNode,
        ]

        allowed_exact_topics = {
            "/cogproj/image_raw",
            "/cogproj/detections",
            "/cogproj/tracked_ores",
            "/cogproj/counts_summary",
            "/cogproj/image_annotated",
            "/cogproj/image_annotated/compressed",
            "/rosout",
            "/parameter_events",
        }

        for factory in node_factories:
            node = self._create_node_safely(factory)
            for pub in node.publishers:
                topic = pub.topic_name
                self.assertIn(
                    topic,
                    allowed_exact_topics,
                    f"Unexpected topic '{topic}' published by {node.get_name()}",
                )

    def test_launch_files_contain_no_actuation_actions(self):
        """Verify that bringup launch files do not reference /cmd_vel or movement topics."""
        bringup_dir = os.path.dirname(os.path.dirname(__file__))
        launch_dir = os.path.join(bringup_dir, "launch")
        launch_files = glob.glob(os.path.join(launch_dir, "*.launch.py"))

        self.assertGreater(len(launch_files), 0, "No launch files found in bringup package")

        forbidden_strings = ["/cmd_vel", "cmd_vel_nav", "cmd_vel_joy", "lyra_control", "nav2", "slam_toolbox"]

        for lf in launch_files:
            with open(lf, "r", encoding="utf-8") as f:
                content = f.read()
                for forbidden in forbidden_strings:
                    self.assertNotIn(
                        forbidden,
                        content,
                        f"SAFETY VIOLATION in launch file {lf}: contains forbidden string '{forbidden}'",
                    )


if __name__ == "__main__":
    unittest.main()
