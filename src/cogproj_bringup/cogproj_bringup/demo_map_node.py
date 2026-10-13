"""Simulated Demo Map Node for CogProj RViz Demonstration.

SAFETY & ARCHITECTURAL NOTICE:
This node is strictly used to demonstrate the RViz layout when physical SLAM is offline.
It publishes ONLY on the isolated, non-authoritative topic '/cogproj/demo_simulated_map'.
It NEVER publishes on the authoritative '/map' topic and is NEVER used as evidence of
physical SLAM mapping.
"""

from typing import List
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


class SimulatedDemoMapNode(Node):
    """Publishes a mock occupancy grid and disclaimer marker for RViz demonstration."""

    def __init__(self) -> None:
        super().__init__("demo_simulated_map_node")

        # Parameters
        self.declare_parameter("map_topic", "/cogproj/demo_simulated_map")
        self.declare_parameter("marker_topic", "/cogproj/demo_simulated_map_markers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("width_m", 6.0)
        self.declare_parameter("height_m", 6.0)

        self.map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.res = self.get_parameter("resolution").get_parameter_value().double_value
        self.w_m = self.get_parameter("width_m").get_parameter_value().double_value
        self.h_m = self.get_parameter("height_m").get_parameter_value().double_value

        # Enforce safety constraint: NEVER allow publishing to authoritative /map
        if self.map_topic.strip() == "/map":
            self.get_logger().error(
                "SAFETY CONTRACT VIOLATION: Simulated demo map cannot publish to '/map'! "
                "Forcing topic to '/cogproj/demo_simulated_map'."
            )
            self.map_topic = "/cogproj/demo_simulated_map"

        # QoS Profiles
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, map_qos)

        self.cached_grid = self._generate_simulated_grid()
        self.cached_markers = self._generate_disclaimer_markers()

        # Publish immediately and periodically at 1 Hz
        self.timer = self.create_timer(1.0, self._publish_demo_map)

        self.get_logger().info("SimulatedDemoMapNode active.")
        self.get_logger().info(f"  Isolated Demo Map:    {self.map_topic}")
        self.get_logger().info(f"  Disclaimer Markers:   {self.marker_topic}")
        self.get_logger().info(f"  Frame ID:             {self.frame_id}")

    def _generate_simulated_grid(self) -> OccupancyGrid:
        """Create a synthetic 2D room occupancy grid labeled 'SIMULATED DEMO MAP'."""
        grid = OccupancyGrid()
        grid.header.frame_id = self.frame_id

        width_cells = int(round(self.w_m / self.res))
        height_cells = int(round(self.h_m / self.res))

        grid.info = MapMetaData()
        grid.info.resolution = float(self.res)
        grid.info.width = width_cells
        grid.info.height = height_cells
        grid.info.origin.position.x = -self.w_m / 2.0
        grid.info.origin.position.y = -self.h_m / 2.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # Occupancy array: 0 = free, 100 = occupied, -1 = unknown
        data = np.zeros((height_cells, width_cells), dtype=np.int8)

        # Outer boundary walls
        data[0:2, :] = 100
        data[-2:, :] = 100
        data[:, 0:2] = 100
        data[:, -2:] = 100

        # Some internal demo partition obstacles
        data[int(height_cells * 0.4):int(height_cells * 0.6), int(width_cells * 0.7):int(width_cells * 0.73)] = 100

        grid.data = data.flatten().tolist()
        return grid

    def _generate_disclaimer_markers(self) -> MarkerArray:
        """Create explicit 3D floating disclaimer text marker for RViz."""
        array = MarkerArray()

        text_marker = Marker()
        text_marker.header.frame_id = self.frame_id
        text_marker.ns = "simulated_demo_disclaimer"
        text_marker.id = 999
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 1.2
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.22  # Text height in meters
        text_marker.color.r = 1.0
        text_marker.color.g = 0.65
        text_marker.color.b = 0.0
        text_marker.color.a = 0.95
        text_marker.text = "[SIMULATED DEMO MAP - NOT REAL SLAM]"

        array.markers.append(text_marker)
        return array

    def _publish_demo_map(self) -> None:
        """Publish the cached demo occupancy grid and disclaimer markers."""
        now = self.get_clock().now().to_msg()
        self.cached_grid.header.stamp = now
        for m in self.cached_markers.markers:
            m.header.stamp = now

        self.map_pub.publish(self.cached_grid)
        self.marker_pub.publish(self.cached_markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulatedDemoMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

