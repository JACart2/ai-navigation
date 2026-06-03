#!/usr/bin/env python
"""
This ROS2 node is used to visualize edges and nodes from a prespecified graph file in RVIZ.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import networkx as nx

import rclpy
import rclpy.qos
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly


class GraphVisual(rclpy.node.Node):

    def __init__(self):
        super().__init__("visualize_graph")

        self.declare_parameter(
            "graph_file", "./src/ai-navigation/navigation/maps/main_shift3.gml"
        )

        latching_qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.global_graph = nx.DiGraph()

        file_name = self.get_parameter("graph_file").get_parameter_value().string_value
        self.load_file(file_name)

        self.visual_pub = self.create_publisher(
            MarkerArray, "/graph_visual", qos_profile=latching_qos
        )

        self.timer = self.create_timer(3, self.timer_cb)
        self.edge_id = 0

    def timer_cb(self):
        """Simple timer callback responsible for publishing the MarkerArray with node/edge information."""
        self.get_logger().info("Looping over path")

        # This first for loop adds every node to some Marker array
        marker_array = MarkerArray()
        id = 0
        for node in self.global_graph:
            temp = Marker()
            new_point = Pose()
            new_point.position.x = self.global_graph.nodes[node]["pos"][0]
            new_point.position.y = self.global_graph.nodes[node]["pos"][1]
            temp.pose = new_point
            temp.header.frame_id = "map"
            temp.id = id
            id += 1
            temp.scale.x = 1.0
            temp.scale.y = 1.0
            temp.scale.z = 1.0
            temp.color.r = 0.0
            temp.color.g = 0.0
            temp.color.b = 1.0
            temp.color.a = 0.5
            temp.type = 2
            temp.action = 0
            marker_array.markers.append(temp)

        # This second for loop adds all the edges to the MarkerArray
        nodes = self.global_graph.nodes
        edges = self.global_graph.edges()
        for edge in edges:
            first_node = nodes[edge[0]]
            second_node = nodes[edge[1]]

            points = []

            first_point = Point()
            first_point.x = first_node["pos"][0]
            first_point.y = first_node["pos"][1]

            second_point = Point()
            second_point.x = second_node["pos"][0]
            second_point.y = second_node["pos"][1]

            points.append(first_point)
            points.append(second_point)

            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = "map"

            marker.ns = "Path_NS"
            marker.id = self.edge_id
            marker.type = Marker.ARROW
            marker.action = 0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.points = points

            marker.scale.x = 0.3
            marker.scale.y = 0.6
            marker.scale.z = 0.0

            marker_array.markers.append(marker)
            self.edge_id += 1
        self.visual_pub.publish(marker_array)
        self.destroy_timer(self.timer)

    def load_file(self, file_name):
        """Loads a file and sets all nodes to active(allowed to take part in pathfinding).

        Args:
            file_name (string): The .gml file to load as the graph, cart_planning/launch/constants.launch
            is where the file path can be found
        """

        try:
            self.global_graph = nx.read_gml(file_name)

            for node in self.global_graph:
                self.global_graph.nodes[node]["active"] = True
        except Exception as e:
            print(e)
            self.log_header(
                f"Unable to launch graph file pointed to in the constants file in {file_name}"
            )

    def log_header(self, msg):
        """Helper method to print noticeable log statements."""
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"{msg}")
        self.get_logger().info("=" * 50)


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = GraphVisual()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
