#!/usr/bin/env python
"""
This is the ROS 2 node that handles the global planning for the JACART.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import time
import serial as sr
import numpy as np
import math
import networkx as nx
from navigation import simple_gps_util

# ROS based import
import tf_transformations
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly


import rclpy
from geometry_msgs.msg import Pose, Point, PoseStamped, PointStamped, TwistStamped
from visualization_msgs.msg import Marker, MarkerArray

class GraphVisual(rclpy.node.Node):

    def __init__(self):
        super().__init__("visualize_graph")

        self.declare_parameter("graph_file", "")

        self.global_graph = nx.DiGraph()
        # self.logic_graph = None

        file_name = self.get_parameter("graph_file").get_parameter_value().string_value
        self.load_file("./src/ai-navigation/navigation/navigation/drive_build.gml")

        self.visual_pub = self.create_publisher(MarkerArray, "/graph_visual", 10)

        self.visualize_edges = self.create_publisher(Marker, "/visual_edges", 10)

        self.timer = self.create_timer(3, self.timer_cb)

    def timer_cb(self):
        self.get_logger().info("Looping over path")
        arr = MarkerArray()
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
            temp.color.a = 1.0
            temp.type = 2
            temp.action = 0
            arr.markers.append(temp)
        arr.markers[0].color.g = 50.0
        arr.markers[-1].color.r = 50.0
        self.visual_pub.publish(arr)

        

        edges = self.global_graph.edges()
        for edge in edges:

            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.ARROW
            marker.pose.position.x = 0.0  # Start point x
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 1.0  # Arrow length
            marker.scale.y = 0.1  # Arrow width
            marker.scale.z = 0.1  # Arrow height
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Alpha

            marker.points = [Point(), Point()]
            marker.points[0].x = 0.0  # Start point x
            marker.points[0].y = 0.0  # Start point y

            marker.points[1].x = 1.0  # End point x
            marker.points[1].y = 1.0  # End point y

            print(self.global_graph.edges[edge]["weight"])
            print(type[edge[0]])
        

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
