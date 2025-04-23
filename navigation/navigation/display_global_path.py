#!/usr/bin/env python
"""
This is the ROS 2 node publishes the global path as a marker array for visualizing in rviz.

Authors: Martin Nester
"""

# ROS based imports
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from navigation_interface.msg import LocalPointsArray
from visualization_msgs.msg import MarkerArray, Marker


class DisplayGlobalPath(rclpy.node.Node):

    def __init__(self):
        super().__init__("display_global_path")

        self.rviz_path_pub = self.create_publisher(MarkerArray, "/visual_path", 10)

        # ROS2 subscribers
        self.path_sub = self.create_subscription(
            LocalPointsArray, "/global_path", self.path_cb, 10
        )

    def path_cb(self, msg):
        """Callback responible for publising the nodes in which the cart must traverse

        Args:
            file_name (string): The .gml file to load as the graph, cart_planning/launch/constants.launch
            is where the file path can be found
        """
        delarr = MarkerArray()
        delmark = Marker()
        delmark.action = 3
        delarr.markers.append(delmark)
        self.rviz_path_pub.publish(delarr)

        arr = MarkerArray()
        id = 0
        for p in msg.localpoints:
            temp = Marker()
            temp.pose = p
            temp.header.frame_id = "map"
            temp.id = id
            id += 1
            temp.scale.x = 1.5
            temp.scale.y = 1.5
            temp.scale.z = 1.5
            temp.color.r = 0.0
            temp.color.g = 255.0
            temp.color.b = 0.0
            temp.color.a = 1.0
            temp.type = 2
            temp.action = 0
            arr.markers.append(temp)
        arr.markers[0].color.g = 0.0
        arr.markers[-1].color.r = 50.0
        arr.markers[-1].color.g = 0.0
        self.rviz_path_pub.publish(arr)


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = DisplayGlobalPath()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
