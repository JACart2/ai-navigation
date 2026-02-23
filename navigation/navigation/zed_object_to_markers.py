#!/usr/bin/env python3
"""
Convert ZED detected objects to RViz markers (3D bounding boxes).
"""

import rclpy
import rclpy.node
from rclpy.duration import Duration

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from zed_msgs.msg import ObjectsStamped


class ZedObjectToMarkers(rclpy.node.Node):
    def __init__(self):
        super().__init__("zed_object_to_markers")

        self.declare_parameter("objects_topic", "/zed_front/zed_node_0/obj_det/objects")
        self.declare_parameter("markers_topic", "/zed_front/zed_node_0/obj_det/markers")
        self.declare_parameter("frame_override", "")
        self.declare_parameter("marker_ns", "zed_obj")
        self.declare_parameter("marker_alpha", 0.35)
        self.declare_parameter("lifetime_sec", 0.2)

        self.objects_topic = (
            self.get_parameter("objects_topic").get_parameter_value().string_value
        )
        self.markers_topic = (
            self.get_parameter("markers_topic").get_parameter_value().string_value
        )
        self.frame_override = (
            self.get_parameter("frame_override").get_parameter_value().string_value
        )
        self.marker_ns = (
            self.get_parameter("marker_ns").get_parameter_value().string_value
        )
        self.marker_alpha = (
            self.get_parameter("marker_alpha").get_parameter_value().double_value
        )
        self.lifetime_sec = (
            self.get_parameter("lifetime_sec").get_parameter_value().double_value
        )

        self.sub = self.create_subscription(
            ObjectsStamped, self.objects_topic, self.on_objects, 10
        )
        self.pub = self.create_publisher(MarkerArray, self.markers_topic, 10)

    def on_objects(self, msg: ObjectsStamped) -> None:
        markers = MarkerArray()

        # Clear previous markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        header = msg.header
        if self.frame_override:
            header.frame_id = self.frame_override

        lifetime = Duration(seconds=self.lifetime_sec).to_msg()

        for i, obj in enumerate(msg.objects):
            marker = Marker()
            marker.header = header
            marker.ns = self.marker_ns
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(obj.position[0])
            marker.pose.position.y = float(obj.position[1])
            marker.pose.position.z = float(obj.position[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = float(obj.dimensions_3d[0])
            marker.scale.y = float(obj.dimensions_3d[1])
            marker.scale.z = float(obj.dimensions_3d[2])

            marker.color = ColorRGBA(r=0.2, g=0.9, b=0.2, a=self.marker_alpha)
            marker.lifetime = lifetime

            markers.markers.append(marker)

        self.pub.publish(markers)


def main():
    rclpy.init()
    node = ZedObjectToMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
