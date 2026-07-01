#!/usr/bin/env python
"""
This ROS node converts detected ZED objects into Obstacles for the cart.

@author Lorenzo Ashurst and Zane Metz
@version 4/9/2022
"""

import rclpy

# Messages
from navigation_interface.msg import Obstacle, ObstacleArray
import rclpy.node
from std_msgs.msg import Header
from zed_msgs.msg import ObjectsStamped
from anomaly_msg.msg import AnomalyMsg

# Display purposes
from visualization_msgs.msg import Marker


class ZedObstacleConverter(rclpy.node.Node):
    def __init__(self):

        super().__init__("zed_object_to_obstacle")
        # This is something I need to figure out soon....
        # ----- Parameters -----

        # # Name of the topic that subscribes to the ZED objects
        # self.objects_in = rospy.get_param(
        #     "objects_in", "/front_cam/front/obj_det/objects"
        # )
        # # Name of the topic that publishes the obstacles
        # self.obstacles_out = rospy.get_param(
        #     "obstacles_out", "/front_cam_obj_obstacles"
        # )
        # self.obstacle_markers_out = rospy.get_param(
        #     "obstacle_markers_out", "/front_cam_obj_obstacle_display"
        # )

        # ----- Node State -----
        # FIXME this subscriber is kind of broken... I need to figure out what the correlation
        # between OG implimentation and my implimentation is here.
        self.object_sub = self.create_subscription(
            ObjectsStamped,
            "/zed_front/zed_node_0/obj_det/objects",
            self.receiveObjects,
            10,
        )
        print("IM HERE")
        self.obstacle_pub = self.create_publisher(ObstacleArray, "/obstacles", 10)
        self.display_pub = self.create_publisher(Marker, "/obs_visualization", 10)
        self.anomaly_pub = self.create_publisher(AnomalyMsg, "/ai_anomaly_logging", 10)
        self.last_object_count = None

    def receiveObjects(self, msg):
        """
        Receives an ObjectsStamped message, converts it to an ObstacleArray and publishes it.

        @param self
        @param msg  ObjectsStamped message
        """

        # rospy.logwarn("[%s] **Object** converter received a message in coordinate frame %s" % (self.name, msg.header.frame_id))

        obstacles = ObstacleArray()
        obstacles.header.frame_id = msg.header.frame_id
        obstacles.header.stamp = msg.header.stamp

        for obj in msg.objects:

            obs = Obstacle()
            obs.header.frame_id = msg.header.frame_id
            obs.header.stamp = msg.header.stamp

            # This float conversion thing here is a bit ugly but gives me errors otherwise.
            obs.pos.point.x = float(obj.position[0])
            obs.pos.point.y = float(obj.position[1])
            obs.pos.point.z = float(obj.position[2])

            # Choose the radius to be the max of width / length
            obs.radius = float(max(obj.dimensions_3d[0], obj.dimensions_3d[2]))

            obstacles.obstacles.append(obs)

        self.obstacle_pub.publish(obstacles)
        object_count = len(obstacles.obstacles)
        if object_count != self.last_object_count:
            severity = AnomalyMsg.INFO
            message = f"ZED obstacle converter published object obstacles: count={object_count}"
            if object_count >= 5:
                severity = AnomalyMsg.WARNING
                message = f"ZED obstacle converter sees dense object field: count={object_count}"
            self.anomaly_logging(message, severity)
            self.last_object_count = object_count
        self.get_logger().info("published the obstacles")
        # rospy.loginfo("[%s] Published %d obstacles!" % (self.name, len(obstacles.obstacles)))
        self.local_display("/zed_front_camera_link", obstacles)

    # Display the obstacles on the LIDAR frame, will appear jittery in Rviz as there is no interpolation
    def local_display(self, frame, object_list):
        for i in range(len(object_list.obstacles)):
            print("PUBLISHING IN LOCAL DISPLAY")
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = frame

            marker.ns = "Object_NS"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = 0
            marker.color.r = 0.5
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.pose.position.x = object_list.obstacles[i].pos.point.x
            print(f"x cord: {object_list.obstacles[i].pos.point.x}")
            marker.pose.position.y = object_list.obstacles[i].pos.point.y
            print(f"y cord: {object_list.obstacles[i].pos.point.y}")

            marker.pose.position.z = 0.0

            radius = object_list.obstacles[i].radius
            marker.scale.x = 10.0
            marker.scale.y = 10.0
            marker.scale.z = 0.3

            self.display_pub.publish(marker)

    def anomaly_logging(self, message, severity):
        anomaly_msg = AnomalyMsg()
        anomaly_msg.header = Header()
        anomaly_msg.header.stamp = self.get_clock().now().to_msg()
        anomaly_msg.header.frame_id = "zed_object_to_obstacle"
        anomaly_msg.node_name = self.get_name()
        anomaly_msg.importance = severity
        anomaly_msg.type = AnomalyMsg.TEXT
        anomaly_msg.msg = message
        self.anomaly_pub.publish(anomaly_msg)


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = ZedObstacleConverter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
