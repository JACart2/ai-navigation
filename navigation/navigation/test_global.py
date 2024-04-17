#!/usr/bin/env python
"""
This is the ROS 2 node that fakes localization data for the purpose of testing global planner.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""

# ROS based imports
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from navigation_interface.msg import LocalPointsArray, VehicleState
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import MarkerArray, Marker

class GlobalTester(rclpy.node.Node):

    def __init__(self):
        super().__init__("global_tester")

        # ROS2 publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/limited_pose", 10)
        self.tar_pub = self.create_publisher(PointStamped, "clicked_point", 10)
        self.vel_pub = self.create_publisher(Float32, "/estimated_vel_mps", 10)
        self.state_pub = self.create_publisher(VehicleState, "/vehicle_state", 10)
        self.rviz_path_pub = self.create_publisher(MarkerArray, "/visual_path", 10)

        # ROS2 subscribers 
        self.path_sub = self.create_subscription(
            LocalPointsArray, "/global_path", self.path_cb, 10
        )

        # Cart position
        self.pose = PoseStamped()
        self.pose.pose.position.x = 82.23206329345703
        self.pose.pose.position.y = 132.16149291992187

        # Target position
        self.target = PointStamped()
        self.target.point.x = 49.95549774169922
        self.target.point.y = 68.27464294433594

        # Cart velocity
        self.vel = Float32()
        self.vel.data = 0.0

        # Cart state
        self.state = VehicleState()
        self.state.stopped = True
        self.state.reached_destination = False
        self.state.is_navigating = False

        self.timer = self.create_timer(3.0, self.timer_cb)
        self.get_logger().info("Begin")

    def timer_cb(self):
        """This timer is only responible for publishing the state variables of the cart. 
        It destroys itself after this task is complete.
        """
        self.destroy_timer(self.timer)
        self.pose_pub.publish(self.pose)
        self.tar_pub.publish(self.target)
        self.vel_pub.publish(self.vel)
        self.state_pub.publish(self.state)

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
        self.rviz_path_pub.publish(arr)



def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = GlobalTester()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
