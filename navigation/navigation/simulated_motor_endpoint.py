#!/usr/bin/env python
"""
This is a ROS2 node that simulates motion rather than sending messages to the arduino controller.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import time
from navigation import steering_position_calc

# ROS based imports
import rclpy
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
from motor_control_interface.msg import VelAngle
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32


class SimulatedMotor(rclpy.node.Node):
    """ROS2 node that handles controlling the motor."""

    def __init__(self):
        super().__init__("motor_simulator")

        # Class constants
        self.NODE_RATE = 20
        self.STEER_RATE = 20.0
        self.VEL_RATE = 0.5

        self.vel = 0.0
        self.angle = 0.0

        # Hard coded cart position
        self.x = 82.23206329345703
        self.y = 132.16149291992187
        self.phi = 0.0

        self.prev_time = time.time()
        self.seen_vel = False

        # ROS2 publishers

        self.pose_pub = self.create_publisher(PoseStamped, "/limited_pose", 10)
        self.vel_pub = self.create_publisher(Float32, "/estimated_vel_mps", 10)
        self.local_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/pcl_pose", 10
        )

        # ROS2 subscribers

        self.planned_motion_subscriber = self.create_subscription(
            VelAngle, "/nav_cmd", self.vel_angle_planned_callback, 10
        )
        self.initial_pose = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initial_pose_callback, 10
        )

        self.timer = self.create_timer(1.0 / self.NODE_RATE, self.timer_callback)

    def initial_pose_callback(self, pose_msg):
        """Callback responsible for getting the initial pose"""
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y
        self.phi = euler_from_quaternion(
            [
                pose_msg.pose.pose.orientation.x,
                pose_msg.pose.pose.orientation.y,
                pose_msg.pose.pose.orientation.z,
                pose_msg.pose.pose.orientation.w,
            ]
        )[2]
        steering_position_calc.prev_phi = self.phi

    def vel_angle_planned_callback(self, planned_vel_angle):
        """
        Callback method to get the target velocity and angle.
        This is achieved by using the subscription we created in init to recieved a
        message (planned_vel_angle) and set the appropriate fields to make the cart drive/turn.
        """

        self.vel = planned_vel_angle.vel

        # Only turn at a max rate
        if planned_vel_angle.angle > self.angle:
            self.angle = min(planned_vel_angle.angle, self.angle + self.STEER_RATE)
        else:
            self.angle = max(planned_vel_angle.angle, self.angle - self.STEER_RATE)

        if self.vel < 0.0:
            # indicates an obstacle, so stop
            self.vel = 0.0
        # set prev_vel for the first calculation
        if not self.seen_vel:
            self.prev_time = time.time()
            self.seen_vel = True

    def timer_callback(self):
        """Main loop timer for updating motor's instructions."""

        self.calculate_endpoint()

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = self.x
        pose.pose.pose.position.y = self.y
        x, y, z, w = quaternion_from_euler(0.0, 0.0, self.phi)
        pose.pose.pose.orientation.x = x
        pose.pose.pose.orientation.y = y
        pose.pose.pose.orientation.z = z
        pose.pose.pose.orientation.w = w

        rpose = PoseStamped()
        rpose.header = pose.header
        rpose.pose = pose.pose.pose

        self.pose_pub.publish(rpose)
        self.local_pose_pub.publish(pose)

        vel = Float32()
        vel.data = self.vel
        self.vel_pub.publish(vel)

    def calculate_endpoint(self):
        """The endpoint for processing and sending instructions to the arduino controller."""

        cur_time = time.time()
        delta_time = cur_time - self.prev_time
        self.prev_time = cur_time

        self.x, self.y, self.phi = steering_position_calc.calc_new_pos(
            delta_time, self.x, self.y, self.vel, self.angle
        )

        # Uneeded unless testing
        # self.get_logger().info(
        #     f"x:{self.x}, y:{self.y}, vel:{self.vel}, steer_angle:{self.angle}, phi:{self.phi}, delta_time:{delta_time}"
        # )


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = SimulatedMotor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
