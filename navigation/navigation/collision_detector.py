#!/usr/bin/env python

"""
Collision Detector node, takes in obstacle data from the obstacle detector endpoint and makes decisions based on the obstacles.

Important Notes Before Continuing:
-Experimental at the moment, very 'prototypey'
-Math is performed in relation to the base_link frame,
    which based on the robot description is rotated half pi radians (counter-clockwise given conventions)
"""

# import math
# import tf
# import numpy as np
# import time

# from std_msgs.msg import Header, Float32
# from visualization_msgs.msg import MarkerArray
# from navigation_msgs.msg import ObstacleArray, Obstacle, Stop, VelAngle
# from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32, Point
# from visualization_msgs.msg import Marker

import math
import time
import numpy as np

import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Header, Float32
from visualization_msgs.msg import MarkerArray
from navigation_interface.msg import ObstacleArray, Obstacle, Stop
from motor_control_interface.msg import VelAngle
from geometry_msgs.msg import (
    Point,
    TwistStamped,
)
from visualization_msgs.msg import Marker
import tf_transformations as tf


class CollisionDetector(rclpy.node.Node):

    def __init__(self):
        super().__init__("collision_detector")

        # self.t = tf.TransformListener()

        self.stop = False
        self.cur_obstacles = None
        # self.cur_pos = None
        self.target_pos = None
        self.prev_distance = None
        self.prev_time = None
        self.prev_obstacle_speed = 0
        self.obstacle_detected = False

        self.resume_confid = 0

        self.lookahead_local = None

        self.requested_steering_angle = 0

        # Vehicle Kinematics
        self.declare_parameter("vehicle_width", 1.1938)
        self.declare_parameter("vehicle_length", 2.4003)
        self.declare_parameter("wheel_base", 2.4003)
        self.declare_parameter("front_axle_track", 0.9017)
        self.declare_parameter("rear_axle_track", 0.9652)
        self.declare_parameter("tire_width", 0.2159)
        self.VEHICLE_WIDTH = (
            self.get_parameter("vehicle_width").get_parameter_value().double_value
        )
        self.VEHICLE_LENGTH = (
            self.get_parameter("vehicle_length").get_parameter_value().double_value
        )
        self.WHEEL_BASE = (
            self.get_parameter("wheel_base").get_parameter_value().double_value
        )
        self.FRONT_AXLE_TRACK = (
            self.get_parameter("front_axle_track").get_parameter_value().double_value
        )
        self.REAR_AXLE_TRACK = (
            self.get_parameter("rear_axle_track").get_parameter_value().double_value
        )
        self.TIRE_WIDTH = (
            self.get_parameter("tire_width").get_parameter_value().double_value
        )

        self.cur_speed = 0.1
        self.stopped = False
        self.cleared_confidence = 0

        # Vehicle Corners (Wheel Positions)
        self.front_right_corner = [0.0, 0.0]
        self.front_left_corner = [0.0, 0.0]
        self.rear_left_corner = [0.0, 0.0]
        self.rear_right_corner = [0.0, 0.0]
        self.front_axle_center = [0.0, 0.0]

        # Collision Bounds
        self.inner_radius = 0
        self.outer_radius = 0
        self.circle_center = [0, 0]
        self.right_turn = False

        # Minimum allowable distance from front of cart to intercept obstacle before emergency stopping
        factor = 1.25
        # self.declare_parameter("min_obstacle_dist", 2.5)
        # self.declare_parameter("min_obstacle_time", 2.5)
        self.declare_parameter("safe_obstacle_dist", 6 * factor)
        self.declare_parameter("safe_obstacle_time", 2 * factor)

        # self.MIN_OBSTACLE_DIST = (
        #     self.get_paramater("min_obstacle_dist").get_paramter_value().float_value
        # )
        # # Minimum allowable transit time to an obstacle allowed before emergency stopping
        # self.MIN_OBSTACLE_TIME = (
        #     self.get_parameter("min_obstacle_time").get_paramter_value().float_value
        # )

        self.SAFE_OBSTACLE_DIST = (
            self.get_parameter("safe_obstacle_dist").get_parameter_value().double_value
        )
        self.SAFE_OBSTACLE_TIME = (
            self.get_parameter("safe_obstacle_time").get_parameter_value().double_value
        )

        self.obstacle_sub = self.create_subscription(
            ObstacleArray, "/obstacles", self.obstacle_callback, 10
        )

        self.angle_sub = self.create_subscription(
            VelAngle, "/nav_cmd", self.angle_callback, 10
        )

        self.cur_speed_sub = self.create_subscription(
            TwistStamped, "/estimate_twist", self.speed_callback, 10
        )  # The original didn't have the callback properly implemented

        self.display_pub = self.create_publisher(Marker, "/corner_display", 10)
        self.stop_pub = self.create_publisher(Stop, "/stop", 10)
        self.display_boundary_pub = self.create_publisher(Marker, "/boundaries", 10)
        # change to 10 if 100 is breaking
        self.display_array = self.create_publisher(
            MarkerArray, "/boundaries_array", 100
        )
        self.collision_pub = self.create_publisher(MarkerArray, "/collision_pub", 100)

        self.speed_pub = self.create_publisher(Float32, "/speed", 10)

        self.obtain_corners()

        # Run collision detection 30 times a second
        self.timer = self.create_timer(1 / 30.0, self.timer_callback)

    def timer_callback(self):
        self.calc_arcs(self.requested_steering_angle)
        if self.cur_obstacles is not None:
            # Calculate the inner and outer radius for arcs and the center of their circ
            self.determine_collision()

    def angle_callback(self, msg):
        self.requested_steering_angle = msg.angle

    def speed_callback(self, msg):
        self.cur_speed = msg.twist.linear.x
        if self.cur_speed < 0.1:
            self.cur_speed = 0.1

    def obtain_corners(self):
        """This could be simplified to simply obtaining the position using the tf library
        Point of experimenting in the next couple days, not sure which is more flexible yet.
        """

        self.rear_left_corner = [
            -(self.WHEEL_BASE / 2),
            ((self.FRONT_AXLE_TRACK / 2) + (self.TIRE_WIDTH / 2)),
        ]
        self.rear_right_corner = [
            -(self.WHEEL_BASE / 2),
            -((self.FRONT_AXLE_TRACK / 2) + (self.TIRE_WIDTH / 2)),
        ]
        self.front_right_corner = [
            (self.WHEEL_BASE / 2),
            -((self.REAR_AXLE_TRACK / 2) + (self.TIRE_WIDTH / 2)),
        ]
        self.front_left_corner = [
            (self.WHEEL_BASE / 2),
            ((self.REAR_AXLE_TRACK / 2) + (self.TIRE_WIDTH / 2)),
        ]
        self.front_axle_center = [(self.WHEEL_BASE / 2), 0.0]

    def calc_arcs(self, steering_angle):
        """Calculates the Inner turn radius arc and outer turn radius arc and the
        center point the two arcs share.

        Args:
            steering_angle(Float): Steering angle of the cart in degrees
        """
        # Used for RViz display
        marker_array = MarkerArray()

        # If steering angle is small/near 0 apply a small angle in the same direction
        if steering_angle < 1 and steering_angle >= 0:
            steering_angle = 1
        elif steering_angle > -1 and steering_angle <= 0:
            steering_angle = -1

        steering_angle = math.radians(steering_angle)

        # Calculate inner radius, which is used to find the circle center for the arcs
        self.inner_radius = self.WHEEL_BASE / math.tan(steering_angle)

        # Turning left
        if steering_angle > 0:
            # calculate outer arc radius and center of the circle for both arcs
            self.outer_radius = self.inner_radius + self.VEHICLE_WIDTH
            self.circle_center = [
                self.rear_left_corner[0],
                self.rear_left_corner[1] + self.inner_radius,
            ]

            # Create markers for RViz display
            inner_arc = self.display_arc(self.circle_center, self.inner_radius, id=10)
            outer_arc = self.display_arc(self.circle_center, self.outer_radius, id=11)
            marker_array.markers.append(inner_arc)
            marker_array.markers.append(outer_arc)

            self.right_turn = False

        # Turning right
        elif steering_angle < 0:
            # calculate outer arc radius and center of the circle for both arcs
            self.outer_radius = self.inner_radius - self.VEHICLE_WIDTH
            self.circle_center = [
                self.rear_right_corner[0],
                self.rear_right_corner[1] + self.inner_radius,
            ]

            # Create markers for RViz display
            inner_arc = self.display_arc(
                self.circle_center, self.inner_radius, id=15, right_turn=True
            )
            outer_arc = self.display_arc(
                self.circle_center, self.outer_radius, id=18, right_turn=True
            )
            marker_array.markers.append(inner_arc)
            marker_array.markers.append(outer_arc)

            self.right_turn = True

        self.display_array.publish(marker_array)

    def determine_collision(self):
        """Simply determines if an obstacle is a potential collision"""
        # For displaying hazardous obstacles
        collision_array = MarkerArray()
        display = None

        # Control for undoing a stop
        clear_path = True

        cur_obstacle_list = self.cur_obstacles
        for obstacle in cur_obstacle_list:
            obstacle_size = 2 * obstacle.radius

            # The distance/radius of the circle center to the obstacle center
            circle_obstacle_dist = self.distance(
                obstacle.pos.point.x,
                obstacle.pos.point.y,
                self.circle_center[0],
                self.circle_center[1],
            )
            # Determine if the obstacle is within bounds of the inner and outer arc
            potential_collision = (
                (abs(self.inner_radius) - obstacle_size)
                < circle_obstacle_dist
                < (abs(self.outer_radius) + obstacle_size)
            )

            # FIXME need to determine if this needs a different frame or not.
            self.display_circle("/base_link", self.front_axle_center, 24, 2, z=1)

            if potential_collision:
                # Prepare an emergency stop message
                stop_msg = Stop()
                stop_msg.stop = True
                stop_msg.sender_id.data = "collision_detector"
                distance = self.distance(
                    obstacle.pos.point.x,
                    obstacle.pos.point.y,
                    self.front_axle_center[0],
                    self.front_axle_center[1],
                )
                impact_time = distance / self.cur_speed

                self.followable_obstacles = [
                    o for o in cur_obstacle_list if o.followable
                ]
                if self.followable_obstacles:
                    # Calculate distance from front of cart to obstacle
                    distance = self.distance(
                        self.followable_obstacles[0].pos.point.x,
                        self.followable_obstacles[0].pos.point.y,
                        # distance = self.distance(obstacle.pos.point.x, obstacle.pos.point.y,
                        self.front_axle_center[0],
                        self.front_axle_center[1],
                    )
                    # Calculate rough time to obstacle impact (seconds)
                    impact_time = distance / self.cur_speed

                    if self.prev_distance is not None and self.prev_time is not None:
                        obstacle_speed = (self.prev_distance - distance) / (
                            self.prev_time - time.time()
                        )
                        # rospy.logwarn("[collision_detector] ********* ********8Obstacle speed: " + str(obstacle_speed))
                        if (
                            obstacle_speed < 10
                            and obstacle_speed > -10
                            and obstacle_speed != 0
                            and abs(obstacle_speed - self.prev_obstacle_speed) < 2
                        ):
                            # rospy.logwarn("Obstacle speed: " + str(obstacle_speed))

                            self.prev_obstacle_speed = obstacle_speed

                            if not self.obstacle_detected:
                                new_speed = (3.23 * obstacle_speed) + 4.254
                                if new_speed < 3 and new_speed > 0:
                                    self.speed_pub.publish(math.ceil(new_speed) + 1)
                                    self.obstacle_detected = True

                    self.prev_distance = distance
                    self.prev_time = time.time()
                # if distance < self.min_obstacle_dist or impact_time < self.min_obstacle_time:
                if distance < (self.SAFE_OBSTACLE_DIST / 3) or impact_time < (
                    self.SAFE_OBSTACLE_TIME / 3
                ):
                    clear_path = False
                    self.cleared_confidence = 0
                    if not self.stopped:
                        self.stopped = True
                        stop_msg.distance = distance
                        self.stop_pub.publish(stop_msg)

                    # Show a red obstacle, an obstacle worth stopping for
                    display = self.show_colliding_obstacle(
                        obstacle.pos.point.x, obstacle.pos.point.y, color=0.0
                    )
                # elif distance < self.safe_obstacle_dist or impact_time < self.safe_obstacle_time:
                #     #Temporay code duplication
                #     clear_path = False
                #     self.cleared_confidence = 0
                #     if not self.stopped:
                #         self.stopped = True
                #         self.gentle_stop_pub.publish(stop_msg)
                #         rospy.logwarn("Requesting a slow stop due to possible collision")
                #     # Show a red obstacle, an obstacle worth stopping for
                #     display = self.show_colliding_obstacle(obstacle.pos.point.x, obstacle.pos.point.y, color=0.0)
                else:
                    # Show a yellow obstacle, obstacle that has potential
                    display = self.show_colliding_obstacle(
                        obstacle.pos.point.x, obstacle.pos.point.y
                    )
            else:
                if self.obstacle_detected:
                    self.resume_confid += 1
                    if self.resume_confid > 8:
                        self.obstacle_detected = False
                        self.speed_pub.publish(10)
                        self.resume_confid = 0

            if display is not None:
                collision_array.markers.append(display)

        # If a run detects a clear path, but we are still stopped. allow nav to continue, and we have confidence is clear (15 spins/3 seconds given rate of 5 hz)
        if clear_path:
            self.cleared_confidence += 1
            if self.stopped and self.cleared_confidence >= 15:
                self.cleared_confidence = 0
                self.stopped = False
                stop_msg = Stop()
                stop_msg.stop = False
                stop_msg.sender_id.data = "collision_detector"
                stop_msg.distance = -1.0
                self.stop_pub.publish(stop_msg)

        self.collision_pub.publish(collision_array)

    def show_colliding_obstacle(self, x, y, color=1.0):
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "/base_link"

        marker.type = marker.CUBE
        marker.action = 0

        marker.color.r = 1.0
        marker.color.g = color
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = Duration(seconds=0.1).to_msg()

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.2

        return marker

    def display_circle(self, frame, corner, id, radius, color=0.0, z=0.05):
        """Create circle marker for RViz"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = frame

        marker.ns = "Object_NS"
        marker.id = id
        marker.type = Marker.CYLINDER
        marker.action = 0
        marker.color.r = color
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.1
        marker.lifetime = Duration(seconds=0.1).to_msg()

        marker.pose.position.x = corner[0]
        marker.pose.position.y = corner[1]
        marker.pose.position.z = 1.0

        radius = float(radius)
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = float(z)

        self.display_pub.publish(marker)

    def obstacle_callback(self, msg):
        self.cur_obstacles = msg.obstacles

    # def position_callback(self, msg):
    #     self.cur_pos = msg.pose

    def display_arc(self, circle, radius, id, right_turn=False):
        """Create and return an arc for RViz using a Line Strip Marker

        Args:
            circle(Array): Tuple [x,y] of circle center
            radius(float): Radius for the circle
            id(Int): ID for the marker
            right_turn(Boolean): Whether the arc is being displayed for a right turn or not
        """
        bound_display = Marker()
        bound_display.header = Header()
        bound_display.id = id
        bound_display.lifetime = Duration(seconds=0.033).to_msg()
        bound_display.type = Marker.LINE_STRIP
        bound_display.header.frame_id = "/base_link"
        bound_display.scale.x = 0.2
        bound_display.color.r = 1.0
        bound_display.color.g = 1.0
        bound_display.color.b = 0.0
        bound_display.color.a = 1.0
        bound_display.action = Marker.ADD

        # Obtain the angle range necessary for a certain arc length in this case 20
        ang = 20 / (2 * radius)

        # Setup arc display range
        if right_turn:
            loop_range = np.arange((math.pi / 2) + ang, math.pi / 2, 0.10)
        else:
            loop_range = np.arange(-(math.pi / 2), (-(math.pi / 2)) + ang, 0.10)

        # Obtain the points along the arc on the circle
        for ang in loop_range:
            arc_point = Point()
            if not right_turn:
                arc_point.x = circle[0] + (radius * math.cos(ang))
                arc_point.y = circle[1] + (radius * math.sin(ang))
            else:
                arc_point.x = circle[0] - (radius * math.cos(ang))
                arc_point.y = circle[1] - (radius * math.sin(ang))

            bound_display.points.append(arc_point)

        return bound_display

    def distance(self, x1, y1, x2, y2):
        dX = (x2 - x1) ** 2
        dY = (y2 - y1) ** 2

        return math.sqrt(dX + dY)


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = CollisionDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
