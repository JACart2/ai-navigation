#!/usr/bin/env python
"""
This is the ROS 2 node that handles the local planning for the JACART.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import time
import numpy as np
import math
from navigation import pure_pursuit, cubic_spline_planner

# ROS based imports
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from nav_msgs.msg import Path
from navigation_interface.msg import (
    LocalPointsArray,
    VehicleState,
    Stop,
    WaypointsArray,
)

# We need to figure out how they are using VelAngle so we can use VelAngle/Vel
# Also the respecitive methods need to be ported over.
from motor_control_interface.msg import Vel, VelAngle

from std_msgs.msg import Float32, String, UInt64, Header
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, Pose, Twist
from visualization_msgs.msg import Marker
import tf_transformations as tf


class LocalPlanner(rclpy.node.Node):
    """ROS2 node that handles local pathing and obstacle avoidance."""

    def __init__(self):
        super().__init__("local_planner")

        # driving constants
        self.METERS = 10.0
        self.SECONDS = 3.6

        # driving variables
        self.tar_speed = self.METERS / self.SECONDS # Target speed?
        self.raw_speed = 0
        self.cur_speed = 0 # Real estiamted speed?

        self.new_path = False
        self.path_valid = False
        self.local_points = []
        self.poll_sample = 0

        self.current_state = VehicleState()
        self.stop_requests = {}

        # ros variables
        self.cur_pose = Pose()  # current position in local coordinates
        self.cur_twist = Twist()  # current velocity (linear x value)

        ## subscribers
        # The points to use for a path coming from global planner
        self.global_path_sub = self.create_subscriber(
            LocalPointsArray, "/global_path", self.global_path_cb, 10
        )

        # The linear and angular velocity of the cart from NDT Matching
        self.twist_sub = self.create_subscriber(
            TwistStamped, "/estimate_twist", self.twist_cb, 10
        )

        # The position of the cart from NDT Matching
        self.pose_sub = self.create_subscriber(
            PoseStamped, "/ndt_pose", self.pose_cb, 10
        )

        # Current speed of the cart in M/s
        self.speed_sub = self.create_subscriber(
            Float32, "/estimated_vel_mps", self.speed_cb, 10
        )

        # Stop requests
        self.stop_sub = self.create_subscriber(Stop, "/stop", self.stop_cb, 10)

        # Change speed
        self.speed_req_sub = self.create_subscriber(
            Float32, "/speed", self.speed_req_cb, 10
        )

        ## Publisher
        # Share the current status of the vehicle's state
        self.vehicle_state_pub = self.create_publisher(
            VehicleState, "/vehicle_state", 10
        )

        # Send out speed and steering requests to motor endpoint
        self.motion_pub = self.create_publisher(VelAngle, "/nav_cmd", 10)

        # Send out velocity estimate to motor endpoint
        # FIXME - Update with correct topic name
        self.vel_state_pub = self.create_publisher(Vel, "/nav_vel", 10)

        # Publish points on the map in rviz
        self.points_pub = self.create_publisher(Path, "/points", 10)

        # Publish the cubic spline path in rviz
        self.path_pub = self.create_publisher(Path, "/path", 10)

        # Publish the next navigating point in the path
        self.target_pub = self.create_publisher(Marker, "/target_point", 10)

        # Publish the current requested steering angle
        self.target_twist_pub = self.create_publisher(Marker, "/target_twist", 10)

        # Publish status update for the server
        self.arrived_pub = self.create_publisher(String, "/arrived", 10)

        # Steering angle PieChart display
        self.steering_pub = self.create_publisher(Float32, "/steering_angle", 10)

        # Publish the ETA
        self.eta_pub = self.create_publisher(UInt64, "/eta", 10)

        ## Timers
        # Calculate ETA
        self.eta_timer = self.create_timer(1, self.calc_eta)

        # Main loop
        self.timer = self.create_timer(0.5, self.timer_cb)

    def timer_cb(self):
        if self.new_path:
            self.path_valid = True
            self.new_path = False
            self.create_path()

    def twist_cb(self, msg):
        self.cur_twist = msg.twist

    def pose_cb(self, msg):
        self.cur_pose = msg.pose

    def stop_cb(self, msg):
        self.stop_requests[str[msg.sender_id.data].lower()] = [msg.stop, msg.distance]
        self.log(f'{str(msg.sender_id.data).lower()} requested stop: {str(msg.stop)} with distance {str(msg.distance)}')

    def speed_cb(self, msg):
        self.tar_speed = msg.data / self.SECONDS
        self.log(f'Speed changed to {str(self.tar_speed)}')

    def vel_cb(self, msg):
        if msg.data < 1.0:
            self.cur_speed = 1.8  # Magic number :)
        else:
            self.poll_sample += 1
            self.raw_speed += msg.data
            if self.poll_sample >= 5:
                self.cur_speed = self.raw_speed / self.poll_sample
                self.raw_speed = 0
                self.poll_sample = 0

    def global_path_cb(self, msg):
        self.local_points = []
        for local_point in msg.localpoints:
            self.local_points.append(local_point.position)

        self.path_valid = False
        self.new_path = True
        self.log(f'Path received: {str(msg)}')

    def create_path(self):
        """ Creates a path for the cart with a set of local_points
        Adds 15 more points between the google points
        Intermediate points are added for a better fitting spline
        """
        # Increase the "resolution" of the path with 15 intermediate points
        local_points_plus = self.local_points # geometry_util.add_intermediate_points(self.local_points, 15.0)
        # TODO - decipher the comment above

        ax = []
        ay = []

        # Create a Path object for displaying the raw path (no spline) in RViz
        display_points = Path()
        display_points.header = Header()
        display_points.header.frame_id = '/map'

        # Set the beginning of the navigation the first point
        last_index = 0
        target_ind = 0

        # Creates a list of the x's and y's to be used when calculating the spline
        for p in local_points_plus:
            display_points.poses.append(create_pose_stamped(p))
            ax.append(p.x)
            ay.append(p.y)

        self.points_pub.publish(display_points)

        # If the path doesn't have any successive points to navigate through, don't try
        if len(ax) > 2:
            # Create a cubic spline from the raw path
            cx, cy, cyaw, ck, cs = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

            # Create Path object which displays the cubic spline in RViz
            path = Path()
            path.header = Header()
            path.header.frame_id = '/map'

            # Add cubic spline points to path
            for i in range(0, len(cx)):
                curve_point = Point()
                curve_point.x = cx[i]
                curve_point.y = cy[i]
                path.poses.append(create_pose_stamped(curve_point))
            
            self.path_pub.publish(path)

            # Set the current state of the cart to navigating
            self.current_state = VehicleState()
            self.current_state.is_navigating = True
            self.vehicle_state_pub.publish(self.current_state)

            target_speed = self.tar_speed

            # initial state
            pose = self.cur_pose
            twist = self.cur_twist

            quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            angles = tf.euler_from_quaternion(quat)
            initial_v = twist.linear.x

            #??? TODO state has to be where we start
            state = State(x=pose.position.x, y=pose.position.y, yaw=angles[2], v=initial_v)

            # last_index represents the last point in the cubic spline, the destination
            last_index = len(cx) - 1
            time = 0.0
            x = [state.x]
            y = [state.y]
            yaw = [state.yaw]
            v = [state.v]
            t = [0.0]
            target_ind = pure_pursuit.calc_target_index(state, cx, cy, 0)

            # Publish the ETA to the destination before we get started
            self.calc_eta(None)
            rate = 1.0 / 30.0 # 30 cycles per second

            # TODO - Can we make this a timer that gets called then destoryed?

            # Continue to loop while we have not hit the target destination, and the path is still valid
            while last_index > target_ind and self.path_valid:
                target_speed = self.global_speed            
                ai = target_speed#pure_pursuit.PIDControl(target_speed, state.v)
                di, target_ind = pure_pursuit.pure_pursuit_control(state, cx, cy, target_ind)

                #publish our desired position
                mkr = create_marker(cx[target_ind], cy[target_ind], '/map')
                self.target_pub.publish(mkr)

                # Arrow that represents steering angle
                arrow = create_marker(0, 0, '/base_link')
                arrow.type = 0 #arrow
                arrow.scale.x = 2.0
                arrow.scale.y = 1.0
                arrow.scale.z = 1.0
                arrow.color.r = 1.0
                arrow.color.g = 0.0
                arrow.color.b = 0.0

                quater = tf.quaternion_from_euler(0, 0, di)
                arrow.pose.orientation.x = quater[0]
                arrow.pose.orientation.y = quater[1]
                arrow.pose.orientation.z = quater[2]
                arrow.pose.orientation.w = quater[3]
                self.target_twist_pub.publish(arrow)

                state = self.update(state, ai, di)

                x.append(state.x)
                y.append(state.y)
                yaw.append(state.yaw)
                v.append(state.v)
                t.append(time)
                # sleep(rate)
                time.sleep(rate)
        else:
            self.path_valid = False
            self.log_header("It appears the cart is already at the destination")

        #Check if we've reached the destination, if so we should change the cart state to finished
        self.log("Done navigating")
        self.current_state = VehicleState()
        self.current_state.is_navigating = False
        self.current_state.reached_destination = True
        notify_server = String()

        # Let operator know why current path has stopped
        if self.path_valid:
            self.log("Reached Destination succesfully without interruption")
            self.arrived_pub.publish(notify_server)
        else:
            self.log("Already at destination, or there may be no path to get to the destination or navigation was interrupted.")
        
        # Update the internal state of the vehicle
        self.vehicle_state_pub.publish(self.current_state)
        plan_msg = VelAngle()
        plan_msg.vel = 0
        plan_msg.angle = 0

        cur_msg = Vel()
        cur_msg.vel = 0

        self.vel_state_pub.publish(cur_msg)
        self.motion_pub.publish(plan_msg)


    def update(self, state, a, delta):
        """ Updates the carts position by a given state and delta
        """
        pose = self.cur_pose
        twist = self.cur_twist
        cur_speed = twist.linear.x

        plan_msg = VelAngle()
        cur_msg = Vel()
        if self.debug:
            self.delay_print -= 1
            if self.delay_print <= 0:
                self.delay_print = 50
                self.log(f'Target Speed: {str(a)}')
                self.log(f'Current Speed: {str(cur_speed)}')
        plan_msg.vel = a # Speed we want from pure pursuit controller
        plan_msg.angle = (delta * 180) / math.pi
        cur_msg.vel = cur_speed

        display_angle = Float32()
        display_angle.data = plan_msg.angle
        
        self.steering_pub.publish(display_angle)

        # Check if any node wants us to stop
        for x in self.stop_requests.values():
            if x[0]: # stop requested
                plan_msg.vel = 0
                if x[1] > 0: # obstacle distance is given
                    plan_msg.vel = -x[1]

        self.vel_state_pub.publish(cur_msg)
        self.motion_pub.publish(plan_msg)

        state.x = pose.position.x
        state.y = pose.position.y

        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        angles = tf.euler_from_quaternion(quat)

        state.yaw = angles[2]

        state.v = twist.linear.x
        
        return state
        

    def calc_eta(self, event):
        """ Calculates the Estimated Time of Arrival to the destination
        """
        # Attempt an update only while driving
        if self.current_state.is_navigating:
            # Where are we at and how much further must we go
            current_node = self.get_closest_point(self.cur_pose.position.x, self.cur_pose.position.y)
            distance_remaining = self.calc_trip_dist(self.local_points, current_node)

            # Remaining time in seconds
            remaining_time = distance_remaining / self.cur_speed
            eta_msg = UInt64()

            # Calculate the ETA to the end
            arrival_time = time.time() + remaining_time

            # Convert the time to milliseconds
            eta_msg.data = int(arrival_time * (1000))
            self.eta_pub.publish(eta_msg)

    def calc_trip_dist(self, points_list, start):
        """ Calculates the trip distance from the "start" index to the end of the "points_list"

        Args:
            points_list(List): The list of path points to calculate the distance of
            start(int): The index of which to start calculating the trip distance  
        """
        sum = 0
        for i in range(start, len(points_list) - 1):
            sum += self.calc_dist(points_list[i].x, points_list[i].y, points_list[i + 1].x, points_list[i + 1].y)
            prev_node = i
        
        return sum

    def get_closest_point(self, pos_x, pos_y):
        """ Get the closest point along the raw path from pos_x, pos_y

        Args:
            pos_x(float): The x position of search center
            pos_y(float): The y position of search center
        """
        min_node = 0
        min_dist = float("inf")
        for i in range(len(self.local_points)):
            dist = self.calc_dist(pos_x, pos_y, self.local_points[i].x, self.local_points[i].y)
            if dist < min_dist:
                min_dist = dist
                min_node = i

    def calc_dist(self, x1, y1, x2, y2):
        return math.sqrt(((x2 - x1) ** 2) + ((y2 - y1) ** 2))

    def log(self, log):
        self.get_logger().info(f'{log}')

    def log_header(self, log):
        self.get_logger().info(f'{'#' * 20}\n{log}\n{'#' * 20}')

## Helper class and methods
class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def create_pose_stamped(point):
    stamped = PoseStamped()
    stamped.header = Header()
    stamped.header.frame_id = '/map'
    stamped.pose.position = point
    return stamped

def create_marker(x, y, frame_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = time.time()
    marker.ns = "my_namespace"
    marker.id = 0
    marker.type = 1 #cube
    marker.action = 0 #add
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    return marker

def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = LocalPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
