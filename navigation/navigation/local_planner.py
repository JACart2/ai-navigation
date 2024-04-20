#!/usr/bin/env python
"""
This is the ROS 2 node that handles the local planning for the JACART.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import math
from navigation import pure_pursuit, cubic_spline_planner

# ROS based imports
import rclpy
from nav_msgs.msg import Path
from navigation_interface.msg import (
    LocalPointsArray,
    VehicleState,
    Stop,
)
from visualization_msgs.msg import Marker
from motor_control_interface.msg import VelAngle
from std_msgs.msg import Float32, String, UInt64, Header
from geometry_msgs.msg import (
    PoseStamped,
    Point,
    TwistStamped,
    Pose,
    PoseWithCovarianceStamped,
)
from visualization_msgs.msg import Marker
import tf_transformations as tf
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly


class LocalPlanner(rclpy.node.Node):
    """ROS2 node that handles local pathing and obstacle avoidance."""

    def __init__(self):
        super().__init__("local_planner")

        # driving constants THIS USED TO BE 10 and 3.6 respectively
        self.METERS = 30.0
        self.SECONDS = 3.6

        # driving variables
        self.cur_vel = 0.0  # current linear velocity from localization
        self.tar_speed = self.METERS / self.SECONDS  # Target speed?

        self.cur_speed = 0  # Another estimate of speed used for eta calculations

        self.new_path = False
        self.path_valid = False
        self.local_points = []

        self.current_state = VehicleState()
        self.stop_requests = {}

        # ros variables
        self.cur_pose = Pose()  # current position in local coordinates

        ## subscribers
        # The points to use for a path coming from global planner
        self.global_path_sub = self.create_subscription(
            LocalPointsArray, "/global_path", self.global_path_cb, 10
        )

        # The linear and angular velocity of the cart from NDT Matching
        self.twist_sub = self.create_subscription(
            TwistStamped, "/estimate_twist", self.twist_cb, 10
        )

        # The position of the cart from NDT Matching

        # The topic name used to be ndt_pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/pcl_pose", self.pose_cb, 10
        )

        # Current speed of the cart in M/s
        self.speed_sub = self.create_subscription(
            Float32, "/estimated_vel_mps", self.speed_cb, 10
        )

        # Stop requests
        self.stop_sub = self.create_subscription(Stop, "/stop", self.stop_cb, 10)

        # Change speed
        self.speed_req_sub = self.create_subscription(
            Float32, "/speed", self.tar_speed_cb, 10
        )

        ## Publishers
        # Share the current status of the vehicle's state
        self.vehicle_state_pub = self.create_publisher(
            VehicleState, "/vehicle_state", 10
        )

        # Send out speed and steering requests to motor endpoint
        self.motion_pub = self.create_publisher(VelAngle, "/nav_cmd", 10)

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

        # Publish the projected turning angle and path
        self.projection_pub = self.create_publisher(Marker, "/projected_path", 10)

        ## Timers
        # Calculate ETA
        self.eta_timer = self.create_timer(1, self.calc_eta)

        # Main loop
        self.timer = self.create_timer(0.05, self.timer_cb)

        # plan_msg = VelAngle()
        # plan_msg.vel = 5.0
        # plan_msg.angle = 5.0

        # self.motion_pub.publish(plan_msg)

    def timer_cb(self):
        """Time callback responsible creating a path. Basically used as a second update function
        If path is already created the create_path method does updating of the carts state
        """
        self.create_path()

    def twist_cb(self, msg):
        """Getting current believed cart vel"""
        self.cur_vel = msg.twist.linear.x

    def pose_cb(self, msg):
        """Getting current believed cart position"""
        self.cur_pose = msg.pose.pose

    def stop_cb(self, msg):
        self.stop_requests[str[msg.sender_id.data].lower()] = [msg.stop, msg.distance]
        self.log(
            f"{str(msg.sender_id.data).lower()} requested stop: {str(msg.stop)} with distance {str(msg.distance)}"
        )

    def tar_speed_cb(self, msg):
        self.tar_speed = msg.data / self.SECONDS
        self.log(f"Speed changed to {str(self.tar_speed)}")

    def speed_cb(self, msg):
        if msg.data < 1.0:
            self.cur_speed = 1.8  # Magic number :)
        else:  # Rolling average for speed estimates, to smooth the changes
            self.cur_speed = 0.8 * self.cur_speed + 0.2 * msg.data

    def global_path_cb(self, msg):
        """This gets the whole path from global planner
        and puts the points inside of the local points array"""
        self.local_points = []
        for local_point in msg.localpoints:
            self.local_points.append(local_point.position)

        self.path_valid = False
        self.new_path = True
        self.log(f"Path received: {str(msg)}")

    def create_path(self):
        """Creates a path for the cart with a set of local_points
        Adds 15 more points between the google points
        Intermediate points are added for a better fitting spline
        """
        if self.new_path:
            self.path_valid = True
            self.new_path = False
            # Increase the "resolution" of the path with 15 intermediate points
            local_points_plus = (
                self.local_points
            )  # geometry_util.add_intermediate_points(self.local_points, 15.0)
            # TODO - decipher the comment above

            ax = []
            ay = []

            # Create a Path object for displaying the raw path (no spline) in RViz
            display_points = Path()
            display_points.header = Header()
            display_points.header.frame_id = "/map"

            # Set the beginning of the navigation the first point
            self.last_index = 0
            self.target_ind = 0

            # Creates a list of the x's and y's to be used when calculating the spline
            for p in local_points_plus:
                display_points.poses.append(create_pose_stamped(p))
                ax.append(p.x)
                ay.append(p.y)

            self.points_pub.publish(display_points)

            # If the path doesn't have any successive points to navigate through, don't try
            if len(ax) > 2:
                # Create a cubic spline from the raw path
                self.cx, self.cy, self.cyaw, self.ck, self.cs = (
                    cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
                )

                # Create Path object which displays the cubic spline in RViz
                path = Path()
                path.header = Header()
                path.header.frame_id = "/map"

                # Add cubic spline points to path
                for i in range(0, len(self.cx)):
                    curve_point = Point()
                    curve_point.x = self.cx[i]
                    curve_point.y = self.cy[i]
                    path.poses.append(create_pose_stamped(curve_point))

                self.path_pub.publish(path)

                # Set the current state of the cart to navigating
                self.current_state = VehicleState()
                self.current_state.is_navigating = True
                self.vehicle_state_pub.publish(self.current_state)

                # target_speed = self.tar_speed

                # initial state
                pose = self.cur_pose

                quat = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
                angles = tf.euler_from_quaternion(quat)

                # ??? TODO state has to be where we start
                self.state = pure_pursuit.State(
                    x=pose.position.x, y=pose.position.y, yaw=angles[2], v=self.cur_vel
                )

                # last_index represents the last point in the cubic spline, the destination
                self.last_index = len(self.cx) - 1
                self.timedelta = 0.0
                self.x = [self.state.x]
                self.y = [self.state.y]
                self.yaw = [self.state.yaw]
                self.v = [self.state.v]
                self.t = [0.0]
                self.target_ind = pure_pursuit.calc_target_index(
                    self.state, self.cx, self.cy, 0
                )

                # Publish the ETA to the destination before we get started
                self.calc_eta()
                rate = 1.0 / 30.0  # 30 cycles per second

                # TODO - Can we make this a timer that gets called then destoryed?

            else:
                self.path_valid = False
                self.log_header("It appears the cart is already at the destination")

        if self.current_state.is_navigating:
            # Continue to loop while we have not hit the target destination, and the path is still valid
            if self.last_index > self.target_ind and self.path_valid:

                # Uneeded unless testing (floods the terminal with messages when active)
                # self.get_logger().info(
                #     f"Cur pos: {self.cur_pose.position},   Tar pos: {self.local_points[-1]}"
                # )
                target_speed = self.tar_speed
                ai = target_speed  # pure_pursuit.PIDControl(target_speed, state.v)
                di, self.target_ind = pure_pursuit.pure_pursuit_control(
                    self.state, self.cx, self.cy, self.target_ind
                )

                # publish our desired position
                mkr = create_marker(
                    self.cx[self.target_ind], self.cy[self.target_ind], "/map"
                )
                self.target_pub.publish(mkr)

                # Arrow that represents steering angle
                arrow = create_marker(0.0, 0.0, "/base_link")
                arrow.type = 0  # arrow
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

                # self.log_header("update is about to be called")
                self.state = self.update(self.state, ai, di)

                self.x.append(self.state.x)
                self.y.append(self.state.y)
                self.yaw.append(self.state.yaw)
                self.v.append(self.state.v)
                self.t.append(self.timedelta)
            else:
                # Check if we've reached the destination, if so we should change the cart state to finished
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
                    self.log(
                        "Already at destination, or there may be no path to get to the destination or navigation was interrupted."
                    )

                # Update the internal state of the vehicle
                self.vehicle_state_pub.publish(self.current_state)
                plan_msg = VelAngle()
                plan_msg.vel = 0.0
                plan_msg.angle = 0.0

                self.motion_pub.publish(plan_msg)

    def update(self, state, a, delta):
        """Updates the carts position by a given state and delta"""

        # self.log_header("update is being called")
        pose = self.cur_pose
        cur_speed = self.cur_vel

        plan_msg = VelAngle()
        plan_msg.vel = a  # Speed we want from pure pursuit controller
        plan_msg.angle = (delta * 180) / math.pi

        display_angle = Float32()
        display_angle.data = plan_msg.angle

        self.steering_pub.publish(display_angle)

        # Check if any node wants us to stop
        for x in self.stop_requests.values():
            if x[0]:  # stop requested
                plan_msg.vel = 0.0
                if x[1] > 0:  # obstacle distance is given
                    plan_msg.vel = -x[1]

        self.motion_pub.publish(plan_msg)

        state.x = pose.position.x
        state.y = pose.position.y

        quat = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        angles = tf.euler_from_quaternion(quat)

        state.yaw = angles[2]

        state.v = self.cur_vel

        # Display lines for projected path
        left, right = self.create_projected_lines(
            state.yaw, plan_msg.angle, pose.position
        )
        self.projection_pub.publish(left)
        self.projection_pub.publish(right)

        return state

    def create_projected_lines(self, angle, steer, pos):
        left = Marker()
        new_point = Pose()
        new_point.position = pos
        left.pose = new_point
        left.header.frame_id = "map"
        left.id = 0
        left.scale.x = 0.1
        left.scale.y = 0.1
        left.scale.z = 0.1
        left.color.r = 0.0
        left.color.g = 2.0
        left.color.b = 0.0
        left.color.a = 1.0
        left.type = 4
        left.action = 0

        right = Marker()
        new_point = Pose()
        new_point.position = pos
        right.pose = new_point
        right.header.frame_id = "map"
        right.id = 1
        right.scale.x = 0.1
        right.scale.y = 0.1
        right.scale.z = 0.1
        right.color.r = 0.0
        right.color.g = 2.0
        right.color.b = 0.0
        right.color.a = 1.0
        right.type = 4
        right.action = 0

        (left.points, right.points) = self.project_points(angle, math.radians(steer))
        return (left, right)

    def project_points(self, angle, steer_r):
        left_arr = []
        right_arr = []
        dist = 0.5
        for i in range(30):
            theta = angle + steer_r * i * 0.3 / 4
            x = math.cos(theta) * 0.3 * i
            y = math.sin(theta) * 0.3 * i

            left_p = Point()
            left_p.x = x + math.cos(theta + math.pi / 2) * dist
            left_p.y = y + math.sin(theta + math.pi / 2) * dist
            left_arr.append(left_p)
            right_p = Point()
            right_p.x = x + math.cos(theta - math.pi / 2) * dist
            right_p.y = y + math.sin(theta - math.pi / 2) * dist
            right_arr.append(right_p)

        return (left_arr, right_arr)

    def calc_eta(self):
        """Calculates the Estimated Time of Arrival to the destination"""
        # Attempt an update only while driving
        if self.current_state.is_navigating:
            # Where are we at and how much further must we go
            current_node = self.get_closest_point(
                self.cur_pose.position.x, self.cur_pose.position.y
            )

            # distance_remaining = self.calc_trip_dist(self.local_points, current_node)

            # # Remaining time in seconds
            # remaining_time = distance_remaining / self.cur_speed
            eta_msg = UInt64()

            # # Calculate the ETA to the end
            # arrival_time = time.time() + remaining_time

            # # Convert the time to milliseconds
            # eta_msg.data = int(arrival_time * (1000))
            eta_msg.data = 0
            self.eta_pub.publish(eta_msg)

    def calc_trip_dist(self, points_list, start):
        """Calculates the trip distance from the "start" index to the end of the "points_list"

        Args:
            points_list(List): The list of path points to calculate the distance of
            start(int): The index of which to start calculating the trip distance
        """
        sum = 0
        for i in range(start, len(points_list) - 1):
            sum += self.calc_dist(
                points_list[i].x,
                points_list[i].y,
                points_list[i + 1].x,
                points_list[i + 1].y,
            )
            prev_node = i

        return sum

    def get_closest_point(self, pos_x, pos_y):
        """Get the closest point along the raw path from pos_x, pos_y

        Args:
            pos_x(float): The x position of search center
            pos_y(float): The y position of search center
        """
        min_node = 0
        min_dist = float("inf")
        for i in range(len(self.local_points)):
            dist = self.calc_dist(
                pos_x, pos_y, self.local_points[i].x, self.local_points[i].y
            )
            if dist < min_dist:
                min_dist = dist
                min_node = i

    def calc_dist(self, x1, y1, x2, y2):
        return math.sqrt(((x2 - x1) ** 2) + ((y2 - y1) ** 2))

    def log(self, log):
        self.get_logger().info(f"{log}")

    def log_header(self, log):
        # self.get_logger().info(f'{'#' * 20}\n{log}\n{'#' * 20}')
        self.log(log)


def create_pose_stamped(point):
    stamped = PoseStamped()
    stamped.header = Header()
    stamped.header.frame_id = "/map"
    stamped.pose.position = point
    return stamped


def create_marker(x, y, frame_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    # marker.header.stamp = Time(seconds=time.time())
    marker.ns = "my_namespace"
    marker.id = 0
    marker.type = 1  # cube
    marker.action = 0  # add
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.0

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
