#!/usr/bin/env python

import math
import rospy
import time
from navigation_msgs.msg import WaypointsArray, VelAngle, LocalPointsArray, VehicleState, Stop
from nav_msgs.msg import Path
from std_msgs.msg import Header, Float32, String, Int8, Bool, UInt64
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, Pose, Twist
from visualization_msgs.msg import Marker
import tf.transformations as tf

import cubic_spline_planner
import pure_pursuit

'''
This class contains the code to track and fit a path
for the cart to follow
'''
class LocalPlanner(object):
    def __init__(self):
        rospy.init_node('local_planner')

        self.debug = False
        #Our current velocity (linear x value)
        self.global_twist = Twist()
        
        #Our current position in local coordinates
        self.global_pose = Pose()

        self.meters = 10.0
        self.seconds = 3.6

        self.delay_print = 0

        self.global_speed = self.meters / self.seconds  # [m/s]

        # Speed before and after averaging
        self.raw_speed = 0
        self.cur_speed = 0

        self.new_path = False
        self.path_valid = False
        self.local_points = []
        self.poll_sample = 0
        self.stop_thresh = 5 #this is how many seconds an object is away

        self.current_state = VehicleState()

        # To allow other nodes to make stop requests mapping like so: Sender_ID : [stop(boolean), distance]
        # If distance == -1, there is no obstacle
        self.stop_requests = {}

        # The points to use for a path, typically coming from global planner                                
        self.local_sub = rospy.Subscriber('/global_path', LocalPointsArray,
                                            self.localpoints_callback, queue_size=10)

        # The Velocity and angular velocity of the cart coming from NDT Matching based on LiDAR                                      
        self.twist_sub = rospy.Subscriber('/estimate_twist', TwistStamped, self.twist_callback, queue_size = 10)

        # Get the current position of the cart from NDT Matching
        self.pose_sub = rospy.Subscriber('/ndt_pose', PoseStamped, self.pose_callback, queue_size = 10)

        # Current Velocity of cart in Meters per second
        self.speed_sub = rospy.Subscriber('/estimated_vel_mps', Float32, self.vel_callback)

        # Allow nodes to make stop requests
        self.stop_sub = rospy.Subscriber('/stop', Stop, self.stop_callback, queue_size=10)

        # Allows changes in speed
        self.set_speed_sub = rospy.Subscriber('/speed', Float32, self.speed_callback)

        # Allow the sharing of the current staus of the vehicle driving
        self.vehicle_state_pub = rospy.Publisher('/vehicle_state', VehicleState, queue_size=10, latch=True)

        # For sending out speed, and steering requests to Motor Endpoint
        self.motion_pub = rospy.Publisher('/nav_cmd', VelAngle, queue_size=10)

        # Show the points on the map in RViz
        self.points_pub = rospy.Publisher('/points', Path, queue_size=10, latch=True)

        # Show the cubic spline path the cart will be taking in RViz
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10, latch=True)

        # Show the current point along the path the cart is attempting to navigate to in RViz
        self.target_pub = rospy.Publisher('/target_point', Marker, queue_size=10)

        # Show the currently requested steering angle in RViz
        self.target_twist_pub = rospy.Publisher('/target_twist', Marker, queue_size=10)

        # Staus update to server
        self.arrived_pub = rospy.Publisher('/arrived', String, queue_size=10)

        # Steering angle PieChart display
        self.steering_pub = rospy.Publisher('/steering_angle', Float32, queue_size=10)

        # Publish the ETA
        self.eta_pub = rospy.Publisher('/eta', UInt64, queue_size=10)
        self.eta_timer = rospy.Timer(rospy.Duration(10), self.calc_eta)

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            # Upon receipt of a new path request, create a new one
            if self.new_path:
                rospy.loginfo("Creating a new path")
                self.path_valid = True
                self.new_path = False
                self.create_path()
            rate.sleep()
    
    def twist_callback(self, msg):
        self.global_twist = msg.twist
        
    def pose_callback(self, msg):
        self.global_pose = msg.pose
        # self.global_pose.position.x = self.global_pose.position.x - .6

    def stop_callback(self, msg):
        self.stop_requests[str(msg.sender_id.data).lower()] = [msg.stop, msg.distance]
        rospy.loginfo(str(msg.sender_id.data).lower() + " requested stop: " + str(msg.stop) + " with distance: " + str(msg.distance))
 
    def speed_callback(self, msg):
        self.global_speed = msg.data / 3.6
        rospy.loginfo("Speed changed to " + str (self.global_speed))

    def vel_callback(self, msg):
        if msg.data < 1.0:
            self.cur_speed = 1.8 # Magic number however this is roughly the observed speed in realtime
        else:
            self.poll_sample += 1
            self.raw_speed += msg.data
            if self.poll_sample >= 5:
                self.cur_speed = (self.raw_speed + msg.data)/self.poll_sample
                self.raw_speed = 0
                self.poll_sample = 0

    def localpoints_callback(self, msg):
        self.local_points = []
        for local_point in msg.localpoints:
            self.local_points.append(local_point.position)
        # path_valid being set to false will end the previous navigation and new_path being true will trigger the creation of a new path
        self.path_valid = False
        self.new_path = True
        rospy.loginfo("path received: " + str(msg))

    '''
    Creates a path for the cart with a set of local_points
    Adds 15 more points between the google points
    Intermediate points are added for a better fitting spline
    '''
    def create_path(self):
        # Increase the "resolution" of the path with 15 intermediate points
        local_points_plus = self.local_points # geometry_util.add_intermediate_points(self.local_points, 15.0)

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
            
            target_speed = self.global_speed

            # initial state
            pose = self.global_pose
            twist = self.global_twist

            quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            angles = tf.euler_from_quaternion(quat)
            initial_v = twist.linear.x
            #TODO state has to be where we start
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
            rate = rospy.Rate(30)
             # Continue to loop while we have not hit the target destination, and the path is still valid
            while last_index > target_ind and self.path_valid and not rospy.is_shutdown():
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
                rate.sleep()
        else:
            self.path_valid = False
            rospy.logwarn("It appears the cart is already at the destination")
            
        #Check if we've reached the destination, if so we should change the cart state to finished
        rospy.loginfo("Done navigating")
        self.current_state = VehicleState()
        self.current_state.is_navigating = False
        self.current_state.reached_destination = True
        notify_server = String()

        # Let operator know why current path has stopped
        if self.path_valid:
            rospy.loginfo("Reached Destination succesfully without interruption")
            self.arrived_pub.publish(notify_server)
        else:
            rospy.loginfo("Already at destination, or there may be no path to get to the destination or navigation was interrupted.")
        
        # Update the internal state of the vehicle
        self.vehicle_state_pub.publish(self.current_state)
        msg = VelAngle()
        msg.vel = 0
        msg.angle = 0
        msg.vel_curr = 0
        self.motion_pub.publish(msg)
        


    '''
    Updates the carts position by a given state and delta
    '''
    def update(self, state, a, delta):
        pose = self.global_pose
        twist = self.global_twist
        current_spd = twist.linear.x

        msg = VelAngle()
        if self.debug:
            self.delay_print -= 1
            if self.delay_print <= 0:
                self.delay_print = 50
                rospy.loginfo("Target Speed: " + str(a))
                rospy.loginfo("Current Speed: " + str(current_spd))
        msg.vel = a #Speed we want from pure pursuit controller
        msg.angle = (delta*180)/math.pi
        msg.vel_curr = current_spd

        display_angle = Float32()
        display_angle.data = msg.angle

        self.steering_pub.publish(display_angle)

        # Check if any node wants us to stop
        for x in self.stop_requests.values():
            if x[0]: # stop requested
                msg.vel = 0
                if x[1] > 0: # obstacle distance is given
                    msg.vel = -x[1] # give distance as a negative

        self.motion_pub.publish(msg)

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
        # Attempt an update only if the cart is driving
        if self.current_state.is_navigating:
            # Where are we at and how much further must we go
            current_node = self.get_closest_point(self.global_pose.position.x, self.global_pose.position.y)
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
        trip_sum = 0
        prev_node = start
        for i in range(prev_node, len(points_list)):
            trip_sum += self.calc_distance(points_list[prev_node].x, points_list[prev_node].y,
            points_list[i].x, points_list[i].y)
            prev_node = i

        return trip_sum

    def get_closest_point(self, pos_x, pos_y):
        """ Get the closest point along the raw path from pos_x, pos_y

        Args:
            pos_x(float): The x position of search center
            pos_y(float): The y position of search center
        """
        min_node = 0
        min_dist = 99999
        for i in range(len(self.local_points)):
            dist = self.calc_distance(pos_x, pos_y, self.local_points[i].x, self.local_points[i].y)
            if dist < min_dist:
                min_dist = dist
                min_node = i
        
        return min_node

    def calc_distance(self, x1, y1, x2, y2):
        return math.sqrt(((x2-x1)**2) + ((y2-y1)**2))


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
    marker.header.stamp = rospy.Time.now()
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

if __name__ == "__main__":
    try:
        LocalPlanner()
    except rospy.ROSInterruptException:
        pass
