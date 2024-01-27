"""Demonstration of using the nav2 action interface in Python.

This node navigates to a goal pose provided on the command line.  This
code also include a demonstration of interacting with OccupancyGrid
messages through the map_util.Map class.

DO NOT MODIFY OR IMPORT THIS FILE.  It is only provided as an
illustration.

Author: Nathan Sprague and Kevin Molloy
Version: 10/24/2023

"""
import argparse
import time
import numpy as np
# import .transformer as tf
from . import transformer as tf
import rclpy
import rclpy.node
from rclpy.action.client import ActionClient
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor
from collections import deque
import statistics
#Change timer
#Work on initial pose and have return to start
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, PoseStamped, Pose, PointStamped, Quaternion
from sensor_msgs.msg import Image
from zeta_competition_interfaces.msg import Victim
from nav_msgs.msg import OccupancyGrid
from jmu_ros2_util import map_utils
import tf_transformations
import tf2_geometry_msgs #  Import is needed, even though not used explicitly
import random
import math
from operator import itemgetter

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import Empty

def create_nav_goal(x, y, theta):
    goal = NavigateToPose.Goal()

    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    
    
    # We need to convert theta to a quaternion....
    quaternion = tf_transformations.quaternion_from_euler(0, 0, theta)
    goal.pose.pose.orientation.x = quaternion[0]
    goal.pose.pose.orientation.y = quaternion[1]
    goal.pose.pose.orientation.z = quaternion[2]
    goal.pose.pose.orientation.w = quaternion[3]
    return goal

def get_yaw(quat): #convert ros quaternion to matrix/list type TODO - DOES NOT WORK FOR NOW?
    mat = [0,0,0,0]
    mat[0] = quat.x
    mat[1] = quat.y
    mat[2] = quat.z
    mat[3] = quat.w
    
 
    return tf_transformations.euler_from_quaternion(mat)[2]

class RescueNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('rescue_node')
        
        group = rclpy.callback_groups.ReentrantCallbackGroup()
        # This QOS Setting is used for topics where the messages
        # should continue to be available indefinitely once they are
        # published. Maps fall into this category.  They typically
        # don't change, so it makes sense to publish them once.
     
        timer_object = self.create_timer(.1, self.timer_callback, callback_group=group)
        
        self.can_goal = False
        latching_qos = QoSProfile(depth=1,
                                  durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(OccupancyGrid, 'map',
                                 self.map_callback,
                                 qos_profile=latching_qos, callback_group=group)

        self.get_initial_pose_callback = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                        self.initial_pose_callback, 10, callback_group=group)

        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.mark_postions_callback, 10, callback_group=group)
        self.explored_locations = set()


        #Just messing with stuff here
        self.quat = None
        self.ID = 0
        self.victims = []
        self.approaching = False
        self.going_home = False

        #Gonna attempt to normalize some things
        self.do_aruco = 50

        self.aruco_help = []
        self.x_list = []
        self.y_list = []
        self.place_check = None
        
        self.create_subscription(PoseArray, 'aruco_poses', self.aruco_callback, 10, callback_group=group)

        self.create_subscription(Image, "/camera/image_raw", self.camera_callback, 10, callback_group=group)
        
        self.image = None

        self.create_subscription(Empty, '/report_requested', self.report_callback, 10, callback_group=group)
    
        self.victim_pub = self.create_publisher(Victim, 'victim', 10, callback_group=group)

        self.place_to_go = self.create_publisher(PoseStamped, "/place", 10, callback_group=group)
        # Create the action client.
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=group)

        # Used if we decide to cancel a goal request. None indicates
        # we haven't canceled.
        # self.cancel_future = None
        self.declare_parameter('time_limit', 720)
        self.total_time = self.get_parameter('time_limit').get_parameter_value().integer_value
        self.get_logger().info(str(self.total_time))

        self.init_time = time.time()
        self.time_to_get_back = 20
        self.get_back_now = False

        self.aruco = None
        self.map = None
        self.timeout = 30.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.initial_position = None


        self.wait_to_move = False
        self.last_seen_vic = None
        self.get_logger().info("end of the init")
        self.tarqueue = deque()

    def camera_callback(self, image):
        # self.get_logger().info(str(time.time()))
        # self.get_logger().info("camera callback")
        if image != None:
            self.image = image
        
        

    def random_goal(self):
        if self.map is None:
            return
        
        # self.get_logger().info("Choosing RANDOM goal\n")
        free = False
        while not free:
            self.goal = create_nav_goal(random.random() * (self.map.width * self.map.resolution) + self.map.origin_x , random.random() * (self.map.height * self.map.resolution) + self.map.origin_y, random.random() * math.pi * 2)
            x = self.goal.pose.pose.position.x
            y = self.goal.pose.pose.position.y
            val = self.map.get_cell(x, y)
            if val == 0:
                free = True
            elif val == 100:
                free = False
            else:
                free = False

        self.cancel_future = None        
        self.send_goal()

    def smart_goal(self):
        if self.map is None:
            return
        # I dont know what this does
        self.wait_to_move = True
        found = False
        while (len(self.tarqueue) != 0): #TODO - Maybe recheck if position is not in self.victims because it could change
            vic = self.tarqueue.popleft()
            if (self.is_new_victim(vic)):
                # vic.orientation.z %= math.pi * 2
                self.get_logger().info(f"Next vic's pose:  x-cord {vic.position.x}, y-cord {vic.position.y}")
                self.get_logger().info("eulers " + str(tf_transformations.euler_from_quaternion([vic.orientation.x, vic.orientation.y, vic.orientation.z, vic.orientation.w])))
                # q_rot = tf_transformations.quaternion_from_euler(-(math.pi / 2), 0, 0)
                # quaternion = tf_transformations.quaternion_multiply(q_rot, [vic.orientation.x, vic.orientation.y, vic.orientation.z, vic.orientation.w])
                # vic.orientation.x, vic.orientation.y, vic.orientation.z, vic.orientation.w = quaternion



                # newx, newy, yaw = self.transform_target_tag(vic)
                npoint = self.transform_target_tag(vic)

                val = self.map.get_cell(npoint[0], npoint[1])
                if val == 0:
                    inb = True
                elif val == 100:
                    inb = False
                else:
                    inb = False
                if not inb:
                    continue
                tempx = vic.position.x - npoint[0]
                tempy = vic.position.y - npoint[1]
                angle = math.atan(tempy / tempx)
                if (tempx < 0):
                    angle += math.pi


                # self.get_logger().info(f"After transformation  x-cord {newx}, y-cord {newy}, rot {yaw}")
                self.get_logger().info(f"After transformation  x-cord {npoint[0]}, y-cord {npoint[1]}, rot {angle}")
                #The original implimentation is this self.goal = create_nav_goal(actual[0], actual[1], (vic.orientation.z + math.pi))
                
                
                #THIS IS ALL MY NEWLY ATTEMPTED IMPLIMENTATION
                # q_orig = Quaternion()
                # q_orig.x = vic.orientation.x
                # q_orig.y = vic.orientation.y
                # q_orig.z = vic.orientation.z
                # q_orig.w = vic.orientation.w
                
                
                # q_rot = tf_transformations.quaternion_from_euler(0, 0, math.pi / 2)
                # quaternion = tf_transformations.quaternion_multiply(q_rot, quaternion)
        
                # initial_euler = tf_transformations.euler_from_quaternion([tag.orientation.x, tag.orientation.y, tag.orientation.z, tag.orientation.w])
                # yaw = initial_euler[2] - (math.pi/2)
                # quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw)

                # self.goal = NavigateToPose.Goal()

                # self.goal.pose.header.frame_id = 'map'
                # self.goal.pose.pose.position.x = newx + vic.position.x
                # self.goal.pose.pose.position.y = newy + vic.position.y
                
                
                # self.goal.pose.pose.orientation.x = quaternion[0]
                # self.goal.pose.pose.orientation.y = quaternion[1]
                # self.goal.pose.pose.orientation.z = quaternion[2]
                # self.goal.pose.pose.orientation.w = quaternion[3]
                
                # #END OF MY ATTEMPT
                self.goal = create_nav_goal(npoint[0], npoint[1], angle)
                to_go = PoseStamped()
                to_go.pose = self.goal.pose.pose
                to_go.header.frame_id = "map"
                self.place_to_go.publish(to_go)
                # self.goal = create_nav_goal(actual[0], actual[1], get_yaw(vic.orientation))
                self.get_logger().info(f"Due to arrive at: x-cord {self.goal.pose.pose.position.x}, y-cord {self.goal.pose.pose.position.y }\n")
                self.approaching = True
                found = True
                self.cancel_future = None        
                self.send_goal()
                self.get_logger().info("Going to target location")
                break

        if not found:
            if len(self.explored_locations) != 0:
                self.get_logger().info("Choosing SMART GOAL\n")
                free = False
                while not free:
                    #self.goal = create_nav_goal(random.random() * (self.map.width * self.map.resolution) + self.map.origin_x , random.random() * (self.map.height * self.map.resolution) + self.map.origin_y, random.random() * math.pi * 2)
                    # x = self.goal.pose.pose.position.x
                    # y = self.goal.pose.pose.position.y
                    best_point = ((0, 0), 0)
                    for i in range(50):
                        temp_goal = create_nav_goal(random.random() * (self.map.width * self.map.resolution) + self.map.origin_x , random.random() * (self.map.height * self.map.resolution) + self.map.origin_y, random.random() * math.pi * 2)
                        temp_x = temp_goal.pose.pose.position.x
                        temp_y = temp_goal.pose.pose.position.y
                        distances_from_temp_goal = []
                        for point in self.explored_locations:
                            distances_from_temp_goal.append(((temp_x, temp_y), math.sqrt((temp_x - point[0])**2 + (temp_y - point[1])**2)))
                        furthest = min(distances_from_temp_goal, key = itemgetter(1))
                        # self.get_logger().info(str(best_point))
                        # self.get_logger().info(str(furthest))

                        if furthest[1] > best_point[1] and self.map.get_cell(furthest[0][0], furthest[0][1]) == 0:
                            best_point = ((temp_x, temp_y), furthest[1])
                
                    self.goal = create_nav_goal(best_point[0][0], best_point[0][1], random.random() * math.pi * 2)
                    
                    
                    to_go = PoseStamped()
                    to_go.pose = self.goal.pose.pose
                    to_go.header.frame_id = "map"
                    self.place_to_go.publish(to_go)
                    x = self.goal.pose.pose.position.x
                    y = self.goal.pose.pose.position.y
                    val = self.map.get_cell(x, y)
                    if val == 0:
                        free = True
                    elif val == 100:
                        free = False
                    else:
                        free = False

                self.cancel_future = None        
                self.send_goal()
            else:
                self.random_goal()

        self.wait_to_move = False
        
    def send_goal(self):
        # self.get_logger().info("WAITING FOR NAVIGATION SERVER...")
        self.ac.wait_for_server()
        # self.get_logger().info("NAVIGATION SERVER AVAILABLE...")
        # self.get_logger().info("SENDING GOAL TO NAVIGATION SERVER...")
        self.start_time = time.time()

        self.goal_future = self.ac.send_goal_async(self.goal)
        self.can_goal = True

        # This will be used by rclpy to know when this node has
        # finished its work.
        self.future_event = Future()

        return self.future_event

    def transform_aruco_frame(self, pose):
        # Convert poses from camera optical rgb coordiante frame to map frame
        return self.buffer.transform(pose, "map")
        
    
    def is_new_victim(self, pose):
        # Check if victim is within certain threshold of all elements in victim set
        for v in self.victims:
            if self.is_same_point(v.point.point, pose.position, 0.4):
                return False
        return True
    
    def not_in_tarqueue(self, pose):
        for v in self.tarqueue:
            if self.is_same_point(v.position, pose.position, 0.32):
                return False
        return True

    def is_same_point(self, a, b, dist=0.3):
        return (abs(a.x - b.x) < dist and abs(a.y - b.y) < dist and abs(a.z - b.z) < dist)
    
    def build_pose_stamped(self, pose, header):
        res = PoseStamped()
        res.header.frame_id = header.frame_id
        res.pose = pose
        
        return res

    def help_position(self, reference):

        for thing in self.aruco_help:
            # self.get_logger().info(f"{thing.position.x}")
            # self.get_logger().info(f"{reference.position.x}")
           
            if (thing.position.x - .3 <= reference.position.x and thing.position.x + .3 >= reference.position.x):
                self.x_list.append(thing.position.x)
               
            if (thing.position.y - .3 <= reference.position.y and thing.position.y + .3 >= reference.position.y):
                self.y_list.append(thing.position.y)
         

        """This method goes through the list of poses that are generated by seeing tags.
        """
    def aruco_callback(self, aruco):
        if self.map == None:
            self.get_logger().info("Map not yet initialized")
            return
        
        if self.going_home:
            return

        if (aruco != None):
            self.aruco = aruco
            
            for v in aruco.poses:
                try:
                    #now in map frame. Its making a pose stamped and 
                    #passing that through to transform aruco and then getting the "pose" of that
                    pose_to_go = self.transform_aruco_frame(self.build_pose_stamped(v, aruco.header)).pose
                    # self.aruco_help.append(pose_to_go)
                    self.last_seen_vic = pose_to_go

                    # if self.place_check == None:
                    #     self.place_check = pose_to_go


                    # if (self.do_aruco != 0):
                    #         self.help_position(self.place_check)
                    #         self.do_aruco = self.do_aruco - 1

                    # if (not self.is_new_victim(self.last_seen_vic) or len(self.x_list) == 0 and len(self.y_list) == 0):
                    #     self.aruco_help.clear()
                    #     self.do_aruco = 50
                    #     self.x_list.clear()
                    #     self.y_list.clear()
                    #     self.place_check = None
                    #     self.get_logger().info("Im stoppping shit cause ive already been here")
                    #     continue

                    # if (len(self.x_list) > 0 and len(self.y_list) > 0):
                    #     x_ret = statistics.mean(self.x_list) 
                    #     y_ret = statistics.mean(self.y_list)  

                    # self.x_list.clear()
                    # self.y_list.clear() 

                    # was originally with pose to go in there
                    if (self.is_new_victim(pose_to_go) and self.not_in_tarqueue(pose_to_go)):
                        dest_to_go = pose_to_go #change to in front of target
                        # I genuinely dont know why we are doing this here.
                        # dest_to_go.position.x -= .3
                        # dest_to_go.position.x = x_ret
                        # dest_to_go.position.y = y_ret
                        self.tarqueue.append(dest_to_go)
                        # self.get_logger().info(f"average: x: {x_ret}, y: {y_ret}")
                        # #BLAH BLAH
                        # self.place_check = None
                        # self.aruco_help.clear()
                        # self.get_logger().info("added")
                        # self.do_aruco = 50

                except Exception as e:
                    self.get_logger().warn(str(e))
                    #self.random_goal()
                    self.smart_goal()
            if (not self.approaching and len(self.tarqueue) != 0):
                #self.random_goal()
                self.smart_goal()

    def initial_pose_callback(self, msg):
        self.get_logger().info("initial pose callback")

        if self.map is not None:
        
            self.initial_position = msg

            self.destroy_subscription(self.get_initial_pose_callback)
            #self.destroy_subscription('amcl_pose')
        
    def mark_postions_callback(self, msg):
        # self.get_logger().info("mark postions callback")
        x = round(msg.pose.pose.position.x, 1)
        y = round(msg.pose.pose.position.y, 1)
        self.explored_locations.add((x, y))
        
    def map_callback(self, map_msg):
        self.get_logger().info("map callback")
        """Process the map message.

        This doesn't really do anything useful, it is purely intended
        as an illustration of the Map class.

        """
        if self.map is None:  # No need to do this every time map is published.

            self.map = map_utils.Map(map_msg)

            # Use numpy to calculate some statistics about the map:
            total_cells = self.map.width * self.map.height
            pct_occupied = np.count_nonzero(self.map.grid == 100) / total_cells * 100
            pct_unknown = np.count_nonzero(self.map.grid == -1) / total_cells * 100
            pct_free = np.count_nonzero(self.map.grid == 0) / total_cells * 100
            map_str = "Map Statistics: occupied: {:.1f}% free: {:.1f}% unknown: {:.1f}%"
            self.get_logger().info(map_str.format(pct_occupied, pct_free, pct_unknown))

            # # Here is how to access map cells to see if they are free:
            # x = self.goal.pose.pose.position.x
            # y = self.goal.pose.pose.position.y
            # val = self.map.get_cell(x, y)
            # if val == 100:
            #     free = "occupied"
            # elif val == 0:
            #     free = "free"self.initial_position is not None
            # else:
            #     free = "unknown"
            # self.get_logger().info(f"HEY! Map position ({x:.2f}, {y:.2f}) is {free}")
            #self.random_goal()
            
            self.smart_goal()

    def publish_victim_data(self):
        for msg in self.victims:
            self.victim_pub.publish(msg)
            self.get_logger().info(f"Target {msg.id} logged at:{msg.point.point.x}, {msg.point.point.y }, {msg.point.point.z}")
            

    def transform_target_tag (self, tag):
        trans= tf.Transformer("map")
        
        # #Was originally 1 in the x field
        point = np.array([0, -0.35, 0])
        
        #This was originally 
        eulers = tf_transformations.euler_from_quaternion([tag.orientation.x,tag.orientation.y,tag.orientation.z,tag.orientation.w])
        tag_trans = tf.trans(tag.position.x, tag.position.y, tag.position.z) @tf.rot_z(eulers[2])
        #Changed it for now because i believe that you should first rotate and then move. Below is my change I WAS 100 WRONG.
        # tag_trans = tf.rot_z(np.pi) @ tf.trans(tag.position.x, tag.position.y, tag.position.z)
        
        #Also this without the rotation gets the same exact thing as  tf.trans(tag.position.x, tag.position.y, tag.position.z) @ tf.rot_z(np.pi)
        # tag_trans = tf.trans(tag.position.x, tag.position.y, tag.position.z)
        trans.add_transform('map', 'tag', tag_trans)
        
        return trans.transform('tag', 'map', point)
        
        
        #Just for fun im going to attempt to do a translation based on cos/sin with a distance of .5

        # initial_euler = tf_transformations.euler_from_quaternion([tag.orientation.x, tag.orientation.y, tag.orientation.z, tag.orientation.w])
        # # yaw = (initial_euler[2] % (2 * math.pi)) 
        # # Was originally with the index being 1 not 0
        # yaw = initial_euler[2] - (math.pi/2)
        # self.get_logger().info("eulers " + str(initial_euler))
        # self.get_logger().info(f"tag's angle is {yaw}")
        # newx = math.cos(yaw) * .8
        # newy = math.sin(yaw) * .8
        
        # return newx, newy, yaw + math.pi

     
    

    def timer_callback(self):
        """Periodically check in on the progress of navigation."""
        #self.get_logger().info(str(time.time() - self.init_time))
        # self.get_logger().info("Timer callback start")
        if (self.initial_position is not None and (time.time() - self.init_time) > (self.total_time - self.time_to_get_back) and not self.going_home):  #time passed > (total amount - time to get back)
            self.get_logger().info("Returning HOME\n")
            self.going_home = True

            initial_quaternion = [0, 0, 0, 0]
            initial_quaternion[0] = self.initial_position.pose.pose.orientation.x
            initial_quaternion[1] = self.initial_position.pose.pose.orientation.y
            initial_quaternion[2] = self.initial_position.pose.pose.orientation.z
            initial_quaternion[3] = self.initial_position.pose.pose.orientation.w
            initial_euler = tf_transformations.euler_from_quaternion(initial_quaternion)
            #self.get_logger().info(str(initial_euler))

            self.goal = create_nav_goal(self.initial_position.pose.pose.position.x, self.initial_position.pose.pose.position.y, initial_euler[2])
            self.future_event.set_result(True)
            self.cancel_future = None
            self.send_goal()
        # if self.initial_position is not None:
        #     self.get_logger().info(str(self.initial_position.pose.pose.position.x))
        #self.get_logger().info(str(self.explored_locations))

        if not self.going_home and not self.wait_to_move and self.can_goal:
            if not self.goal_future.done():
                self.get_logger().info("NAVIGATION GOAL NOT YET ACCEPTED")

            elif self.cancel_future is not None:  # We've cancelled and are waiting for ack.
                if self.cancel_future.done():
                    self.get_logger().info("SERVER HAS ACKNOWLEDGED CANCELLATION")

                    self.approaching = False
                    
                    self.future_event.set_result(False)
                    self.smart_goal()
                    #self.random_goal()
            else:

                if self.goal_future.result() is not None and self.goal_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info("Navigation SUCCESS\n")
                    temp_image = self.image
                    if (self.approaching and self.aruco is not None):
                        pos = self.last_seen_vic  #TODO - GET TARGET POSITION SOMEHOW (FIRST ENTRY IN ARUCO?)
                        if (self.is_new_victim(pos)):
                            vic = Victim()
                            vic.id = self.ID
                            self.ID += 1
                            vic.image = temp_image
                            ps = PointStamped()
                            ps.header.frame_id = 'map'
                            ps.point = pos.position
                            vic.point = ps
                            self.victims.append(vic)
                        
                    
                        
                    self.approaching = False
                    
                    self.future_event.set_result(True)
                    self.smart_goal()
                    #self.random_goal()

                elif self.goal_future.result() is not None and self.goal_future.result().status == GoalStatus.STATUS_ABORTED:
                    self.get_logger().info("NAVIGATION SERVER HAS ABORTED. EXITING!")

                    self.approaching = False
                    
                    self.future_event.set_result(False)
                    self.smart_goal()
                    #self.random_goal()

                elif time.time() - self.start_time > self.timeout:
                    self.get_logger().info("Time out! CANCEL goal\n")
                    # self.cancel_future = self.goal_future.result().cancel_goal_async()
                    self.approaching = False
                    self.smart_goal()
                    #self.random_goal()
                # elif (not self.approaching and is_same_point(location of self, goal location)) #Maybe cancel early if close enough to goal and goal is not for a target
                #     self.smart_goal()

    def report_callback(self, msg):
        # self.get_logger().info("report callback")
        self.publish_victim_data()


def main():

    rclpy.init()

    node = RescueNode()
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()