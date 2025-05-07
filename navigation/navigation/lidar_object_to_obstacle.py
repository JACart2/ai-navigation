#!/usr/bin/env python
"""Convert transformed LaserScan data to an ObstacleArray.

    This file is a ros2 port of 'obstacle_detector.py' from the ros1 code.
    It uses the pointcloud_to_laserscan library (installed locally as a node
    in a git module) to filter points that could be obstacles. It then adds
    those obstacles to the ObstacleArray and publishes them so that the collision
    avoidance code can dodge whatever the filtering identifies as an obstacle.

    Authors: Nate Baker, Rafael Dietsch
    Last Updated: 04/09/2025

"""
import math

import rclpy
import rclpy.node
import tf_transformations
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from builtin_interfaces.msg import Duration
from builtin_interfaces.msg import Time

# ROS2 Messages.
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from navigation_interface.msg import Obstacle, ObstacleArray
from rclpy.qos import QoSProfile, ReliabilityPolicy # needed to match QoS settings of /scanner/scan
from navigation_interface.msg import Obstacle, ObstacleArray
from visualization_msgs.msg import Marker
from tf2_ros import Buffer

class Point(object):

    # Constants.
    DIST_THRESHOLD = 0.10

    def __init__(self, x, y):
        self.x = x
        self.y = y

    # Compares self to another Point and determines whether they can be clustered.
    def equals(self, other_point):

        sx = self.x
        sy = self.y

        ox = other_point.x
        oy = other_point.y

        dist = math.sqrt( (ox - sx)**2 + (oy - sy)**2 )

        return dist < Point.DIST_THRESHOLD


class LidarObjectToObstacle(rclpy.node.Node):
    def __init__(self):
        super().__init__("lidar_object_to_obstacle")
        
        self.tf_buffer = Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_scanned = None

        # Data related to incoming LaserScan (set in laserscan_callback).
        self.angle_max = 3.141592653589
        self.angle_min = 0.0
        self.angle_increment = 0.0

        self.curr_data = None # most recent lidar scan data (after conversion to LaserScan)
        self.cluster_list = [] # raw clusters of points where obstacles may be
        self.obstacles = ObstacleArray() # stores the converted obstacles to be sent to /obstacles topic

        # listen for velodyne output
        self.lidar_ptcloud_sub = self.create_subscription(
            PointCloud2, "/velodyne_points", self.lidar_callback, 1
        )

        # communication with the pointcloud_to_laserscan node
        qos_scanner = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.converter_sub = self.create_subscription(
            LaserScan, "/scanner/scan", self.laserscan_callback, qos_scanner # Receives the converted LaserScan data back to be processed
        )
        self.converter_pub = self.create_publisher (PointCloud2, "/cloud_in", 10) # where received velodyne PointCloud data gets sent to be converted.

        # Transmit aggregated ObstacleArray data to a topic to be handled separately.
        self.obstacle_pub = self.create_publisher (ObstacleArray, "/obstacles", 10)

        # Publish created markers to be displayed on RVIZ2.
        self.display_pub = self.create_publisher (Marker, "lidar_obstacle_display", 10)

    def cluster_points(self):
        # LaserScan msg being processed.
        cur_scan = self.curr_data
        self.last_stamped = self.curr_data.header.stamp

        # Extract the range data.
        arr = cur_scan.ranges

        # Angle increase between each measurement.
        step_angle = self.curr_data.angle_increment

        # Starting angle of the LiDAR.
        cur_angle = cur_scan.angle_min

        # Extract the cluster.
        cluster_list = self.cluster_list
        cur_cluster = []

        # Initialize the starting Point to the beginning.
        default_x, default_y = self.get_point (cur_angle, arr[0])
        cur_point = Point(default_x, default_y)
        last_point = cur_point

        # Begin iterating through the received ranges.
        for i in range(len(arr)):

            cur_angle = self.angle_min + (i * step_angle)
            ray_dist = arr[i]

            # Only process scans 8 meters ahead; limit view to front 180 degrees.
            if ray_dist <= 8 and ( (cur_angle > (-math.pi/2)) and (cur_angle < (math.pi/2)) ):

                # Convert scan info to Point object.
                cur_x, cur_y = self.get_point (cur_angle, ray_dist)
                cur_point = Point(cur_x, cur_y)

                # Cluster Point instances that are close together; clear current cluster if it's getting large.
                if cur_point.equals(last_point) and len(cur_cluster) < 70:
                    cur_cluster.append (cur_point)
                else:
                    # Keep only significant objects.
                    if len(cur_cluster) > 1:
                        cluster_list.append(cur_cluster)

                    # Prepare to process a new cluster.
                    cur_cluster = []

            last_point = cur_point

    """Compute Cartesian coordinates given an angle and distance.

    Params:
        angle : Angle in radians.
        distance: distance from the origin along the angle direction.
    Returns:
        A pair of (x, y) coordinates.
    """
    def get_point(self, angle, distance):

        x = math.sin(angle) * distance
        y = math.cos(angle) * distance

        return x, y

    def lidar_callback(self, msg):
        # send the pointcloud to be converted to a laserscan
        self.get_logger ().info ("LIDAR - Received Pointcloud, transforming...")
        self.converter_pub.publish(msg)

        # Get the time of the message and store it.
        self.last_scanned = Time()
        self.last_scanned.sec = msg.header.stamp.sec
        self.last_scanned.nanosec = msg.header.stamp.nanosec

    def laserscan_callback(self, msg):
        # Extract data from the last converted LaserScan data.
        self.get_logger ().info ("LIDAR - LaserScan received!")
        self.curr_data = msg
        self.angle_max = msg.angle_max
        self.angle_min = msg.angle_min

        # Cluster the raw LaserScan data.
        self.cluster_points ()

        # Construct Obstacle data from clustered points.
        self.circularize () #TODO - untested

        # Publish all detected Obstacles, then clear the buffer.
        self.obstacle_pub.publish (self.obstacles)

        # TODO! - untested function
        self.local_display("base_link")

        # Clear the buffer.
        self.cluster_list = []

    def circularize(self):
        # Turn the point clusters stored in self.cluster_list to Obstacles with radii that cover the entire cluster
        self.obstacles = ObstacleArray()
        self.obstacles.header.frame_id = "velodyne"
        # self.get_logger().info(str(type(self.last_scanned))) # debugging!
        # TODO - This throws errors sometimes between launches due to messages remaining in the ros2 /scanner/scan topic.
        # I see two potential ways to fix this:
        #   1 - Check for None values and give the stamp field a default value in case it hasn't been set yet.
        #   2 - Use some sort of subscriber configuration to only get messages sent AFTER this node starts running.
        #       That way, the execution flow follows the expected flow of execution.
        # 
        # Discuss potential solutions w/ Nate and Sprague to see what works best.
        if self.last_scanned != None:
            self.obstacles.header.stamp = self.last_scanned

            for cluster in self.cluster_list:
                cur_circle = Obstacle()
                radius = 0

                first_point = cluster[0]
                last_point = cluster[len(cluster) - 1]

                # Local to the base_laser_link
                centerX = (first_point.x + last_point.x) / 2
                centerY = (first_point.y + last_point.y) / 2

                # avg point spacing is maybe ~3 inches? Adjust the calculation if needed
                radius = 0.01 * len(cluster)

                # Wait for transform to be available
                try:
                    transform = self.tf_buffer.lookup_transform('base_link', 'velodyne', self.last_scanned)
                except (tf2_ros.TransformException) as e:
                    self.get_logger().warn(f'LO2O: Could not transform: {e}')
                    continue

                # Transforming the center point to 'base_link'
                global_point = PointStamped()
                global_point.header.frame_id = 'velodyne'
                global_point.header.stamp = self.last_scanned
                global_point.point.x = centerX
                global_point.point.y = centerY
                global_point.point.z = 0.0

                # Apply the transform to convert from 'velodyne' frame to 'base_link'
                try:
                    transformed_point = self.tf_buffer.transform(global_point, 'base_link')
                    transformed_point = do_transform_point(global_point, transform)
                except (tf2_ros.TransformException) as e:
                    self.get_logger().warn(f'Could not transform point: {e}')
                    continue

                # self.get_logger().info("Transformed point to :: " + str(transformed_point)) # DEBUGGING!
                # Create a new circle around the obstacle
                cur_circle = Obstacle()
                cur_circle.header.frame_id = 'base_link'
                cur_circle.header.stamp = self.last_scanned
                cur_circle.pos = transformed_point
                cur_circle.radius = radius
                cur_circle.followable = True

                # Add the obstacle to the list
                self.obstacles.obstacles.append(cur_circle)

    def local_display(self, frame):
        # Display the obstacles on the LIDAR frame, will appear jittery in Rviz as there is no interpolation
        object_list = self.obstacles.obstacles
        for i in range(len(object_list)):
            marker = Marker()
            marker.header.frame_id = frame

            marker.ns = "Object_NS"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            # Use rclpy duration for marker lifetime
            dur = Duration()
            dur.nanosec = 1 * 10 ** 8
            self.get_logger().info("Setting marker duration to :: " + str(type(dur))) # DEBUGGING!
            marker.lifetime = dur

            marker.pose.position.x = object_list[i].pos.point.x
            marker.pose.position.y = object_list[i].pos.point.y
            marker.pose.position.z = 0.0

            radius = object_list[i].radius
            marker.scale.x = radius
            marker.scale.y = radius
            marker.scale.z = 0.3

            # Publish marker
            self.display_pub.publish(marker)

def main():
    # node startup.
    rclpy.init()
    node = LidarObjectToObstacle()

    # Running the node.
    rclpy.spin(node)

    # Explicitly clean up when not running.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
