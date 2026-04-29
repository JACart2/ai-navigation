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
import tf2_geometry_msgs  # noqa: F401 Needed to register PointStamped transforms.
import tf2_ros
from builtin_interfaces.msg import Duration

# ROS2 Messages.
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from navigation_interface.msg import Obstacle, ObstacleArray
from rclpy.qos import QoSProfile, ReliabilityPolicy # needed to match QoS settings of /scanner/scan
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
        self.sensor_frame = "velodyne"

        # Data related to incoming LaserScan (set in laserscan_callback).
        self.angle_max = 3.141592653589
        self.angle_min = 0.0
        self.angle_increment = 0.0

        self.curr_data = None # most recent lidar scan data (after conversion to LaserScan)
        self.cluster_list = [] # raw clusters of points where obstacles may be
        self.obstacles = ObstacleArray() # stores the converted obstacles to be sent to /obstacles topic
        self.sensor_obstacles = {
            "lidar": ObstacleArray(),
            "radar": ObstacleArray(),
        }
        self.sensor_last_seen = {
            "lidar": None,
            "radar": None,
        }
        self.declare_parameter("fusion_match_distance", 0.6)
        self.declare_parameter("sensor_stale_timeout", 0.35)
        self.fusion_match_distance = (
            self.get_parameter("fusion_match_distance")
            .get_parameter_value()
            .double_value
        )
        self.sensor_stale_timeout = (
            self.get_parameter("sensor_stale_timeout")
            .get_parameter_value()
            .double_value
        )

        # Rate limiting for radar
        self.last_radar_time = self.get_clock().now()

        # listen for velodyne output
        self.lidar_ptcloud_sub = self.create_subscription(
            PointCloud2, "/velodyne_points", self.lidar_callback, 1
        )

        # listen for radar output
        self.radar_ptcloud_sub = self.create_subscription(
            PointCloud2, "/ti_mmwave/radar_scan_pcl", self.radar_callback, 1
        )

        # communication with the pointcloud_to_laserscan nodes
        qos_scanner = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.lidar_converter_sub = self.create_subscription(
            LaserScan,
            "/scanner/lidar_scan",
            lambda msg: self.laserscan_callback(msg, "lidar"),
            qos_scanner,
        )
        self.radar_converter_sub = self.create_subscription(
            LaserScan,
            "/scanner/radar_scan",
            lambda msg: self.laserscan_callback(msg, "radar"),
            qos_scanner,
        )
        self.lidar_converter_pub = self.create_publisher(PointCloud2, "/cloud_in_lidar", 10)
        self.radar_converter_pub = self.create_publisher(PointCloud2, "/cloud_in_radar", 10)

        # Transmit aggregated ObstacleArray data to a topic to be handled separately.
        self.obstacle_pub = self.create_publisher (ObstacleArray, "/obstacles", 10)

        # Publish created markers to be displayed on RVIZ2.
        self.lidar_display_pub = self.create_publisher(Marker, "/lidar_obstacle_display", 10)
        self.radar_display_pub = self.create_publisher(Marker, "/radar_obstacle_display", 10)
        self.current_sensor = "lidar"

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
        min_cluster_size = 1 if self.current_sensor == "radar" else 2
        last_point = None

        # Begin iterating through the received ranges.
        for i in range(len(arr)):

            cur_angle = self.angle_min + (i * step_angle)
            ray_dist = arr[i]

            # Only process scans 8 meters ahead; limit view to front 180 degrees.
            if (
                math.isfinite(ray_dist)
                and ray_dist <= 8
                and ((cur_angle > (-math.pi / 2)) and (cur_angle < (math.pi / 2)))
            ):

                # Convert scan info to Point object.
                cur_x, cur_y = self.get_point (cur_angle, ray_dist)
                cur_point = Point(cur_x, cur_y)

                # Cluster Point instances that are close together; clear current cluster if it's getting large.
                if (
                    last_point is not None
                    and cur_point.equals(last_point)
                    and len(cur_cluster) < 70
                ):
                    cur_cluster.append (cur_point)
                else:
                    # Keep only significant objects.
                    if len(cur_cluster) >= min_cluster_size:
                        cluster_list.append(cur_cluster)

                    # Prepare to process a new cluster, including this first point.
                    cur_cluster = [cur_point]

                last_point = cur_point
            else:
                if len(cur_cluster) >= min_cluster_size:
                    cluster_list.append(cur_cluster)

                cur_cluster = []
                last_point = None

        if len(cur_cluster) >= min_cluster_size:
            cluster_list.append(cur_cluster)

    """Compute Cartesian coordinates given an angle and distance.

    Params:
        angle : Angle in radians.
        distance: distance from the origin along the angle direction.
    Returns:
        A pair of (x, y) coordinates.
    """
    def get_point(self, angle, distance):
        # LaserScan polar->Cartesian in ROS convention:
        # x forward, y left, angle measured from +x toward +y.
        x = math.cos(angle) * distance
        y = math.sin(angle) * distance

        return x, y

    def lidar_callback(self, msg):
        # send the pointcloud to be converted to a laserscan
        self.get_logger().info("LIDAR - Received Pointcloud, transforming...")
        self.lidar_converter_pub.publish(msg)

    def radar_callback(self, msg):
        # Rate limiting: only process radar data every 0.1 seconds
        current_time = self.get_clock().now()
        if (current_time - self.last_radar_time).nanoseconds / 1e9 < 0.1:
            return
        self.last_radar_time = current_time

        # send the radar PointCloud2 to be converted to a laserscan
        self.get_logger().info("RADAR - Received Pointcloud, transforming...")
        self.radar_converter_pub.publish(msg)

    def laserscan_callback(self, msg, sensor_name=None):
        # Extract data from the last converted LaserScan data.
        self.get_logger().info("LaserScan received from %s" % msg.header.frame_id)
        self.curr_data = msg
        self.angle_max = msg.angle_max
        self.angle_min = msg.angle_min
        self.sensor_frame = msg.header.frame_id
        self.last_scanned = msg.header.stamp
        if sensor_name is None:
            sensor_name = (
                "radar"
                if "ti_mmwave" in msg.header.frame_id or "radar" in msg.header.frame_id
                else "lidar"
            )
        self.current_sensor = sensor_name

        # Cluster the raw LaserScan data.
        self.cluster_points()

        # Construct Obstacle data from clustered points.
        self.circularize()

        self.sensor_obstacles[self.current_sensor] = self.obstacles
        self.sensor_last_seen[self.current_sensor] = self.get_clock().now()
        self.publish_fused_obstacles()
        self.local_display("base_link")

        # Clear the buffer.
        self.cluster_list = []

    def obstacle_distance(self, first, second):
        return math.sqrt(
            (first.pos.point.x - second.pos.point.x) ** 2
            + (first.pos.point.y - second.pos.point.y) ** 2
        )

    def copy_obstacle(self, obstacle):
        copied = Obstacle()
        copied.header.frame_id = obstacle.header.frame_id
        copied.header.stamp = obstacle.header.stamp
        copied.pos.header.frame_id = obstacle.pos.header.frame_id
        copied.pos.header.stamp = obstacle.pos.header.stamp
        copied.pos.point.x = obstacle.pos.point.x
        copied.pos.point.y = obstacle.pos.point.y
        copied.pos.point.z = obstacle.pos.point.z
        copied.radius = obstacle.radius
        copied.followable = obstacle.followable
        return copied

    def sensor_is_recent(self, sensor_name):
        last_seen = self.sensor_last_seen[sensor_name]
        if last_seen is None:
            return False

        age = (self.get_clock().now() - last_seen).nanoseconds / 1e9
        return age <= self.sensor_stale_timeout

    def merge_obstacle(self, fused_obstacle, radar_obstacle):
        # LiDAR is usually the better geometric estimate; keep its center and
        # expand the safety radius when radar agrees with it.
        fused_obstacle.radius = max(fused_obstacle.radius, radar_obstacle.radius)
        fused_obstacle.followable = fused_obstacle.followable or radar_obstacle.followable

    def publish_fused_obstacles(self):
        fused = ObstacleArray()
        fused.header.frame_id = "base_link"
        fused.header.stamp = self.last_scanned

        if self.sensor_is_recent("lidar"):
            for obstacle in self.sensor_obstacles["lidar"].obstacles:
                fused.obstacles.append(self.copy_obstacle(obstacle))

        if self.sensor_is_recent("radar"):
            for radar_obstacle in self.sensor_obstacles["radar"].obstacles:
                best_index = None
                best_distance = None

                for index, fused_obstacle in enumerate(fused.obstacles):
                    distance = self.obstacle_distance(fused_obstacle, radar_obstacle)
                    if distance <= self.fusion_match_distance and (
                        best_distance is None or distance < best_distance
                    ):
                        best_index = index
                        best_distance = distance

                if best_index is None:
                    fused.obstacles.append(self.copy_obstacle(radar_obstacle))
                else:
                    self.merge_obstacle(fused.obstacles[best_index], radar_obstacle)

        self.obstacle_pub.publish(fused)

    def circularize(self):
        # Turn the point clusters stored in self.cluster_list to Obstacles with radii that cover the entire cluster
        self.obstacles = ObstacleArray()
        self.obstacles.header.frame_id = "base_link"
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
                if self.current_sensor == "radar":
                    radius = max(0.25, 0.01 * len(cluster))
                else:
                    radius = 0.01 * len(cluster)

                # Transforming the center point to 'base_link'
                global_point = PointStamped()
                global_point.header.frame_id = self.sensor_frame
                global_point.header.stamp = self.last_scanned
                global_point.point.x = centerX
                global_point.point.y = centerY
                global_point.point.z = 0.0

                # Apply the transform to convert from sensor frame to 'base_link'
                if self.sensor_frame == "base_link":
                    transformed_point = global_point
                else:
                    try:
                        transformed_point = self.tf_buffer.transform(global_point, 'base_link')
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

            if self.current_sensor == "radar":
                marker.ns = "Radar_Objects"
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.ns = "Lidar_Objects"
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0

            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.color.a = 1.0

            # Use rclpy duration for marker lifetime
            dur = Duration()
            dur.nanosec = 1 * 10 ** 8
            marker.lifetime = dur

            marker.pose.position.x = object_list[i].pos.point.x
            marker.pose.position.y = object_list[i].pos.point.y
            marker.pose.position.z = 0.0

            radius = object_list[i].radius
            marker.scale.x = radius
            marker.scale.y = radius
            marker.scale.z = 0.3

            if self.current_sensor == "radar":
                self.radar_display_pub.publish(marker)
            else:
                self.lidar_display_pub.publish(marker)

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
