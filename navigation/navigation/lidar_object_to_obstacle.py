"""This file is a ros2 port of 'obstacle_detector.py' from the ros1 code. It uses the pointcloud_to_laserscan library (installed locally as a node in a git module) to filter points that could be obstacles. It then adds those obstacles to the ObstacleArray and publishes them so that the collision avoidance code can dodge whatever the filtering identifies as an obstacle.
Author: Nate Baker
"""

import rclpy
import rclpy.node

from sensor_msgs.msg import LaserScan, PointCloud2
from navigation_interface.msg import Obstacle, ObstacleArray


class LidarObjectToObstacle(rclpy.node.Node):
    def __init__(self):
        super().__init__("lidar_object_to_obstacle")

        # listen for velodyne output
        self.lidar_ptcloud_sub = self.create_subscription(
            LaserScan, "/velodyne_points", self.lidar_callback, 1
        )

        # communication with the pointcloud_to_laserscan node
        self.converter_sub = self.create_subscription(
            LaserScan, "/scan", self.laserscan_callback, 10
        )
        self.converter_pub = self.create_publisher(PointCloud2, "/cloud_in", 10)

        # most recent lidar scan data (after conversion to LaserScan)
        self.curr_data = None

    def lidar_callback(self, msg):
        # send the pointcloud to be converted to a laserscan
        self.converter_pub.publish(msg)

    def laserscan_callback(self, msg):
        self.curr_data = msg


def main():
    """node startup"""
    rclpy.init()
    node = LidarObjectToObstacle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
