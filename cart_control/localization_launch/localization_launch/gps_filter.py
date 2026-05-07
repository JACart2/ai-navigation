#!/usr/bin/env python3
"""
GPS filter node that filters out NavSatFix messages without altitude data.
Subscribes to /fix and republishes valid messages to /fix_filtered.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math


class GPSFilterNode(Node):
    def __init__(self):
        super().__init__('gps_filter_node')
        
        # Create subscription to /fix topic
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )
        
        # Create publisher for filtered GPS data
        self.publisher = self.create_publisher(NavSatFix, '/fix_filtered', 10)
        
        self.get_logger().info('GPS filter node started')
    
    def gps_callback(self, msg: NavSatFix):
        """
        Filter GPS messages to only pass those with valid altitude data.
        """
        # Check if altitude is valid (not NaN)
        if not math.isnan(msg.altitude):
            self.publisher.publish(msg)
        else:
            self.get_logger().debug('Filtered out GPS message without altitude data')


def main(args=None):
    rclpy.init(args=args)
    gps_filter = GPSFilterNode()
    rclpy.spin(gps_filter)
    gps_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
