#!/usr/bin/env python

import unittest
from motor_control_interface.msg import VelCurr, VelAnglePlanned
import rclpy


class MotorEndpointTest(rclpy.node.Node):

    def __init__(self):
        super().__init__("motor_endpoint_test")



        self.pub_motion = self.create_publisher(
            VelAnglePlanned, "/nav_cmd", 10
        )
    
    def drive_two_seconds(self):
        
        vel_angle = VelAnglePlanned()
        
        
        
        
    