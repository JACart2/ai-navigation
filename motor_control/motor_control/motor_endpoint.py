#!/usr/bin/env python
"""
This is the python class that allows us to control the golf cart 
via teleop.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
#Python based imports
import serial as sr
import numpy as np




#ROS based imports
import tf_transformations
import tf2_geometry_msgs #  Import is needed, even though not used explicitly
import rclpy



class MotorEndpoint(rclpy.node.Node):

    def __init__(self):
        
        super().__init__('motor_node')
        
        #constants but prolly should make these as launch paramaters
        self.BRAKE_TIME = 3
        self.NODE_RATE = 10
        self.STEERING_TOLERANCE = 50
      
        #We need to look into getting this to be the right value/launch parameter
        self.cart_port = '/dev/ttyUSB9'
        self.serial_connected = False
        self.heartbeat = b''
        
        
        
        try:
            #Im guessing this 57600 is the "baudrate"... Will need to look into what that actually does
            self.arduino_ser = sr.Serial(self.cart_port, 57600, write_timeout=0, timeout=.01)
            self.serial_connected = True
            self.get_logger().info("==========================================================================")
            self.get_logger().info("Connected to arduino")
            self.get_logger().info("==========================================================================")
        except Exception as e:
            self.get_logger().info("==========================================================================")
            self.get_logger().info("Motor_endpoint: " + str(e))
            self.get_logger().info("==========================================================================")
            serial_connected = False
    
        # For now Im just ripping this straight from the old motor endpoint.
        # Need to figure out if we should keep the same subscribers and how to port them if needed
        self.motion_subscriber = rospy.Subscriber('/nav_cmd', VelAngle, self.motion_callback,
                                                  queue_size=10)
        self.debug_subscriber = rospy.Subscriber('/realtime_debug_change', Bool, self.debug_callback, queue_size=10)
        self.heart_pub = rospy.Publisher('/heartbeat', String, queue_size=10)
        
    
        """
        The main method that actually handles spinning up the node.
        """
    def main():

        rclpy.init()

        node = MotorEndpoint()

        rclpy.spin(node)

        node.destroy_node()
        rclpy.shutdown()
    