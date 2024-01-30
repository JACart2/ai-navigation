#!/usr/bin/env python
"""
This is the python class that allows us to control the golf cart 
via teleop.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import serial as sr
import numpy as np


# ROS based imports
import tf_transformations
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy


class MotorEndpoint(rclpy.node.Node):
    def __init__(self):
        super().__init__("motor_node")

        # constants but prolly should make these as launch paramaters
        self.BRAKE_TIME = 3
        self.NODE_RATE = 10
        self.STEERING_TOLERANCE = 50
        self.serial_connected = False

        # We need to look into getting this to be the right value/launch parameter
        # I think we can use this USB port but I wont know til i try
        # These are launch paramaters for now. They are given default values which are shown in the code blocks below
        self.declare_parameter("baudrate", "57600")
        # For this port we can do ls /dev/tty* and find the actual thing
        self.declare_parameter("arduino_poirt", "/dev/ttyACM0")

        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        arduino_port = (
            self.get_parameter("arduino_poirt").get_parameter_value().string_value
        )

        try:
            # Im guessing this 57600 is the "baudrate"... Will need to look into what that actually does
            self.arduino_ser = sr.Serial(
                arduino_port, baudrate=baudrate, write_timeout=0, timeout=0.01
            )
            self.serial_connected = True
            self.get_logger().info(
                "=========================================================================="
            )
            self.get_logger().info("Connected to arduino")
            self.get_logger().info(
                "=========================================================================="
            )
        except Exception as e:
            self.get_logger().info(
                "=========================================================================="
            )
            self.get_logger().info("Motor_endpoint: " + str(e))
            self.get_logger().info(
                "=========================================================================="
            )
            serial_connected = False


def main():
    """
    The main method that actually handles spinning up the node."""

    rclpy.init()
    node = MotorEndpoint()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
