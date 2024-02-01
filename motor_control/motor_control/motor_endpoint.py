#!/usr/bin/env python
"""
This is the python class that allows us to control the golf cart 
via teleop.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import time
import serial as sr
import numpy as np
import bitstruct


# ROS based imports
import tf_transformations
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from navigation_msgs.msg import VelAngle
from std_msgs.msg import Bool, String

# State constants
MOVING = 0
BRAKING = 1
STOPPED = 2


class MotorEndpoint(rclpy.node.Node):
    def __init__(self):
        super().__init__("motor_endpoint")

        # constants but prolly should make these as launch paramaters
        self.BRAKE_TIME = 3
        self.NODE_RATE = 10
        self.STEERING_TOLERANCE = 50

        self.serial_connected = False
        self.heartbeat = b""

        # We need to look into getting this to be the right value/launch parameter
        # I think we can use this USB port but I wont know til i try
        # These are launch paramaters for now. They are given default values which are shown in the code blocks below
        self.declare_parameter("baudrate", "57600")
        # For this port we can do ls /dev/tty* and find the actual thing
        self.declare_parameter("arduino_port", "/dev/ttyACM0")

        self.BAUDRATE = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )
        self.ARDUINO_PORT = (
            self.get_parameter("arduino_port").get_parameter_value().string_value
        )

        try:
            self.arduino_ser = sr.Serial(
                self.ARDUINO_PORT, baudrate=self.BAUDRATE, write_timeout=0, timeout=0.01
            )
            self.serial_connected = True
            self.log_header("CONNECTED TO ARDUINO")
        except Exception as e:
            self.log_header("MOTOR ENDPOINT: " + str(e))
            serial_connected = False

        # For now Im just ripping this straight from the old motor endpoint.
        # Need to figure out if we should keep the same subscribers and how to port them if needed
        self.motion_subscriber = self.create_subscription(
            VelAngle, "/nav_cmd", self.motion_callback, 10
        )
        self.debug_subscriber = self.create_subscription(
            Bool, "/realtime_debug_change", self.debug_callback, 10
        )
        self.heart_pub = self.create_publishers(String, "/heartbeat", 10)

        self.timer = self.create_timer(self.NODE_RATE, self.timer_callback)

    def motion_callback(self, vel_angle):
        """
        Callback for driving commands.
        """
        self.vel_curr = vel_angle.vel_curr
        self.vel = vel_angle.vel
        self.angle = vel_angle.angle

        if self.vel < 0:
            # indicates an obstacle
            self.obstacle_distance = abs(self.vel)
            self.vel = 0
        else:
            # reset obstacle distance and brake time
            self.obstacle_distance = -1
            self.brake_time_used = 0
            self.full_stop_count = 0

        if (
            self.vel > 0
            and (self.state == STOPPED or self.state == BRAKING)
            and (time.time() - self.stopping_time) > 10
        ):
            self.state = MOVING
            self.brake = 0  # take the foot off the brake
        elif self.state == MOVING and self.vel <= 0:  # Brakes are hit
            self.state = BRAKING
            self.brake = 0  # ramp up braking from 0
            self.stopping_time = time.time()

        self.new_vel = True

    def timer_callback(self):
        """
        Main loop timer for updating motor's instructions"""
        if not self.serial_connected:
            self.log_header("RETRYING SERIAL CONNECTION")
            try:
                self.arduino_ser = sr.Serial(
                    self.ARDUINO_PORT,
                    baudrate=self.BAUDRATE,
                    write_timeout=0,
                    timeout=0.01,
                )
                self.serial_connected = True
            except Exception as e:
                self.log_header("MOTOR ENDPOINT: " + str(e))
                serial_connected = False
                return

    def send_packet(self, throttle, brake, steer_angle):
        data = bytearray(b"\x00" * 5)
        bitstruct.pack_into(
            "u8u8u8u8u8", data, 0, 42, 21, abs(throttle), brake, steer_angle + 10
        )
        self.arduino_ser.write(data)

    def log_header(self, msg):
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"{msg}")
        self.get_logger().info("=" * 50)


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
