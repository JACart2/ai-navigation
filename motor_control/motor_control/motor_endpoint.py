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
import math

# ROS based imports
import tf_transformations
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from motor_control_interface.msg import VelAngle
from std_msgs.msg import Bool, String

# State constants
MOVING = 0
BRAKING = 1
STOPPED = 2
STEERING_CORRECTION = 10


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
                self.serial_connected = False
                return
    

    def calculate_endpoint(self):
        if self.new_vel:
            #The first time we get a new target speed and angle we must convert it        
            self.vel_cart_units = self.vel
            self.vel_curr_cart_units = self.vel_curr

            self.new_vel = False

            # Why was this hard coded this way? What is it even doing
            self.vel_cart_units *= 50       # Rough conversion from m/s to cart controller units
            self.vel_curr_cart_units *= 50  # Rough conversion from m/s to cart controller units
            
            #i guess this logic makes sense but also i dont understand what "cart controller units" are
            #and is it possible to get a better estimate?
            if self.vel_cart_units > 254:
                self.vel_cart_units = 254
            if self.vel_cart_units < -254:
                self.vel_cart_units = -254
            if self.vel_curr_cart_units > 254:
                self.vel_curr_cart_units = 254
            if self.vel_cart_units < 0:
                rospy.logwarn("NEGATIVE VELOCITY REQUESTED FOR THE MOTOR ENDPOINT!")
        
        target_speed = int(self.vel_cart_units) #float64
        current_speed = int(self.vel_curr_cart_units) #float64

        #adjust the target_angle range from (-45 <-> 45) to (0 <-> 100)
        # rospy.loginfo("Angle before adjustment: " + str(self.cmd_msg.angle))

        if(self.angle < -40):
            self.angle = self.steering_tolerance * -1
        if(self.angle > 40):
            self.angle = self.steering_tolerance
        target_angle = 100 - int(( (self.angle + self.steering_tolerance) / 90 ) * 100)
        
        #if debug printing is requested print speed and angle info
        if self.debug:
            self.delay_print -= 1
            if self.delay_print <= 0:
                self.delay_print = 5
                rospy.loginfo("Endpoint Angle: " + str(target_angle))
                rospy.loginfo("Endpoint Speed: " + str(target_speed))

        if self.state == STOPPED:
            self.brake = 0
            target_speed = 0
        elif self.state == BRAKING:
            if self.obstacle_distance > 0:
                # there exists an obstacle in the cart's path we need to stop for
               
                self.brake_time_used += (1.0/self.node_rate) # 1 sec / rate per sec (10)
                obstacle_brake_time = self.obstacle_distance/self.vel_curr - (1.0/self.node_rate) # we decrease by one node rate initially to account for rounding
                
                y = (0.1) * ((2550) ** (self.brake_time_used/obstacle_brake_time))

                if (y >= 255):
                    self.full_stop_count += 1

                self.brake = float(min(255, math.ceil(y)))
            else:
                # comfortable stop, no obstacle/deadline given

                self.brake_time_used += (1.0/self.node_rate) # 1 sec / rate per sec (10)
                brake_time = self.comfortable_stop_dist - (1.0/self.node_rate) # we decrease by one node rate initially to account for rounding
                
                y = (0.1) * ((2550) ** (self.brake_time_used/brake_time))

                if (y >= 255):
                    self.full_stop_count += 1

                self.brake = float(min(255, math.ceil(y)))
            if self.brake >= 255 and self.full_stop_count > 10:  # We have reached maximum braking!
                self.state = STOPPED
                # reset brake time used
                self.brake_time_used = 0
                self.full_stop_count = 0
            target_speed = 0

        self.pack_send(target_speed, int(self.brake), target_angle)
    

    def send_packet(self, throttle, brake, steer_angle):
        """This method is used to send instructions to the arduino that was connected in init.

        Args:
            throttle (_type_): _description_
            brake (_type_): _description_
            steer_angle (_type_): _description_
        """
        
        # This is a buffer used in pack_into essentially making 5 empty bytes
        data = bytearray(b"\x00" * 5)
        
        # We are assuming 42 21 is the magic number for the arduino
        bitstruct.pack_into(
            "u8u8u8u8u8", data, 0, 42, 21, abs(throttle), brake, steer_angle + STEERING_CORRECTION
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
