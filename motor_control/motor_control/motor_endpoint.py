#!/usr/bin/env python
"""
This is the python class that allows us to control the golf cart.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import time
import serial as sr
import numpy as np
import bitstruct
import math

# ROS based imports
# import tf_transformations
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from motor_control_interface.msg import VelCurr, VelAnglePlanned
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
        self.NODE_RATE = 0.1
        self.STEERING_TOLERANCE = 50
        self.COMFORT_STOP_DIST = 4.0
        self.STEERING_CORRECTION = 10

        # Driving vars
        self.state = STOPPED
        self.obstacle_distance = -1
        self.brake_time_used = 0
        self.brake = 0
        self.stopping_time = 0

        self.vel_planned = None
        self.angle_planned = None
        self.vel_curr = 1

        # Serial vars
        self.serial_connected = False
        self.heartbeat = b""
        self.prev_time = 0.0
        self.delta_time = 0.0

        self.manual_control = True

        # We need to look into getting this to be the right value/launch parameter
        # I think we can use this USB port but I wont know til i try
        # These are launch paramaters for now. They are given default values which are shown in the code blocks below
        self.declare_parameter("baudrate", "57600")
        # For this port we can do ls /dev/tty* and find the actual thing
        # ttyUSB0   ttyACM0
        self.declare_parameter("arduino_port", "/dev/ttyUSB9")

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

        #           self.debug_subscriber = self.create_subscription(
        #     Bool, "/realtime_debug_change", self.debug_callback, 10
        # )

        # self.curr_motion_subscriber = self.create_subscription(
        #     VelCurr, "/nav_cmd", self.vel_curr_callback, 10
        # )

        self.planned_motion_subscriber = self.create_subscription(
            VelAnglePlanned, "/nav_cmd", self.vel_angle_planned_callback, 10
        )

        self.heart_pub = self.create_publisher(String, "/heartbeat", 10)

        self.timer = self.create_timer(self.NODE_RATE, self.timer_callback)

    def vel_angle_planned_callback(self, planned_vel_angle):

        self.vel_planned = planned_vel_angle.vel_planned
        self.angle_planned = planned_vel_angle.angle_planned

        self.log_header(f"Planned Angle: {planned_vel_angle}")

        if self.vel_planned < 0:
            # indicates an obstacle
            self.obstacle_distance = abs(self.vel_planned)
            self.vel_planned = 0
        else:
            # reset obstacle distance and brake time
            self.obstacle_distance = -1
            self.brake_time_used = 0
            self.full_stop_count = 0

        if (
            self.vel_planned > 0
            and (self.state == STOPPED or self.state == BRAKING)
            and (time.time() - self.stopping_time) > 10
        ):
            self.state = MOVING
            self.brake = 0  # take the foot off the brake
        elif self.state == MOVING and self.vel_planned <= 0:  # Brakes are hit
            self.state = BRAKING
            self.brake = 0  # ramp up braking from 0
            self.stopping_time = time.time()

        self.new_vel = True

    # DONT WORRY AB THIS FOR NOW XD
    def vel_curr_callback(self, vel_angle):
        pass

    #     """
    #     Callback for driving commands.
    #     """
    #     self.vel_curr = vel_angle.vel_curr

    #     if self.vel < 0:
    #         # indicates an obstacle
    #         self.obstacle_distance = abs(self.vel)
    #         self.vel = 0
    #     else:
    #         # reset obstacle distance and brake time
    #         self.obstacle_distance = -1
    #         self.brake_time_used = 0
    #         self.full_stop_count = 0

    #     if (
    #         self.vel > 0
    #         and (self.state == STOPPED or self.state == BRAKING)
    #         and (time.time() - self.stopping_time) > 10
    #     ):
    #         self.state = MOVING
    #         self.brake = 0  # take the foot off the brake
    #     elif self.state == MOVING and self.vel <= 0:  # Brakes are hit
    #         self.state = BRAKING
    #         self.brake = 0  # ramp up braking from 0
    #         self.stopping_time = time.time()

    #     self.new_vel = True

    def timer_callback(self):
        """
        Main loop timer for updating motor's instructions"""
        if not self.serial_connected:
            self.log_header("RETRYING SERIAL CONNECTION")

            # FIXME MAKE THIS IN A METHOD
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

                # We are doing this instead of rate.sleep() and continue in the original implementation main loop
                return

        # Need to do this but better somehow and i dont know what they are doing tbh.
        if self.vel_planned is not None and self.angle_planned is not None:
            if self.manual_control:
                self.manual_endpoint()
            else:
                self.calculate_endpoint()
        self.prev_time = time.time()

        try:
            self.log_header("getting in the heartbeat try")
            self.heartbeat = self.arduino_ser.read_until()
        except Exception as e:
            self.log_header("THE ARDUINO HAS BEEN DISCONNECTED")
            self.serial_connected = False

            # Add a method that attempts to reconnect to the arduino and then return
            return

        if self.heartbeat != "":

            # Some of these variables need to be renamed/check on
            self.heart_pub.publish(self.heartbeat)
            self.delta_time = time.time() - self.prev_time
            self.log_header("Heartbeat message:")
            self.log_header(f"{self.heartbeat} | Time since last message: ")
            heartbeat_delta_t = time.time() - self.prev_time

            # This check is here because the time between the first and 2nd heartbeat is always ~2.4s
            # I believe this is because of the rest of the setup taking place at the same time
            # We need to initialize first_heartbeat first
            if heartbeat_delta_t >= 2.0:
                self.log_header("TIME BETWEEN HEARTBEATS, > 2.0s | Things may be fine")

            self.log_header(f"Heartbeat delta: {heartbeat_delta_t}")
        # This is here to emmulate rate.sleep() from the previous implementation
        return

    def manual_endpoint(self):

        if self.new_vel:
            # The first time we get a new target speed and angle we must convert it
            # Need to come back to vel_cart_units later because it involves getting the planned velocity
            # self.vel_cart_units = self.vel_planned

            self.vel_cart_units = self.vel_planned
            self.new_vel = False

            # Why was this hard coded this way? What is it even doing
            self.vel_cart_units *= (
                50  # Rough conversion from m/s to cart controller units
            )

            # i guess this logic makes sense but also i dont understand what "cart controller units" are
            # and is it possible to get a better estimate?
            if self.vel_cart_units > 254:
                self.vel_cart_units = 254
            if self.vel_cart_units < -254:
                self.vel_cart_units = -254
            if self.vel_cart_units < 0:
                self.self.log_header(
                    "NEGATIVE VELOCITY REQUESTED FOR THE MOTOR ENDPOINT!"
                )

        target_speed = int(self.vel_cart_units)  # float64

        # adjust the target_angle range from (-45 <-> 45) to (0 <-> 100)

        if self.angle_planned < -40:
            self.angle_planned = self.STEERING_TOLERANCE * -1
        if self.angle_planned > 40:
            self.angle_planned = self.STEERING_TOLERANCE
        target_angle = 100 - int(
            ((self.angle_planned + self.STEERING_TOLERANCE) / 90) * 100
        )

        if self.state == STOPPED:
            self.brake = 0
            target_speed = 0

        elif self.state == BRAKING:

            # comfortable stop, no obstacle/deadline given

            self.brake_time_used += 1.0 / self.NODE_RATE  # 1 sec / rate per sec (10)
            brake_time = self.COMFORT_STOP_DIST - (
                1.0 / self.NODE_RATE
            )  # we decrease by one node rate initially to account for rounding

            brake_rate = (0.1) * ((2550) ** (self.brake_time_used / brake_time))

            if brake_rate >= 255:
                self.full_stop_count += 1

            self.brake = float(min(255, math.ceil(brake_rate)))
            if (
                self.brake >= 255 and self.full_stop_count > 10
            ):  # We have reached maximum braking!
                self.state = STOPPED
                # reset brake time used
                self.brake_time_used = 0
                self.full_stop_count = 0
        ## DEL THIS
        if self.brake < 0:
            self.brake = 0
        self.send_packet(target_speed, int(self.brake), target_angle)

    def calculate_endpoint(self):
        if self.new_vel:
            # The first time we get a new target speed and angle we must convert it
            # Need to come back to vel_cart_units later because it involves getting the planned velocity
            # self.vel_cart_units = self.vel_planned

            self.vel_cart_units = self.vel_planned
            self.vel_curr_cart_units = self.vel_curr
            self.new_vel = False

            # Why was this hard coded this way? What is it even doing
            self.vel_cart_units *= (
                50  # Rough conversion from m/s to cart controller units
            )

            self.vel_curr_cart_units *= (
                50  # Rough conversion from m/s to cart controller units
            )

            # i guess this logic makes sense but also i dont understand what "cart controller units" are
            # and is it possible to get a better estimate?
            if self.vel_cart_units > 254:
                self.vel_cart_units = 254
            if self.vel_cart_units < -254:
                self.vel_cart_units = -254
            if self.vel_curr_cart_units > 254:
                self.vel_curr_cart_units = 254
            if self.vel_cart_units < 0:
                self.log_header("NEGATIVE VELOCITY REQUESTED FOR THE MOTOR ENDPOINT!")

        target_speed = int(self.vel_cart_units)  # float64
        # current_speed = int(self.vel_curr_cart_units) float64

        # adjust the target_angle range from (-45 <-> 45) to (0 <-> 100)

        if self.angle_planned < -40:
            self.angle_planned = self.STEERING_TOLERANCE * -1
        if self.angle_planned > 40:
            self.angle_planned = self.STEERING_TOLERANCE
        target_angle = 100 - int(
            ((self.angle_planned + self.STEERING_TOLERANCE) / 90) * 100
        )

        if self.state == STOPPED:
            self.brake = 0
            target_speed = 0

        elif self.state == BRAKING:

            target_speed = 0

            # THIS IS ALL THE MATH FOR THE ACTUAL BREAKING
            if self.obstacle_distance > 0:
                # there exists an obstacle in the cart's path we need to stop for

                self.brake_time_used += (
                    1.0 / self.NODE_RATE
                )  # 1 sec / rate per sec (10)
                obstacle_brake_time = self.obstacle_distance / self.vel_curr - (
                    1.0 / self.NODE_RATE
                )  # we decrease by one node rate initially to account for rounding

                brake_rate = (0.1) * (
                    (2550) ** (self.brake_time_used / obstacle_brake_time)
                )

                if brake_rate >= 255:
                    self.full_stop_count += 1

                self.brake = float(min(255, math.ceil(brake_rate)))
            else:
                # comfortable stop, no obstacle/deadline given

                self.brake_time_used += (
                    1.0 / self.NODE_RATE
                )  # 1 sec / rate per sec (10)
                brake_time = self.COMFORT_STOP_DIST - (
                    1.0 / self.NODE_RATE
                )  # we decrease by one node rate initially to account for rounding

                brake_rate = (0.1) * ((2550) ** (self.brake_time_used / brake_time))

                if brake_rate >= 255:
                    self.full_stop_count += 1

                self.brake = float(min(255, math.ceil(brake_rate)))
            if (
                self.brake >= 255 and self.full_stop_count > 10
            ):  # We have reached maximum braking!
                self.state = STOPPED
                # reset brake time used
                self.brake_time_used = 0
                self.full_stop_count = 0

        self.send_packet(target_speed, int(self.brake), target_angle)

    def send_packet(self, throttle, brake, steer_angle):
        """This method is used to send instructions to the arduino that was connected in init.

        Args:
            throttle (_type_): _description_
            brake (_type_): _description_
            steer_angle (_type_): _description_
        """

        # This is a buffer used in pack_into essentially making 5 empty bytes
        data = bytearray(b"\x00" * 5)
        self.log_header(f"Steer angle: {steer_angle}")
        # We are assuming 42 21 is the magic number for the arduino
        bitstruct.pack_into(
            "u8u8u8u8u8",
            data,
            0,
            42,
            21,
            abs(throttle),
            brake,
            steer_angle + self.STEERING_CORRECTION,
        )
        self.arduino_ser.write(data)
        self.log_header("\nHERE\n")

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
