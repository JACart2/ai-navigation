#!/usr/bin/env python
"""
This is the ROS2 node that allows us to control the golf cart.
It sends messages to the arduino controller based on information received from ROS2 topics.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
# Python based imports
import time
import serial as sr
import bitstruct
import math

# ROS based imports
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from motor_control_interface.msg import VelAngle
from std_msgs.msg import Bool, String
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

# Annolmaly Logging based imports
from std_msgs.msg import Float32 
from anomaly_msg.msg import AnomalyMsg 
import struct

# State constants
MOVING = 0
BRAKING = 1
STOPPED = 2



class MotorEndpoint(rclpy.node.Node):
    """ROS2 node that handles controlling the motor."""

    def __init__(self):
        super().__init__("motor_endpoint")

        # Class constants
        self.BRAKE_TIME = 3
        self.NODE_RATE = 10
        self.STEERING_TOLERANCE = 50
        self.COMFORT_STOP_DIST = 4.0
        self.STEERING_CORRECTION = 10
        
        self.declare_parameter("enable_aad", True)

        self.AAD_LOGGING_ENABLED = (
            self.get_parameter("enable_aad")
            .get_parameter_value()
            .bool_value
        )

        # Driving vars
        self.state = STOPPED
        self.obstacle_distance = -1
        self.brake_time_used = 0
        self.brake = 0
        self.stopping_time = 0
        self.vel = 0
        self.vel_planned = None
        self.angle_planned = None
        self.vel_curr = 0

        # Serial vars
        self.serial_connected = False
        self.heartbeat = b""
        self.prev_time = time.time()
        self.last_heartbeat_time = time.time()
        self.last_reported_state = self.state
        self.last_reported_manual_control = None
        self.heartbeat_was_unhealthy = False
        self.serial_retry_reported = False

        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("arduino_port", "/dev/ttyUSB0")
        self.declare_parameter("manual_control", False)

        self.BAUDRATE = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )
        self.ARDUINO_PORT = (
            self.get_parameter("arduino_port").get_parameter_value().string_value
        )
        self.manual_control = (
            self.get_parameter("manual_control").get_parameter_value().bool_value
        )  # Sets the cart to use teleop control logic instead of autonomous control


        # Sets up publishing to /ai_anomaly_logging
        if self.AAD_LOGGING_ENABLED:
            self.aad_pub = self.create_publisher(
                AnomalyMsg,
                "/ai_anomaly_logging",
                10)

        # Need to sleep after first connection to let serial establish
        try:
            self.arduino_ser = sr.Serial(
                self.ARDUINO_PORT, baudrate=self.BAUDRATE, write_timeout=0, timeout=0.01
            )
            time.sleep(2)
            self.serial_connected = True
            self.serial_retry_reported = False
            self.log_header("CONNECTED TO ARDUINO")

            self.log_aad(AnomalyMsg.INFO, "CONNECTED TO ARDUINO")
                
        except Exception as e:
            self.log_header("MOTOR ENDPOINT: " + str(e))
            self.serial_connected = False
            
            self.log_aad(AnomalyMsg.ERROR, "MOTOR ENDPOINT: " + str(e))

        # ROS2 SUBSCRIBERS

        self.planned_motion_subscriber = self.create_subscription(
            VelAngle, "/nav_cmd", self.vel_angle_planned_callback, 10
        )

        # The linear and angular velocity of the cart from NDT Matching
        self.twist_sub = self.create_subscription(
            TwistStamped, "/estimate_twist", self.vel_curr_callback, 10
        )

        self.manual_sub = self.create_subscription(
            Bool, "/set_manual_control", self.manual_callback, 10
        )

        # ROS2 PUBLISHERS

        # heartbeat is used to ensure that we have a stable connection with the ardiuno
        self.heart_pub = self.create_publisher(String, "/heartbeat", 10)

        # ROS2 TIMERS
        self.timer = self.create_timer(1.0 / self.NODE_RATE, self.timer_callback)

    def vel_angle_planned_callback(self, planned_vel_angle):
        """
        Callback method to get the target velocity and angle.
        This is achieved by using the subscription we created in init to recieved a
        message (planned_vel_angle) and set the appropriate fields to make the cart drive/turn.
        """

        self.vel_planned = planned_vel_angle.vel
        self.angle_planned = planned_vel_angle.angle

        self.log_header(f"Planned Angle: {planned_vel_angle}")

        # This logic should be changed in the future, but basically if the velocity that we plan to go is negative,
        # then the velocity is interpreted as the distance to an obstacle. The braking using this variable is handled in
        # calculate endpoint. We have not made use of this as of 4/10/24.

        if self.vel_planned < 0:
            # indicates an obstacle
            self.obstacle_distance = abs(self.vel_planned)
            self.vel_planned = 0
            self.log_aad(
                AnomalyMsg.WARNING,
                f"Obstacle braking command received: distance={self.obstacle_distance:.2f}m",
            )
        else:
            # reset obstacle distance and brake time
            self.obstacle_distance = -1
            self.brake_time_used = 0
            self.full_stop_count = 0

        # Setting some class variables about the state of the cart here given what our instructed velocity/angle is
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

        self.report_state_change()
        self.new_vel = True

    def vel_curr_callback(self, vel_twist):
        """Callback for getting the estimated current velocity. As of right now this speed
        estimate is coming from a ROS2 node called speed_node.py."""
        if vel_twist != None:
            self.vel_curr = vel_twist.twist.linear.x

    def manual_callback(self, msg):
        """Callback that sets manual control bool to indicate teleop vs auto control."""
        self.manual_control = msg.data
        if self.last_reported_manual_control != self.manual_control:
            mode = "manual" if self.manual_control else "autonomous"
            self.log_aad(AnomalyMsg.INFO, f"Motor endpoint control mode changed to {mode}")
            self.last_reported_manual_control = self.manual_control

    def connect_arduino(self):
        """Simple method for retrying/trying serial connection."""
        was_connected = self.serial_connected
        try:
            self.arduino_ser = sr.Serial(
                self.ARDUINO_PORT,
                baudrate=self.BAUDRATE,
                write_timeout=0,
                timeout=0.01,
            )
            self.serial_connected = True
            self.serial_retry_reported = False
            if not was_connected:
                self.log_aad(AnomalyMsg.INFO, "Arduino serial connection restored")
        except Exception as e:
            self.log_header("MOTOR ENDPOINT: " + str(e))

            self.log_aad(
                AnomalyMsg.ERROR,
                "MOTOR ENDPOINT: " + str(e),
            )
            
            self.serial_connected = False
            
    def timer_callback(self):
        """Main loop timer for updating motor's instructions."""

        if not self.serial_connected:
            self.log("RETRYING SERIAL CONNECTION")
            if not self.serial_retry_reported:
                self.log_aad(
                    AnomalyMsg.WARNING,
                    f"Retrying Arduino serial connection on {self.ARDUINO_PORT}",
                )
                self.serial_retry_reported = True
            self.connect_arduino()
            # Wait for the timer to start over in the event of an error
            if not self.serial_connected:
                return
    
        # Check if we have received a target yet
        if self.vel_planned is not None and self.angle_planned is not None:

            # Switching between our autonomous implimentation and our "teleop" implimentation here.
            if self.manual_control:
                # Use a different endpoint for driving if ignoring current velocity
                self.manual_endpoint()
            else:
                # Use the autonomous implimentation
                self.calculate_endpoint()

        # The heartbeat is a message sent from the arduino which provides the steering target, throttle target,
        # and brake target as comma separated numbers
        try:
            self.heartbeat = self.arduino_ser.read_until()
        except Exception as e:
            self.log_header("THE ARDUINO HAS BEEN DISCONNECTED")
            
            self.log_aad(AnomalyMsg.ERROR, "THE ARDUINO HAS BEEN DISCONNECTED")

            # Same thing as above. if the ardiuno had some problems... ie: it disconnected attempt to retry the connection.
            # Return to end the current instance of the time callback we are in if it fails to connect.
            self.connect_arduino()
            if not self.serial_connected:
                return
        cur_time = time.time()
        if self.heartbeat != "":
            self.heart_pub.publish(self.heartbeat)
            heartbeat_delta_t = time.time() - self.prev_time
            self.last_heartbeat_time = cur_time
            self.log_header(
                f"Heartbeat message:\n{self.heartbeat} | Time since last message: {heartbeat_delta_t}"
            )
            if self.heartbeat_was_unhealthy:
                self.log_aad(
                    AnomalyMsg.INFO,
                    f"Arduino heartbeat recovered: delta={heartbeat_delta_t:.2f}s",
                )
                self.heartbeat_was_unhealthy = False

            # This check is here because the time between the first and 2nd heartbeat is always ~2.4s
            # This is because of the rest of the setup taking place at the same time
  
            if heartbeat_delta_t >= 2.0:
                self.log_header("TIME BETWEEN HEARTBEATS, > 2.0s | Things may be fine")
                
                if not self.heartbeat_was_unhealthy:
                    self.log_aad(
                        AnomalyMsg.WARNING,
                        f"Time between Arduino heartbeats is high: delta={heartbeat_delta_t:.2f}s",
                    )
                self.heartbeat_was_unhealthy = True
        elif self.serial_connected and (cur_time - self.last_heartbeat_time) >= 2.0:
            if not self.heartbeat_was_unhealthy:
                self.log_aad(
                    AnomalyMsg.WARNING,
                    f"No Arduino heartbeat received for {cur_time - self.last_heartbeat_time:.2f}s",
                )
            self.heartbeat_was_unhealthy = True

        self.prev_time = cur_time
        return

    def manual_endpoint(self):
        """Alternative endpoint for processing and sending instructions to arduino for use
        when current velocity is ignored. This is helpful when using teleop for control.
        """

        if self.new_vel:
            self.vel_cart_units = self.vel_planned
            self.new_vel = False

            # The first time we get a new target velocity we must convert it for the arduino.
            # May need to get a better estimate later on.
            self.vel_cart_units *= (
                50  # Rough conversion from m/s to cart controller units
            )

            if self.vel_cart_units > 254:
                self.vel_cart_units = 254
            if self.vel_cart_units < -254:
                self.vel_cart_units = -254
            if self.vel_cart_units < 0:
                self.log_header("NEGATIVE VELOCITY REQUESTED FOR THE MOTOR ENDPOINT!")
                
                self.log_aad(AnomalyMsg.ERROR, "NEGATIVE VELOCITY REQUESTED FOR THE MOTOR ENDPOINT!")

        target_speed = int(self.vel_cart_units)  # float64

        # Adjusts the target_angle range from (-40 <-> 40) to (0 <-> 100)
        angle_planned = max(-40, min(40, self.angle_planned))
        target_angle = 100 - int(
            ((angle_planned + self.STEERING_TOLERANCE) / 90) * 100
        )

        if self.state == STOPPED:
            self.brake = 0
            target_speed = 0

        elif self.state == BRAKING:

            # Comfortable stop, no obstacle/deadline given
            self.brake_time_used += 1.0 / self.NODE_RATE  # 1 sec / rate per sec (10)
            brake_time = self.COMFORT_STOP_DIST - (
                1.0 / self.NODE_RATE
            )  # Decrease by one node rate initially to account for rounding

            brake_rate = (0.1) * ((2550) ** (self.brake_time_used / brake_time))

            if brake_rate >= 255:
                self.full_stop_count += 1

            self.brake = float(min(255, math.ceil(brake_rate)))
            if (
                self.brake >= 255 and self.full_stop_count > 10
            ):  # We have reached maximum braking!
                self.state = STOPPED
                self.report_state_change()
                # Reset brake time used
                self.brake_time_used = 0
                self.full_stop_count = 0

        # Should not be needed, accounts for invalid braking
        if self.brake < 0:
            self.brake = 0
        self.send_packet(target_speed, int(self.brake), target_angle)

    def calculate_endpoint(self):
        """The endpoint for processing and sending instructions to the arduino controller.
        As opposed to manual endpoint this is used for autonomous driving"""
        if self.new_vel:

            self.vel_cart_units = self.vel_planned
            self.vel_curr_cart_units = self.vel_curr
            self.new_vel = False

            # The first time we get a new target velocity we must convert it for the arduino.
            # May need to get a better estimate later on.
            self.vel_cart_units *= (
                50  # Rough conversion from m/s to cart controller units
            )

            self.vel_curr_cart_units *= (
                50  # Rough conversion from m/s to cart controller units
            )

            # Planned velocity checks
            if self.vel_cart_units > 254:
                self.vel_cart_units = 254
            if self.vel_cart_units < -254:
                self.vel_cart_units = -254

            # Current velocity checks
            if self.vel_curr_cart_units > 254:
                self.vel_curr_cart_units = 254
            if self.vel_cart_units < 0:
                self.log_header("NEGATIVE VELOCITY REQUESTED FOR THE MOTOR ENDPOINT!")
                
                self.log_aad(AnomalyMsg.ERROR, "NEGATIVE VELOCITY REQUESTED FOR THE MOTOR ENDPOINT!")
                    
        target_speed = int(self.vel_cart_units)  # float64

        # Adjust the target_angle range from (-40 <-> 40) to (0 <-> 100)
        angle_planned = max(-40, min(40, self.angle_planned))

        target_angle = 100 - int(
            ((angle_planned + self.STEERING_TOLERANCE) / 90) * 100
        )

        if self.state == STOPPED:
            self.brake = 0
            target_speed = 0

        elif self.state == BRAKING:

            target_speed = 0

            # Calculation for braking
            if self.obstacle_distance > 0:
                # There exists an obstacle in the cart's path we need to stop for

                self.brake_time_used += (
                    1.0 / self.NODE_RATE
                )  # 1 sec / rate per sec (10)

                obstacle_brake_time = self.obstacle_distance / self.vel_curr - (
                    1.0 / self.NODE_RATE
                )  # We decrease by one node rate initially to account for rounding

                brake_rate = (0.1) * (
                    (2550) ** (self.brake_time_used / obstacle_brake_time)
                )

                if brake_rate >= 255:
                    self.full_stop_count += 1
            else:
                # Comfortable stop, no obstacle/deadline given
                self.brake_time_used += (
                    1.0 / self.NODE_RATE
                )  # 1 sec / rate per sec (10)
                brake_time = self.COMFORT_STOP_DIST - (
                    1.0 / self.NODE_RATE
                )  # We decrease by one node rate initially to account for rounding

                brake_rate = (0.1) * ((2550) ** (self.brake_time_used / brake_time))

                if brake_rate >= 255:
                    self.full_stop_count += 1

            self.brake = float(min(255, math.ceil(brake_rate)))
            if (
                self.brake >= 255 and self.full_stop_count > 10
            ):  # We have reached maximum braking!
                self.state = STOPPED
                self.report_state_change()
                # Reset brake time used
                self.brake_time_used = 0
                self.full_stop_count = 0

        self.send_packet(target_speed / 1.7, int(self.brake), target_angle)

    def send_packet(self, throttle, brake, steer_angle):
        """This method is used to send instructions to the arduino that was connected in init."""

        # This is a buffer used in pack_into essentially making 5 empty bytes
        data = bytearray(b"\x00" * 5)

        # 42 21 is the magic number for the arduino
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

    def log_header(self, msg):
        """Helper method to print noticeable log statements."""
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"{msg}")
        self.get_logger().info("=" * 50)

    def log(self, msg):
        """Helper method to print  log tatements."""
        self.get_logger().info(f"{msg}")

    def report_state_change(self):
        if self.state == self.last_reported_state:
            return

        state_name = {
            MOVING: "MOVING",
            BRAKING: "BRAKING",
            STOPPED: "STOPPED",
        }.get(self.state, f"UNKNOWN({self.state})")
        importance = AnomalyMsg.WARNING if self.state == BRAKING else AnomalyMsg.INFO
        self.log_aad(
            importance,
            f"Motor endpoint state changed to {state_name}; planned_vel={self.vel_planned}, current_vel={self.vel_curr:.2f}",
        )
        self.last_reported_state = self.state

    # This is for publishing to anomaly logging
    def log_aad(
        self,
        importance: int,
        motor_endpoint_msg: str,
    ):
        if not self.AAD_LOGGING_ENABLED:
            return

        anomaly = AnomalyMsg() 
        
        # Header 
        anomaly.header = Header()
        anomaly.header.stamp = self.get_clock().now().to_msg() 
        anomaly.header.frame_id = "motor_endpoint_frame" 
        
        # Required fields 
        anomaly.node_name = self.get_name() 
        anomaly.importance = importance
        anomaly.type = AnomalyMsg.TEXT 
        
        # Human-readable message 
        anomaly.msg = f"Received Motor Endpoint Info: {motor_endpoint_msg}" 
        #Publish 
        self.aad_pub.publish(anomaly) 

def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = MotorEndpoint()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
