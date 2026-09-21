#!/usr/bin/env python
"""
This is the ROS2 node that allows us to control the golf cart.
It sends messages to the arduino controller based on information received from ROS2 topics.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
"""
import math
import time

import bitstruct
import serial as sr

# ROS based imports
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly
import rclpy
from motor_control_interface.msg import VelAngle
from std_msgs.msg import Bool, String
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

# Annolmaly Logging based imports
from std_msgs.msg import Float32 
try:
    from anomaly_msg.msg import AnomalyMsg
    LEGACY_ANOMALY_MSG = True
except ImportError:
    from anomaly_msg.msg import AnomalyLog as AnomalyMsg
    LEGACY_ANOMALY_MSG = False
    AnomalyMsg.INFO = "INFO"
    AnomalyMsg.WARNING = "WARNING"
    AnomalyMsg.ERROR = "ERROR"
    AnomalyMsg.TEXT = "TEXT"
 
import struct

# State constants
MOVING = 0
BRAKING = 1
STOPPED = 2
DEFAULT_AUTONOMOUS_THROTTLE_DIVIDER = 1.8
TELEMETRY_RATE_HZ = 1.0



class MotorEndpoint(rclpy.node.Node):
    """ROS2 node that handles controlling the motor."""

    def __init__(self):
        super().__init__("motor_endpoint")

        # Class constants
        self.BRAKE_TIME = 3
        self.NODE_RATE = 10  # Hz
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
        self.full_stop_count = 0
        self.brake = 0
        self.stopping_time = 0
        self.vel = 0
        self.vel_planned = None
        self.angle_planned = None
        self.vel_curr = 0
        self.last_vel_curr_received_monotonic = None

        # Serial vars
        self.serial_connected = False
        self.heartbeat = b""
        self.prev_time = time.time()
        self.last_heartbeat_time = time.time()
        self.last_reported_state = self.state
        self.last_reported_manual_control = None
        self.heartbeat_was_unhealthy = False
        self.serial_retry_reported = False
        self.last_arduino_throttle_command = 0
        self.last_arduino_brake_command = 0
        self.last_arduino_steering_command = None
        self.estimated_yaw_rate = 0.0

        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("arduino_port", "/dev/ttyUSB0")
        self.declare_parameter("manual_control", False)
        self.declare_parameter("autonomous_mps_to_controller_units", 75.0)
        self.declare_parameter("wheel_base", 2.4003)
        self.declare_parameter(
            "anomaly_telemetry_period_seconds", 1.0 / TELEMETRY_RATE_HZ
        )

        self.BAUDRATE = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )
        self.ARDUINO_PORT = (
            self.get_parameter("arduino_port").get_parameter_value().string_value
        )
        self.manual_control = (
            self.get_parameter("manual_control").get_parameter_value().bool_value
        )  # Sets the cart to use teleop control logic instead of autonomous control
        self.autonomous_mps_to_controller_units = (
            self.get_parameter("autonomous_mps_to_controller_units")
            .get_parameter_value()
            .double_value
        )
        self.wheel_base = (
            self.get_parameter("wheel_base").get_parameter_value().double_value
        )
        self.anomaly_telemetry_period_seconds = max(
            0.0,
            self.get_parameter("anomaly_telemetry_period_seconds")
            .get_parameter_value()
            .double_value,
        )
        # Sets up publishing to /ai_anomaly_logging
        if self.AAD_LOGGING_ENABLED:
            self.aad_pub = self.create_publisher(
                AnomalyMsg,
                "/ai_anomaly_logging",
                10)

        self.connect_arduino(initial_connection=True)

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
        self.telemetry_timer = None
        if (
            self.AAD_LOGGING_ENABLED
            and self.anomaly_telemetry_period_seconds > 0.0
        ):
            self.telemetry_timer = self.create_timer(
                self.anomaly_telemetry_period_seconds,
                self.publish_anomaly_telemetry,
            )

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
            self.estimated_yaw_rate = vel_twist.twist.angular.z
            self.last_vel_curr_received_monotonic = time.monotonic()

    def manual_callback(self, msg):
        """Callback that sets manual control bool to indicate teleop vs auto control."""
        self.manual_control = msg.data
        if self.last_reported_manual_control != self.manual_control:
            mode = "manual" if self.manual_control else "autonomous"
            self.log_aad(AnomalyMsg.INFO, f"Motor endpoint control mode changed to {mode}")
            self.last_reported_manual_control = self.manual_control

    def report_state_change(self):
        """Publish an anomaly log when the motor endpoint state changes."""
        if self.last_reported_state == self.state:
            return

        state_names = {
            MOVING: "moving",
            BRAKING: "braking",
            STOPPED: "stopped",
        }
        previous_state = state_names.get(
            self.last_reported_state,
            str(self.last_reported_state),
        )
        current_state = state_names.get(self.state, str(self.state))
        self.log_aad(
            AnomalyMsg.INFO,
            f"Motor endpoint state changed from {previous_state} to {current_state}",
        )
        self.last_reported_state = self.state

    def connect_arduino(self, initial_connection=False):
        """Simple method for retrying/trying serial connection."""
        was_connected = self.serial_connected
        try:
            self.arduino_ser = sr.Serial(
                self.ARDUINO_PORT,
                baudrate=self.BAUDRATE,
                write_timeout=0,
                timeout=0.01,
            )
            if initial_connection:
                # Give the serial device time to finish its initial setup.
                time.sleep(2)
            self.serial_connected = True
            self.serial_retry_reported = False
            if initial_connection:
                self.log_header("CONNECTED TO ARDUINO")
                self.log_aad(AnomalyMsg.INFO, "CONNECTED TO ARDUINO")
            elif not was_connected:
                self.log_aad(AnomalyMsg.INFO, "Arduino serial connection restored")
        except Exception as e:
            self.log_header("MOTOR ENDPOINT: " + str(e))

            self.log_aad(
                AnomalyMsg.ERROR,
                "MOTOR ENDPOINT: " + str(e),
            )
            
            self.serial_connected = False

    def speed_to_controller_units(self, speed_mps, multiplier):
        """Convert an m/s request to the Arduino's signed controller range."""
        controller_units = speed_mps * multiplier
        controller_units = max(-254, min(254, controller_units))
        if controller_units < 0:
            message = "NEGATIVE VELOCITY REQUESTED FOR THE MOTOR ENDPOINT!"
            self.log_header(message)
            self.log_aad(AnomalyMsg.ERROR, message)
        return controller_units

    def steering_to_controller_units(self, angle):
        """Convert a steering request in degrees to the Arduino's 0-100 range."""
        angle = max(-40, min(40, angle))
        return 100 - int(((angle + self.STEERING_TOLERANCE) / 90) * 100)

    def calculate_comfort_brake(self):
        """Advance and return the comfortable-stop brake ramp."""
        self.brake_time_used += 1.0 / self.NODE_RATE
        brake_time = self.COMFORT_STOP_DIST - (1.0 / self.NODE_RATE)
        return (0.1) * ((2550) ** (self.brake_time_used / brake_time))

    def apply_brake_rate(self, brake_rate):
        """Apply a brake ramp value and finish the stop once braking settles."""
        if brake_rate >= 255:
            self.full_stop_count += 1

        self.brake = float(min(255, math.ceil(brake_rate)))
        if self.brake >= 255 and self.full_stop_count > 10:
            self.state = STOPPED
            self.report_state_change()
            self.brake_time_used = 0
            self.full_stop_count = 0
            
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
        if self.heartbeat:
            heartbeat_msg = String()
            heartbeat_msg.data = self.heartbeat.decode("utf-8", errors="replace").strip()
            self.heart_pub.publish(heartbeat_msg)
            heartbeat_delta_t = time.time() - self.prev_time
            self.last_heartbeat_time = cur_time
            self.log_header(
                f"Heartbeat message:\n{heartbeat_msg.data} | Time since last message: {heartbeat_delta_t}"
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
            self.vel_cart_units = self.speed_to_controller_units(
                self.vel_planned, 50
            )
            self.new_vel = False

        target_speed = int(self.vel_cart_units)  # float64

        target_angle = self.steering_to_controller_units(self.angle_planned)

        if self.state == STOPPED:
            self.brake = 0
            target_speed = 0

        elif self.state == BRAKING:

            self.apply_brake_rate(self.calculate_comfort_brake())

        # Should not be needed, accounts for invalid braking
        if self.brake < 0:
            self.brake = 0
        self.send_packet(target_speed, int(self.brake), target_angle)

    def calculate_endpoint(self):
        """The endpoint for processing and sending instructions to the arduino controller.
        As opposed to manual endpoint this is used for autonomous driving"""
        if self.new_vel:

            self.vel_cart_units = self.speed_to_controller_units(
                self.vel_planned, self.autonomous_mps_to_controller_units
            )
            self.vel_curr_cart_units = min(254, self.vel_curr * 50)
            self.new_vel = False

            # The first time we get a new target velocity we must convert it for the arduino.
            # May need to get a better estimate later on.
                    
        target_speed = int(self.vel_cart_units)  # float64

        # Adjust the target_angle range from (-40 <-> 40) to (0 <-> 100)
        target_angle = self.steering_to_controller_units(self.angle_planned)

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

            else:
                # Comfortable stop, no obstacle/deadline given
                brake_rate = self.calculate_comfort_brake()

            self.apply_brake_rate(brake_rate)

        self.send_packet(target_speed, int(self.brake), target_angle)

    def send_packet(self, throttle, brake, steer_angle):
        """This method is used to send instructions to the arduino that was connected in init."""

        # Preserve the exact effective values for diagnostics. Throttle is an
        # unsigned controller command, not a physical speed or km/h value.
        self.last_arduino_throttle_command = max(
            0,
            min(255, int(abs(throttle))),
        )
        self.last_arduino_brake_command = max(
            0,
            min(255, int(brake)),
        )
        self.last_arduino_steering_command = max(
            0,
            min(255, int(steer_angle + self.STEERING_CORRECTION)),
        )

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

    def _anomaly_telemetry_message(self):
        """Return the available motor and steering observations as plain text."""
        requested_steering = MotorEndpoint._format_steering_angle(self.angle_planned)
        estimated_steering = "unavailable"
        if (
            self.last_vel_curr_received_monotonic is not None
            and abs(self.vel_curr) >= 0.1
        ):
            estimated_angle_deg = math.degrees(
                math.atan(self.wheel_base * self.estimated_yaw_rate / self.vel_curr)
            )
            estimated_steering = MotorEndpoint._format_steering_angle(
                estimated_angle_deg
            )

        return (
            f"Motor telemetry: requested_steering={requested_steering}, "
            f"estimated_steering={estimated_steering}, "
            f"arduino_steering_command={self.last_arduino_steering_command}"
        )

    @staticmethod
    def _format_steering_angle(angle_deg):
        """Format a steering angle as a magnitude in degrees and a direction."""
        if angle_deg is None:
            return "unavailable"
        if angle_deg > 0:
            direction = "left"
        elif angle_deg < 0:
            direction = "right"
        else:
            direction = "straight"
        return f"{abs(angle_deg):.2f} deg {direction}"

    def publish_anomaly_telemetry(self):
        """Periodically provide plain-text motor context to anomaly detection."""
        self.log_aad(
            AnomalyMsg.INFO,
            self._anomaly_telemetry_message(),
            node_name="motor_endpoint",
        )

    def log_header(self, msg):
        """Helper method to print noticeable log statements."""
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"{msg}")
        self.get_logger().info("=" * 50)

    def log(self, msg):
        """Helper method to print  log tatements."""
        self.get_logger().info(f"{msg}")

    # This is for publishing to anomaly logging
    def log_aad(
        self,
        importance: int,
        motor_endpoint_msg: str,
        node_name=None,
    ):
        """Publish motor endpoint info to anomaly logging."""
        if not self.AAD_LOGGING_ENABLED:
            return

        anomaly = AnomalyMsg()

        if LEGACY_ANOMALY_MSG:
            anomaly.header = Header()
            anomaly.header.stamp = self.get_clock().now().to_msg()
            anomaly.header.frame_id = "motor_endpoint_frame"
            anomaly.node_name = node_name or self.get_name()
            anomaly.importance = importance
            anomaly.type = AnomalyMsg.TEXT
            anomaly.msg = f"Motor Endpoint Info: {motor_endpoint_msg}"
        else:
            anomaly.stamp = self.get_clock().now().to_msg()
            anomaly.node_name = node_name or self.get_name()
            anomaly.source = "motor_control"
            anomaly.description = f"{importance}: Motor Endpoint Info: {motor_endpoint_msg}"
            anomaly.topic_name = "/motor_endpoint"
            anomaly.data_type = "text"
            anomaly.data = motor_endpoint_msg.encode("utf-8")

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
