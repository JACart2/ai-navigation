#!/usr/bin/env python

import serial
import rospy
import bitstruct
from navigation_msgs.msg import VelAngle
from std_msgs.msg import Int8, Bool, String
import time
import math

#Should be fixed when doing the ROS2 port. This should probably be made a launch parameter
cart_port = '/dev/ttyUSB9' #hardcoded depending on computer

# STATES:
MOVING = 0 
BRAKING = 1
STOPPED = 2


class MotorEndpoint(object):

    def __init__(self):
        
        #Going thru class and grouping everything together that seems related

        global cart_port


        self.current_speed = 0.0
        self.new_vel = True
        self.vel_curr = 0
        self.vel_curr_cart_units = 0
        self.vel = 0
        self.vel_cart_units = 0
        self.angle = 0


        self.debug = False


        self.stop = False
        self.gentle_stop = False
        self.brake = int(0)
        self.drove_since_braking = True
        #brake time is never used
        self.brake_time = 3
        self.stopping_time = 0
        self.step_size = 255.0/(self.node_rate*self.brake_time)
        self.state = STOPPED
        self.obstacle_distance = -1
        self.brake_time_used = 0
        self.comfortable_stop_dist = 4.0
        # init local values to store & change msg params
        self.steering_tolerance = 50 # default was 45

        self.delay_print = 0
        self.cmd_msg = None
        # Time (seconds) to ramp up to full brakes
        
        self.node_rate = 10
        self.heartbeat = b''  # Byte array for reading heartbeat from arduino
        self.delta_time = 0.0
        self.prev_time  = 0.0
        self.first_heartbeat = True

     

       
  
        """ Set up the node. """
        rospy.init_node('motor_endpoint')
        rospy.loginfo("Starting motor node!")
        #Connect to arduino for sending speed
        self.serial_connected = True

        #This would be useful for us considering it is about arduino stuff
        try:
            #Need to find out how to do this serial stuff but I guess its the same for ros2, its just a python thing
            self.arduino_ser = serial.Serial(cart_port, 57600, write_timeout=0, timeout=.01)
        except Exception as e:
            rospy.loginfo("==========================================================================")
            rospy.loginfo( "Motor_endpoint: " + str(e))
            rospy.loginfo("==========================================================================")
            rospy.logerr("Motor_endpoint: " + str(e))
            serial_connected = False
        

        rospy.loginfo("Speed serial established")
        """
        Connect to arduino for steering
        """
        self.motion_subscriber = rospy.Subscriber('/nav_cmd', VelAngle, self.motion_callback,
                                                  queue_size=10)
        self.debug_subscriber = rospy.Subscriber('/realtime_debug_change', Bool, self.debug_callback, queue_size=10)
        self.heart_pub = rospy.Publisher('/heartbeat', String, queue_size=10)
        
        rate = rospy.Rate(self.node_rate)
        

        #This loop might be important for when we actually want to use teleop.
        while not rospy.is_shutdown():
            
            if not self.serial_connected:
                print("==========================================================================")
                print("                       RETRYING SERIAL CONNECTION                         ")
                print("==========================================================================")
                try:
                    self.arduino_ser = serial.Serial(cart_port, 57600, write_timeout=0)
                    self.serial_connected = True
                except Exception as e:
                    rospy.logerr("Motor_endpoint: " + str(e))
                    self.serial_connected = False
                    rate.sleep()
                    continue
            
            self.message = None
            
            if self.cmd_msg is not None:
                self.endpoint_calc()
            self.prev_time = time.time()
            
            try:
                self.heartbeat = self.arduino_ser.read_until()
            except Exception as e:
                print("==========================================================================")
                print("                    THE ARDUINO HAS BEEN DISCONNECTED                     ")
                print("==========================================================================")
                self.serial_connected = False
                rate.sleep()
                continue
            if self.heartbeat != "":
                self.heart_pub.publish(self.heartbeat)
                self.delta_time = time.time() - self.prev_time
                rospy.loginfo("Heartbeat message:")
                rospy.loginfo(self.heartbeat + "| Time since last message: ")
                heartbeat_delta_t = time.time() - self.prev_time
                
                # This check is here because the time between the first and 2nd heartbeat is always ~2.4s
                # I believe this is because of the rest of the setup taking place at the same time
                if not self.first_heartbeat: 
                    if heartbeat_delta_t >= 2.0:
                        rospy.loginfo("==========================================================================")
                        rospy.loginfo("           TIME BETWEEN HEARTBEATS, > 2.0s | Things may be fine           ")
                        rospy.loginfo("==========================================================================")
                        
                rospy.loginfo(heartbeat_delta_t)

            rate.sleep()

    def motion_callback(self, planned_vel_angle):
        self.cmd_msg = planned_vel_angle

        # set local velocity from msg values
        self.vel_curr = self.cmd_msg.vel_curr
        self.vel = self.cmd_msg.vel
        self.angle = self.cmd_msg.angle

        if self.vel < 0:
            # indicates an obstacle
            self.obstacle_distance = abs(self.vel)
            self.vel = 0
        else:
            # reset obstacle distance and brake time
            self.obstacle_distance = -1
            self.brake_time_used = 0
            self.full_stop_count = 0

        if self.vel > 0 and (self.state == STOPPED or self.state == BRAKING) and (time.time()- self.stopping_time) > 10:
            self.state = MOVING
            self.brake = 0 # make sure we take the foot off the brake

        elif self.state == MOVING and self.vel <= 0:  # Brakes are hit
            self.state = BRAKING
            self.brake = 0 # ramp up braking from 0
            self.stopping_time = time.time()
            
        self.new_vel = True     

    def debug_callback(self, msg):
        self.debug = msg.data

    
    def endpoint_calc(self):
        if self.new_vel:
            #The first time we get a new target speed and angle we must convert it        
            self.vel_cart_units = self.vel
            self.vel_curr_cart_units = self.vel_curr

            self.new_vel = False

            self.vel_cart_units *= 50       # Rough conversion from m/s to cart controller units
            self.vel_curr_cart_units *= 50  # Rough conversion from m/s to cart controller units
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
    
    def pack_send(self, throttle, brake, steer_angle):
        data = bytearray(b'\x00' * 5)
        bitstruct.pack_into('u8u8u8u8u8', data, 0, 42, 21, abs(throttle), brake, steer_angle + 10)
        self.arduino_ser.write(data)

if __name__ == "__main__":
    try:
        MotorEndpoint()
    except rospy.ROSInterruptException:
        pass