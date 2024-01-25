#!/usr/bin/env python
"""
This is the python class that allows us to control the golf cart 
via teleop.

"""
import serial


class MotorEndpoint(object):

    def __init__(self):
        #constants
        self.BRAKE_TIME = 3
        self.NODE_RATE = 10
        self.STEERING_TOLERANCE = 50