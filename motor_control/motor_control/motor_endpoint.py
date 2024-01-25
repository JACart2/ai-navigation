#!/usr/bin/env python

import serial



class MotorEndpoint(object):

    def __init__(self):
        #constants
        self.BRAKE_TIME = 3
        self.NODE_RATE = 10
        self.STEERING_TOLERANCE = 50
