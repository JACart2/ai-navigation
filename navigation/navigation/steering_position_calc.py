#!/usr/bin/python3

"""
    Ackermann Steering Simulator with Tkinter and Pyplot

    Enrique Mireles 
    18.02.2019

    Kevin Molloy 26.03.2024 
    Ripped out all GUI stuff and made this stand alone

"""

import numpy as np
import time

LF = 1.25  # Front distance from the center of mass of the cart.
LB = 1.25  # Back distance from the center of mass of the cart.
LW = 1  # Half of the width of the cart.
prev_phi = 0
steering_angle = 0
vel_mps = 0


def calc_new_pos(deltaT, x, y, next_vel_mps, next_steering_angle):
    """
    Compute new position
    """
    global LB
    global LF
    global LW
    global prev_phi
    global steering_angle
    global vel_mps

    steering = np.deg2rad(steering_angle)

    # Calculate heading of the vehicle.
    beta = np.arctan(LB * np.tan(steering) / (LB + LF))
    phi = prev_phi + vel_mps * deltaT * np.cos(beta) * np.tan(steering) / (LB + LF)
    prev_phi = phi

    # Calculate vehicles's position.
    new_x = x + vel_mps * deltaT * np.cos(beta + phi)
    new_y = y + vel_mps * deltaT * np.sin(beta + phi)

    steering_angle = next_steering_angle
    vel_mps = next_vel_mps

    return new_x, new_y, beta + phi


def main():
    x = y = 0
    while True:

        new_x, new_y = calc_new_pos(
            deltaT=0.05, x=x, y=y, vel_mps=0.5, steering_angle=20.0
        )
        print(new_x, new_y)
        x = new_x
        y = new_y


if __name__ == "__main__":
    main()
