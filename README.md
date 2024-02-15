# ai-navigation
ROS2 Code for JACart2

## Structure

### motor_control
Contains the code for controlling the arduino board and processes navigation instructions. The first step in getting the cart setup.

### teleop
Used to manually control the cart using keyboard input. Requires the motor_endpoint to be running first.

## Setup
For help setting up the project and getting it ready to run, see the README in the motor_control package.

## Quick Commands
- ```ros2 launch motor_control motor_endpoint``` Starts the node that controls the motor
- ```ros2 run teleop teleop``` Starts the teleop controller
