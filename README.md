# ai-navigation
ROS2 Code for JACart2 - Please read the following before continuing

## Structure

### motor_control
Contains the code for controlling the arduino board and processes navigation instructions. This package houses [the first steps](motor_control/README.md) in getting the cart setup. It is recommened to follow all of the instructions located here before continuing.

### motor_control_interface
This package contains the messages that actually allow us to communicate with the motor control nodes

### navigation
Contains all code and information for the processing and computing navigation, including local and global planners.

### navigation_interface
This package contains the messages that actually allow us to communicate with the navigation nodes

### teleop
Used to manually control the cart using keyboard input. Requires the motor_endpoint documented in [the motor control](motor_control/README.md) package to be running first.

## Setup
For help setting up the project and getting it ready to run, see the README in the [motor_control](motor_control/README.md#setuphow-to-run) package.

## Quick Commands
- ```ros2 launch motor_control motor.launch.py``` Starts the node that controls the motor
- ```ros2 run teleop teleop_node``` Starts the teleop controller
