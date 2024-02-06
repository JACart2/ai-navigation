# Motor Control

This package contains 3 folders: launch, motor_control, and resource; along with ROS2 auto-generated files.

## launch
This contains the launch file.
## motor_control
This contains the motor_endpoint python script that connects the arduino to cooperate with ROS2.
## resource
This contains the old scripts from Jacart1 and other resources used to transition into ROS2.

##### To find the TTY* port that Arduino uses, run "udevadm monitor -u" before plugging it in, then it should give the information you need.
