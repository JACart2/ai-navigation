# Motor Control

* This package's [structure](#structure) contains 3 folders: launch, motor_control, and resource; along with ROS2 auto-generated files.
* [Here](#setuphow-to-run) is a tutorial.

## Structure
### launch
This contains the launch file.
### motor_control
This contains the motor_endpoint python script that connects the arduino to cooperate with ROS2.
### resource
This contains the old scripts from Jacart1 and other resources used to transition into ROS2.

##### To find the TTY* port that Arduino uses, run "udevadm monitor -u" before plugging it in, then it should give the information you need.

## Setup/How to run
First, you must install all of the required packages (noted in requirements.txt)
```
sudo pip install <packages>
```
Second, you must run the launch file
```
ros2 launch <file> <baud_rate> <arduino_port>
```
Third, open a new terminal and you can start up the teleop
```
ros2 run <teleop>
```
## This is how you send the message to the node that you want to turn the thing > 
```
ros2 topic pub /nav_cmd motor_control_interface/msg/VelAnglePlanned "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
vel_planned: 0.0
angle_planned: 50.0
"
```


