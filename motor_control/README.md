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
##### An issue that has presented itself relating to connecting the arduino port can be fixed by deleting the Brltty package in linux
```
sudo apt remove brltty
```

## Setup/How to run

1. Ensure {https://docs.ros.org/en/humble/Installation.html} (ROS2 humble) is installed on your machine and navigate to /dev_ws/src by either creating the directory or going into a premade one.
```
mkdir -p dev_ws/src
cd dev_ws/src
```
2.
3.
5.
6.
7.  you must install all of the required packages (noted in requirements.txt)
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
## This is the message to send to the motor_control node when its running.
It should be noted that the values of vel_planned and angle_planned should be changed based on upon how you would like the cart to act.
 
- Negative values for the angle_planned paramters turn the car right and vice versa for positive values.
- Negative values passed to vel_planned indicate breaking while positive values indicate acceleration

```
ros2 topic pub /nav_cmd motor_control_interface/msg/VelAnglePlanned "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
vel_planned: 0.0
angle_planned: 0.0
"
```


