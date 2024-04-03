# Motor Control

* This package's [structure](#structure) contains 3 folders: launch, motor_control, and resource; along with ROS2 auto-generated files.
* [Here](#setuphow-to-run) is a tutorial.

## Structure
### launch
This contains the launch file.
### motor_control
This contains the motor_endpoint python code that connects the arduino to cooperate with ROS2.
### resource
This contains the old scripts from Jacart1 and other resources used to transition into ROS2.

## Setup/How to run
Ensure that you have completed the initial setup that is outlined in [ai-navigation README](../README.md) before continuing on to these steps.

1. Turn on the cart and connect the laptop via the USB-C cable located in the rear of the cart
2. Run the launch file
```
ros2 launch motor_control motor.launch.py
```
or
```
ros2 launch motor_control motor.launch.py baudrate:=57600 arduino_port:=dev/ttyUSB9
```
3. Open a new terminal and you can start up the teleop
```
source install/setup.bash
ros2 run teleop teleop_node
```

## ROS Info
#### motor_endpoint
```
Subscribes to: /nav_cmd
Publishes to: /heartbeat
Params: baudrate, arduino port
```
### Template nav_cmd message
This is the message to send to the motor_control node when it's running.
It should be noted that the values of vel and angle should be changed based on upon how you would like the cart to act.
 
- Negative values for the angle paramters turn the car right and vice versa for positive values.
- Negative values passed to vel indicate braking while positive values indicate acceleration

```
ros2 topic pub /nav_cmd motor_control_interface/msg/VelAngle "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
vel: 0.0
angle: 0.0
"
```


