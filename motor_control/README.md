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

1. Ensure [ROS2 humble](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) is installed on your machine and navigate to /dev_ws/src by either creating the directory or going into a premade one.
```
mkdir -p dev_ws/src
cd dev_ws/src
```
2. Clone the ai-navigation repository into dev_ws/src folder. After the clone is complete you should cd back into dev_ws.
```
git clone https://github.com/JACart2/ai-navigation.git
cd ..
```
3. Run the setup script
```
./src/ai-navigation/motor_control/resource/startup_script.sh
```
4. Run the launch file
```
ros2 launch motor_control motor.launch.py
```
or
```
ros2 launch motor_control motor.launch.py baudrate:=57600 arduino_port:=dev/ttyUSB9
```
5. Open a new terminal and you can start up the teleop
```
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
It should be noted that the values of vel_planned and angle_planned should be changed based on upon how you would like the cart to act.
 
- Negative values for the angle_planned paramters turn the car right and vice versa for positive values.
- Negative values passed to vel_planned indicate braking while positive values indicate acceleration

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


