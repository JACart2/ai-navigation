# ai-navigation
ROS2 Code for JACart2 - Please read the following before continuing

## Structure

### motor_control
Contains the code for controlling the arduino board and processes navigation instructions. This package houses [the first steps](motor_control/README.md) in getting the cart setup. It is recommened to follow all of the instructions located here before continuing.

### motor_control_interface
This package contains the messages that actually allow us to communicate with the motor control nodes

### navigation
This package houses graph files, navigation nodes and utility scripts for the gold cart. To read more please see the [navigation README](navigation/README.md) .

### navigation_interface
This package houses all the necessary msgs to communicate with the cart for navigation purposes. For more informtaion on the msg types please see the [navigation_interface README](navigation_interface/README.md).
Contains all code and information for the processing and computing navigation, including local and global planners.

### teleop
Used to manually control the cart using keyboard input. Requires the motor_endpoint documented in [the motor control](motor_control/README.md) package to be running first.

## Setup
Once this initial setup procedure is completed you should be able to follow along with the continued setup sections in other packages.

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
3. Run the setup script from the dev_ws directory.
```
./src/ai-navigation/motor_control/resource/startup_script.sh
```

## Quick Commands
- ```ros2 launch motor_control motor.launch.py``` Starts the node that controls the motor
- ```ros2 run teleop teleop_node``` Starts the teleop controller
