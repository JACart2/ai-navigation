# Autonomous Launch Package
## Overview

This package runs the following components include:
<<<<<<< HEAD
    Localization Launch file: 
    Navigation Stack: Includes all necessary nodes for the robot's navigation (E.g., local planner, global planner).
=======
    Localization Launch file: Example node for lidar sensor data (replace with actual sensor nodes as necessary).
    Navigation Stack: Includes all necessary nodes for the robot's navigation (modify to suit your configuration).
>>>>>>> origin/walker/zed-custom-launch-files
    Custom RViz: Runs a custom RViz configuration to visualize the robot's state, sensor data, and other relevant information.

Prerequisites

Before running this package, ensure you have the following installed:

<<<<<<< HEAD
1. ROS2 (Robot Operating System 2)
2. Any dependencies for specific nodes (e.g., lidar, navigation)
3. Have done the previous installation steps for all required parts of the JACART which includes
    [ai-navigation installation](https://github.com/JACart2/ai-navigation/blob/main/README.md) and the 
    [localization installation](https://github.com/JACart2/jacart2_documentation/blob/main/sensors_README.md)
=======
    ROS2 (Robot Operating System 2)
    Any dependencies for specific nodes (e.g., lidar, navigation)
>>>>>>> origin/walker/zed-custom-launch-files

## Usage
Launch the Autonomous System

To start the autonomous system along with the custom RViz visualization, use the following command:

```
ros2 launch cart_launch autonomous_launcher.launch.py
```


