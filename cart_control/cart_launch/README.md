# Autonomous Launch Package
## Overview

This package runs the following components include:
    Localization Launch file: Example node for lidar sensor data (replace with actual sensor nodes as necessary).
    Navigation Stack: Includes all necessary nodes for the robot's navigation (modify to suit your configuration).
    Custom RViz: Runs a custom RViz configuration to visualize the robot's state, sensor data, and other relevant information.

Prerequisites

Before running this package, ensure you have the following installed:

    ROS2 (Robot Operating System 2)
    Any dependencies for specific nodes (e.g., lidar, navigation)

## Usage
Launch the Autonomous System

To start the autonomous system along with the custom RViz visualization, use the following command:

```
ros2 launch cart_launch autonomous_launcher.launch.py
```


