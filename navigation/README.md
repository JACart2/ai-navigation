# Navigation

* This package's [structure](#structure) contains 3 folders: navigation, resource, and launch; along with ROS2 auto-generated files.
* [Here](#setuphow-to-run) is a tutorial.

## Structure
### launch
This contains the launch file.
### navigation
This contains the navigation python code that allows the JACART2 to navigate.
### resource
This contains the old scripts from Jacart1 and other resources used to transition into ROS2.

## Running the test script
Ensure that you have completed the initial setup that is outlined in [ai-navigation README](../../ai-navigation/README.md#setup) before continuing on to these steps.

1. rviz2 -d src/ai-navigation/navigation/test/visualize_path.rviz
2. ros2 run navigation global_planner
3. ros2 run navigation global_tester

This will open an rviz window to visualize the path, run global_planner to create a path when given a start and end point, and a test node that simulates position and input data.

## Setup/How to Run
Ensure that you have completed the initial setup that is outlined in [ai-navigation README](../../ai-navigation/README.md#setup) before continuing on to these steps.

1. Turn on the cart and connect the laptop via the USB-C cable located in the rear of the cart
2. Run the global planner (for now until navigation is fully completed)
```
ros2 run navigation global_planner
```
3. Send an example starting location in another terminal
```
source install/setup.bash
ros2 topic pub /limited_pose geometry_msgs/msg/PoseStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
"
```
4. Send a sample destination in a new terminal
```
source install/setup.bash
ros2 topic pub /clicked_point geometry_msgs/msg/PointStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
point:
    x: 0.0
    y: 0.0
    z: 0.0
"
```

## ROS Info
#### Global Planner

Subscribes to:
- /limited_pose [PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)
- /vehicle_state [VehicleState](../navigation_interface/msg/VehicleState.msg)
- /clicked_point [PointStamped](https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PointStamped.html)
- /gps_request [LatLongPoint](../navigation_interface/msg/LatLongPoint.msg)
- /estimated_vel_mps Float32


Publishes to: 
- /display_gps [Marker](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html)
- /global_path [LocalPointsArray](../navigation_interface/msg/LocalPointsArray.msg)
- /gps_global_path [LatLongArray](../navigation_interface/msg/LatLongArray.msg)
- /gps_send [LatLongPoint](../navigation_interface/msg/LatLongPoint.msg)

#### Local Planner
Subscribes to:
- /global_path [LocalPointsArray](../navigation_interface/msg/LocalPointsArray.msg)
- /estimate_twist [TwistStamped]
- /ndt_pose [PoseStamped]
- /estimated_vel_mps Float32
- /stop [Stop]
- /speed Float32

Publishes to: 
- /vehicle_state [VehicleState](../navigation_interface/msg/VehicleState.msg)
- /nav_cmd [VelAngle]
- ...todo
