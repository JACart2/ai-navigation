# Navigation

* This package's [structure](#structure) contains 3 folders: navigation, resource, and launch; along with ROS2 auto-generated files.
* [Here](#setuphow-to-run) is a tutorial.

## Structure
### navigation
This contains the navigation python code that allows the JACART2 to navigate.
### resource
This contains the old scripts from Jacart1 and other resources used to transition into ROS2.
### launch
This contains the launch file.

## Setup/How to Run
Eventually there will be a tutorial here.



## ROS Info
#### navigation (9 topics)
```
Subscribes to:
    /limited_pose
    /vehicle_state
    /clicked_point
    /estimated_vel_mps
    /gps_request
Publishes to:
    /display_gps
    /global_path
    /gps_global_path
    /gps_send
Params: X, Y
```
