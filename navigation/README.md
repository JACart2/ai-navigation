# Navigation

* This package's [structure](#structure) contains 2 folders: navigation and resource; along with ROS2 auto-generated files.
* [Here](#setuphow-to-run) is a tutorial.

## Structure
### navigation
This contains the navigation python code that allows the JACART2 to navigate.
### resource
This contains the old scripts from Jacart1 and other resources used to transition into ROS2.

## Setup/How to Run
Eventually there will be a tutorial here.



## ROS Info
#### navigation
```
Subscribes to: /XXXXXXX
Publishes to: /XXXXX
Params: X, Y
```


Pose Stamped message type
```
"header:
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
