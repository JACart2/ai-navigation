# Motor Control Interfaces
This package houses the messages that are used to communicate with the motor controller.

## Messages

LatLongArray
```
std_msgs/Header header
LatLongPoint[] gpspoints
```

LatLongPoint
```
std_msgs/Header header
float64 latitude
float64 longitude
float64 elevation
```

LocalPointsArray
```
std_msgs/Header header
geometry_msgs/Pose[] localpoints
std_msgs/Float64 total_distance
```

Stop
```
std_msgs/Header header
bool stop
std_msgs/String sender_id
float64 distance
```

VehicleState
```
std_msgs/Header header
bool is_navigating
bool reached_destination
bool stopped
```

WaypointsArray
```
std_msgs/Header header
sensor_msgs/NavSatFix[] waypoints
```