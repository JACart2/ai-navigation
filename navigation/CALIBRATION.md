# GPS Calibration System Documentation

This document explains how the landmark-based GPS calibration system works, how to configure it, and how to convert map files between ROS local coordinates and GPS coordinates.

## Overview

The navigation system uses a **landmark-based calibration** approach to convert between GPS coordinates (latitude/longitude) and local ROS coordinates (x, y in meters). This allows the robot to operate seamlessly using both coordinate systems.

### Key Concepts

- **Local ROS Coordinates**: A flat 2D coordinate system (x, y in meters) relative to a local origin. Maps are typically created in this format using tools like RViz.
- **GPS Coordinates**: Real-world coordinates expressed as latitude and longitude.
- **Landmarks**: Known physical locations with corresponding coordinates in both systems. These are used to calibrate the transformation.
- **Calibration Parameters**: The rotation angle and translation vectors computed from landmarks that enable coordinate conversions.

## How to Change the Landmark Config File Source

The landmark calibration YAML file can be sourced from different locations and filenames depending on your use case.

### Default Configuration

By default, the system looks for calibration files in the `navigation/maps/` directory. The launch file passes these parameters to the nodes:

### Method 1: Modify via Launch File Arguments

When launching the navigation system, override the parameters:

```bash
ros2 launch navigation navigation.launch.py \
  calibration_config_dir:=/path/to/configs \
  calibration_config_file:=my_landmarks.yaml
```

### Method 2: Modify the Launch File Directly

Edit [navigation.launch.py](launch/navigation.launch.py) to change default values:

```python
DeclareLaunchArgument(
    "calibration_config_dir",
    default_value="/path/to/your/config/dir",  # Change this
),
DeclareLaunchArgument(
    "calibration_config_file",
    default_value="your_config.yaml",  # Change this
),
```

### Method 3: Runtime Configuration via ROS Parameters

If you need to change the config file at runtime (before nodes are initialized), you can set parameters directly:

```bash
ros2 param set global_planner calibration_config_dir /new/path
ros2 param set global_planner calibration_config_file new_file.yaml
```

## How to Make a Calibration File

A calibration file is a YAML file that defines landmark points in both local ROS and GPS coordinate systems.

### Step-by-Step: Creating a Calibration File

Right now, there is no way to auto-generate a config file for a waypoint graph /shrug, so they will have to be made maunally on a per waypoint graph basis.

1. **Get Local ROS Coordinates**
   - Use RViz to identify the corresponding local (x, y) position (pointcloud points are best)

2. **Get GPS Coordinates**
   - Record GPS coordinates at the corresponding location (e.g., 38.43155945255256)

3. **Create the YAML File**
   - Create a new `.yaml` file and upload it to https://w3.cs.jmu.edu/spragunr/rosmaps/
   - Add landmarks using the example format below

### Example: Complete Calibration File

```yaml
landmarks:
  - name: "south_building_corner"
    local:
      x: 250.0
      y: 95.0
    gps:
      latitude: 38.43156
      longitude: -78.86153

  - name: "north_plaza_center"
    local:
      x: 105.0
      y: 217.5
    gps:
      latitude: 38.43270
      longitude: -78.85990

  - name: "east_gate_entrance"
    local:
      x: 98.0
      y: 13.6
    gps:
      latitude: 38.43304
      longitude: -78.86220

  - name: "west_quad_junction"
    local:
      x: -446.3
      y: -1209.3
    gps:
      latitude: 38.43943
      longitude: -78.87513

  - name: "map_origin_reference"
    local:
      x: 1.0
      y: 5.5
    gps:
      latitude: 38.43390
      longitude: -78.86211
```

## How to Enable Auto-Converting a .gml GPS File to ROS Coordinates

tl;dr - in navigation.launch.py, change graph_coordinate_format:=gps => ros and vice versa.

The `global_planner` node can automatically convert GPS-format GML maps to local ROS coordinates when the map is loaded, using a calibration .yaml file.

### Method 1: Via Launch Arguments (Recommended)

Set the `graph_coordinate_format` parameter to `"gps"`:

```bash
ros2 launch navigation navigation.launch.py \
  graph_file:=path/to/your/map_gps.gml \
  graph_coordinate_format:=gps \
  calibration_config_file:=your_landmarks.yaml
```

### Method 2: Modify Launch File Default

Edit [navigation.launch.py](launch/navigation.launch.py):

```python
DeclareLaunchArgument(
    "graph_coordinate_format",
    default_value="gps",  # Change from "ros" to "gps"
),
```

## How to Convert a .gml File from ROS Coordinates to GPS Lat-Long

The `gml_to_gps` CLI tool converts GML maps from local ROS coordinates to GPS coordinates. This will need a calibration YAML file with landmarks

### Step Process

```bash
# 1. Create a ROS map in RViz (saved as main_local.gml)
# 2. Create a landmarks config file (landmarks.yaml) with 4+ reference points

# 3. Convert the map to GPS coordinates
# The landmarks file (landmarks.yaml) is assumed to be in /maps. If the landmarks file is somewhere else, change the source directory in navigation.launch.py.

# Remember to colcon build and cd into dev_ws
ros2 run navigation gml_to_gps \
  --input src/ai-navigation/navigation/maps/main_shift3.gml \
  --output src/ai-navigation/navigation/maps/main_shift3_gps.gml \
  --config-dir /maps \
  --config-file SpeedBoiMap.yaml

# Output:
# Loaded 5 landmark point(s) from navigation/maps/landmarks.yaml
# Calibrated with landmarks: rotation -2.345672°
# Written: navigation/maps/main_gps.gml
```

## Using the Calibration API Directly

If you need to use the calibration functions in your own code:

```python
from navigation.simple_gps_util import (
    load_landmark_calibration,
    calibrate_with_landmarks,
    gps_to_local,
    local_to_gps,
    convert_gml_to_gps_landmarks,
)

# Load landmarks from YAML
local_points, gps_points = load_landmark_calibration("config.yaml")

# Compute calibration parameters
ref_lat, ref_lon, cx_local, cy_local, cx_gps, cy_gps, theta = \
    calibrate_with_landmarks(local_points, gps_points)

# Convert GPS to local
local_x, local_y = gps_to_local(
    lat=38.43156, lon=-78.86153,
    ref_lat=ref_lat, ref_lon=ref_lon,
    cx_local=cx_local, cy_local=cy_local,
    cx_gps=cx_gps, cy_gps=cy_gps,
    theta_degrees=theta
)

# Convert local to GPS
gps_lat, gps_lon = local_to_gps(
    x=250.0, y=95.0,
    ref_lat=ref_lat, ref_lon=ref_lon,
    cx_local=cx_local, cy_local=cy_local,
    cx_gps=cx_gps, cy_gps=cy_gps,
    theta_degrees=theta
)

# Convert entire GML file
convert_gml_to_gps_landmarks(
    input_path="map_local.gml",
    output_path="map_gps.gml",
    ref_lat=ref_lat, ref_lon=ref_lon,
    cx_local=cx_local, cy_local=cy_local,
    cx_gps=cx_gps, cy_gps=cy_gps,
    theta_degrees=theta
)
```
