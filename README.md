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

_Note: Any README that asks you to install packages with "sudo apt" is old and not fully relevant._
_Packages are now installed within Docker containers, which are the primary way of executing the cart logic._

---

## Radar (TI IWR6843AOP) Pipeline

The cart can run with a TI mmWave radar in addition to the velodyne lidar. The radar pipeline is layered so noisy returns are cropped first, then live-tunable from a slider window, then converted directly into obstacles that the collision detector acts on.

### Nodes (all live in the `navigation` package)

| Node | Subscribes | Publishes | What it does |
|------|------------|-----------|--------------|
| `radar_xyz_filter` | `/ti_mmwave/radar_scan_pcl` | `/ti_mmwave/radar_scan_pcl_filtered`, `/ti_mmwave/radar_scan_pcl_rejected`, `/radar_filter_box` | Crops the radar cloud to a 3-D box. 6 dynamic ROS 2 params (`x_min`, `x_max`, `y_min`, `y_max`, `z_min`, `z_max`) update live. Also publishes a green wireframe Marker showing the active box. |
| `radar_filter_sliders` | (Tk window) | (calls `SetParameters` on the filter node) | Six sliders + Refresh / Reset buttons. Moves on a slider go straight to the filter as live param changes. |
| `radar_pcl_to_obstacles` | `/ti_mmwave/radar_scan_pcl_filtered` | `/obstacles`, `/radar_obstacle_display` | 3-D distance clusters the filtered cloud, transforms each cluster centroid to `base_link`, and publishes ObstacleArray. Replaces the old radar→LaserScan→cluster path which dropped sparse radar returns. The marker is height-true: jump up and down and it follows. |
| `lidar_object_to_obstacle` | velodyne PCL + `/scanner/lidar_scan` | `/obstacles`, `/lidar_obstacle_display` | Unchanged for lidar. Radar is intentionally skipped in `laserscan_callback` so it doesn't race the new node on `/obstacles`. |

### Default filter bounds (radar frame `ti_mmwave_0`, x = forward, y = left, z = up)

| Axis | min | max |
|------|----:|----:|
| x | 0.25 | 3.58 |
| y | -2.10 | 5.00 |
| z | -0.65 | 1.11 |

These defaults live in five files; all five must stay in sync if you change them:

1. `navigation/navigation/radar_xyz_filter.py` (`DEFAULT_BOUNDS`)
2. `navigation/navigation/radar_filter_sliders.py` (`DEFAULTS`)
3. `navigation/launch/obstacle_conversion.launch.py`
4. `navigation/launch/navigation.launch.py`
5. `cart_control/cart_launch/launch/autonomous_launcher.launch.py`

To save new defaults after live tuning, read them off the running node and update those five files:

```bash
for p in x_min x_max y_min y_max z_min z_max; do
  ros2 param get /radar_xyz_filter $p
done
```

### RViz: two windows side by side

`autonomous_launcher.launch.py` opens two RViz windows automatically:

* `cart_launch/rviz/localization.rviz` — top-down map view with the cart, path, lidar obstacles, and radar overlays (green = kept, red = rejected, green wireframe = filter box).
* `cart_launch/rviz/radar_pov.rviz` — locked to `ti_mmwave_0`. Three saved camera presets: **Radar POV (Forward)**, **Radar Birds-eye**, **Radar Behind**.

## Launching the cart with radar

Inside the backend container shell (see the [docker_files README](https://github.com/JACart2/docker_files) for how to get one — `./dev-run-backend.sh` drops you straight into it):

```bash
ros2 launch cart_launch autonomous_launcher.launch.py \
  motor_arduino_port:=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00448R-if00-port0 \
  radar_command_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_011D1A9D-if00-port0 \
  radar_data_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_011D1A9D-if01-port0
```

This one launch brings up:

* the TI radar driver (`ti_mmwave_rospkg` via the OpaqueFunction in `autonomous_launcher.launch.py`)
* `radar_xyz_filter` + slider GUI + `radar_pcl_to_obstacles`
* the lidar stack (velodyne + localization)
* the navigation stack (global planner, local planner, collision detector, motor endpoint)
* rosbridge (port 9090) for the frontend UI
* swri_console
* both RViz windows

`enable_radar:=true` and `radar_cfg_file:=6843AOP_Standard.cfg` are already the defaults — drop them. The three `*_port` args are the only ones that aren't defaulted to something matching the cart's actual hardware (the launch falls back to `/dev/ttyUSB0` / `/dev/ttyUSB1`, which don't match).

### Useful overrides

```bash
# Disable the Tk slider window (e.g. running headless)
radar_filter_gui:=false

# Override bounds at launch (skips the saved defaults above)
radar_filter_x_max:=5.0 radar_filter_z_min:=-0.4

# Slower cruise for indoor testing — local_planner default is 30 km/h
cruise_speed_kph:=5.0
```

### What "working" looks like

* `ros2 topic hz /ti_mmwave/radar_scan_pcl_filtered` ≈ 30 Hz.
* `ros2 topic hz /obstacles` ≈ 30–40 Hz when the radar pipeline is healthy.
* Walking in front of the cart puts green spheres into the radar-POV window with red cylinders on them, and `collision_detector` logs `requested stop: True with distance X`.
* `lidar_localization` fitness score should drop **under 2.0** after you set the initial pose in RViz. If it's stuck at 10+ the cart will steer poorly even when radar is fine — re-set the 2D Pose Estimate.

---

_It's been a long, successful semester. Good cart._
