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

The canonical full-stack MOLA/autonomy launch methods are:

```bash
# Outside the container:
scripts/launch_mola_stack.sh --cart james

# Inside the container:
ros2 launch cart_launch mola_autonomy.launch.py cart_name:=james
```

`cart_name` accepts `james` or `madison`. The older `cart:=...` launch
argument is still accepted for compatibility.

To enable the GPS-assisted MOLA relocalization supervisor:

```bash
# Outside the container:
scripts/launch_mola_stack.sh --cart james \
  enable_mola_auto_localization:=true \
  use_gps_relocalize:=true \
  gps_yaw_sweep_enabled:=true

# Inside the container:
ros2 launch cart_launch mola_autonomy.launch.py cart_name:=james \
  enable_mola_auto_localization:=true \
  use_gps_relocalize:=true \
  gps_yaw_sweep_enabled:=true
```

By default, GPS-assisted recovery is only confirmed after at least three fresh
post-candidate MOLA pose samples, fresh healthy MOLA diagnostics, direct
match-quality evidence from the diagnostics stream, pose stability for the
confirmation window, and proximity to the frozen GPS seed. A merely promising
pose does not enter lockout. Set `acceptance_allow_quality_fallback:=true` only
when MOLA diagnostics do not expose a direct match-quality metric.

Safe outdoor relocalization test command with motors disabled:

```bash
scripts/launch_mola_stack.sh --cart james \
  enable_motor:=false \
  enable_mola_auto_localization:=true \
  use_gps_relocalize:=true \
  gps_yaw_sweep_enabled:=true \
  gps_topic:=/fix_valid \
  gps_yaw_sweep_mode:=absolute \
  acceptance_allow_quality_fallback:=false
```
