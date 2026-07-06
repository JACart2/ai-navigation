# Localization Launch File
This is the custom JACART launch file created to launch the components associated with localization
This includes launching:
1. Velodyne Lidar Packages - Converts data to raw velodyne packets then to velodyne pointcloud data
2. ZED Multi Camera - Runs the ZED packages in order to collect the Data coming off the zed cameras
3. Lidar_localization_ros2 - Takes that Velodyne point cloud data and the odomentry provided by the ZED and actually runs the localization processs

## MOLA auto-localization supervisor

`mola_auto_localization_supervisor` is a conservative LiDAR-only helper for the
MOLA stack. It watches `/velodyne_points`, `/lidar_odometry/pose`, optional
`/pcl_pose` visibility, and `/mola_diagnostics/lidar_odom/status`. When enabled,
it can request `/relocalize_near_pose` after LiDAR is healthy on startup and
again after obvious localization loss.

For this first version, "lost" only means obvious or suspicious failures:
missing or stale MOLA pose, LiDAR alive while pose is not updating, diagnostics
that report inactive/poor ICP/high dropped frames for a sustained grace period,
impossible pose jumps, impossible pose-derived speed, impossible yaw-rate, or a
large yaw flip. Without GPS, ZED, or another global reference, this node cannot
prove that a physically plausible pose is globally wrong, so it intentionally
does not chase subtle wrong-pose cases.

Launch with the supervisor disabled, preserving current behavior:

```bash
scripts/launch_james_mola_stack.sh
```

Launch with the supervisor enabled for testing:

```bash
scripts/launch_james_mola_stack.sh enable_mola_auto_localization:=true
```

Tune thresholds in:

```text
cart_control/localization_launch/param/mola_auto_localization_supervisor.yaml
```

The relocalization service type is discovered from the ROS graph at runtime. If
the target MOLA environment does not advertise the type before the request is
needed, set `relocalize_service_type` in the YAML to the exact service type.
