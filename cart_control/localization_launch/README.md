# Localization Launch File
This is the custom JACART launch file created to launch the components associated with localization
This includes launching:
1. Velodyne Lidar Packages - Converts data to raw velodyne packets then to velodyne pointcloud data
2. ZED Multi Camera - Runs the ZED packages in order to collect the Data coming off the zed cameras
3. Lidar_localization_ros2 - Takes that Velodyne point cloud data and the odomentry provided by the ZED and actually runs the localization processs

## MOLA auto-localization supervisor

`mola_auto_localization_supervisor` is a conservative LiDAR-first helper for the
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

### GPS-assisted recovery

The supervisor can optionally use `/fix` as an independent recovery hint after
MOLA localization is suspected lost. GPS is not fused continuously and does not
override a healthy LiDAR localization; it only seeds `/relocalize_near_pose`
with a broad map-frame pose so LiDAR can confirm the final alignment. When
`enable_gps_recovery:=true`, the live autonomy launch also starts the Garmin
GPS 18x driver so `/fix` is available during the run.

By default, GPS recovery uses `enable_gps_auto_anchor:=true`. That means it
does not require manually typing the map GPS origin. Instead, after MOLA is
healthy, it averages several good GPS fixes, pairs them with the current MOLA
pose, then waits for healthy motion to estimate `gps_map_yaw_from_enu`.

```bash
scripts/launch_james_mola_stack.sh \
  enable_mola_auto_localization:=true \
  enable_gps_recovery:=true
```

Manual origin/yaw inputs are still available as a fallback if auto-anchor is
disabled or unavailable:

```bash
scripts/launch_james_mola_stack.sh \
  enable_mola_auto_localization:=true \
  enable_gps_recovery:=true \
  enable_gps_auto_anchor:=false \
  gps_map_origin_lat:=38.433825 \
  gps_map_origin_lon:=-78.862175 \
  gps_map_origin_alt:=422.1 \
  gps_map_yaw_from_enu:=0.0
```

Optional GPS driver launch arguments are `gps_port:=/dev/ttyACM0`,
`gps_baud:=4800`, and `gps_frame:=gps`.

`gps_map_yaw_from_enu` is radians. It rotates local ENU GPS coordinates into the
MOLA map frame. Auto-anchor learns this value after the cart drives far enough
while MOLA localization is healthy.

GPS recovery rejects stale fixes, no-fix statuses, and fixes whose estimated
xy standard deviation is above `gps_max_xy_std`. Auto-anchor also requires
`gps_auto_anchor_min_samples` recent good fixes and `gps_auto_anchor_min_move_m`
of healthy motion before it can use GPS for recovery. Tune the GPS thresholds in:

```text
cart_control/localization_launch/param/mola_auto_localization_supervisor.yaml
```

The relocalization service type is discovered from the ROS graph at runtime. If
the target MOLA environment does not advertise the type before the request is
needed, set `relocalize_service_type` in the YAML to the exact service type.
