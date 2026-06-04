# Codex Session Notes

## Current Branch

- Working branch: `Localization_fix`
- Repository path: `/root/dev_ws/src/ai-navigation`
- Current pending changes are in `cart_control/localization_launch`.

## What Was Added Earlier

- `cart_control/localization_launch/launch/localization_full_launcher.launch.py`
  starts the full localization stack from one ROS 2 launch entrypoint:
  Velodyne driver, Velodyne pointcloud transform, lidar localization, and ZED cameras.
- It exposes launch arguments for:
  - `cart_config_path`
  - `lidar_device_ip`
  - `lidar_port`
  - `lidar_frame_id`

## TF Flicker Investigation

The runtime symptom was RViz repeatedly reporting that Velodyne data could not transform to `map`.

The likely TF chain RViz needs is:

```text
map -> odom -> base_link -> velodyne
```

The suspected issue was unstable TF ownership:

- ZED was publishing `map -> odom`.
- `lidar_localization_ros2` was configured around `map`, `odom`, and `base_link`, but was not explicitly enabled to own `map -> odom`.
- The static ZED/base transform had `base_link` as the parent and `zed_front_camera_link` as the child, while ZED odometry publishes tracking to the camera frame. That can create competing or disconnected TF paths.

## Pending Fixes

Changed files:

- `cart_control/localization_launch/param/localization.yaml`
  - Added `enable_map_odom_tf: true`
  - Intent: let lidar localization publish `map -> odom`.
- `cart_control/localization_launch/launch/zed_multi_camera.launch.py`
  - Changed `publish_map_tf` to `false`
  - Intent: stop ZED from also publishing `map -> odom`.
- `cart_control/localization_launch/launch/cameras.launch.py`
  - Changed the static transform direction to `zed_front_camera_link -> base_link`
  - Intent: connect ZED odometry to the cart base without making ZED and base frames fight for parents.

## Verification Commands

After rebuilding and sourcing:

```bash
cd ~/dev_ws
colcon build --packages-select localization_launch
source install/setup.bash
```

Run the system, then check TF:

```bash
ros2 run tf2_ros tf2_echo map velodyne
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link velodyne
```

Also useful:

```bash
ros2 run tf2_tools view_frames
```

If `map -> velodyne` stays live instead of cutting in and out, the TF ownership issue was likely the cause.

## Latest Runtime Log Diagnosis

The pasted runtime output had three different classes of messages:

- Repeated `lidar_localization` warnings:
  - `Could not get transform base_link to odom: Lookup would require extrapolation into the future`
  - This is a real TF timing/synchronization warning. `lidar_localization` is asking for `base_link -> odom` at the pointcloud timestamp, but the newest odom TF data is slightly older.
  - The node still reported `has converged: 1`, fitness scores, and final transformations after these warnings, so this is not an immediate launch/build failure.
- Many `process has died`, `exit code -2`, and `KeyboardInterrupt` traces:
  - These happened after `^C` / SIGINT.
  - Treat these mostly as shutdown noise unless the same node dies before Ctrl-C.
- One actual hard failure seen near shutdown:
  - `zed_rear.zed_node_1: Error opening camera: INVALID FUNCTION CALL`
  - The ZED wrapper also printed `Please verify the camera connection`.
  - This points at the rear ZED camera path: connection, serial config, camera already in use, USB/device state, or ZED SDK startup state.

The current working theory before rebuild:

- The code builds and launch descriptions load.
- The remaining live issue is likely runtime TF timing plus possible rear ZED camera startup failure.
- After rebuilding, first verify whether `zed_rear` opens before worrying about shutdown tracebacks.

Useful focused checks after launch:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map velodyne
```

Useful focused ZED check:

```bash
ros2 launch localization_launch cameras.launch.py cart_config_path:=/root/dev_ws/src/ai-navigation/cart_control/cart_launch/config/cart_james.yaml
```

Use the cart config that matches the physical cart. The known config files are:

- `cart_control/cart_launch/config/cart_james.yaml`
- `cart_control/cart_launch/config/cart_madison.yaml`

## RViz Lidar Visibility Update

A later runtime paste showed lidar data is actually flowing:

- `lidar_localization` repeatedly printed:
  - `number of filtered cloud points: 82-85`
  - `has converged: 1`
  - fitness scores
  - final transformations
- That means `/velodyne_points` is being produced and consumed by `lidar_localization`.
- So the remaining symptom "lidar does not show up in RViz" is probably RViz display configuration or TF timing, not a dead Velodyne driver.

One likely RViz-specific issue was fixed:

- `cart_control/cart_launch/rviz/localization.rviz`
  - `PointCloudVelodyne` is already subscribed to `/velodyne_points`.
  - Its QoS was set to `Reliable`.
  - Live Velodyne/PointCloud2 sensor streams are commonly `Best Effort`.
  - Changed `PointCloudVelodyne -> Topic -> Reliability Policy` from `Reliable` to `Best Effort`.

Because this workspace was built with symlink install, the installed RViz config resolves back to the source file:

```text
install/cart_launch/share/cart_launch/rviz/localization.rviz
  -> cart_control/cart_launch/rviz/localization.rviz
```

So the RViz QoS edit should take effect after restarting the cart launch/RViz. Rebuilding is still fine.

## Rebuild Notes

Recommended rebuild before the next run:

```bash
cd /root/dev_ws/src/ai-navigation
colcon build --symlink-install --packages-select cart_launch localization_launch navigation
source install/setup.bash
```

If launching from `/root/dev_ws` instead, use that workspace's install setup consistently:

```bash
cd /root/dev_ws
colcon build --symlink-install --packages-select cart_launch localization_launch navigation
source install/setup.bash
```

Use one workspace prefix consistently per terminal. Mixing `/root/dev_ws/install/setup.bash` and `/root/dev_ws/src/ai-navigation/install/setup.bash` can make ROS launch stale files.

## Post-Rebuild Checks

After the cart launch starts, check the lidar topic directly:

```bash
ros2 topic hz /velodyne_points
ros2 topic info /velodyne_points -v
```

In RViz, inspect:

```text
Displays -> PointCloudVelodyne -> Status
```

Interpretation:

- If RViz status says QoS/no messages:
  - Confirm `Reliability Policy: Best Effort` in the RViz display.
  - Confirm `/velodyne_points` has publishers with `ros2 topic info /velodyne_points -v`.
- If RViz status says transform/extrapolation:
  - The lidar topic is live, but RViz cannot transform `velodyne` into the fixed frame `map`.
  - Continue with TF timing/chain checks.

Useful TF checks:

```bash
ros2 run tf2_ros tf2_echo map velodyne
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link velodyne
```

If `/velodyne_points` is live and RViz still does not show the cloud, try temporarily changing RViz `Global Options -> Fixed Frame` from `map` to `velodyne`.

- If the cloud appears in fixed frame `velodyne`, the data is fine and the issue is TF from `map` to `velodyne`.
- If the cloud still does not appear, focus on the RViz display settings and `/velodyne_points` QoS/message status.

## Known Remaining Runtime Warning

The repeated warning:

```text
Could not get transform base_link to odom:
Lookup would require extrapolation into the future
```

means `lidar_localization` is asking for the `base_link -> odom` transform at the pointcloud timestamp, but the newest ZED odom TF is slightly older. The observed gap was usually small, roughly 10-45 ms. This is a TF timing/tolerance issue to tune after confirming RViz can see `/velodyne_points`.

## Git Auth Note

GitHub CLI authentication was already working, but plain `git push` was still using a broken VS Code credential helper. The fix that worked was:

```bash
gh auth setup-git
git push --set-upstream origin Localization_fix
```

## RViz Map/Lidar Screenshot Follow-up

The screenshot showed:

- RViz fixed frame `map` was OK.
- The large colored point cloud map was visible, so `/initial_map` or `/map` point cloud display was working.
- `PointCloudVelodyne` was receiving `/velodyne_points`, but RViz dropped all live lidar points because it could not transform from the Velodyne frame into `map`.
- The GML route graph display was not visible.

Follow-up code changes:

- `lidar_localization.launch.py`
  - Changed static TF publishers to named arguments.
  - Delayed the lifecycle configure transition by 1 second so lifecycle services are ready.
  - Explicitly overrides `enable_map_odom_tf`, `global_frame_id`, `odom_frame_id`, `base_frame_id`, `enable_timer_publishing`, and `pose_publish_frequency`.
- `localization.yaml`
  - Enables timer publishing of localization TF at 30 Hz.
- `navigation.launch.py`
  - Passes the launch-time `graph_file` argument into `visualize_graph`, not only `global_planner`.
- `cart_launch/rviz/localization.rviz`
  - Sets `RouteGraph` durability to `Transient Local` so it can receive the one-shot `/graph_visual` marker array after publication.

Rebuilt successfully:

```bash
cd /root/dev_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select localization_launch navigation cart_launch
```

After restarting launch/RViz, useful checks:

```bash
ros2 lifecycle get /lidar_localization
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link velodyne
ros2 topic echo /velodyne_points --once --field header.frame_id
ros2 topic echo /graph_visual --once
```

## Live Lidar Root Cause

The GML/route graph came back, but the live Velodyne cloud still did not render.

The latest `lidar_localization` logs showed the concrete blocker:

```text
Could not get transform base_link to odom:
Lookup would require extrapolation into the future
```

The requested scan times were consistently ahead of the newest ZED odom TF by roughly 20-100 ms. Because `lidar_localization_ros2` computes `map -> odom` by looking up `odom -> base_link` at the lidar scan timestamp, that lookup failure prevents it from publishing a usable `map -> odom` TF. RViz then receives `/velodyne_points` but drops every point because it cannot transform `velodyne` into fixed frame `map`.

Follow-up fix:

- `zed_multi_camera.launch.py` now passes `localization_launch/config/common.yaml` into each ZED camera. Previously the copied config was not actually being used.
- `common.yaml` now sets:

```yaml
pos_tracking:
  transform_time_offset: 0.15
```

This future-dates ZED `odom -> camera_link` TF enough to cover the measured Velodyne/ZED timestamp skew.

Rebuilt and confirmed both install prefixes contain the updated config:

```bash
cd /root/dev_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select localization_launch cart_launch navigation
```

After restarting, if live lidar is still red, check that the extrapolation warning is gone from the newest localizer log:

```bash
ls -t /root/.ros/log/lidar_localization_node_*.log | head -1
rg "Could not get transform base_link to odom|Lookup would require extrapolation" /root/.ros/log/lidar_localization_node_*.log
```

## 2026-06-03 Current State

Cart did not move after a goal:

- The path command chain is `/clicked_point -> /global_path -> /nav_cmd -> motor_endpoint -> Arduino`.
- `autonomous_launcher.launch.py` includes `motor.launch.py` but does not forward `arduino_port`.
- `motor.launch.py` defaults to `/dev/ttyUSB0`.
- The udev rule creates the Arduino symlink as `/dev/ttyUSB9`.
- If the physical Arduino is on `/dev/ttyUSB9`, `motor_endpoint` receives nothing on serial and returns before writing packets. Check `/heartbeat` and consider forwarding `arduino_port` through cart launch.

RViz rate:

- `cart_control/cart_launch/rviz/localization.rviz` now has `Frame Rate: 10`.

Localization while driving fix:

- `common.yaml`: `pos_tracking.transform_time_offset: 0.15`.
- `localization.yaml`: `use_odom: true`, `max_twist_prediction_dt: 0.35`, `enable_map_odom_tf: true`.
- `zed_multi_camera.launch.py`: ZED publishes `odom -> zed_front_camera_link`; ZED `map -> odom` remains disabled; local `common.yaml` is passed as `config_path`.
- `cameras.launch.py`: static transform is now `zed_front_camera_link -> base_link`.
- Intended TF ownership:

```text
map -> odom                 lidar_localization
odom -> zed_front_camera_link  ZED
zed_front_camera_link -> base_link  static TF
base_link -> velodyne       static TF
```

Verified with:

```bash
colcon build --symlink-install --packages-select localization_launch
ros2 launch localization_launch cameras.launch.py --show-args
ros2 launch localization_launch localization_full_launcher.launch.py --show-args
ros2 launch cart_launch autonomous_launcher.launch.py --show-args
```

After restart, confirm the warning is gone:

```bash
rg "Could not get transform base_link to odom|Lookup would require extrapolation" /root/.ros/log/lidar_localization_node_*.log
```
