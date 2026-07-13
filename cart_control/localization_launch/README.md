# Localization Launch File
This is the custom JACART launch file created to launch the components associated with localization
This includes launching:
1. Velodyne Lidar Packages - Converts data to raw velodyne packets then to velodyne pointcloud data
2. ZED Multi Camera - Runs the ZED packages in order to collect the Data coming off the zed cameras
3. Lidar_localization_ros2 - Takes that Velodyne point cloud data and the odomentry provided by the ZED and actually runs the localization processs

## MOLA auto-localization supervisor

`mola_auto_localization_supervisor` is a conservative helper for the MOLA stack.
It watches `/velodyne_points`, `/lidar_odometry/pose`, optional `/pcl_pose`
visibility, `/initialpose`, and `/mola_diagnostics/lidar_odom/status`. When
enabled, it can request `/relocalize_near_pose` after LiDAR is healthy on
startup and again after obvious localization loss.

For this first version, "lost" only means obvious or suspicious failures:
missing or stale MOLA pose, LiDAR alive while pose is not updating, diagnostics
that report inactive/poor ICP/high dropped frames for a sustained grace period,
impossible pose jumps, impossible pose-derived speed, impossible yaw-rate, or a
large yaw flip. Without GPS, ZED, or another global reference, this node cannot
prove that a physically plausible pose is globally wrong, so it intentionally
does not chase subtle wrong-pose cases.

Optional GPS recovery seeding is available but disabled by default. Set
`use_gps_relocalize: true` to subscribe to `gps_topic` (`/gps` by default).
Valid `sensor_msgs/msg/NavSatFix` messages are lightly checked for status,
message age, finite latitude/longitude, and configurable covariance, then
converted continuously to a cached map-frame x/y seed through the landmark
calibration config. GPS updates do not overwrite healthy MOLA localization.
When MOLA health is lost or stale, the supervisor can send the latest fresh GPS
map seed to `/relocalize_near_pose` with broad yaw uncertainty. If GPS is
missing, stale, invalid, or cannot be converted, recovery falls back to the
settled last-known-pose seed.

GPS is only a rough automatic seed, similar to an RViz "2D Pose Estimate".
MOLA/LiDAR pose freshness and diagnostics decide whether recovery is accepted.
Manual `/initialpose` updates suppress automatic recovery during the configured
manual-pose cooldown so GPS does not fight a human correction.
By default, GPS-assisted recovery requires three fresh post-candidate MOLA pose
samples, fresh healthy MOLA diagnostics, direct match-quality evidence from the
diagnostics stream, pose stability for the confirmation window, and proximity
to the frozen GPS seed. A fresh pose without match quality is treated as
promising, not confirmed, and does not enter lockout. Set
`acceptance_allow_quality_fallback:=true` only when MOLA diagnostics do not
expose a direct match-quality metric; the fallback still requires diagnostics,
stability, fresh pose updates, and the confirmation window.

Canonical full-stack launch commands:

```bash
# Outside the container:
scripts/launch_mola_stack.sh --cart james

# Inside the container:
ros2 launch cart_launch mola_autonomy.launch.py cart_name:=james
```

Launch with the GPS-assisted MOLA supervisor enabled:

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

The same GPS tuning arguments are exposed by the main launch file:

```bash
ros2 launch cart_launch mola_autonomy.launch.py cart_name:=james \
  enable_mola_auto_localization:=true \
  use_gps_relocalize:=true \
  gps_yaw_sweep_enabled:=true \
  gps_topic:=/gps \
  gps_fix_timeout_sec:=3.0 \
  gps_seed_max_age_sec:=3.0 \
  gps_max_covariance:=100.0 \
  gps_relocalize_cooldown_sec:=8.0 \
  acceptance_min_fresh_pose_count:=3 \
  acceptance_require_diagnostics_ok:=true \
  acceptance_require_match_quality:=true \
  acceptance_allow_quality_fallback:=false \
  acceptance_confirmation_window_sec:=2.0 \
  acceptance_near_seed_m:=10.0
```

Safe outdoor relocalization test command with motors disabled:

```bash
scripts/launch_mola_stack.sh --cart james \
  enable_motor:=false \
  enable_mola_auto_localization:=true \
  use_gps_relocalize:=true \
  gps_yaw_sweep_enabled:=true \
  gps_topic:=/fix_valid \
  gps_yaw_sweep_mode:=absolute \
  yaw_candidate_settle_sec:=1.0 \
  yaw_candidate_timeout_sec:=4.0 \
  post_recovery_lockout_sec:=25.0 \
  acceptance_min_fresh_pose_count:=3 \
  acceptance_require_match_quality:=true \
  acceptance_allow_quality_fallback:=false
```

Keep `gps_yaw_candidates_deg` values as floats in YAML, for example
`[0.0, 45.0, ...]`, so ROS 2 loads the parameter as a double array. The older
`gps_yaw_sweep_angles_deg` key is still accepted as a legacy alias.

Monitor yaw sweep logs:

```bash
ros2 topic echo /rosout \
  | grep -E 'Starting GPS recovery episode|frozen seed|GPS yaw candidate|Accepted recovery|Rejected GPS yaw candidate|lockout|Manual'
```

Tune thresholds in:

```text
cart_control/localization_launch/param/mola_auto_localization_supervisor.yaml
```

Offline helper tests for GPS seed validation and recovery gating can run
without a cart, GPS, LiDAR, motors, or a ROS graph:

```bash
python3 -m pytest \
  cart_control/localization_launch/test/test_mola_auto_localization_supervisor.py \
  -q
```

The relocalization service type is discovered from the ROS graph at runtime. If
the target MOLA environment does not advertise the type before the request is
needed, set `relocalize_service_type` in the YAML to the exact service type.
