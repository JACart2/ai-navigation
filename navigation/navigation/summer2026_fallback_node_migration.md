# Velodyne Fallback Node Migration Notes for a `summer2026`-Based Branch

This document lists the changes needed to move `velodyne_pointcloud_tf_fallback.py` onto a new branch that is based on `summer2026`.

Planned workflow:
1. Open the new `summer2026`-based branch.
2. Copy the node file into the repo.
3. Run Codex on this document to make the remaining wiring changes.

## Goal

Add a fallback node that:
- subscribes to the original Velodyne cloud on `/velodyne_points`
- publishes a stabilized cloud on `/velodyne_points_stable`
- uses the live TF tree when available
- falls back to a hardcoded `base_link -> velodyne` transform when needed
- falls back to the cart's last known `base_link -> map` transform when map TF is briefly unavailable

## Node File to Add

Place this file here:

- `src/ai-navigation/cart_control/localization_launch/localization_launch/velodyne_pointcloud_tf_fallback.py`

That file should remain a Python module inside the `localization_launch` package.

## Required Changes

### 1. Expose the node as a ROS executable

File:

- `src/ai-navigation/cart_control/localization_launch/setup.py`

Add this entry under `entry_points["console_scripts"]`:

```python
'velodyne_pointcloud_tf_fallback = localization_launch.velodyne_pointcloud_tf_fallback:main',
```

Why:

- Without this, ROS cannot launch the node with `executable="velodyne_pointcloud_tf_fallback"`.

### 2. Add Python/runtime dependencies for the node

File:

- `src/ai-navigation/cart_control/localization_launch/package.xml`

Make sure these runtime dependencies are present:

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>tf2_ros</depend>
<depend>tf2_sensor_msgs</depend>
<depend>python3-tf-transformations</depend>
```

Keep the existing dependencies too, including:

```xml
<depend>rviz2</depend>
<depend>velodyne_driver</depend>
<depend>velodyne_pointcloud</depend>
```

Why:

- The fallback node imports and uses all of the packages above directly.
- `summer2026` may build or run without them being explicitly declared today, but this node depends on them.

### 3. Launch the fallback node from the localization stack

File:

- `src/ai-navigation/cart_control/localization_launch/launch/localization_full_launcher.launch.py`

Add a `Node(...)` entry for:

- package: `localization_launch`
- executable: `velodyne_pointcloud_tf_fallback`
- name: `velodyne_pointcloud_tf_fallback`

Recommended parameters:

```python
parameters=[{
    "input_topic": "/velodyne_points",
    "output_topic": "/velodyne_points_stable",
    "target_frame": "map",
    "parent_frame": "base_link",
    "child_frame": "velodyne",
    "x": 1.0,
    "y": 0.0,
    "z": 1.9,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "prefer_fallback_lidar_tf": True,
    "republish_original_if_possible": True,
    "downsample_stride": 2,
}]
```

Then include that node in the returned `LaunchDescription`.

Why:

- This is the actual wiring that makes the node run.
- `input_topic` must stay on the original Velodyne stream so the node has raw data to work with.
- `output_topic` is the stabilized stream that downstream consumers can use.

Notes:

- `downsample_stride: 2` is optional but was used to reduce output cloud size and improve the effective stable-point throughput.
- `prefer_fallback_lidar_tf: True` biases toward the hardcoded lidar transform when the source frame matches the expected lidar child frame.

### 4. Point lidar localization at the stabilized cloud

File:

- `src/ai-navigation/cart_control/localization_launch/launch/lidar_localization.launch.py`

Change the cloud remap from:

```python
("/cloud", "/velodyne_points")
```

to:

```python
("/cloud", "/velodyne_points_stable")
```

Why:

- If this remap is left on raw `/velodyne_points`, localization will ignore the fallback node.

### 5. Point RViz at the stabilized cloud

File:

- `src/ai-navigation/cart_control/cart_launch/rviz/localization.rviz`

In the `PointCloudVelodyne` display, change the topic from:

```yaml
Value: /velodyne_points
```

to:

```yaml
Value: /velodyne_points_stable
```

Why:

- Otherwise RViz will still visualize the raw cloud and the fallback node will appear unused.

Important:

- Keep RViz reliability on whatever the `summer2026` file already uses unless there is a confirmed QoS mismatch during testing.
- In our current branch we backed out an attempted RViz QoS change because it caused visibility issues. So topic rewiring is required, but QoS changes are not part of the recommended migration set.

## Optional Changes

These are not required just to get the fallback node working, but they may matter depending on what you want the rest of the stack to consume.

### A. Obstacle conversion can stay on raw points or move to stable points

File:

- `src/ai-navigation/navigation/navigation/lidar_object_to_obstacle.py`

Current raw subscription pattern:

```python
PointCloud2, "/velodyne_points", ...
```

Options:

- Leave it on `/velodyne_points` if you want obstacle conversion to continue using the raw cloud.
- Move it to `/velodyne_points_stable` if you want obstacle conversion to consume the fallback output too.

This is a behavior decision, not a requirement for the node transplant.

### B. Velodyne RPM tuning

If the new branch also needs higher raw Velodyne frequency, the Velodyne driver launch can be given:

```python
{"model": "VLP16", "rpm": 600.0}
```

This is not required for the fallback node to function.

## Build Step Required After Changes

After the file is added and the launch/package wiring is changed:

```bash
colcon build --packages-select localization_launch
source install/setup.bash
```

Why:

- The new `console_scripts` entry will not exist until the package is rebuilt.

## Validation Checklist

After launching the stack, verify these:

1. The fallback node is present:

```bash
ros2 node list | grep velodyne_pointcloud_tf_fallback
```

2. The stabilized topic exists:

```bash
ros2 topic list | grep velodyne_points_stable
```

3. The fallback node subscribes to raw and publishes stable:

```bash
ros2 node info /velodyne_pointcloud_tf_fallback
```

You should see:

- subscriber on `/velodyne_points`
- publisher on `/velodyne_points_stable`

4. The stable topic is active:

```bash
ros2 topic hz /velodyne_points_stable
```

5. Lidar localization is actually consuming the stable stream:

```bash
ros2 node info /lidar_localization
```

Check that its cloud input resolves to `/velodyne_points_stable`.

## Minimum Necessary Change Set

If you want the shortest possible list for Codex on the new branch, this is it:

1. Add `velodyne_pointcloud_tf_fallback.py` to `localization_launch/localization_launch/`
2. Add the `console_scripts` entry in `localization_launch/setup.py`
3. Add the needed dependencies in `localization_launch/package.xml`
4. Launch the node from `localization_full_launcher.launch.py`
5. Remap lidar localization to `/velodyne_points_stable`
6. Point RViz `PointCloudVelodyne` to `/velodyne_points_stable`
7. Rebuild `localization_launch`

## Suggested Prompt for the Next Codex Run

You can paste something like this on the new branch:

> I already copied `velodyne_pointcloud_tf_fallback.py` into `cart_control/localization_launch/localization_launch/`. Please use `summer2026_fallback_node_migration.md` to make the remaining required changes so the fallback node builds, launches, feeds `/velodyne_points_stable`, and is consumed by lidar localization and RViz.

