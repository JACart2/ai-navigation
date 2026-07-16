# Mapping System Instruction Manual V4

This manual describes the mapping workflow used in this workspace when you want to do all of the following in a single run:

- Build a live map with MOLA from lidar and IMU data.
- Record a ROS 2 bag at the same time for later replay or debugging.
- Include GNSS data in the bag so the resulting map can be georeferenced later.
- Monitor whether GPS and MOLA odometry have enough healthy overlap to learn an automatic recovery anchor.
- Preserve the `.simplemap` as the main mapping artifact and generate `.mm` products from it.

This version keeps the structure of `mapping_instruction_manual_v2.md` and adds bag recording plus GNSS capture and georeferencing steps.

## 1. How The Mapping System Works

The mapping system builds a map incrementally from lidar data. The first lidar scan acts as the seed for the map. Each successive lidar scan is registered against the existing map estimate, and the mapper grows the map as the robot moves.

The current mapping system makes use of the Velodyne for SLAM and the ZED IMU for assisted localization. GNSS is optional during mapping, but if it is recorded and passed into MOLA it can later be used for georeferencing. During live runs, the GPS auto-anchor monitor can also compare `/fix` against MOLA odometry to confirm whether GPS recovery would have a useful map-frame anchor.

### Map File Types

The mapping workflow uses three main data formats:

- ROS 2 bag (`.mcap` or bag directory): A time-recorded dataset of topics such as lidar, IMU, TF, and GNSS. This is the raw replayable sensor log.
- `.simplemap`: The final raw mapping dataset. This stores the keyframes, observations, poses, and supporting data produced by the mapper. It is the main artifact to keep because it can be reused later to generate different `.mm` maps.
- `.mm`: A metric map file. This is the format used by MOLA tools for localization, visualization, map processing, and georeferencing output.

There are two common uses of `.mm` maps:

- Display `.mm` map: A flattened point-cloud map used for viewing the whole mapped area in `mm-viewer`.
- Localization `.mm` map: A structured map used by live localization or SLAM.

### Important GNSS Note

If GNSS is available during mapping and `MOLA_GNSS_TOPIC` is set, the `.simplemap` can contain the GNSS observations needed for later georeferencing.

However, the georeferenced output is written later into an `.mm` file with `mola-sm-georeferencing`. The `.simplemap` should still be preserved as the source artifact.

## 2. Mapping Process With Live Bag Recording

### 2.1 Source The Workspace

Before running MOLA or ROS commands, source the workspace:

```bash
source /root/dev_ws/install/setup.bash
```

### 2.2 Publish TF Frames

In a terminal, start the static transform publisher for the lidar frame so the mapper knows where the lidar is compared to `base_link`.

For Madison:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.5 --y 0.0 --z 1.75 \
  --yaw 0.0 --pitch 0.0 --roll 0.0 \
  --frame-id base_link \
  --child-frame-id velodyne
```

For James:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 1.524 --y 0.0254 --z 1.778 \
  --yaw 0.0 --pitch 0.0 --roll 0.0 \
  --frame-id base_link \
  --child-frame-id velodyne
```

### 2.3 Start The Velodyne

In another terminal, start the lidar driver.

```bash
ros2 launch localization_launch velodyne_only.launch.py cart:=madison
```

Use `cart:=james` if mapping with James.


### 2.4 Start The ZED IMU

In another terminal, start the ZED IMU.

For Madison:

```bash
ros2 launch localization_launch cameras.launch.py \
  cart_config_path:=/root/dev_ws/src/ai-navigation/cart_control/cart_launch/config/cart_madison.yaml
```

For James:

```bash
ros2 launch localization_launch cameras.launch.py \
  cart_config_path:=/root/dev_ws/src/ai-navigation/cart_control/cart_launch/config/cart_james.yaml
```


### 2.5 Start The GPS

In another terminal, start the Garmin GPS driver so the bag and MOLA can capture GNSS fixes on `/fix`.

```bash
ros2 launch localization_launch garmin_gps18x.launch.py \
  port:=/dev/serial/by-id/usb-Microchip_Technology_Inc._MCP2221_USB-I2C_UART_Combo-if00
```


If your system uses a filtered GNSS topic from another node, adjust the later bag-record and `MOLA_GNSS_TOPIC` commands accordingly. The current in-repo GPS launch publishes `/fix`.

### 2.6 Publish The GPS TF Frame

The Garmin GPS driver publishes fixes with `frame_id: gps`. MOLA needs a TF from `base_link` to `gps` before it can attach GNSS observations to the simplemap.

Use the measured GPS antenna offset if it is known. If the antenna offset has not been measured yet, use this temporary transform only to confirm the workflow, then replace it before collecting final georeferenced data:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.2 \
  --yaw 0.0 --pitch 0.0 --roll 0.0 \
  --frame-id base_link \
  --child-frame-id gps
```


### 2.7 Sensor And TF Validation Gate

Run this validation gate after the lidar, IMU, GPS, lidar TF, and GPS TF terminals are all running. Do not start bag recording or MOLA mapping until the required topics and transforms pass these checks.

List the active ROS graph:

```bash
ros2 topic list
ros2 node list
```

Verify the Velodyne point cloud:

```bash
ros2 topic hz /velodyne_points
```

The expected point-cloud frame is `velodyne`. The expected rate for the current VLP-16 setup is about 10 Hz.

Verify the IMU topic that will be recorded and passed into MOLA:

```bash
ros2 topic hz /zed_front/zed_node_0/imu/data
```

If you are using a different IMU source, such as the Inertial Sense IMU on `/imu`, replace the topic in the bag-record and MOLA export commands before continuing.

Verify GNSS fixes:

```bash
ROS_LOCALHOST_ONLY=1 ros2 topic echo /fix --once
```

For georeferencing, do not continue until the message reports a valid fix status. `status: -1` means the driver is publishing but the GPS does not currently have a usable fix.

Verify required TF transforms:

```bash
ros2 run tf2_ros tf2_echo base_link velodyne
ros2 run tf2_ros tf2_echo base_link gps
ros2 run tf2_ros tf2_echo base_link zed_front_imu_link
```

If using `/imu` instead of the ZED IMU, verify the IMU frame for that source instead. The frame must match the IMU message `header.frame_id` and must connect back to `base_link`.

### 2.8 Start Bag Recording

In another terminal, create a bag output directory and start recording the core mapping topics.

```bash
mkdir -p /root/dev_ws/bagfiles

ros2 bag record \
  --storage mcap \
  --topics \
  /tf \
  /tf_static \
  /velodyne_points \
  /zed_front/zed_node_0/imu/data \
  /fix \
  -o /root/dev_ws/bagfiles/<bag_name>
```

This records the minimum topic set needed for later offline replay plus GNSS-based georeferencing work.

If you also have a filtered or validated GNSS topic, you may additionally record it:

```bash
ros2 bag record \
  --storage mcap \
  --topics \
  /tf \
  /tf_static \
  /velodyne_points \
  /zed_front/zed_node_0/imu/data \
  /fix \
  /fix_filtered \
  -o /root/dev_ws/bagfiles/<bag_name>
```

### 2.9 Configure The Mapper With Export Commands

In another terminal, set the key environment variables that control how often map keyframes are saved, how often the map is updated, and which GNSS topic MOLA should consume.

Each of these need to be run individually instead of using ROS params because ROS kills the mapper while saving the maps by escalating from SIGTERM to SIGKILL.

See part 3 for configuration details.

```bash
mkdir -p /root/dev_ws/maps

export MOLA_LIDAR_TOPIC=/velodyne_points
export MOLA_LIDAR_TOPIC_TYPE=PointCloud2
export MOLA_START_ACTIVE=True
export MOLA_MAPPING_ENABLED=True
export MOLA_GENERATE_SIMPLEMAP=true

export MOLA_SIMPLEMAP_MIN_XYZ=1.0
export MOLA_SIMPLEMAP_MIN_ROT=20.0

export MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES=0.25
export MOLA_MIN_ROT_BETWEEN_MAP_UPDATES=5.0
export MOLA_PUBLISH_LOCAL_MAP_UPDATES_EVERY_N=40

export MOLA_TF_BASE_LINK=base_link
export MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION=True

export MOLA_OBS_VALIDITY_MIN_POINTS=100
export MOLA_ENABLE_OBS_VALIDITY_FILTER=True

export MOLA_IMU_TOPIC=/zed_front/zed_node_0/imu/data
export MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU
export MOLA_LO_INITIAL_LOCALIZATION_METHOD=InitLocalization::PitchAndRollFromIMU
export MOLA_IMU_GRAVITY_CORRECTION=true
export MOLA_IMU_GRAVITY_SIGMA_DEG=3.0

export MOLA_GNSS_TOPIC=/fix

export MOLA_SIMPLEMAP_OUTPUT=/root/dev_ws/maps/<map_name>.simplemap
export MOLA_SAVE_MM=/root/dev_ws/maps/<map_name>.mm
```

`MOLA_SIMPLEMAP_OUTPUT` and `MOLA_SAVE_MM` specify the output map files.

`MOLA_GNSS_TOPIC` tells MOLA to listen for GNSS observations while mapping so those observations are carried into the mapping dataset for later georeferencing.

### 2.10 Start The Mapper

In the same terminal, launch the mapper.

```bash
/opt/ros/jazzy/lib/mola_launcher/mola-cli \
  /opt/ros/jazzy/share/mola_lidar_odometry/mola-cli-launchs/lidar_odometry_ros2.yaml \
  --ros-args -r /tf:=tf -r /tf_static:=tf_static
```

### 2.11 Monitor GPS Auto-Anchor Readiness

In another terminal, start the GPS auto-anchor monitor. This does not change the mapper or fuse GPS into the map. It only watches `/fix` and `/lidar_odometry/pose` and reports whether the run has enough good GPS plus healthy MOLA motion to learn the GPS-to-map anchor and yaw.

```bash
ros2 run localization_launch gps_auto_anchor_monitor --ros-args \
  -p fix_topic:=/fix \
  -p pose_topic:=/lidar_odometry/pose \
  -p gps_max_xy_std:=12.0 \
  -p min_samples:=5 \
  -p min_move_m:=8.0
```

Good signs:

- The monitor prints `GPS auto-anchor captured` after MOLA odometry and GPS are both healthy.
- After driving at least 8 meters, it prints `GPS auto-anchor yaw learned`.
- Once yaw is learned, GPS can be used as a broad recovery suggestion if live localization is lost.

Warning signs:

- It keeps waiting for good GPS fixes, which usually means `/fix` has `status: -1`, stale data, or covariance/noise is too high.
- It captures an anchor but never learns yaw, which usually means the cart has not moved far enough while MOLA localization was healthy.
- GPS and map movement distances disagree strongly, which suggests GPS quality, TF, or localization quality should be checked before trusting georeferencing.

### 2.12 Monitor Live Map Quality While Recording

During the run, use the live map in RViz or MOLA Viz as the primary quality check for whether the bag is worth keeping. The topic and TF checks should already have passed in section 2.7.

Good signs:

- The live map grows smoothly as the cart moves.
- The latest scan aligns with the existing map instead of double-drawing edges.
- Sharp turns do not cause long freezes or large map smearing.
- The live map keeps updating while the cart moves.

Warning signs:

- The live map stops updating.
- The current scan visibly separates from the map.
- Straight structures become duplicated or warped.
- The cart must frequently stop to recover localization.

If any warning sign appears, stop driving and return to the validation gate in section 2.7 before continuing the run.

### 2.13 Drive The Cart While Mapping

Drive the cart around the mapping area. Make sure the cart maintains a consistent slow speed. Also slow down during sharp turns.

Keep track of where the cart started when mapping. At the end of mapping, try to end the test in the same location that the cart started.

Make sure that there is a substantial amount of overlap between the start and end of the scanned area. This is important for loop closure.

If the live map does not update as fast, that means localization is getting more difficult, and the cart needs to slow down.

If the live map stops updating, that means that localization has been lost and the map has stopped growing. Try to align the cart with the location in the live map to attempt relocalization.

### 2.14 Save The Generated Map And Bag

When the mapping run is complete:

1. Stop the mapper with `Ctrl + C`.
2. Stop the bag recorder with `Ctrl + C`.

The mapper should save the generated `.simplemap` on shutdown. This `.simplemap` is the raw mapping result and should be preserved.

The bag in `/root/dev_ws/bagfiles/<bag_name>` should also be preserved so the run can be replayed later if needed.

## 2.15 Optional: Rebuild The Map Offline From The Recorded Bag

If you later want to rebuild or compare the map offline from the recorded bag, reuse the same MOLA topic environment variables and replay with `mola-lo-gui-rosbag2`.

```bash
source /root/dev_ws/install/setup.bash

export MOLA_GENERATE_SIMPLEMAP=true
export MOLA_SIMPLEMAP_OUTPUT=/root/dev_ws/maps/<map_name>_offline.simplemap
export MOLA_LO_INITIAL_LOCALIZATION_METHOD=InitLocalization::PitchAndRollFromIMU
export MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU
export MOLA_IMU_TOPIC=/zed_front/zed_node_0/imu/data
export MOLA_LIDAR_TOPIC=/velodyne_points
export MOLA_TF_BASE_LINK=base_link
export MOLA_GNSS_TOPIC=/fix

mola-lo-gui-rosbag2 /root/dev_ws/bagfiles/<bag_name>
```

You can also point `mola-lo-gui-rosbag2` at the actual `.mcap` file inside the bag directory. Passing the bag directory is usually easier because it keeps the bag metadata and storage file together.

## 2.15 Run Loop Closure

Loop closure corrects accumulated drift by adding constraints between places that the robot revisited.

For large maps, the frame-to-frame loop closure pipeline is usually safer than the full submap/simplemap loop closure pipeline because it uses less memory.

Example manual frame-to-frame loop closure:

```bash
source /root/dev_ws/install/setup.bash

USE_GNSS=false mola-sm-lc-cli \
  -i /root/dev_ws/maps/<map_name>.simplemap \
  -o /root/dev_ws/maps/<map_name>_lc.simplemap \
  -p /root/dev_ws/src/mola_sm_loop_closure/pipelines/loop-closure-f2f-lidar3d-gicp.yaml \
  -a mola::FrameToFrameLoopClosure
```

For best results, add several timestamp pairs between matching places near the beginning and end of the route. A single start/end constraint may not be enough if the odometry drift is large.

## 2.16 Generate A Visual `.mm` Map From The Loop-Closed `.simplemap`

To view the full corrected map, generate a display `.mm` map without passing a localization pipeline:

```bash
source /root/dev_ws/install/setup.bash

sm2mm \
  -i /root/dev_ws/maps/<map_name>_lc.simplemap \
  -o /root/dev_ws/maps/<map_name>_visual.mm
```

This creates a flattened point-cloud map with a `raw` layer that is suitable for viewing.

## 2.17 Generate A Localization `.mm` Map From The Loop-Closed `.simplemap`

To generate a localization map, use the localization map pipeline:

```bash
source /root/dev_ws/install/setup.bash

sm2mm \
  -i /root/dev_ws/maps/<map_name>_lc.simplemap \
  -o /root/dev_ws/maps/<map_name>_localization.mm \
  -p /root/dev_ws/maps/sm2mm_gicp_localmap.yaml \
  -l libmola_metric_maps.so
```

This creates a structured `.mm` map intended for localization rather than simple viewing.

## 2.18 Georeference The GNSS-Enabled Map

If GNSS data was available during mapping and `MOLA_GNSS_TOPIC` was set, you can georeference the resulting map after loop closure.

First, solve for the georeferencing metadata from the GNSS observations stored in the `.simplemap`:

```bash
source /root/dev_ws/install/setup.bash

LD_PRELOAD=/opt/ros/jazzy/lib/x86_64-linux-gnu/libmola_metric_maps.so \
mola-sm-georeferencing \
  -i /root/dev_ws/maps/<map_name>_lc.simplemap \
  -o /root/dev_ws/maps/<map_name>_georef.georef
```

Then generate a georeferenced `.mm` from the loop-closed `.simplemap`:

```bash
sm2mm \
  -i /root/dev_ws/maps/<map_name>_lc.simplemap \
  -o /root/dev_ws/maps/<map_name>_georef.mm \
  -g /root/dev_ws/maps/<map_name>_georef.georef
```

This creates a new `.mm` whose poses are expressed with the georeferencing metadata applied.

If you already generated an `.mm` file and only need to inject georeferencing metadata into that existing map, use:

```bash
LD_PRELOAD=/opt/ros/jazzy/lib/x86_64-linux-gnu/libmola_metric_maps.so \
mola-sm-georeferencing \
  -i /root/dev_ws/maps/<map_name>_lc.simplemap \
  --write-into /root/dev_ws/maps/<existing_map>.mm
```

If you also want to extract the resulting georeferencing metadata:

```bash
mm-georef \
  --extract-from-map \
  -m /root/dev_ws/maps/<map_name>_georef.mm \
  -g /root/dev_ws/maps/<map_name>_georef.yaml \
  -l /opt/ros/jazzy/lib/x86_64-linux-gnu/libmola_metric_maps.so
```

## 2.19 View The Generated Visual `.mm` Map

Open the visual map with:

```bash
mm-viewer /root/dev_ws/maps/<map_name>_visual.mm
```

If the visual map is very large, create a smaller preview first:

```bash
sm2mm \
  -i /root/dev_ws/maps/<map_name>_lc.simplemap \
  -o /root/dev_ws/maps/<map_name>_visual_preview.mm \
  --decimate-nth 5
```

Then view the preview:

```bash
mm-viewer /root/dev_ws/maps/<map_name>_visual_preview.mm
```

## 3. Appendix: Export Statements

These 4 starting parameters are the most important for maintaining localization and controlling end filesize of the end map.

### `MOLA_SIMPLEMAP_MIN_XYZ`

```bash
export MOLA_SIMPLEMAP_MIN_XYZ=1.0
```

Minimum translation, in meters, before saving another keyframe into the `.simplemap`.

Higher values reduce file size at the expense of detail.

### `MOLA_SIMPLEMAP_MIN_ROT`

```bash
export MOLA_SIMPLEMAP_MIN_ROT=20.0
```

Minimum rotation, in degrees, before saving another keyframe into the `.simplemap`.

Higher values reduce file size at the expense of turning detail.

### `MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES`

```bash
export MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES=0.25
```

Minimum translation, in meters, before updating the live map.

Lower values make the live map update more frequently and help keep localization at the expense of CPU usage. Since the live `.mm` cleans itself over time, it is recommended for the update density to be higher or equal to the simplemap settings.

### `MOLA_MIN_ROT_BETWEEN_MAP_UPDATES`

```bash
export MOLA_MIN_ROT_BETWEEN_MAP_UPDATES=5.0
```

Minimum rotation, in degrees, before updating the live map.

Lower values make the live map update more frequently and help keep localization at the expense of CPU usage. Since the live `.mm` cleans itself over time, it is recommended for the update density to be higher or equal to the simplemap settings.

### `MOLA_GNSS_TOPIC`

```bash
export MOLA_GNSS_TOPIC=/fix
```

Topic name for GNSS fixes of type `sensor_msgs/msg/NavSatFix`.

Set this only when GNSS is actively available and correctly timestamped during the run. If you are using a filtered GNSS topic instead, replace `/fix` with that filtered topic name.
