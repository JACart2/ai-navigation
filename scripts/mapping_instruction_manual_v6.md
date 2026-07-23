# Mapping System Instruction Manual V6

This manual describes the map generation process, which has been simplified to two scripts:

- `scripts/launch_mola_bag_mapper.sh` starts the sensors, validates topics and TF, records the bag, and provides a live MOLA map while driving.
- `scripts/launch_mola_post_processing` rebuilds the authoritative map offline, performs loop closure, levels the result, and creates a georeferenced live-localization map.

## How to Make a Map
### 1. Before Mapping

Open a terminal and source the workspace:

```bash
cd /root/dev_ws/src/ai-navigation
source /root/dev_ws/install/setup.bash
```

Name the test:
```bash
TEST_NAME=<test_name>
```

Confirm the hardware is connected:

- Velodyne lidar
- Front ZED IMU
- Garmin GPS

### 2. Collect The Bag

Launch options are listed with:
```bash
./scripts/launch_mola_bag_mapper.sh --help
```

For James:
```bash
./scripts/launch_mola_bag_mapper.sh --cart james $TEST_NAME \
  --gps-port /dev/ttyACM0
```

For Madison:
```bash
./scripts/launch_mola_bag_mapper.sh --cart madison $TEST_NAME \
  --gps-port /dev/ttyACM0
```

Before driving, check the Validation Summary in the terminal to confirm that all sensors and transforms are live. You should see the message:

```bash
Validation Summary
------------------
No missing required sensors or transforms were detected by the validation gate. 
```

### 3. Drive The Route

Drive slowly and smoothly, especially during sharp turns.

During the run, confirm that the live map continues growing and that the map does not freeze or suddenly jump. If the map freezes, that means localization has been lost, and you will need to return the cart to the last known location and orientation (the location in the live map).

The live mapper is a quality monitor. The offline map rebuilt from the bag is the authoritative result because it uses full 6-DoF terrain motion, IMU roll/pitch initialization, IMU deskew, and gravity correction.

Stop in the same place as you started, and make sure that the start and end of the route have predictable overlap, such as a long, straight corridor. For east_campus_mola_georef.mm, I used the astronomy park hedges as my area of overlap.  

When finished, press `Ctrl+C` once (Pressing more than once will terminate the saving process) in the collection terminal. The bag file should be saved in:

```text
/root/dev_ws/bagfiles/TEST_NAME/
```

Check the bag's contents:

```bash
ros2 bag info "/root/dev_ws/bagfiles/$TEST_NAME"
```

It should contain `/velodyne_points`, `/zed_front/zed_node_0/imu/data`, `/tf`, `/tf_static`, and `/fix`.

### 4. Run Post-Processing

Run the complete offline workflow:

```bash
./src/ai-navigation/scripts/launch_mola_post_processing "$TEST_NAME"
```

This performs, in order:

1. Terrain-aware offline mapping from the recorded bag. This generates a non loop-closed simplemap.
2. Generate endpoint loop candidates from the first and last 5% of keyframe poses.
3. KISS-Matcher initialization using non-ground, 11-keyframe submaps. This creates a coarse relation between candidate submaps.
4. ICP refines the matches, GNC rejects outliers, and GNSS remains disabled during loop closure.
5. Rigid leveling of the loop-closed SimpleMap.
6. Add GNSS georeferencing.
7. Structured GICP localization `.mm` generation.

Large bags can take significant time and memory. Messages saying the desired real-time rate was not achieved are expected during offline processing and do not mean the map failed.

Do not interrupt the offline mapping stage after the bag reaches 100%. MOLA performs an automatic clean shutdown and then serializes the map.

### 5. Outputs

The default outputs are:

```text
/root/dev_ws/maps/TEST_NAME_offline.simplemap
/root/dev_ws/maps/TEST_NAME_lc.simplemap
/root/dev_ws/maps/TEST_NAME_georef_localization.mm
```

- The Bag: The raw data useful for producing all other outputs starting from step 4. 
- `TEST_NAME_offline.simplemap`: The source map.
- `TEST_NAME_lc.simplemap`: The loop-closed and leveled source map.
- `TEST_NAME_georef_localization.mm`: The structured live-localization map with georeferencing.

### 6. Inspect The Result

Open the map in Metric Map Viewer:

```bash
  mm-viewer "/root/dev_ws/maps/${TEST_NAME}_georef_localization.mm"
```

Look for discontinuities and incomplete loop closure. If features that were driven by more than once show up multiple times, if the ground has multiple layers, or if there is a global tilt, loop closure has likely failed and needs to be regenerated.

## Mapping from existing Bag files or simplemaps.

You can run post processing from any step. To run starting from an existing bag file:

```bash
./scripts/launch_mola_post_processing <new_name> \
  --bag "/root/dev_ws/bagfiles/$TEST_NAME"
```

If the bag has already generated a simplemap and you just want to run loop closure:

```bash
./scripts/launch_mola_post_processing <new_name> \
  --bag "/root/dev_ws/bagfiles/$TEST_NAME" \
  --simplemap-output /root/dev_ws/maps/TEST_NAME_offline.simplemap \
  --skip-offline-map
```

Use `--skip-georef --skip-georef-mm` when GNSS is absent or invalid.

An EOF error while loading a SimpleMap means the file is incomplete or corrupt. Do not run loop closure or `sm2mm` on it; regenerate it from the preserved bag.

The scripts refuse to overwrite existing outputs by default. Prefer a new test/output name. Use `--allow-existing-output` only when overwriting is intentional.

## Troubleshooting

### The live map looks tilted or layered

The live mapper is a quality monitor and uses conservative settings for real-time operation. Preserve the bag if all required sensor streams are healthy, then judge the authoritative offline result. The post-processing mapper disables planar-motion enforcement and uses IMU roll/pitch initialization, gravity correction, and IMU deskew.

### Offline replay reports that it cannot achieve the desired real-time rate

This is expected for a CPU-heavy offline run. It means processing is slower than the requested replay rate, not that observations were dropped. Do not interrupt the run when bag progress reaches 100 percent; wait for the clean shutdown and final-map save messages.

### SimpleMap loading reports EOF or zero keyframes

The SimpleMap is incomplete or corrupt. Do not feed it into loop closure or `sm2mm`. Regenerate it from the preserved ROS bag. A valid run prints the saved keyframe count and produces a substantial nonzero file.

### Loop closure does not align revisited geometry

Confirm that the route contains distinctive overlapping structure, not only flat ground. Review the manual timestamp hints in the configured pipeline. Those hints must identify the same physical areas in the early and late trajectory. Keep `trust_as_inlier: false` so GNC can reject a bad constraint.

### Start and end poses are not numerically identical

That is not automatically a failure. Loop closure estimates relative scene alignment instead of forcing two sensor origins to the same coordinate. Inspect overlapping walls, poles, curbs, and terrain surfaces.

### The result has a uniform global tilt

The post-processing script runs `sm-cli level` after loop closure. This applies one rigid rotation to the complete map, preserving all relative geometry. The unleveled intermediate is temporary unless `--save-unleveled-simplemap` is supplied. Real hills and local slopes remain in the leveled map.

### GNSS or georeferencing is unavailable

Geometric mapping and loop closure can still be completed. Run post-processing with `--skip-georef --skip-georef-mm`. Do not trust georeferenced products when `/fix` reported `status: -1`, stale timestamps, or excessive covariance.

## Script Input Parameters And Automatic Settings

Both scripts accept command-line parameters. Run either script with `--help` to
see the installed version's authoritative option list. Options change only the
current run; they do not edit cart configuration files.

### 1 Bag-generation script parameters

Basic form:

```bash
./scripts/launch_mola_bag_mapper.sh --cart james|madison TEST_NAME [options]
```

`TEST_NAME` identifies the run. If omitted, the script generates a name from the
cart and current timestamp. Prefer an explicit, unique name in field work.

Sensor and transform inputs:

| Option | Default | Purpose and guidance |
|---|---|---|
| `--cart james|madison` | `madison` | Selects the cart launch/configuration. Always specify it explicitly in field work. |
| `--gps-port PATH` | `/dev/ttyACM0` | Garmin serial device. Change when device enumeration differs. |
| `--gps-baud BAUD` | `4800` | GPS serial baud rate. It must match the receiver. |
| `--gps-frame FRAME` | `gps` | Frame assigned to GNSS fixes. |
| `--lidar-topic TOPIC` | `/velodyne_points` | LiDAR topic validated, recorded, and passed to MOLA. |
| `--lidar-frame FRAME` | `velodyne` | Child frame of the static LiDAR transform. |
| `--imu-topic TOPIC` | `/zed_front/zed_node_0/imu/data` | IMU topic validated and recorded. |
| `--base-frame FRAME` | `base_link` | Vehicle reference frame used by sensor transforms and MOLA. |
| `--gps-x/y/z M` | `0, 0, 0.2` m | GNSS antenna translation from `base_link`. Measure this for reliable georeferencing. |
| `--gps-roll/pitch/yaw R` | `0, 0, 0` rad | GNSS-frame rotation from `base_link`; values are radians. |
| `--enable-rear-zed` | off | Launches the rear ZED in addition to the front unit. It is normally unnecessary for this LiDAR mapping workflow. |
| `--disable-rear-zed` | default | Explicitly selects front-ZED-only operation. |
| `--use-imu-roll-pitch` | off | Initializes the live map's roll/pitch from gravity. Use only with a calibrated IMU transform. |
| `--use-imu-deskew` | off | Uses IMU motion compensation in the live mapper. Linear deskew is the safer live-monitor default. |

Output and execution controls:

| Option | Default | Purpose and guidance |
|---|---|---|
| `--bag-dir DIR` | `/root/dev_ws/bagfiles` | Parent directory for recorded bags. |
| `--bag-name NAME` | `TEST_NAME` | Overrides only the bag directory name. |
| `--maps-dir DIR` | `/root/dev_ws/maps` | Parent directory for live mapper outputs. |
| `--map-name NAME` | `TEST_NAME` | Overrides the live mapper output base name. |
| `--allow-existing-output` | off | Allows reuse of existing paths. Avoid it unless overwriting/reuse is intentional. |
| `--no-bag-record` | off | Runs sensors/live mapper without recording. Do not use for a production mapping run. |
| `--no-mapper` | off | Records sensors without the live MOLA quality monitor. |
| `--no-gps-anchor-monitor` | off | Disables only the GNSS anchor/yaw monitor. |
| `--skip-validation` | off | Skips topic and TF checks. Not recommended in field work. |
| `--keep-running` | enabled | Keeps sensors, recorder, and mapper running after validation. This is normal collection behavior. |
| `--exit-after-validation` | off | Stops after validation instead of collecting a route. |

The bag is recorded as MCAP and contains `/tf`, `/tf_static`, the selected LiDAR
and IMU topics, and `/fix`. The live mapper uses planar-motion enforcement,
0.25 m/5 degree local-map update thresholds, a 100-point scan-validity minimum,
and linear deskew/fixed-pose initialization unless the IMU options are enabled.
Its `.mm` is a live quality-monitor artifact; post-processing rebuilds the
authoritative map from the bag.

### 2 Post-processing script parameters

Basic form:

```bash
./scripts/launch_mola_post_processing TEST_NAME [options]
```

Input, sensor, and pipeline selection:

| Option | Default | Purpose and guidance |
|---|---|---|
| `--bag PATH` | `bagfiles/TEST_NAME` | Explicit input bag directory. Use this when reprocessing under a new output name. |
| `--bag-dir DIR` | `/root/dev_ws/bagfiles` | Bag parent directory when `--bag` is not supplied. |
| `--maps-dir DIR` | `/root/dev_ws/maps` | Parent directory for persistent processed outputs. |
| `--lidar-topic TOPIC` | `/velodyne_points` | LiDAR topic replayed into offline MOLA. Must match the bag. |
| `--imu-topic TOPIC` | `/zed_front/zed_node_0/imu/data` | IMU topic used for offline initialization and deskew. |
| `--gnss-topic TOPIC` | `/fix` | GNSS observations retained for georeferencing. |
| `--base-frame FRAME` | `base_link` | Offline MOLA vehicle frame. |
| `--pipeline PATH` | `maps/final1_v8_nonground_submap.yaml` | Loop-closure configuration and manual overlap hints. Replace/review it for a different route. |
| `--endpoint-window-percent PCT` | `5.0` | Restricts automatic hints to the first and last PCT of keyframe poses; maximum 5%. |
| `--endpoint-loop-pairs N` | `3` | Number of early samples paired with their nearest late-window poses. |
| `--no-auto-endpoint-loop` | off | Uses the manual constraints already present in `--pipeline`. |
| `--localization-pipeline PATH` | `maps/sm2mm_gicp_localmap.yaml` | Converts the final SimpleMap into the structured localization map. Normally leave unchanged. |

Offline motion settings:

| Option | Default | Purpose and guidance |
|---|---|---|
| `--use-imu-roll-pitch` / `--no-imu-roll-pitch` | enabled | Enables or disables gravity-based initial roll/pitch. |
| `--use-imu-deskew` / `--no-imu-deskew` | enabled | Selects IMU or linear scan motion compensation. |

Output-path and retention settings:

| Option | Default | Purpose and guidance |
|---|---|---|
| `--simplemap-output PATH` | `TEST_NAME_offline.simplemap` | Offline map before loop closure. |
| `--lc-simplemap-output PATH` | `TEST_NAME_lc.simplemap` | Final loop-closed and leveled SimpleMap. |
| `--georef-mm-output PATH` | `TEST_NAME_georef_localization.mm` | Final live-localization metric map with embedded georeferencing. |
| `--save-unleveled-simplemap` | off | Retains the otherwise temporary pre-leveling SimpleMap. |
| `--save-georef-file` | off | Retains the otherwise temporary `.georef` sidecar. |
| `--georef-output PATH` | temporary | Selects and retains an explicit `.georef` path. |
| `--save-trajectory-files` | off | Enables diagnostic TUM trajectories and `.cov` sidecars. |
| `--allow-existing-output` | off | Permits reuse/overwrite of output paths. Prefer a new name. |
| `--dry-run` | off | Prints the commands and resolved paths without running them. |

Stage controls:

| Option | Effect |
|---|---|
| `--skip-offline-map` | Reuses `--simplemap-output`; the specified file must already exist. |
| `--skip-loop-closure` | Reuses the final `--lc-simplemap-output`; leveling is skipped with loop closure. |
| `--skip-georef` | Reuses an existing `--georef-output` instead of fitting GNSS. |
| `--skip-georef-mm` | Does not create the final localization `.mm`. |

### 3 Automatic post-processing settings

The script applies these mapping decisions automatically:

- Offline keyframes are retained after approximately 1 meter or 20 degrees.
- Local-map updates occur after approximately 0.25 meter or 5 degrees.
- Offline mapping allows full 6-DoF terrain motion; planar enforcement is disabled.
- IMU roll/pitch initialization, IMU deskew, and gravity correction are enabled by default.
- Three endpoint hints are generated from the first/last 5% pose windows. They are candidates, never forced inliers.
- Loop closure uses KISS-Matcher followed by ICP on non-ground submaps containing the center keyframe plus five neighboring keyframes on each side.
- GNSS is disabled inside geometric loop closure so GPS does not distort scan registration; GNSS is fitted afterward for global placement.
- GNC robust optimization decides which candidate loop constraints remain inliers.
- Loop-closure logs are decimated by 20; large 3D scenes and TUM/COV files are disabled by default.
- The loop-closed map is rigidly leveled before georeferencing and localization-map conversion.
- Localization conversion keeps up to 10,000 points per 1 m keyframe with a 0.15 m adaptive voxel filter. Temporary point layers are cleared after every frame.
- The standalone `.georef` and unleveled SimpleMap are temporary unless their retention flags are supplied.

Use script options instead of manually exporting replacements unless deliberately
testing a new configuration. Record overrides with the bag and resulting maps.


### 4 Automatic endpoint loop-closure configuration

The default base pipeline is:

```text
/root/dev_ws/maps/final1_v8_nonground_submap.yaml
```

The filename is historical; the hard-coded `final1` timestamps are not reused
for new maps. During each run, post-processing exports the new SimpleMap's
keyframe poses and creates a temporary pipeline configuration:

1. It limits each endpoint search window to the first and last 5% of keyframes.
2. It samples three early keyframes evenly across the first window.
3. It pairs each one with the spatially nearest keyframe in the last window,
   according to the preliminary trajectory.
4. Around both keyframes it constructs non-ground submaps from the center frame
   plus five adjacent frames on each side.
5. KISS-Matcher estimates a coarse alignment, ICP refines it, and GNC rejects
   inconsistent loop factors.

Here, “5% of points” means 5% of the stored keyframe poses, not 5% of the
individual LiDAR returns. Generated constraints use `trust_as_inlier: false`;
they are candidates that registration must validate, not commands that force
the selected poses to become equal.

Change the endpoint window or number of pairs with:

```bash
./scripts/launch_mola_post_processing "$TEST_NAME" \
  --endpoint-window-percent 3 \
  --endpoint-loop-pairs 3
```

The endpoint percentage must be greater than zero and no more than five. A
smaller value narrows the search closer to the recording endpoints. This
automation assumes that the final part of the route revisits geometry mapped
near the beginning. It cannot manufacture a valid closure when the two windows
do not contain overlapping, distinctive geometry.

For a route that does not return to its starting area—or when known overlap
occurs elsewhere—disable automatic endpoint selection and supply deliberate
manual constraints:

```bash
./scripts/launch_mola_post_processing "$TEST_NAME" \
  --pipeline /path/to/loop_closure.yaml \
  --no-auto-endpoint-loop
```

Inspect all available options or preview commands without running them:

```bash
./scripts/launch_mola_post_processing --help
./scripts/launch_mola_post_processing "$TEST_NAME" --dry-run
```
