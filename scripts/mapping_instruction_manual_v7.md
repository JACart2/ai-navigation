# Mapping System Instruction Manual V7

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

It is very important that the start and end of the mapping run have predictable overlap, such as a long, straight corridor. In the next step, we will demarcate the overlapping segments using keyframes. For east_campus_mola_georef.mm, I used the astronomy park hedges as my area of overlap.

When finished, press `Ctrl+C` once (Pressing more than once will terminate the saving process) in the collection terminal. The bag file should be saved in:

```text
/root/dev_ws/bagfiles/TEST_NAME/
```

Check the bag's contents:

```bash
ros2 bag info "/root/dev_ws/bagfiles/$TEST_NAME"
```

It should contain `/velodyne_points`, `/zed_front/zed_node_0/imu/data`, `/tf`, `/tf_static`, and `/fix`.

### 4.1 Generate the Raw Simplemap With Post-Processing

post-processing is a stack of multiple operations, it preforms, in order:

1. Terrain-aware offline mapping (simplemap generation) from the recorded bag.
2. Resolve the two bag-time overlap ranges to offline SimpleMap keyframes.
3. Sample six ordered seed pairs across the overlap ranges.
4. Run broad non-ground KISS+ICP registration on ±15-keyframe submaps; require at least four mutually consistent registrations and optimize with GNC.
5. Run the f1.1.10 narrow retroactive pass: 0.25–2.5 m candidates, ±10-keyframe non-ground submaps, 20 candidates per round, and three rounds.
6. Rigidly level the final loop-closed SimpleMap.
7. Fit GNSS georeferencing.
8. Generate the structured GICP localization `.mm`.

Large bags can take significant time and memory. Messages saying the desired real-time rate was not achieved are expected during offline processing and do not mean the map failed.

Do not interrupt the offline mapping stage after the bag reaches 100%. MOLA performs an automatic clean shutdown and then serializes the map.

Run the simplemap generation component of post_processing. This should bring up a replay of the bag run. If you fullscreen the bag replay, you should see the current added keyframe in the bottom left corner, labeled "simplemap:" For the starting and ending components of the mapping run, record...

1. when the first traversal enters the overlap; 
2. when the first traversal leaves the overlap; 
3. when the final traversal enters the overlap; 
4. when the final traversal leaves the overlap. 

```bash
./scripts/launch_mola_post_processing "$TEST_NAME" \
  --skip-loop-closure \
  --skip-georef \
  --skip-georef-mm
```

### 4.2 Generate the loop-closed Simplemap and Metric Map With Post-Processing

With the keyframes recorded, 

```bash
./scripts/launch_mola_post_processing "$TEST_NAME" \
  --skip-offline-map \
  --loop-start-keyframes START_ENTER:START_LEAVE \
  --loop-end-keyframes END_ENTER:END_LEAVE \
  --loop-traversal reverse
```

This should finish generating the loop-closed simplemap and localization metric map.

Use `--loop-traversal same` when both passes were driven in the same direction.
An alternative to keyframes are `--loop-start-times SEC:SEC` and `--loop-end-times SEC:SEC`


### 5. Outputs

The default outputs are:

```text
/root/dev_ws/maps/TEST_NAME_offline.simplemap
/root/dev_ws/maps/TEST_NAME_lc.simplemap
/root/dev_ws/maps/TEST_NAME_georef_localization.mm
```

- The Bag: The raw data useful for producing all other outputs starting from step 4. 
- `TEST_NAME_offline.simplemap`: The source map.
- `TEST_NAME_lc.simplemap`: The two-pass loop-closed and rigidly leveled source map.
- `TEST_NAME_georef_localization.mm`: The structured live-localization map with georeferencing.
- `TEST_NAME_lc_seed.log`: Seed registration, atomic-consensus, and GNC record.
- `TEST_NAME_lc_retroactive.log`: Automatic candidate and final GNC record.

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
| `--pipeline PATH` | `scripts/loop_closure_waypoint_retroactive.yaml` | Base KISS+ICP configuration. Waypoint and retroactive temporary pipelines are generated from it. |
| `--loop-start-times SEC:SEC` | required for new bags | Absolute bag/ROS timestamps for entering and leaving the first overlap traversal. |
| `--loop-end-times SEC:SEC` | required for new bags | Absolute bag/ROS timestamps for entering and leaving the final overlap traversal. |
| `--loop-start-keyframes A:B` | expert alternative | Inclusive keyframes from the authoritative offline SimpleMap; never use live metric-map IDs. |
| `--loop-end-keyframes A:B` | expert alternative | Corresponding final-traversal offline keyframe range. |
| `--loop-traversal same|reverse` | `reverse` | Describes how the two ranges correspond in travel order. |
| `--loop-waypoint-pairs N` | `6` | Number of ordered seed registrations sampled across the ranges. |
| `--legacy-auto-endpoint-loop` | off | Explicitly selects the older v6 first/last-percentage workflow. |
| `--endpoint-window-percent PCT` | `5.0` | Legacy mode only: first/last pose percentage. |
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
| `--save-seed-loop-simplemap` | off | Retains the otherwise temporary output of the first waypoint-seed pass. Seed and retroactive text logs are always retained. |
| `--save-unleveled-simplemap` | off | Retains the otherwise temporary output of the retroactive pass before leveling. |
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

- Offline mapping allows full 6-DoF terrain motion with IMU initialization, gravity correction, and IMU deskew.
- Operator-selected bag timestamps are resolved to the nearest authoritative offline SimpleMap keyframes.
- Six seed pairs are sampled in travel order across the user-bounded overlap; reverse traversal reverses the first range before pairing.
- Seed registration uses KISS-Matcher plus ICP on non-ground ±15-keyframe submaps.
- At least four seed registrations must succeed, and their corrections must agree within 3 m and 6 degrees before the hypothesis is accepted.
- The seed graph uses 0.04 m/m translational and 0.10 degree/m angular odometry uncertainty.
- The retroactive pass considers only 0.25–2.5 m candidates, evaluates 20 per round for three rounds, and uses ±10-keyframe non-ground submaps.
- GNSS and planar-world constraints remain disabled during both geometric loop-closure passes.
- The current loop-closure build reports zero ICP quality for otherwise converged KISS+ICP registrations, so the reproduced f1.1.10 workflow uses transform consensus and GNC instead of that broken scalar gate.
- The final loop-closed map is rigidly leveled before GNSS fitting and localization-map conversion.
- Temporary seed, unleveled, and georeference artifacts are removed unless their save options are supplied.


### 4 Waypoint-guided two-pass loop closure

The default v7 workflow does not guess overlap from the first and last five
percent of a drifted trajectory. The operator records bag timestamps at the start and end of both
traversals of a recognizable overlap area. Post-processing samples ordered
pairs inside those ranges and lets KISS+ICP estimate the actual six-degree-of-
freedom transformations.

Ranges must be written in the order each traversal was driven. For a reverse
return through a hedge corridor, use `--loop-traversal reverse`; the first range
is paired from its last keyframe toward its first while the final range is
paired from first toward last.

Before accepting the seed, verify the terminal reports:

- six waypoint registrations attempted;
- at least four succeeded;
- the atomic hypothesis was accepted;
- GNC retained a coherent set of seed factors.

The second pass should report nearby automatic candidates and a final GNC
inlier/outlier count. A zero-factor seed, rejected atomic hypothesis, or zero
automatic candidates is not a successful reproduction. Post-processing now
stops before leveling/georeferencing in each of those cases. Inspect
`TEST_NAME_lc_seed.log` and `TEST_NAME_lc_retroactive.log`.

To inspect intermediate maps, add:

```bash
--save-seed-loop-simplemap --save-unleveled-simplemap --save-trajectory-files
```

The older v6 endpoint-percentage workflow remains available for comparison:

```bash
./scripts/launch_mola_post_processing "$TEST_NAME" \
  --legacy-auto-endpoint-loop
```

It is not recommended for production because nearest poses in a drifted
trajectory are not reliable physical correspondences.

