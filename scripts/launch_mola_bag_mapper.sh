#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  launch_mola_bag_mapper.sh [--cart james|madison] TEST_NAME [options]
  launch_mola_bag_mapper.sh james TEST_NAME [options]
  launch_mola_bag_mapper.sh madison TEST_NAME [options]

Runs mapping_instruction_manual_v4 steps 2.1 through 2.11:
  2.1 source /root/dev_ws/install/setup.bash
  2.2 publish base_link -> velodyne
  2.3 launch Velodyne
  2.4 launch ZED cameras/IMU
  2.5 launch Garmin GPS
  2.6 publish base_link -> gps
  2.7 run sensor and TF validation checks
  2.8 record a ROS 2 bag
  2.9 configure MOLA map outputs
  2.10 start the MOLA mapper
  2.11 monitor GPS auto-anchor readiness

Required/primary parameters:
  --cart james|madison          Cart configuration to use. Default: madison
  TEST_NAME                     Output name for bag, .simplemap, and .mm files

Common options:
  --gps-port PATH               GPS serial port. Default: /dev/ttyACM0
  --gps-baud BAUD               GPS baud rate. Default: 4800
  --gps-frame FRAME             GPS frame_id. Default: gps
  --imu-topic TOPIC             IMU topic to validate/record. Default: /zed_front/zed_node_0/imu/data
  --use-imu-roll-pitch          Use IMU gravity for initial roll/pitch. Not recommended on James until IMU TF is calibrated.
  --use-imu-deskew              Use IMU motion compensation. Default is linear deskew for safer live checking.
  --lidar-topic TOPIC           Lidar topic to validate. Default: /velodyne_points
  --lidar-frame FRAME           Lidar TF child frame. Default: velodyne
  --base-frame FRAME            Base TF frame. Default: base_link
  --gps-x M --gps-y M --gps-z M GPS antenna transform from base_link. Default: 0.0 0.0 0.2
  --gps-roll R --gps-pitch P --gps-yaw Y
                                GPS rotation from base_link, radians. Default: 0.0 0.0 0.0
  --enable-rear-zed             Launch rear ZED too. Default is front-ZED-only for mapping.
  --disable-rear-zed            Explicitly launch only the front ZED IMU/camera. Default.
  --bag-dir DIR                 Bag output parent directory. Default: /root/dev_ws/bagfiles
  --maps-dir DIR                Map output parent directory. Default: /root/dev_ws/maps
  --bag-name NAME               Override bag directory name. Default: TEST_NAME
  --map-name NAME               Override .simplemap/.mm base name. Default: TEST_NAME
  --no-bag-record               Do not run step 2.8 bag recording.
  --no-mapper                   Do not run steps 2.9-2.10 MOLA mapper.
  --no-gps-anchor-monitor       Do not run step 2.11 GPS auto-anchor monitor.
  --allow-existing-output       Allow existing bag/map paths instead of refusing to start.
  --skip-validation             Launch sensors/TF only; do not run validation checks.
  --keep-running                Keep launched processes alive after validation. Default behavior.
  --exit-after-validation       Stop launched processes after validation finishes.
  -h, --help                    Show this help.

Examples:
  scripts/launch_mola_bag_mapper.sh --cart madison test1
  scripts/launch_mola_bag_mapper.sh james test1 --gps-port /dev/ttyACM0
  scripts/launch_mola_bag_mapper.sh --cart james test1 --gps-x 0.1 --gps-y -0.2 --gps-z 1.4
  scripts/launch_mola_bag_mapper.sh --cart james test1 --enable-rear-zed

After this script passes validation, leave it running while driving the cart.
Press Ctrl+C when the mapping run is finished; this stops bag recording, MOLA,
and the sensor/TF helper processes.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$script_dir/.." && pwd)"
ws_dir="${DEV_WS:-/root/dev_ws}"
setup_file="$ws_dir/install/setup.bash"

source_ros_setup() {
  # ROS/colcon setup files may reference optional variables such as COLCON_TRACE.
  # Keep the rest of this script strict, but do not let nounset break setup.bash.
  set +u
  # shellcheck source=/root/dev_ws/install/setup.bash
  source "$setup_file"
  set -u
}

cart="madison"
gps_port="/dev/ttyACM0"
gps_baud="4800"
gps_frame="gps"
imu_topic="/zed_front/zed_node_0/imu/data"
use_imu_roll_pitch=false
use_imu_deskew=false
lidar_topic="/velodyne_points"
lidar_frame="velodyne"
base_frame="base_link"
gps_x="0.0"
gps_y="0.0"
gps_z="0.2"
gps_roll="0.0"
gps_pitch="0.0"
gps_yaw="0.0"
rear_zed_enabled=false
run_validation=true
keep_running=true
test_name=""
bag_dir="$ws_dir/bagfiles"
maps_dir="$ws_dir/maps"
bag_name=""
map_name=""
record_bag=true
start_mapper=true
start_gps_anchor_monitor=true
allow_existing_output=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    james|madison)
      cart="$1"
      shift
      ;;
    --cart|--cart-name)
      cart="${2:?$1 requires james or madison}"
      shift 2
      ;;
    cart:=*)
      cart="${1#cart:=}"
      shift
      ;;
    --gps-port)
      gps_port="${2:?--gps-port requires a path}"
      shift 2
      ;;
    --gps-baud)
      gps_baud="${2:?--gps-baud requires a baud rate}"
      shift 2
      ;;
    --gps-frame)
      gps_frame="${2:?--gps-frame requires a frame id}"
      shift 2
      ;;
    --imu-topic)
      imu_topic="${2:?--imu-topic requires a topic}"
      shift 2
      ;;
    --use-imu-roll-pitch)
      use_imu_roll_pitch=true
      shift
      ;;
    --use-imu-deskew)
      use_imu_deskew=true
      shift
      ;;
    --lidar-topic)
      lidar_topic="${2:?--lidar-topic requires a topic}"
      shift 2
      ;;
    --lidar-frame)
      lidar_frame="${2:?--lidar-frame requires a frame id}"
      shift 2
      ;;
    --base-frame)
      base_frame="${2:?--base-frame requires a frame id}"
      shift 2
      ;;
    --gps-x)
      gps_x="${2:?--gps-x requires a value in meters}"
      shift 2
      ;;
    --gps-y)
      gps_y="${2:?--gps-y requires a value in meters}"
      shift 2
      ;;
    --gps-z)
      gps_z="${2:?--gps-z requires a value in meters}"
      shift 2
      ;;
    --gps-roll)
      gps_roll="${2:?--gps-roll requires radians}"
      shift 2
      ;;
    --gps-pitch)
      gps_pitch="${2:?--gps-pitch requires radians}"
      shift 2
      ;;
    --gps-yaw)
      gps_yaw="${2:?--gps-yaw requires radians}"
      shift 2
      ;;
    --enable-rear-zed)
      rear_zed_enabled=true
      shift
      ;;
    --disable-rear-zed)
      rear_zed_enabled=false
      shift
      ;;
    --bag-dir)
      bag_dir="${2:?--bag-dir requires a directory}"
      shift 2
      ;;
    --maps-dir)
      maps_dir="${2:?--maps-dir requires a directory}"
      shift 2
      ;;
    --bag-name)
      bag_name="${2:?--bag-name requires a name}"
      shift 2
      ;;
    --map-name)
      map_name="${2:?--map-name requires a name}"
      shift 2
      ;;
    --no-bag-record)
      record_bag=false
      shift
      ;;
    --no-mapper)
      start_mapper=false
      start_gps_anchor_monitor=false
      shift
      ;;
    --no-gps-anchor-monitor)
      start_gps_anchor_monitor=false
      shift
      ;;
    --allow-existing-output)
      allow_existing_output=true
      shift
      ;;
    --skip-validation)
      run_validation=false
      shift
      ;;
    --keep-running)
      keep_running=true
      shift
      ;;
    --exit-after-validation)
      keep_running=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [[ -z "$test_name" ]]; then
        test_name="$1"
        shift
      else
        echo "Unknown argument: $1" >&2
        usage >&2
        exit 2
      fi
      ;;
  esac
done

cart="${cart,,}"

if [[ -z "$test_name" ]]; then
  test_name="${cart}_$(date +%Y%m%d_%H%M%S)"
  echo "No TEST_NAME provided; using generated test name: $test_name"
fi

bag_name="${bag_name:-$test_name}"
map_name="${map_name:-$test_name}"
bag_output="$bag_dir/$bag_name"
simplemap_output="$maps_dir/$map_name.simplemap"
mm_output="$maps_dir/$map_name.mm"

if [[ "$cart" != "james" && "$cart" != "madison" ]]; then
  echo "Invalid cart '$cart'. Expected 'james' or 'madison'." >&2
  exit 2
fi

if [[ ! -f "$setup_file" ]]; then
  echo "Workspace setup file not found: $setup_file" >&2
  echo "Build/source the workspace first, or set DEV_WS to the correct workspace." >&2
  exit 1
fi

mkdir -p "$bag_dir" "$maps_dir"

if [[ "$allow_existing_output" != true ]]; then
  if [[ "$record_bag" == true && -e "$bag_output" ]]; then
    echo "Bag output already exists: $bag_output" >&2
    echo "Use a new TEST_NAME, --bag-name, or --allow-existing-output." >&2
    exit 1
  fi
  if [[ "$start_mapper" == true && ( -e "$simplemap_output" || -e "$mm_output" ) ]]; then
    echo "Map output already exists:" >&2
    [[ -e "$simplemap_output" ]] && echo "  $simplemap_output" >&2
    [[ -e "$mm_output" ]] && echo "  $mm_output" >&2
    echo "Use a new TEST_NAME, --map-name, or --allow-existing-output." >&2
    exit 1
  fi
fi

source_ros_setup

cart_config="$repo_dir/cart_control/cart_launch/config/cart_${cart}.yaml"
if [[ ! -f "$cart_config" ]]; then
  echo "Cart config not found: $cart_config" >&2
  exit 1
fi

effective_cart_config="$cart_config"
if [[ "$rear_zed_enabled" != true ]]; then
  effective_cart_config="/tmp/mola_bag_mapper_${cart}_front_zed_only.yaml"
  cp "$cart_config" "$effective_cart_config"
  if grep -q '^zed_rear_enabled:' "$effective_cart_config"; then
    sed -i 's/^zed_rear_enabled:.*/zed_rear_enabled: false/' "$effective_cart_config"
  else
    printf '\nzed_rear_enabled: false\n' >> "$effective_cart_config"
  fi
fi

case "$cart" in
  james)
    lidar_x="1.524"
    lidar_y="0.0254"
    lidar_z="1.778"
    ;;
  madison)
    lidar_x="0.5"
    lidar_y="0.0"
    lidar_z="1.75"
    ;;
esac
lidar_roll="0.0"
lidar_pitch="0.0"
lidar_yaw="0.0"

pids=()
cleaned_up=false
log_dir="${MOLA_BAG_MAPPER_LOG_DIR:-/tmp/mola_bag_mapper_${cart}_logs}"
mkdir -p "$log_dir"

cleanup() {
  local status=$?
  if [[ "$cleaned_up" == true ]]; then
    exit "$status"
  fi
  cleaned_up=true
  if ((${#pids[@]} > 0)); then
    echo
    echo "Stopping launched mapping setup processes..."
    kill -TERM "${pids[@]}" 2>/dev/null || true
    sleep 2
    kill -KILL "${pids[@]}" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

start_process() {
  local name="$1"
  shift
  local log_file="$log_dir/${name}.log"
  echo "Starting $name"
  echo "  log: $log_file"
  "$@" >"$log_file" 2>&1 &
  pids+=("$!")
}

topic_exists() {
  local topic="$1"
  local topics
  topics="$(ros2 topic list 2>/dev/null || true)"
  grep -Fxq "$topic" <<<"$topics"
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="$2"
  local start_s
  start_s="$(date +%s)"
  while true; do
    if topic_exists "$topic"; then
      return 0
    fi
    if (( $(date +%s) - start_s >= timeout_s )); then
      echo "Timed out waiting for topic: $topic" >&2
      return 1
    fi
    sleep 1
  done
}

run_timed() {
  local timeout_s="$1"
  shift
  if command -v timeout >/dev/null 2>&1; then
    timeout "$timeout_s" "$@"
  else
    "$@"
  fi
}

validation_failures=()

record_validation_failure() {
  validation_failures+=("$1")
}

check_topic_available() {
  local topic="$1"
  local label="$2"
  if topic_exists "$topic"; then
    echo "OK: $label topic is present ($topic)"
    return 0
  fi
  echo "MISSING: $label topic is not present ($topic)"
  record_validation_failure "$label topic missing: $topic"
  return 1
}

check_topic_sample() {
  local topic="$1"
  local label="$2"
  if run_timed 8s ros2 topic echo "$topic" --once >/tmp/mola_bag_mapper_sample.out 2>/tmp/mola_bag_mapper_sample.err; then
    echo "OK: received one $label message from $topic"
    sed -n '1,25p' /tmp/mola_bag_mapper_sample.out || true
    return 0
  fi
  echo "MISSING/UNHEALTHY: could not receive one $label message from $topic"
  sed -n '1,20p' /tmp/mola_bag_mapper_sample.err || true
  record_validation_failure "$label message sample failed: $topic"
  return 1
}

check_tf_available() {
  local parent="$1"
  local child="$2"
  local label="$3"
  local output_file="/tmp/mola_bag_mapper_tf_${label}.out"
  local err_file="/tmp/mola_bag_mapper_tf_${label}.err"

  # tf2_echo streams forever; timeout exit status alone is not a failure.
  run_timed 8s ros2 run tf2_ros tf2_echo "$parent" "$child" >"$output_file" 2>"$err_file" || true

  if grep -q 'Translation:' "$output_file" && grep -q 'Rotation:' "$output_file"; then
    echo "OK: $label TF is available ($parent -> $child)"
    sed -n '1,20p' "$output_file" || true
    return 0
  fi

  echo "MISSING: $label TF is not available ($parent -> $child)"
  sed -n '1,20p' "$output_file" || true
  sed -n '1,20p' "$err_file" || true
  record_validation_failure "$label TF missing: $parent -> $child"
  return 1
}

wait_for_tf_available() {
  local parent="$1"
  local child="$2"
  local label="$3"
  local timeout_s="$4"
  local start_s
  start_s="$(date +%s)"
  while true; do
    local before_count=${#validation_failures[@]}
    if check_tf_available "$parent" "$child" "$label"; then
      return 0
    fi
    # This is a waiting loop, not the final validation summary; remove the transient failure entry.
    while ((${#validation_failures[@]} > before_count)); do
      unset 'validation_failures[-1]'
    done
    if (( $(date +%s) - start_s >= timeout_s )); then
      record_validation_failure "$label TF missing after warmup: $parent -> $child"
      return 1
    fi
    sleep 1
  done
}

print_validation_summary() {
  echo
  echo "Validation Summary"
  echo "------------------"
  if ((${#validation_failures[@]} == 0)); then
    echo "No missing required sensors or transforms were detected by the validation gate."
    return
  fi
  echo "Missing or unhealthy required items detected:"
  local item
  for item in "${validation_failures[@]}"; do
    echo "  - $item"
  done
  echo
  echo "Do not trust this mapping run until the items above are fixed."
}

echo "Launching MOLA bag-mapping run for cart: $cart"
echo "Test name: $test_name"
echo "Cart config: $cart_config"
echo "ZED launch config: $effective_cart_config"
echo "Rear ZED enabled: $rear_zed_enabled"
echo "Use IMU roll/pitch initialization: $use_imu_roll_pitch"
echo "Use IMU deskew: $use_imu_deskew"
echo "GPS auto-anchor monitor: $start_gps_anchor_monitor"
echo "Bag output: $bag_output"
echo "Simplemap output: $simplemap_output"
echo "MM output: $mm_output"
echo "Logs: $log_dir"
echo

start_process "lidar_tf" \
  ros2 run tf2_ros static_transform_publisher \
    --x "$lidar_x" --y "$lidar_y" --z "$lidar_z" \
    --yaw "$lidar_yaw" --pitch "$lidar_pitch" --roll "$lidar_roll" \
    --frame-id "$base_frame" \
    --child-frame-id "$lidar_frame"

start_process "velodyne" \
  ros2 launch localization_launch velodyne_only.launch.py cart:="$cart"

start_process "zed_cameras" \
  ros2 launch localization_launch cameras.launch.py cart_config_path:="$effective_cart_config"

start_process "garmin_gps18x" \
  ros2 launch localization_launch garmin_gps18x.launch.py \
    port:="$gps_port" \
    baud:="$gps_baud" \
    frame_id:="$gps_frame"

start_process "gps_tf" \
  ros2 run tf2_ros static_transform_publisher \
    --x "$gps_x" --y "$gps_y" --z "$gps_z" \
    --yaw "$gps_yaw" --pitch "$gps_pitch" --roll "$gps_roll" \
    --frame-id "$base_frame" \
    --child-frame-id "$gps_frame"

echo
echo "Waiting for primary topics..."
wait_for_topic "$lidar_topic" 45 || true
wait_for_topic "$imu_topic" 45 || true
wait_for_topic "/fix" 30 || true

if [[ "$run_validation" == true ]]; then
  echo
  echo "2.7 Sensor And TF Validation Gate"
  echo "---------------------------------"

  # echo
  # echo "Active ROS topics:"
  # ros2 topic list || true

  # echo
  # echo "Active ROS nodes:"
  # ros2 node list || true

  echo
  echo "Required topic presence checks:"
  check_topic_available "$lidar_topic" "Velodyne point cloud" || true
  check_topic_available "$imu_topic" "IMU" || true
  check_topic_available /fix "GNSS fix" || true

  echo
  echo "Velodyne rate check: $lidar_topic"
  run_timed 8s ros2 topic hz "$lidar_topic" || true

  echo
  echo "Velodyne sample: $lidar_topic"
  check_topic_sample "$lidar_topic" "Velodyne point cloud" || true

  echo
  echo "IMU sample: $imu_topic"
  check_topic_sample "$imu_topic" "IMU" || true

  echo
  echo "IMU rate check: $imu_topic"
  run_timed 8s ros2 topic hz "$imu_topic" --window 20 || true

  echo
  echo "GNSS sample: /fix"
  if ROS_LOCALHOST_ONLY=1 run_timed 8s ros2 topic echo /fix --once >/tmp/mola_bag_mapper_fix.out 2>/tmp/mola_bag_mapper_fix.err; then
    echo "OK: received one GNSS fix message from /fix"
    sed -n '1,40p' /tmp/mola_bag_mapper_fix.out || true
    if grep -q 'status: -1' /tmp/mola_bag_mapper_fix.out; then
      echo "UNHEALTHY: GNSS is publishing but reports status -1, which is not usable for georeferencing."
      record_validation_failure "GNSS /fix status is -1"
    fi
  else
    echo "MISSING/UNHEALTHY: could not receive one GNSS fix message from /fix"
    sed -n '1,20p' /tmp/mola_bag_mapper_fix.err || true
    record_validation_failure "GNSS message sample failed: /fix"
  fi

  imu_frame="$(run_timed 8s ros2 topic echo "$imu_topic" --once --field header.frame_id 2>/dev/null | awk 'NF {print; exit}' || true)"
  if [[ -z "$imu_frame" ]]; then
    imu_frame="zed_front_imu_link"
    echo "Could not read IMU frame from $imu_topic; falling back to expected frame: $imu_frame"
    record_validation_failure "could not read IMU frame_id from $imu_topic"
  fi

  echo
  echo "TF warmup and checks after sensors are publishing:"
  wait_for_tf_available "$base_frame" "$lidar_frame" "lidar" 20 || true
  wait_for_tf_available "$base_frame" "$gps_frame" "GPS" 20 || true
  wait_for_tf_available "$base_frame" "$imu_frame" "IMU" 20 || true

  echo
  echo "Validation gate finished."
  print_validation_summary
fi

if [[ "$keep_running" != true ]]; then
  echo
  echo "Exiting after validation; bag recording and MOLA mapper were not started."
  exit 0
fi

if [[ "$record_bag" == true ]]; then
  echo
  echo "2.8 Start Bag Recording"
  echo "-----------------------"
  start_process "bag_record" \
    ros2 bag record \
      --storage mcap \
      --topics \
      /tf \
      /tf_static \
      "$lidar_topic" \
      "$imu_topic" \
      /fix \
      -o "$bag_output"
fi

if [[ "$start_mapper" == true ]]; then
  echo
  echo "2.9 Configure The Mapper With Export Commands"
  echo "---------------------------------------------"
  export MOLA_LIDAR_TOPIC="$lidar_topic"
  export MOLA_LIDAR_TOPIC_TYPE=PointCloud2
  export MOLA_START_ACTIVE=True
  export MOLA_MAPPING_ENABLED=True
  export MOLA_GENERATE_SIMPLEMAP=false

  export MOLA_SIMPLEMAP_MIN_XYZ=1.0
  export MOLA_SIMPLEMAP_MIN_ROT=20.0

  export MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES=0.25
  export MOLA_MIN_ROT_BETWEEN_MAP_UPDATES=5.0
  export MOLA_PUBLISH_LOCAL_MAP_UPDATES_EVERY_N=40

  export MOLA_TF_BASE_LINK="$base_frame"
  export MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION=true
  if [[ "$use_imu_roll_pitch" == true || "$use_imu_deskew" == true ]]; then
    export MOLA_NAVSTATE_IMU_SENSOR_NAME='.*'
  else
    export MOLA_NAVSTATE_IMU_SENSOR_NAME='__disabled_for_mapping__'
  fi

  export MOLA_OBS_VALIDITY_MIN_POINTS=100
  export MOLA_ENABLE_OBS_VALIDITY_FILTER=True

  export MOLA_IMU_TOPIC="$imu_topic"
  if [[ "$use_imu_deskew" == true ]]; then
    export MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU
  else
    export MOLA_DESKEW_METHOD=MotionCompensationMethod::Linear
  fi

  if [[ "$use_imu_roll_pitch" == true ]]; then
    export MOLA_LO_INITIAL_LOCALIZATION_METHOD=InitLocalization::PitchAndRollFromIMU
    export MOLA_IMU_GRAVITY_CORRECTION=true
  else
    export MOLA_LO_INITIAL_LOCALIZATION_METHOD=InitLocalization::FixedPose
    export MOLA_IMU_GRAVITY_CORRECTION=false
  fi
  export MOLA_IMU_GRAVITY_SIGMA_DEG=3.0

  export MOLA_GNSS_TOPIC=/fix

  export MOLA_SIMPLEMAP_OUTPUT="$simplemap_output"
  export MOLA_SAVE_MM="$mm_output"

  echo "MOLA_SIMPLEMAP_OUTPUT=$MOLA_SIMPLEMAP_OUTPUT"
  echo "MOLA_SAVE_MM=$MOLA_SAVE_MM"
  echo "MOLA_LO_INITIAL_LOCALIZATION_METHOD=$MOLA_LO_INITIAL_LOCALIZATION_METHOD"
  echo "MOLA_DESKEW_METHOD=$MOLA_DESKEW_METHOD"
  echo "MOLA_IMU_GRAVITY_CORRECTION=$MOLA_IMU_GRAVITY_CORRECTION"
  echo "MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION=$MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION"
  echo "MOLA_NAVSTATE_IMU_SENSOR_NAME=$MOLA_NAVSTATE_IMU_SENSOR_NAME"

  echo
  echo "2.10 Start The Mapper"
  echo "---------------------"
  start_process "mola_mapper" \
    /opt/ros/jazzy/lib/mola_launcher/mola-cli \
      /opt/ros/jazzy/share/mola_lidar_odometry/mola-cli-launchs/lidar_odometry_ros2.yaml
fi

if [[ "$start_gps_anchor_monitor" == true ]]; then
  echo
  echo "2.11 Monitor GPS Auto-Anchor Readiness"
  echo "--------------------------------------"
  echo "This does not change the mapper. It watches /fix and /lidar_odometry/pose"
  echo "and logs when GPS has a healthy anchor plus enough motion to estimate yaw."
  start_process "gps_auto_anchor_monitor" \
    ros2 run localization_launch gps_auto_anchor_monitor --ros-args \
      -p fix_topic:=/fix \
      -p pose_topic:=/lidar_odometry/pose \
      -p gps_max_xy_std:=12.0 \
      -p min_samples:=5 \
      -p min_move_m:=8.0
fi

if [[ "$keep_running" == true ]]; then
  echo
  echo "Mapping run is active. Drive the cart now. Press Ctrl+C when finished."
  echo "Outputs will be written to:"
  echo "  bag:       $bag_output"
  echo "  simplemap: $simplemap_output"
  echo "  mm:        $mm_output"
  if [[ "$start_gps_anchor_monitor" == true ]]; then
    echo "  gps anchor log: $log_dir/gps_auto_anchor_monitor.log"
  fi
  wait
else
  exit 0
fi

