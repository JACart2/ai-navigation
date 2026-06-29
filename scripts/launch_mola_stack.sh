#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

open_ui_page() {
    local url="http://localhost:5173/"
    if command -v xdg-open >/dev/null 2>&1; then
        xdg-open "$url" >/dev/null 2>&1 &
    else
        echo "Open UI manually: $url"
    fi
}


usage() {
  cat <<'EOF'
Usage: launch_mola_stack.sh [--cart NAME] [--no-motor] [--build-image] [--no-cleanup-old-stack] [--no-docker] [--no-ui] [ros2 launch args...]

Examples:
  scripts/launch_mola_stack.sh --cart james
  scripts/launch_mola_stack.sh --cart madison
  scripts/launch_mola_stack.sh cart:=madison
  scripts/launch_mola_stack.sh --cart madison --no-motor
  scripts/launch_mola_stack.sh --cart james enable_mola_auto_localization:=true

Motor control is enabled by default. Use --no-motor or enable_motor:=false for
safe no-motor testing.

Expected UI URLs:
  http://localhost:5173/
  or the alternate URL printed by Vite, for example http://localhost:5174/
EOF
}

cart="madison"
cleanup_old_stack=true
use_docker=true
start_ui=true
motor_enabled=true
build_image=false
cart_forwarded=false
motor_forwarded=false
motor_port_forwarded=false
motor_baudrate_forwarded=false
cart_config_path=""
launch_args=()

cart_config_value() {
  local config_path="$1"
  local key="$2"

  python3 - "$config_path" "$key" <<'PY'
import sys

path, wanted_key = sys.argv[1:3]

try:
    import yaml
except ImportError:
    yaml = None

try:
    if yaml is not None:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        value = data.get(wanted_key)
        if value is not None:
            print(value)
        sys.exit(0)

    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#") or ":" not in stripped:
                continue
            key, value = stripped.split(":", 1)
            if key.strip() == wanted_key:
                print(value.strip().strip("'\""))
                break
except Exception:
    pass
PY
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cart)
      cart="${2:?--cart requires a cart name}"
      shift 2
      ;;
    --enable-motor)
      motor_enabled=true
      shift
      ;;
    --no-motor)
      motor_enabled=false
      shift
      ;;
    --build-image)
      build_image=true
      shift
      ;;
    --cleanup-old-stack)
      cleanup_old_stack=true
      shift
      ;;
    --no-cleanup-old-stack)
      cleanup_old_stack=false
      shift
      ;;
    --no-docker)
      use_docker=false
      shift
      ;;
    --no-ui)
      start_ui=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    cart:=*)
      cart="${1#cart:=}"
      launch_args+=("$1")
      cart_forwarded=true
      shift
      ;;
    cart_name:=*)
      cart="${1#cart_name:=}"
      launch_args+=("cart:=$cart")
      cart_forwarded=true
      shift
      ;;
    cart_config_path:=*)
      cart_config_path="${1#cart_config_path:=}"
      launch_args+=("$1")
      shift
      ;;
    enable_motor:=true)
      motor_enabled=true
      launch_args+=("$1")
      motor_forwarded=true
      shift
      ;;
    enable_motor:=false)
      motor_enabled=false
      launch_args+=("$1")
      motor_forwarded=true
      shift
      ;;
    motor_port:=*)
      launch_args+=("$1")
      motor_port_forwarded=true
      shift
      ;;
    motor_baudrate:=*)
      launch_args+=("$1")
      motor_baudrate_forwarded=true
      shift
      ;;
    *)
      launch_args+=("$1")
      shift
      ;;
  esac
done

cart="${cart,,}"

if [[ "$cart" != "james" && "$cart" != "madison" ]]; then
  echo "Invalid cart '$cart'. Expected 'james' or 'madison'." >&2
  exit 2
fi

if [[ "$cart_forwarded" != true ]]; then
  launch_args=("cart:=$cart" "${launch_args[@]}")
fi

if [[ -z "$cart_config_path" ]]; then
  cart_config_path="$repo_root/cart_control/cart_launch/config/cart_${cart}.yaml"
fi

if [[ -f "$cart_config_path" ]]; then
  if [[ "$motor_port_forwarded" != true ]]; then
    config_motor_port="$(
      cart_config_value "$cart_config_path" motor_port
    )"
    if [[ -n "$config_motor_port" ]]; then
      launch_args=("motor_port:=$config_motor_port" "${launch_args[@]}")
    fi
  fi

  if [[ "$motor_baudrate_forwarded" != true ]]; then
    config_motor_baudrate="$(
      cart_config_value "$cart_config_path" motor_baudrate
    )"
    if [[ -n "$config_motor_baudrate" ]]; then
      launch_args=("motor_baudrate:=$config_motor_baudrate" "${launch_args[@]}")
    fi
  fi
fi

if [[ "$motor_enabled" == true ]]; then
  echo "WARNING: motor endpoint will be started. Use --no-motor or enable_motor:=false for safe testing."
fi

if [[ "$motor_forwarded" != true ]]; then
  launch_args=("enable_motor:=$motor_enabled" "${launch_args[@]}")
fi

start_host_ui() {
  local ui_dir="${UI_DIR:-$HOME/ui}"
  local ui_log="${UI_LOG:-/tmp/jacart_mola_ui.log}"

  if [[ "$start_ui" != true ]]; then
    return
  fi

  if curl -fsS http://localhost:5173/ >/dev/null 2>&1; then
    echo "UI appears to already be running: http://localhost:5173/"
    open_ui_page
    return
  fi
  if curl -fsS http://localhost:5174/ >/dev/null 2>&1; then
    echo "UI appears to already be running: http://localhost:5174/"
    return
  fi

  if [[ ! -f "$ui_dir/package.json" ]]; then
    echo "UI package not found at $ui_dir; skipping host UI dev server."
    return
  fi

  : > "$ui_log"
  (
    cd "$ui_dir"
    npm run dev -- --host 0.0.0.0
  ) > "$ui_log" 2>&1 &
  ui_pid=$!

  echo "Started host UI dev server from $ui_dir as PID $ui_pid"
  echo "UI log: $ui_log"

  local printed_url=false
  for _ in {1..30}; do
    if grep -Eq 'http://(localhost|[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+):[0-9]+/' "$ui_log"; then
      echo "UI URLs reported by Vite:"
      grep -Eo 'http://(localhost|[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+):[0-9]+/' "$ui_log" | sort -u
      printed_url=true
      break
    fi
    sleep 0.5
  done

  if [[ "$printed_url" != true ]]; then
    echo "UI is starting; expected URL is usually http://localhost:5173/ or http://localhost:5174/"
    open_ui_page
  fi
}

cleanup_ui() {
  if [[ -n "${ui_pid:-}" ]]; then
    kill "$ui_pid" 2>/dev/null || true
  fi
}
trap cleanup_ui EXIT

read -r -d '' cleanup_command <<'EOS' || true
cleanup_old_autolaunch_children() {
  local shell_pid old_pids still_running
  shell_pid=$$
  mapfile -t old_pids < <(
    ps -eo pid=,ppid=,args= | awk -v shell_pid="$shell_pid" '
      {
        pid=$1
        ppid=$2
        $1=""
        $2=""
        sub(/^ +/, "")
        cmd=$0

        if (pid == 1 || pid == shell_pid || ppid == shell_pid) next

        if (cmd ~ /ros2 launch cart_launch mola_autonomy\.launch\.py/) print pid
        else if (cmd ~ /mola_bridge_ros2/) print pid
        else if (cmd ~ /rosbridge_websocket/) print pid
        else if (cmd ~ /velodyne_driver_node|velodyne_transform_node/) print pid
        else if (cmd ~ /(^|\/)localization\.rviz/) print pid
        else if (cmd ~ /rviz2/) print pid
        else if (cmd ~ /swri_console/) print pid
        else if (cmd ~ /base_link_to_velodyne_tf|lidar_tf/) print pid
        else if (cmd ~ /pcl_pose_relay/) print pid
        else if (cmd ~ /mola_auto_localization_supervisor/) print pid
        else if (cmd ~ /robot_state_publisher/) print pid
        else if (cmd ~ /global_planner|local_planner|display_global_path|visualize_graph|speed_node/) print pid
        else if (cmd ~ /zed_object_to_obstacle|collision_detector|collision_avoidance_aad_log/) print pid
        else if (cmd ~ /dummy_pointcloud_publisher|pointcloud_to_laserscan_node/) print pid
        else if (cmd ~ /motor_endpoint/) print pid
      }
    ' | sort -n -u
  )

  if ((${#old_pids[@]} == 0)); then
    echo "No old autolaunch child processes matched cleanup patterns."
    return
  fi

  echo "Stopping old autolaunch child processes:"
  ps -o pid,ppid,args -p "$(IFS=,; echo "${old_pids[*]}")" || true
  kill -TERM "${old_pids[@]}" 2>/dev/null || true
  sleep 2

  mapfile -t still_running < <(
    for pid in "${old_pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        echo "$pid"
      fi
    done
  )

  if ((${#still_running[@]} > 0)); then
    echo "Force stopping remaining old child processes: ${still_running[*]}"
    kill -KILL "${still_running[@]}" 2>/dev/null || true
  fi
}
EOS

ros_setup_command='
set -e
source /opt/ros/jazzy/setup.bash
source /opt/ros_ws/install/setup.bash
if [ -f /root/dev_ws/install/setup.bash ]; then
  source /root/dev_ws/install/setup.bash
fi
'

if [[ "$cleanup_old_stack" == true ]]; then
  cleanup_invocation='cleanup_old_autolaunch_children'
else
  cleanup_invocation='echo "Skipping old-stack cleanup."'
fi

launch_command='exec ros2 launch cart_launch mola_autonomy.launch.py "$@"'

start_host_ui

if [[ "$use_docker" == true ]] && ! command -v docker >/dev/null 2>&1; then
  echo "Docker is required for the normal helper path. Use --no-docker only for explicit local ROS testing." >&2
  exit 1
fi

if [[ "$use_docker" == true ]]; then
  docker_dir="${DOCKER_FILES_DIR:-$HOME/docker_files}"
  if [[ -f "$docker_dir/compose.yaml" ]]; then
    container_script="$(mktemp /tmp/jacart_mola_container.XXXXXX.sh)"
    cleanup_container_script() {
      rm -f "$container_script"
    }
    trap 'cleanup_ui; cleanup_container_script' EXIT
    {
      printf '%s\n' "$ros_setup_command"
      printf '%s\n' "$cleanup_command"
      printf '%s\n' "$cleanup_invocation"
      printf '%s\n' "$launch_command"
    } > "$container_script"
    chmod +x "$container_script"
    bash -n "$container_script"

    (
      cd "$docker_dir"
      if [[ -x ./initialize_host.sh ]]; then
        ./initialize_host.sh
      fi
      if [[ "$build_image" == true ]]; then
        echo "Building backend image because --build-image was requested."
        BACKEND_COMMAND="tail -f /dev/null" docker compose up backend -d --build --remove-orphans
      else
        BACKEND_COMMAND="tail -f /dev/null" docker compose up backend -d --no-build --remove-orphans
      fi
      echo "Launching ROS stack inside backend container..."
      docker compose exec -T -e DISPLAY="${DISPLAY:-}" -w /root/dev_ws backend bash -s -- "${launch_args[@]}" < "$container_script"
    )
    exit $?
  fi
  echo "Docker compose file not found at $docker_dir/compose.yaml. Use --no-docker only for explicit local ROS testing." >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
if [[ -f /opt/ros_ws/install/setup.bash ]]; then
  source /opt/ros_ws/install/setup.bash
fi
if [[ -f /root/dev_ws/install/setup.bash ]]; then
  source /root/dev_ws/install/setup.bash
elif [[ -f "$HOME/dev_ws/install/setup.bash" ]]; then
  source "$HOME/dev_ws/install/setup.bash"
fi

if [[ "$cleanup_old_stack" == true ]]; then
  eval "$cleanup_command"
  cleanup_old_autolaunch_children
else
  echo "Skipping old-stack cleanup."
fi

exec ros2 launch cart_launch mola_autonomy.launch.py "${launch_args[@]}"
