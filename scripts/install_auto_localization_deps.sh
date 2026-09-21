#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: install_auto_localization_deps.sh [--build] [--ros-distro DISTRO]

Installs system dependencies needed by the auto_localization branch.

Options:
  --build              Run colcon build --symlink-install after installing deps.
  --ros-distro DISTRO  ROS distro to install packages for. Defaults to $ROS_DISTRO
                       when set, otherwise jazzy.
  -h, --help           Show this help.

Run from anywhere inside the checkout. The script assumes this repo is inside
a ROS workspace at: <workspace>/src/ai-navigation.
USAGE
}

run_build=false
ros_distro="${ROS_DISTRO:-jazzy}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build)
      run_build=true
      shift
      ;;
    --ros-distro)
      if [[ $# -lt 2 ]]; then
        echo "error: --ros-distro requires a value" >&2
        exit 2
      fi
      ros_distro="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "error: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
workspace_root="$(cd "${repo_root}/../.." && pwd)"
ros_setup="/opt/ros/${ros_distro}/setup.bash"

if [[ ! -f "${ros_setup}" ]]; then
  echo "error: ${ros_setup} not found. Install ROS ${ros_distro} before running this script." >&2
  exit 1
fi

if [[ ! -d "${workspace_root}/src" ]]; then
  echo "error: could not infer a ROS workspace root from ${repo_root}" >&2
  exit 1
fi

if [[ "${EUID}" -eq 0 ]]; then
  sudo_cmd=()
else
  sudo_cmd=(sudo)
fi

ros_pkg() {
  printf 'ros-%s-%s' "${ros_distro}" "$1"
}

apt_packages=(
  python3-colcon-common-extensions
  python3-pip
  python3-rosdep
  python3-serial
  python3-yaml
  libflann-dev
  liblz4-dev
  "$(ros_pkg ament-cmake)"
  "$(ros_pkg ament-cmake-gtest)"
  "$(ros_pkg gtsam)"
  "$(ros_pkg mola)"
  "$(ros_pkg mola-common)"
  "$(ros_pkg mola-georeferencing)"
  "$(ros_pkg mola-gtsam-factors)"
  "$(ros_pkg mola-test-datasets)"
  "$(ros_pkg rosbridge-server)"
  "$(ros_pkg rviz2)"
  "$(ros_pkg swri-console)"
  "$(ros_pkg velodyne-driver)"
  "$(ros_pkg velodyne-pointcloud)"
)

echo "Using ROS distro: ${ros_distro}"
echo "Workspace root: ${workspace_root}"

"${sudo_cmd[@]}" apt-get update
"${sudo_cmd[@]}" apt-get install -y "${apt_packages[@]}"

if [[ ! -d /etc/ros/rosdep/sources.list.d ]]; then
  "${sudo_cmd[@]}" rosdep init || true
fi

rosdep update

# shellcheck source=/dev/null
source "${ros_setup}"
rosdep install --from-paths "${workspace_root}/src" --ignore-src -r -y --rosdistro "${ros_distro}"

if [[ -f "${repo_root}/motor_control/resource/requirements.txt" ]]; then
  python3 -m pip install -r "${repo_root}/motor_control/resource/requirements.txt"
fi

if [[ "${run_build}" == true ]]; then
  cd "${workspace_root}"
  colcon build --symlink-install
fi

cat <<EOF

auto_localization dependencies are installed.

For a new shell, run:
  source /opt/ros/${ros_distro}/setup.bash
  source ${workspace_root}/install/setup.bash
EOF
