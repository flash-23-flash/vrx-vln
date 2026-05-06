#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

unset CONDA_DEFAULT_ENV CONDA_EXE CONDA_PREFIX PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset GZ_SIM_RESOURCE_PATH GZ_GUI_PLUGIN_PATH GZ_SIM_SYSTEM_PLUGIN_PATH
unset IGN_GAZEBO_RESOURCE_PATH IGN_GUI_PLUGIN_PATH
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:$PATH"

source /opt/ros/humble/setup.bash

if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
  echo "[VRX_VLN] install/setup.bash not found. Run ./build_vrx_vln.sh first."
  exit 1
fi

source "$SCRIPT_DIR/install/setup.bash"

append_resource_path_unique() {
  local candidate="$1"
  [ -d "$candidate" ] || return 0
  if [[ -z "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
    GZ_SIM_RESOURCE_PATH="$candidate"
    return 0
  fi
  case ":$GZ_SIM_RESOURCE_PATH:" in
    *":$candidate:"*) ;;
    *) GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:$candidate" ;;
  esac
}

GZ_SIM_RESOURCE_PATH=""
append_resource_path_unique "$SCRIPT_DIR/src/vrx_gz/models"
append_resource_path_unique "$SCRIPT_DIR/install/share/vrx_gz/models"
append_resource_path_unique "$SCRIPT_DIR/src/vrx_urdf"
append_resource_path_unique "$SCRIPT_DIR/install/share"
append_resource_path_unique "$SCRIPT_DIR/src/vrx_vln_bringup/worlds"
append_resource_path_unique "$SCRIPT_DIR/install/share/vrx_vln_bringup/worlds"
export GZ_SIM_RESOURCE_PATH
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"

HAS_HEADLESS_ARG=0
HAS_RVIZ_ARG=0
for arg in "$@"; do
  case "$arg" in
    headless:=*) HAS_HEADLESS_ARG=1 ;;
    rviz:=*) HAS_RVIZ_ARG=1 ;;
  esac
done

EXTRA_ARGS=()
if [[ "$HAS_HEADLESS_ARG" -eq 0 ]]; then
  EXTRA_ARGS+=("headless:=False")
fi
if [[ "$HAS_RVIZ_ARG" -eq 0 ]]; then
  EXTRA_ARGS+=("rviz:=True")
fi

WORLD_FILE="$SCRIPT_DIR/install/share/vrx_vln_bringup/worlds/vrx_vln_buoy_test.sdf"
if [ -f "$WORLD_FILE" ]; then
  echo "[VRX_VLN] launch_vrx_vln: default UI mode (headless:=False, rviz:=True)."
  echo "[VRX_VLN] pass headless:=True to disable Gazebo UI."
  exec ros2 launch vrx_vln_bringup empty_usv.launch.py world:="$WORLD_FILE" "${EXTRA_ARGS[@]}" "$@"
fi

echo "[VRX_VLN] buoy test world not found, fallback to default empty world."
echo "[VRX_VLN] launch_vrx_vln: default UI mode (headless:=False, rviz:=True)."
echo "[VRX_VLN] pass headless:=True to disable Gazebo UI."
exec ros2 launch vrx_vln_bringup empty_usv.launch.py "${EXTRA_ARGS[@]}" "$@"
