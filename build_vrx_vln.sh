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

COMMON_ARGS=(--merge-install --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3)
BRINGUP_PACKAGES=(
  cluster_msg
  lidar_processor
  vrx_gazebo
  vrx_ros
  wamv_description
  wamv_gazebo
  vrx_gz
  vrx_vln_bringup
  marine_vln_vrx
)

if [ "${VRX_VLN_BUILD_ALL:-0}" = "1" ]; then
  echo "[VRX_VLN] Building the full copied workspace with system Python."
  colcon build "${COMMON_ARGS[@]}"
  exit 0
fi

echo "[VRX_VLN] Building the minimal VRX_VLN bringup stack."
colcon build "${COMMON_ARGS[@]}" --packages-select "${BRINGUP_PACKAGES[@]}"
