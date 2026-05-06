#!/usr/bin/env bash
set -e

echo "===== VRX_VLN environment helper ====="

ROS_DISTRO="${ROS_DISTRO:-humble}"
echo "Using ROS_DISTRO=${ROS_DISTRO}"

cat <<EOF

Recommended Ubuntu / ROS dependency commands:

sudo apt update
sudo apt install -y \\
  build-essential cmake git python3-pip python3-rosdep \\
  python3-colcon-common-extensions python3-vcstool

sudo apt install -y \\
  ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-ros-gz-bridge \\
  ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-robot-state-publisher

Then initialize rosdep if this machine has not done so before:

sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

Build and launch:

source /opt/ros/${ROS_DISTRO}/setup.bash
./build_vrx_vln.sh
source install/setup.bash
./launch_marine_vln_classic.sh

EOF

echo "This helper prints commands only; run the commands you need for your machine."
