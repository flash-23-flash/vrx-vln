#!/usr/bin/env bash
set -e

echo "===== VRX_VLN 环境设置助手 ====="

if [ -z "${ROS_DISTRO}" ]; then
  echo "请先在 shell 中设置 ROS_DISTRO，例如：export ROS_DISTRO=humble"
  echo "或者在命令前设置：ROS_DISTRO=humble ./scripts/setup_env.sh"
  exit 1
fi

echo "检测到 ROS_DISTRO=${ROS_DISTRO}"

echo "下面是推荐的安装命令（请根据你的系统选择执行）："
echo
echo "sudo apt update"
echo "sudo apt install -y build-essential python3-colcon-common-extensions python3-pip git"
echo "sudo apt install -y ros-${ROS_DISTRO}-desktop"
echo "sudo apt install -y ros-${ROS_DISTRO}-ros-gz-bridge ros-${ROS_DISTRO}-ros-gz"

echo
echo "如果你希望脚本自动安装，请在具备 sudo 权限的 shell 中运行："
echo "  sudo bash -c 'apt update && apt install -y build-essential python3-colcon-common-extensions python3-pip git ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-ros-gz-bridge ros-${ROS_DISTRO}-ros-gz'"

echo
echo "安装完成后，请运行："
echo "  python3 -m pip install -U pip"
echo "  cd $(pwd)"
echo "  ./build_vrx_vln.sh"
echo "  source install/setup.bash"
echo "  ./launch_marine_vln_classic.sh"

echo "脚本结束。若需更多自动化，请手动编辑本脚本以添加 apt/pip 安装步骤。"
