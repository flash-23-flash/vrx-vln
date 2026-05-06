# VRX_VLN

**Public GitHub repository name:** vrx-vln

## 项目概述

`VRX_VLN` 是一个精简的基于 ROS 2 的海上自主航行（Visual-Location Navigation, VLN）示例工程，面向 VRX（仿真竞赛）场景。该仓库保留了两个核心 demo：

- 漂浮门穿越（pass gate）
- 沿航道前进并靠近码头（to dock）

工程采用 colcon 构建，使用 ROS 2 节点和 Gazebo / Ignition（通过 `ros_gz`）进行仿真。

## 特性

- 最小化的 Marine-VLN 原型（基于 `marine_vln_vrx` 包）。
- 提供用于构建与启动的脚本：`build_vrx_vln.sh`、`launch_marine_vln.sh`、`launch_marine_vln_classic.sh`。
- 演示脚本位于 `scripts/`：`run_demo_pass_gate.sh`、`run_demo_to_dock.sh`。

## 目录结构（重要文件）

- `build_vrx_vln.sh`：构建整个工作区的便捷脚本。
- `launch_marine_vln.sh`、`launch_marine_vln_classic.sh`：不同场景的启动脚本。
- `scripts/`：包含 demo 启动与 VLM 服务脚本。
- `src/marine_vln_vrx/`：主功能包（`package.xml` 指明为 ROS 2、`ament_python` 构建）。

（更多文件请参见仓库顶层）

## 先决条件

- 操作系统：Ubuntu（建议配合对应 ROS 2 发布版的系统）。
- ROS 2：一套已安装且配置好的 ROS 2（例如 `humble`、`galactic`、`foxy` 等），请根据你的系统选择合适的发行版。
- `colcon`：用于构建 ROS 2 工作区。安装方法见下文。
- Gazebo / Ignition 与 `ros_gz`（如果需要仿真）。

注意：`src/marine_vln_vrx/package.xml` 指明了项目依赖 `rclpy`、`launch`、`launch_ros`、`nav_msgs`、`ros_gz_interfaces` 等。

## 一键（或半自动）配置依赖（建议流程）

以下命令为示例，请将 `<ros-distro>` 替换为你选择的 ROS 发行版（如 `humble`）。

1) 安装系统依赖（Debian/Ubuntu）：

```bash
sudo apt update
sudo apt install -y build-essential python3-colcon-common-extensions python3-pip git
sudo apt install -y ros-<ros-distro>-desktop
```

2) 安装 `ros_gz`（如果需要仿真）与其它常见包：

```bash
sudo apt install -y ros-<ros-distro>-ros-gz-bridge ros-<ros-distro>-ros-gz
```

3) Python 依赖（若仓内 Python 包需要）：

```bash
python3 -m pip install -U pip
# 如果需要，安装本地开发依赖：
# pip install -r requirements.txt
```

4) 构建与运行：

```bash
cd /path/to/VRX_VLN
./build_vrx_vln.sh
source install/setup.bash
./launch_marine_vln_classic.sh
# 在另一个终端运行 demo：
./scripts/run_demo_pass_gate.sh
```

## 使用 Git 和 推送到 GitHub（我已为你选择仓库名）

- 推荐仓库名：`vrx-vln`（公开仓库）。
- 我无法在无凭证的情况下直接替你把代码推上 GitHub。以下是两个推荐选项：

1) 使用 GitHub CLI（推荐；本机安全认证一次即可）

```bash
# 在本机上登录一次：
gh auth login
# 创建远程仓库（public）并将本地仓库关联：
gh repo create your-github-username/vrx-vln --public --source=. --push
```

2) 手动在 GitHub 网页上创建 `vrx-vln` 仓库，然后在本地执行：

```bash
git init
git add .
git commit -m "Initial import of VRX_VLN"
git branch -M main
git remote add origin https://github.com/<your-username>/vrx-vln.git
git push -u origin main
```

安全提示：不要通过聊天发送你的访问令牌（PAT）。如果需要我“远程”替你创建仓库并推送，请在安全通道提供一个短期 PAT，我可以给出基于 `curl` 的示例命令，但强烈建议你在本机运行 `gh` 命令。

## 启动与演示

快速启动：

```bash
./build_vrx_vln.sh
source install/setup.bash
./launch_marine_vln_classic.sh
# 新终端：
./scripts/run_demo_pass_gate.sh
```

## 故障排查要点

- 若 `colcon build` 失败，先看 `install` 和 `build` 输出，按提示安装缺少的 ROS 包。
- 若 Gazebo/ros_gz 连接失败，确认 `ros_gz` 版本与本机 Gazebo/Ignition 兼容。

## 许可证

此仓库使用 `Apache-2.0`（与 `src/marine_vln_vrx/package.xml` 一致）。

---

如果你同意我将仓库命名为 `vrx-vln` 并设置为公开，我会：

- 在仓库根目录添加 `README.md`（已完成）、`.gitignore`、`LICENSE` 和 `scripts/setup_env.sh`（见仓内）。
- 给出你需在本机运行的逐步命令来初始化 Git、提交并推送到 GitHub（含 `gh` 示例）。

请选择：

1. 请我现在生成并显示用于在你机器上运行的完整命令（我不会要求凭证）。
2. 你愿意通过 `gh auth login` 在本机登录并让我在这里直接给出 `gh repo create ... --push` 命令执行建议（我不在你机器上运行这些命令）。
3. 你想把仓库创建与推送的事情全部交给我（这需要你通过安全方式临时提供 PAT；不推荐在聊天中发送）。

我推荐选项 1 或 2（安全且简单）。
