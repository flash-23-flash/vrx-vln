# VRX_VLN

VRX_VLN is a ROS 2 workspace for marine visual-language navigation experiments in the VRX / Gazebo simulation environment. It keeps a compact end-to-end demo stack for a WAM-V style unmanned surface vehicle: parse natural-language navigation commands, build semantic subgoals, plan a local route, and send safe thruster commands in simulation.

The current repository focuses on two reproducible demos:

- pass between red and green buoys
- follow the channel and approach the dock

## Highlights

- ROS 2 Humble workspace built with `colcon`
- Gazebo / VRX-style WAM-V simulation assets and launch files
- Layered Marine-VLN pipeline: instruction parsing, semantic mapping, subgoal planning, local planning, control, and safety monitoring
- Optional OpenAI-compatible VLM endpoint support, with helper scripts for local Qwen2.5-VL / vLLM service management
- Ready-to-run shell scripts for build, launch, demo commands, and status inspection

## Repository Layout

```text
.
|-- build_vrx_vln.sh                 # Build the minimal demo workspace
|-- launch_marine_vln.sh             # Launch the default Marine-VLN stack
|-- launch_marine_vln_classic.sh     # Launch the long-range classic scene
|-- launch_vrx_vln.sh                # Launch a basic VRX_VLN scene
|-- docs/
|   `-- COMMAND_QUICKSTART.md        # Short command reference
|-- scripts/
|   |-- run_demo_pass_gate.sh        # Demo: pass between buoys
|   |-- run_demo_to_dock.sh          # Demo: approach dock
|   |-- start_qwen25_vl_server.sh    # Optional local VLM server helper
|   |-- check_qwen25_vl_server.sh
|   |-- stop_qwen25_vl_server.sh
|   `-- show_vlm_route_status.sh
`-- src/
    |-- marine_vln_vrx/              # Marine-VLN ROS 2 Python package
    |-- vrx_vln_bringup/             # Worlds, launch files, RViz configs
    |-- vrx_gz/, vrx_ros/            # Gazebo / ROS bridge support
    |-- vrx_urdf/                    # WAM-V descriptions and Gazebo assets
    `-- *_msg, *_processor, *_planner
```

## Requirements

Recommended platform:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo / `ros_gz`
- Python 3 and `colcon`
- A GPU-capable Python environment if you want to run Qwen2.5-VL locally

Install common system dependencies:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git python3-pip python3-rosdep \
  python3-colcon-common-extensions python3-vcstool

sudo apt install -y \
  ros-humble-desktop ros-humble-ros-gz ros-humble-ros-gz-bridge \
  ros-humble-rviz2 ros-humble-xacro ros-humble-robot-state-publisher
```

Initialize `rosdep` if needed:

```bash
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## Build

```bash
cd /path/to/VRX_VLN
./build_vrx_vln.sh
```

By default, the script builds the minimal bringup stack needed for the demos. To build every package copied into the workspace:

```bash
VRX_VLN_BUILD_ALL=1 ./build_vrx_vln.sh
```

## Run

Launch the recommended classic long-range scene:

```bash
cd /path/to/VRX_VLN
./launch_marine_vln_classic.sh
```

Launch the default Marine-VLN scene:

```bash
./launch_marine_vln.sh
```

Useful launch arguments:

```bash
./launch_marine_vln_classic.sh headless:=True rviz:=False
./launch_marine_vln.sh vlm_model:=Qwen/Qwen2.5-VL-3B-Instruct-AWQ
```

## Demo Commands

Open a second terminal after the simulator is running:

```bash
cd /path/to/VRX_VLN
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/run_demo_pass_gate.sh
./scripts/run_demo_to_dock.sh
```

Inspect the latest VLM / rule-fallback route decision:

```bash
./scripts/show_vlm_route_status.sh
```

You can also publish a command directly:

```bash
ros2 topic pub --once /vln/instruction_text std_msgs/msg/String \
  "{data: 'Pass between the red and green buoys'}"
```

## Optional VLM Server

The parser can use an OpenAI-compatible vision-language model endpoint. The default API base is:

```text
http://127.0.0.1:8000/v1
```

Start, check, and stop the helper-managed local Qwen2.5-VL service:

```bash
./scripts/start_qwen25_vl_server.sh --3b
./scripts/check_qwen25_vl_server.sh
./scripts/stop_qwen25_vl_server.sh
```

To point the launch scripts at another compatible endpoint:

```bash
export QWEN25_VL_BASE_URL=http://127.0.0.1:8000/v1
./launch_marine_vln_classic.sh
```

If the VLM endpoint is unavailable, the system can fall back to rule-based parsing in hybrid mode.

## Configuration

The main runtime parameters live in:

```text
src/marine_vln_vrx/config/marine_vln_params.yaml
```

Important sections:

- `instruction_manager`: command parsing mode, VLM endpoint, model name, retry and logging policy
- `scene_parser`: sensor topics and scene object files
- `semantic_mapper`: semantic map publishing and benchmark perturbations
- `subgoal_planner`: pass-between, dock-approach, obstacle-avoidance parameters
- `local_planner`, `controller`, `safety_monitor`: path smoothing, thruster control, and safety gates

## Troubleshooting

- If `install/setup.bash` is missing, run `./build_vrx_vln.sh` first.
- If Gazebo cannot find models or worlds, rebuild and launch from the repository root so the scripts can set `GZ_SIM_RESOURCE_PATH`.
- If `ros_gz` packages are missing, install `ros-humble-ros-gz` and run `rosdep install --from-paths src --ignore-src -r -y --rosdistro humble`.
- If VLM calls return model-not-found errors, start the local server first or pass `vlm_model:=...` explicitly.
- The ignored folders `build/`, `install/`, and `log/` are generated locally and are not intended to be committed.

## Related Work

This project follows the layout and publishing style commonly used by ROS / Gazebo open-source workspaces:

- [osrf/vrx](https://github.com/osrf/vrx): Virtual RobotX simulation environment for unmanned surface vehicles
- [gazebosim/ros_gz](https://github.com/gazebosim/ros_gz): ROS and Gazebo integration packages

## License

This repository is released under the Apache-2.0 license. See [LICENSE](LICENSE).
