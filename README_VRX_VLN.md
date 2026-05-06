# VRX_VLN Quick Overview

This file is a short companion to the main [README.md](README.md).

VRX_VLN is a simplified ROS 2 Humble workspace for Marine-VLN experiments in a VRX / Gazebo simulation scene. The retained demo surface is intentionally small:

- pass between a red buoy and a green buoy
- navigate along the channel and approach the dock

Primary entry points:

```bash
./build_vrx_vln.sh
./launch_marine_vln_classic.sh
./scripts/run_demo_pass_gate.sh
./scripts/run_demo_to_dock.sh
```

Optional local VLM helpers:

```bash
./scripts/start_qwen25_vl_server.sh --3b
./scripts/check_qwen25_vl_server.sh
./scripts/stop_qwen25_vl_server.sh
```

Generated `build/`, `install/`, and `log/` directories are intentionally ignored by Git.
