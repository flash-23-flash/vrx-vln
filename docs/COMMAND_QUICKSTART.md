# VRX_VLN Command Quickstart

Run all commands from the repository root.

## Clean Build

```bash
rm -rf build install log
./build_vrx_vln.sh
```

## Launch The Recommended Scene

```bash
./launch_marine_vln_classic.sh
```

For a headless run:

```bash
./launch_marine_vln_classic.sh headless:=True rviz:=False
```

## Optional Local VLM

```bash
./scripts/start_qwen25_vl_server.sh --3b
./scripts/check_qwen25_vl_server.sh
```

## Run One Demo

```bash
./scripts/run_demo_pass_gate.sh
./scripts/run_demo_to_dock.sh
```

## Inspect Parser / Route Status

```bash
./scripts/show_vlm_route_status.sh
```

## Manual Instruction

```bash
ros2 topic pub --once /vln/instruction_text std_msgs/msg/String \
  "{data: 'Go through the channel and approach the dock'}"
```
