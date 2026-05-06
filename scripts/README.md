# Scripts

These scripts are the public command surface for the simplified VRX_VLN demos.

## VLM Service Helpers

- `start_qwen25_vl_server.sh`: start a local OpenAI-compatible Qwen2.5-VL service
- `check_qwen25_vl_server.sh`: check whether the local endpoint is reachable
- `stop_qwen25_vl_server.sh`: stop the helper-managed service

Compatibility aliases:

- `check_qwen_vl_server.sh`
- `stop_qwen25_server.sh`

## Demo Commands

- `run_demo_pass_gate.sh`: publish the pass-between-buoys instruction
- `run_demo_to_dock.sh`: publish the dock-approach instruction

## Status

- `show_vlm_route_status.sh`: summarize the latest parser route, fallback reason, intent, and target

## Typical Flow

```bash
cd /path/to/VRX_VLN
./build_vrx_vln.sh
./scripts/start_qwen25_vl_server.sh --3b
./launch_marine_vln_classic.sh
```

Open a second terminal:

```bash
cd /path/to/VRX_VLN
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_demo_pass_gate.sh
```
