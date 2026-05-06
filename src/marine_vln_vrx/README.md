# marine_vln_vrx

`marine_vln_vrx` is the ROS 2 Python package that implements the Marine-VLN pipeline used by the top-level VRX_VLN demos.

Pipeline:

```text
instruction text
  -> instruction_manager
  -> semantic task JSON
  -> subgoal_planner
  -> local_planner
  -> controller
  -> safety_monitor
  -> WAM-V thruster topics
```

Key design choices:

- the VLM, when enabled, outputs semantic JSON rather than direct control commands
- `parser_mode: hybrid` allows rule-based fallback when the VLM endpoint is unavailable or low confidence
- planning and safety remain explicit ROS nodes so behavior can be inspected with topics and logs

Main configuration:

```text
config/marine_vln_params.yaml
```

Default VLM endpoint:

```text
http://127.0.0.1:8000/v1
```

Useful commands from the repository root:

```bash
./scripts/start_qwen25_vl_server.sh --3b
./launch_marine_vln_classic.sh
./scripts/run_demo_pass_gate.sh
./scripts/show_vlm_route_status.sh
```

See the root [README.md](../../README.md) for full installation, build, and launch instructions.
