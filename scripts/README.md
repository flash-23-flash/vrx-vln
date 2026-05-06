# scripts 目录说明（精简版）

本目录仅保留基础运行必需脚本。

## 1) VLM 服务管理

- `start_qwen25_vl_server.sh`
- `check_qwen25_vl_server.sh`
- `stop_qwen25_vl_server.sh`

兼容别名：
- `check_qwen_vl_server.sh` -> `check_qwen25_vl_server.sh`
- `stop_qwen25_server.sh` -> `stop_qwen25_vl_server.sh`

## 2) Demo（已拆分）

- `run_demo_pass_gate.sh`
  - 仅执行“从红绿浮标中间穿过去”
- `run_demo_to_dock.sh`
  - 仅执行“沿航道前进到码头附近”

## 3) 状态检查

- `show_vlm_route_status.sh`
  - 查看最近任务的 route/fallback/intent/target

## 建议流程

```bash
cd /home/pkuiflab/VRX_VLN
./build_vrx_vln.sh
./scripts/start_qwen25_vl_server.sh --3b
./launch_marine_vln_classic.sh
```

另开终端执行单个 demo：

```bash
./scripts/run_demo_pass_gate.sh
# 或
./scripts/run_demo_to_dock.sh
```
