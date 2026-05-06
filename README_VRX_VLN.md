# VRX_VLN（精简版）

当前仓库已清理为“只保留两个基础 demo”的版本。

## 保留目标

- Demo 1：移动过门（从红绿浮标中间通过）
- Demo 2：沿航道前进并到达码头附近

## 当前保留入口

- 构建：`./build_vrx_vln.sh`
- 启动（基础）：`./launch_marine_vln.sh`
- 启动（经典场景）：`./launch_marine_vln_classic.sh`
- Demo（过门）：`./scripts/run_demo_pass_gate.sh`
- Demo（到码头）：`./scripts/run_demo_to_dock.sh`

## VLM 服务脚本

- `./scripts/start_qwen25_vl_server.sh`
- `./scripts/check_qwen25_vl_server.sh`
- `./scripts/stop_qwen25_vl_server.sh`

## 重建状态

已执行 `build/ install/ log/` 彻底清空并重建。

## 快速开始

```bash
cd /home/pkuiflab/VRX_VLN
./launch_marine_vln_classic.sh
```

另开终端运行单个 demo：

```bash
./scripts/run_demo_pass_gate.sh
# 或
./scripts/run_demo_to_dock.sh
```

## 结果目录

- `results/demos/`（仅保留 demo 结果目录）
