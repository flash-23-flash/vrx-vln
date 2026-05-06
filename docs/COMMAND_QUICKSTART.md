# VRX_VLN 快速命令清单（精简版）

## 1. 彻底清空并重建

```bash
cd /home/pkuiflab/VRX_VLN
rm -rf build install log
./build_vrx_vln.sh
```

## 2. 启动本地 VLM（可选）

```bash
./scripts/start_qwen25_vl_server.sh --3b
./scripts/check_qwen25_vl_server.sh
```

## 3. 启动场景（推荐经典场景）

```bash
./launch_marine_vln_classic.sh
```

说明：默认 `headless:=False` 且 `rviz:=True`。

## 4. 运行单个 demo（已拆分）

过门 demo：

```bash
./scripts/run_demo_pass_gate.sh
```

到码头 demo：

```bash
./scripts/run_demo_to_dock.sh
```

## 5. 查看最近解析链路

```bash
./scripts/show_vlm_route_status.sh
```
