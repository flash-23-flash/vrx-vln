# marine_vln_vrx

基于 ROS 2 + VRX 的分层式 Marine-VLN 原型，当前已接入本地 Qwen2.5-VL（vLLM OpenAI-compatible）并支持严格 JSON 语义解析与 hybrid 回退。

## 架构

自然语言 -> `instruction_manager`（rules / vlm / hybrid）-> 语义任务 JSON -> `subgoal_planner` -> `local_planner` -> `controller` -> `safety_monitor`

核心原则：
- VLM 只输出语义 JSON，不直接输出推进器控制量
- planner / controller 继续走传统可解释链路
- parser 默认 `hybrid`，失败自动回退 rules

## 默认配置

`config/marine_vln_params.yaml` 默认：

```yaml
instruction_manager:
  ros__parameters:
    parser_mode: hybrid
    vlm_api_base: "http://127.0.0.1:8000/v1"
    vlm_model: "Qwen/Qwen2.5-VL-7B-Instruct-AWQ"
    vlm_confidence_threshold: 0.55
    vlm_max_retries: 1
    vlm_save_debug_image: true
    vlm_log_raw_response: true
```

轻量模型可切到：

```yaml
vlm_model: "Qwen/Qwen2.5-VL-3B-Instruct-AWQ"
```

## 1) 启动本地 Qwen2.5-VL 服务

在工作区根目录执行：

```bash
cd /home/pkuiflab/VRX_VLN
./scripts/start_qwen25_vl_server.sh
```

检查状态：

```bash
./scripts/check_qwen25_vl_server.sh
```

停止服务：

```bash
./scripts/stop_qwen25_vl_server.sh
```

切换 3B：

```bash
./scripts/start_qwen25_vl_server.sh --3b
```

## 2) VRX + Marine-VLN 运行

```bash
cd /home/pkuiflab/VRX_VLN
./build_vrx_vln.sh
./launch_marine_vln.sh
```

默认情况下，`launch_marine_vln.sh` 会自动从 `http://127.0.0.1:8000/v1/models` 获取服务端模型名并对齐 `vlm_model`，避免 `http_404` 模型名不匹配问题。
此外，`instruction_manager` 在遇到 `http_404` 时会自动查询服务端模型并重试一次（运行期兜底）。

### 2.2) 经典比赛风格长距离场景

保留原有近距离简化场景不变，新增经典长距离场景：
- 多段航道浮标（近/中/远）
- 中途静态障碍块
- 远距离码头目标（~150m 级别）

一键启动：

```bash
cd /home/pkuiflab/VRX_VLN
./launch_marine_vln_classic.sh
```

对应 world / 对象配置：
- `vrx_vln_bringup/worlds/vrx_vln_classic_longrange.sdf`
- `marine_vln_vrx/data/scene_objects_classic_longrange.yaml`

该经典场景当前包含：
- 近/中/远三段 gate（`gate_left/right`, `gate_left/right_mid`, `gate_left/right_far`）
- 远距离 dock 目标（与 world 坐标一致）
- 偏航道布局的静态障碍块（降低“卡在门后反复急停/转圈”的概率）

可直接跑长距离任务示例：

```bash
cd /home/pkuiflab/VRX_VLN
./scripts/run_classic_longrange_demo.sh
```

### 2.4) 更高难度动态挑战场景

新增 dynamic challenge：
- S 弯航道 + 4 组 gate（近/中/远/末段）
- 更密集静态障碍
- 动态障碍（Gazebo 模型真实移动 + 语义动态同步）

启动：

```bash
cd /home/pkuiflab/VRX_VLN
./launch_marine_vln_dynamic.sh
```

示例任务（更困难）：

```bash
ros2 topic pub --once /vln/instruction_text std_msgs/msg/String "{data: '沿着航道往前走到码头附近。'}"
```

动态障碍演示脚本：

```bash
cd /home/pkuiflab/VRX_VLN
./scripts/run_dynamic_challenge_demo.sh
```

发指令示例：

```bash
ros2 topic pub --once /vln/instruction_text std_msgs/msg/String "{data: 'Pass between the red and green buoys'}"
```

## 2.1) 如何确认 VLM 真的在工作

1. 先看在线解析输出（`/vln/task_json` 里已包含 `route/fallback_reason/parser_latency_ms`）：

```bash
ros2 topic echo --once /vln/task_json
```

2. 再看运行日志汇总（推荐）：

```bash
cd /home/pkuiflab/VRX_VLN
./scripts/show_vlm_route_status.sh
```

若看到 `route=vlm`，说明该条指令由 VLM 解析成功；若是 `route=rules_fallback`，说明触发了 hybrid 回退。

3. 若在 dynamic challenge 中看不到动态障碍运动，再检查动态驱动状态：

```bash
cd /home/pkuiflab/VRX_VLN
./scripts/show_dynamic_obstacle_status.sh
```

若提示 `dynamic driver log missing`，说明当前不是通过动态场景启动链路，或没有启用 `dynamic_obstacle_driver`。

### 2.3) RViz 前视图 VLM 可视化

系统默认启动 `vlm_visualizer_node`，发布：

- `/vln/debug/front_camera_overlay`

默认 RViz 的 Front Camera 已切到该话题，会叠加显示：
- 顶部状态条：`route / intent / target / fallback`
- 语义对象伪检测框（基于语义地图与相对方位投影）

注意：当前是“可解释调试叠加层”，不是视觉检测网络的真实检测框。

## 3) 一键评测（Qwen + VRX）

```bash
cd /home/pkuiflab/VRX_VLN
./scripts/eval_qwen_vln_vrx.sh
```

评测脚本会：
- 自动检查/启动 vLLM 服务
- 先跑 parser smoke test
- 再在 VRX 中执行 5 条固定指令
- 生成：
  - `/tmp/marine_vln_eval/<ts>/summary.json`
  - `/tmp/marine_vln_eval/<ts>/report.md`

## 4) 导出微调数据（新 schema）

```bash
ros2 run marine_vln_vrx export_vlm_sft_data \
  --log-root /tmp/marine_vln_logs \
  --latest-only \
  --output /tmp/marine_vln_parser_sft.jsonl
```

每条样本至少包含：
- `instruction`
- `ego_state`
- `candidate_objects`
- `local_context_text`
- `image_path`
- `target_json_label`

并附带 `messages` 字段，便于直接做 SFT。

## 5) 严格 JSON + Hybrid 回退

VLM 输出统一 schema（`intent/target_id/.../confidence/use_rule_fallback`），并执行本地校验。

会触发 rules 回退的场景：
- HTTP 失败 / 超时
- 非法 JSON 或 schema 校验失败
- `use_rule_fallback=true`
- `confidence < vlm_confidence_threshold`
- `intent=unknown`
- `target_id` 不在候选对象 ID 集合内

## 日志

默认日志目录：

`/tmp/marine_vln_logs/<run_id>/*.jsonl`

`instruction_manager.jsonl` 会记录：
- parser latency
- model / api_base
- fallback reason
- raw response（可配置）
- semantic_parse 结果
