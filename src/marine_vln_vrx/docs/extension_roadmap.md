# 后续扩展路线

## 当前状态（2026-03-27）

- 已支持真实 VLM 解析接入（OpenAI-compatible API）：
  - `instruction_manager.parser_mode = rules | vlm | hybrid`
  - `hybrid` 模式支持 VLM 失败自动回退规则解析
- 已提供微调数据工具链：
  - 运行日志导出：`ros2 run marine_vln_vrx export_vlm_sft_data`
  - 种子数据：`data/vlm_sft_seed.jsonl`

## 扩展到 VLM/LLM 高层决策

1. 把 `instruction_manager` 的规则解析替换为 LLM 函数调用输出（保持 JSON schema 不变）
2. 增加 `task_validator`，对 LLM 输出做约束检查和安全过滤
3. 把 `subgoal_planner` 改为行为树/任务图执行器（可回退到规则策略）

## 扩展到真实感知

1. 在 `scene_parser` 中接入检测器输出（目标框/实例分割/跟踪）
2. 在 `semantic_mapper` 融合多帧观测并维护置信度与时间衰减
3. 在 `safety_monitor` 引入 LiDAR TTC（time-to-collision）与动态重规划触发

## 扩展到更强控制

1. `local_planner` 从线性插值升级到 DWA/TEB/MPC
2. `controller` 从纯差动映射升级到 LOS + 船舶动力学补偿
3. 引入轨迹跟踪评估指标（横向误差、航向误差、控制平滑度）
