# VLM 接入与水面微调

## 1. 目标

把 `instruction_manager` 从纯规则升级到真实 VLM 解析，并用 VRX 水面任务数据持续微调（SFT/LoRA）。

## 2. 本地服务部署（Qwen2.5-VL + vLLM）

```bash
cd /home/pkuiflab/VRX_VLN
./scripts/start_qwen25_vl_server.sh          # 默认 7B AWQ
# ./scripts/start_qwen25_vl_server.sh --3b   # 切 3B AWQ
./scripts/check_qwen25_vl_server.sh
```

默认端点：

`http://127.0.0.1:8000/v1`

## 3. 在线推理接入

在 `config/marine_vln_params.yaml` 设置：

```yaml
instruction_manager:
  ros__parameters:
    parser_mode: hybrid
    vlm_api_base: "http://127.0.0.1:8000/v1"
    vlm_model: "Qwen/Qwen2.5-VL-7B-Instruct-AWQ"
    vlm_api_key_env: OPENAI_API_KEY
    use_vlm_image: true
    vlm_timeout_ms: 8000
    vlm_max_retries: 1
    vlm_confidence_threshold: 0.55
    vlm_enable_guided_json: true
```

说明：
- `hybrid` 推荐用于实船/仿真测试，VLM 不可用时自动回退规则解析。
- `use_vlm_image=true` 时会把当前图像落盘到 `vlm_media_dir`，通过 `file://...` 发给 vLLM。
- 图像只做辅助消歧，语义地图候选对象是权威集合。

## 4. 严格 JSON schema 与回退

VLM 输出必须匹配统一 schema（intent/target_id/.../confidence/use_rule_fallback）。

触发 rules 回退条件：
- HTTP 失败、超时、非法响应
- 非法 JSON 或 schema 校验失败
- `use_rule_fallback=true`
- `confidence < 0.55`
- `intent="unknown"`
- `target_id` 不在候选对象 ID 中

## 5. 数据采集与导出

运行任务后，`instruction_manager` 会产生日志：

```bash
/tmp/marine_vln_logs/<run_id>/instruction_manager.jsonl
```

导出为 SFT 数据：

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

## 6. 水面域增强建议

建议把以下样本加入训练集：
- 波浪/风扰下的含糊指令（如“稳一点靠近码头”）
- 浮标密集/颜色混淆（红绿白黄）
- 同义表达（gate/channel/marker/buoy）中英混用
- 多步骤长指令（先避障再穿门再停靠）

标签保持统一 JSON schema：
- `intent`
- `target_id`
- `secondary_target_id`
- `target_type`
- `direction_hint`
- `distance_hint_m`
- `speed_scale`
- `stop_condition`
- `confidence`
- `use_rule_fallback`
- `brief_reason`

## 7. LoRA 微调（示例流程）

下面是通用命令形态（以你使用的训练框架为准）：

```bash
# 伪示例：请替换为你的训练框架命令
train_lora \
  --model Qwen/Qwen2.5-VL-7B-Instruct-AWQ \
  --train_file /tmp/marine_vln_parser_sft.jsonl \
  --learning_rate 1e-5 \
  --epochs 3 \
  --lora_rank 16 \
  --output_dir /tmp/qwen2_5_vl_marine_lora
```

微调后可通过 vLLM / LMDeploy 部署成 OpenAI-compatible 服务，再回填到 `vlm_api_base` 与 `vlm_model`。

## 8. VRX 评测

```bash
cd /home/pkuiflab/VRX_VLN
./scripts/eval_qwen_vln_vrx.sh
```

输出：
- `/tmp/marine_vln_eval/<ts>/summary.json`
- `/tmp/marine_vln_eval/<ts>/report.md`

## 9. 评估指标

最小建议：
- 指令解析准确率（action/target/sequence）
- JSON 合法率
- 回退率（hybrid 中 VLM 失败占比）
- 任务成功率（穿门、到点、避障）
