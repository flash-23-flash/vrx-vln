# instruction_manager

职责：
- 订阅自然语言指令（`/vln/instruction_text`）
- 解析模式支持：
  - `rules`：纯规则模板
  - `vlm`：真实 VLM（OpenAI-compatible `.../v1`，内部走 chat completions）
  - `hybrid`：优先 VLM，失败后回退规则
- 发布任务到 `/vln/task_json`

关键输出 JSON 字段：
- `sequence`: 兼容下游 planner/controller 的动作序列
- `semantic_parse`: 严格 schema 的语义解析结果（intent/target_id/confidence/...）

参数：
- `instruction_topic`
- `task_topic`
- `semantic_map_topic`
- `parser_mode`
- `include_scene_context`
- `use_vlm_image`
- `camera_compressed_topic`
- `vlm_api_base`
- `vlm_model`
- `vlm_api_key_env`
- `vlm_timeout_ms`
- `vlm_max_retries`
- `vlm_confidence_threshold`
- `vlm_temperature`
- `vlm_max_tokens`
- `vlm_media_dir`
- `vlm_enable_guided_json`
- `vlm_save_debug_image`
- `vlm_log_raw_response`
- `vlm_system_prompt_file`
- `vlm_user_prompt_file`
- `vlm_max_candidates`
- `vlm_candidate_max_distance_m`
- `log_root`

快速示例（VLM 混合模式）：

```bash
export OPENAI_API_KEY=your_key
ros2 launch marine_vln_vrx marine_vln_system.launch.py \
  params_file:=/path/to/VRX_VLN/src/marine_vln_vrx/config/marine_vln_params.yaml
```

并在参数里设置：

```yaml
instruction_manager:
  ros__parameters:
    parser_mode: hybrid
    vlm_api_base: "http://127.0.0.1:8000/v1"
    vlm_model: "Qwen/Qwen2.5-VL-7B-Instruct-AWQ"
    # vlm_model: "Qwen/Qwen2.5-VL-3B-Instruct-AWQ"
    vlm_confidence_threshold: 0.55
    vlm_enable_guided_json: true
```

hybrid 回退条件：
- HTTP/timeout/非法响应
- JSON 解析失败或 schema 校验失败
- `use_rule_fallback=true`
- `confidence < vlm_confidence_threshold`
- `intent="unknown"`
- `target_id` 或 `secondary_target_id` 不在候选对象 ID 中
