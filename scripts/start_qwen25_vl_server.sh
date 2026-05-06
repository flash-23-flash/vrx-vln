#!/usr/bin/env bash
set -euo pipefail

HOST="${QWEN25_VL_HOST:-0.0.0.0}"
PORT="${QWEN25_VL_PORT:-8000}"
MODEL_7B="Qwen/Qwen2.5-VL-7B-Instruct-AWQ"
MODEL_3B="Qwen/Qwen2.5-VL-3B-Instruct-AWQ"
MODEL="${QWEN25_VL_MODEL:-$MODEL_7B}"
SERVED_MODEL_NAME="${QWEN25_VL_SERVED_MODEL_NAME:-}"
PREFERRED_PY="${QWEN25_VL_PYTHON:-}"
# ROS/conda injected PYTHONPATH may force old distro packages ahead of user site.
# Default behavior is to launch vLLM with PYTHONPATH/PYTHONHOME unset.
KEEP_PY_ENV="${QWEN25_VL_KEEP_PY_ENV:-0}"
STARTUP_TIMEOUT_S="${QWEN25_VL_STARTUP_TIMEOUT_S:-600}"
LIMIT_MM_PER_PROMPT="${QWEN25_VL_LIMIT_MM_PER_PROMPT:-{\"image\":1}}"
HF_ENDPOINT_VAL="${QWEN25_VL_HF_ENDPOINT:-}"
HF_HOME_VAL="${QWEN25_VL_HF_HOME:-$HOME/.cache/huggingface}"
# Compatibility-first defaults for mixed torch/vLLM environments.
USE_V1_ENGINE="${QWEN25_VL_USE_V1:-}"
ENFORCE_EAGER="${QWEN25_VL_ENFORCE_EAGER:-1}"
COMPILATION_CONFIG="${QWEN25_VL_COMPILATION_CONFIG:-}"
GPU_MEMORY_UTILIZATION="${QWEN25_VL_GPU_MEMORY_UTILIZATION:-0.88}"
DTYPE="${QWEN25_VL_DTYPE:-auto}"
TENSOR_PARALLEL_SIZE="${QWEN25_VL_TENSOR_PARALLEL_SIZE:-1}"
FORCE_CUDA_PLATFORM="${QWEN25_VL_FORCE_CUDA_PLATFORM:-1}"
EAGER_FLAG=""
if [[ "$ENFORCE_EAGER" == "1" ]]; then
  EAGER_FLAG="--enforce-eager"
fi
COMPILATION_ARGS=()
if [[ -n "$COMPILATION_CONFIG" ]]; then
  COMPILATION_ARGS=(--compilation-config "$COMPILATION_CONFIG")
fi
V1_ENV_ARGS=()
if [[ -n "$USE_V1_ENGINE" ]]; then
  V1_ENV_ARGS=("VLLM_USE_V1=$USE_V1_ENGINE")
fi
FORCED_CUDA_BOOTSTRAP='import runpy, vllm.platforms as p; from vllm.platforms.cuda import CudaPlatform; p.current_platform = CudaPlatform(); runpy.run_module("vllm.entrypoints.openai.api_server", run_name="__main__")'

if [[ "${1:-}" == "--3b" ]]; then
  MODEL="$MODEL_3B"
elif [[ "${1:-}" == "--7b" ]]; then
  MODEL="$MODEL_7B"
elif [[ -n "${1:-}" ]]; then
  MODEL="$1"
fi

if [[ -z "$SERVED_MODEL_NAME" ]]; then
  if [[ -d "$MODEL" ]]; then
    # keep API model name stable when loading from local path
    SERVED_MODEL_NAME="$MODEL_7B"
  else
    SERVED_MODEL_NAME="$MODEL"
  fi
fi

MEDIA_DIR="${QWEN25_VL_MEDIA_DIR:-/tmp/marine_vln_vlm_media}"
LOG_DIR="${QWEN25_VL_LOG_DIR:-/tmp/marine_vln_vlm_logs}"
PID_FILE="${QWEN25_VL_PID_FILE:-/tmp/qwen25_vl_server.pid}"
LOG_FILE="$LOG_DIR/qwen25_vl_server_$(date +%Y%m%d_%H%M%S).log"

mkdir -p "$MEDIA_DIR" "$LOG_DIR"

SERVER_ARGS=(
  --host "$HOST"
  --port "$PORT"
  --model "$MODEL"
  --served-model-name "$SERVED_MODEL_NAME"
  --trust-remote-code
  --allowed-local-media-path "$MEDIA_DIR"
  --limit-mm-per-prompt "$LIMIT_MM_PER_PROMPT"
  --tensor-parallel-size "$TENSOR_PARALLEL_SIZE"
  --gpu-memory-utilization "$GPU_MEMORY_UTILIZATION"
  --dtype "$DTYPE"
  --max-model-len 4096
)
if [[ -n "$EAGER_FLAG" ]]; then
  SERVER_ARGS+=("$EAGER_FLAG")
fi
if [[ ${#COMPILATION_ARGS[@]} -gt 0 ]]; then
  SERVER_ARGS+=("${COMPILATION_ARGS[@]}")
fi

have_rg=0
if command -v rg >/dev/null 2>&1; then
  have_rg=1
fi

if [[ -f "$PID_FILE" ]]; then
  OLD_PID="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$OLD_PID" ]] && kill -0 "$OLD_PID" 2>/dev/null; then
    echo "[QWEN_VL] server already running: pid=$OLD_PID"
    echo "[QWEN_VL] endpoint: http://127.0.0.1:${PORT}/v1"
    exit 0
  fi
  rm -f "$PID_FILE"
fi

port_in_use=0
SS_OUT="$(ss -ltn 2>/dev/null || true)"
if [[ -n "$SS_OUT" ]]; then
  if (( have_rg == 1 )); then
    printf "%s\n" "$SS_OUT" | rg -q ":${PORT}\\b" && port_in_use=1 || true
  else
    printf "%s\n" "$SS_OUT" | grep -Eq "[:.]${PORT}([[:space:]]|$)" && port_in_use=1 || true
  fi
fi
if (( port_in_use == 1 )); then
  echo "[QWEN_VL] port ${PORT} already in use."
  echo "[QWEN_VL] run scripts/stop_qwen25_vl_server.sh or choose another port (QWEN25_VL_PORT)."
  exit 1
fi

print_network_hint() {
  local _log_file="$1"
  if grep -q "huggingface.co" "$_log_file" 2>/dev/null && \
     grep -Eq "Network is unreachable|couldn't connect|LocalEntryNotFoundError" "$_log_file" 2>/dev/null; then
    cat <<'EOT'
[QWEN_VL] Detected Hugging Face network/cache failure.
[QWEN_VL] Options:
  1) Download model to local path first, then start with local dir:
     ./scripts/start_qwen25_vl_server.sh /absolute/path/to/Qwen2.5-VL-7B-Instruct-AWQ
  2) Use mirror endpoint when downloading:
     export HF_ENDPOINT=https://hf-mirror.com
  3) If you already downloaded model, ensure --model points to that local directory.
EOT
  fi
}

print_compile_hint() {
  local _log_file="$1"
  if grep -q "FakeTensorMode" "$_log_file" 2>/dev/null; then
    cat <<'EOT'
[QWEN_VL] Detected torch/vLLM compile path compatibility issue.
[QWEN_VL] Try conservative flags (already default in this script):
  export QWEN25_VL_ENFORCE_EAGER=1
  export QWEN25_VL_COMPILATION_CONFIG='{"mode":0}'
[QWEN_VL] You can also try toggling engine generation:
  export QWEN25_VL_USE_V1=0   # legacy path
  # or
  export QWEN25_VL_USE_V1=1   # v1 path
[QWEN_VL] If still failing, pin torch+vllm versions known compatible on your machine.
EOT
  fi
}

declare -a py_candidates=()
if [[ -n "$PREFERRED_PY" ]]; then
  py_candidates+=("$PREFERRED_PY")
fi
if command -v python3 >/dev/null 2>&1; then
  py_candidates+=("$(command -v python3)")
fi
if [[ -x "/usr/bin/python3" ]]; then
  py_candidates+=("/usr/bin/python3")
fi

PY_BIN=""
for py in "${py_candidates[@]}"; do
  if [[ -x "$py" ]] && env -u PYTHONPATH -u PYTHONHOME "$py" -c "import vllm" >/dev/null 2>&1; then
    PY_BIN="$py"
    break
  fi
done

if [[ -z "$PY_BIN" ]]; then
  DEFAULT_PY="$(command -v python3 || true)"
  if [[ -n "$DEFAULT_PY" ]]; then
    DEFAULT_VER="$("$DEFAULT_PY" -V 2>/dev/null || true)"
  else
    DEFAULT_VER="not found"
  fi
  SYS_VER="$(/usr/bin/python3 -V 2>/dev/null || true)"
  echo "[QWEN_VL] vLLM is not available in current python env."
  echo "[QWEN_VL] python3=$DEFAULT_PY ($DEFAULT_VER)"
  echo "[QWEN_VL] /usr/bin/python3 ($SYS_VER)"
  echo "[QWEN_VL] install example:"
  echo "  /usr/bin/python3 -m pip install -U pip"
  echo "  /usr/bin/python3 -m pip install vllm"
  echo "[QWEN_VL] or specify another interpreter:"
  echo "  export QWEN25_VL_PYTHON=/path/to/python3"
  exit 1
fi

if ! env -u PYTHONPATH -u PYTHONHOME "$PY_BIN" - <<'PY' >/dev/null 2>&1
import PIL
from PIL import Image
assert hasattr(Image, "Resampling"), f"Pillow too old: {getattr(PIL, '__version__', 'unknown')}"
PY
then
  PILLOW_VER="$(env -u PYTHONPATH -u PYTHONHOME "$PY_BIN" - <<'PY'
import PIL
print(getattr(PIL, "__version__", "unknown"))
PY
)"
  echo "[QWEN_VL] Pillow is too old in selected python: version=${PILLOW_VER}"
  echo "[QWEN_VL] fix example:"
  echo "  $PY_BIN -m pip install -U \"pillow>=10.0.0\""
  exit 1
fi

if ! env -u PYTHONPATH -u PYTHONHOME "$PY_BIN" - <<'PY' >/dev/null 2>&1
import numpy
from scipy.optimize import linear_sum_assignment
_ = (numpy.__version__, linear_sum_assignment)
PY
then
  echo "[QWEN_VL] scipy/numpy runtime mismatch detected."
  echo "[QWEN_VL] fix example:"
  echo "  $PY_BIN -m pip install -U \"scipy>=1.13.0\""
  echo "[QWEN_VL] then retry start script."
  exit 1
fi

set +e
if [[ "$KEEP_PY_ENV" == "1" ]]; then
  if [[ "$FORCE_CUDA_PLATFORM" == "1" ]]; then
    nohup env HF_HOME="$HF_HOME_VAL" HF_ENDPOINT="$HF_ENDPOINT_VAL" "${V1_ENV_ARGS[@]}" "$PY_BIN" -c "$FORCED_CUDA_BOOTSTRAP" \
      "${SERVER_ARGS[@]}" \
      >"$LOG_FILE" 2>&1 &
  else
    nohup env HF_HOME="$HF_HOME_VAL" HF_ENDPOINT="$HF_ENDPOINT_VAL" "${V1_ENV_ARGS[@]}" "$PY_BIN" -m vllm.entrypoints.openai.api_server \
      "${SERVER_ARGS[@]}" \
      >"$LOG_FILE" 2>&1 &
  fi
else
  if [[ "$FORCE_CUDA_PLATFORM" == "1" ]]; then
    nohup env -u PYTHONPATH -u PYTHONHOME HF_HOME="$HF_HOME_VAL" HF_ENDPOINT="$HF_ENDPOINT_VAL" "${V1_ENV_ARGS[@]}" "$PY_BIN" -c "$FORCED_CUDA_BOOTSTRAP" \
      "${SERVER_ARGS[@]}" \
      >"$LOG_FILE" 2>&1 &
  else
    nohup env -u PYTHONPATH -u PYTHONHOME HF_HOME="$HF_HOME_VAL" HF_ENDPOINT="$HF_ENDPOINT_VAL" "${V1_ENV_ARGS[@]}" "$PY_BIN" -m vllm.entrypoints.openai.api_server \
      "${SERVER_ARGS[@]}" \
      >"$LOG_FILE" 2>&1 &
  fi
fi
NEW_PID=$!
set -e

echo "$NEW_PID" > "$PID_FILE"

echo "[QWEN_VL] launching... pid=$NEW_PID model=$MODEL"
echo "[QWEN_VL] served_model_name=$SERVED_MODEL_NAME"
echo "[QWEN_VL] python=$PY_BIN"
echo "[QWEN_VL] hf_home=$HF_HOME_VAL"
if [[ -n "$USE_V1_ENGINE" ]]; then
  echo "[QWEN_VL] use_v1=$USE_V1_ENGINE"
else
  echo "[QWEN_VL] use_v1=default"
fi
echo "[QWEN_VL] force_cuda_platform=$FORCE_CUDA_PLATFORM"
echo "[QWEN_VL] eager=$ENFORCE_EAGER tp=$TENSOR_PARALLEL_SIZE dtype=$DTYPE gpu_mem_util=$GPU_MEMORY_UTILIZATION"
if [[ -n "$COMPILATION_CONFIG" ]]; then
  echo "[QWEN_VL] compilation_config=$COMPILATION_CONFIG"
else
  echo "[QWEN_VL] compilation_config=default"
fi
if [[ -n "$HF_ENDPOINT_VAL" ]]; then
  echo "[QWEN_VL] hf_endpoint=$HF_ENDPOINT_VAL"
fi
if [[ "$KEEP_PY_ENV" == "1" ]]; then
  echo "[QWEN_VL] python env mode=inherit"
else
  echo "[QWEN_VL] python env mode=isolated (unset PYTHONPATH/PYTHONHOME)"
fi
echo "[QWEN_VL] log_file=$LOG_FILE"

for i in $(seq 1 "$STARTUP_TIMEOUT_S"); do
  if ! kill -0 "$NEW_PID" 2>/dev/null; then
    rm -f "$PID_FILE"
    echo "[QWEN_VL] server exited early. check logs: $LOG_FILE"
    echo "[QWEN_VL] recent log tail:"
    tail -n 80 "$LOG_FILE" || true
    print_network_hint "$LOG_FILE"
    print_compile_hint "$LOG_FILE"
    exit 1
  fi

  if curl -fsS "http://127.0.0.1:${PORT}/v1/models" >/dev/null 2>&1; then
    echo "[QWEN_VL] server is ready: http://127.0.0.1:${PORT}/v1"
    break
  fi
  if (( i % 15 == 0 )); then
    echo "[QWEN_VL] waiting for server... ${i}s/${STARTUP_TIMEOUT_S}s"
  fi
  sleep 1
done

if ! curl -fsS "http://127.0.0.1:${PORT}/v1/models" >/dev/null 2>&1; then
  if ! kill -0 "$NEW_PID" 2>/dev/null; then
    rm -f "$PID_FILE"
    echo "[QWEN_VL] server exited early. check logs: $LOG_FILE"
  else
    echo "[QWEN_VL] startup timeout. check logs: $LOG_FILE"
    kill -TERM "$NEW_PID" >/dev/null 2>&1 || true
    rm -f "$PID_FILE"
  fi
  echo "[QWEN_VL] recent log tail:"
  tail -n 80 "$LOG_FILE" || true
  print_network_hint "$LOG_FILE"
  print_compile_hint "$LOG_FILE"
  exit 1
fi

cat <<EOM

[QWEN_VL] quick curl test:
curl -s http://127.0.0.1:${PORT}/v1/models | jq

[QWEN_VL] multimodal curl test (replace file path):
curl -s http://127.0.0.1:${PORT}/v1/chat/completions \\
  -H "Content-Type: application/json" \\
  -d '{
    "model": "'"${SERVED_MODEL_NAME}"'",
    "messages": [
      {"role":"system","content":"Output JSON only."},
      {"role":"user","content":[
        {"type":"text","text":"Describe the scene as JSON."},
        {"type":"image_url","image_url":{"url":"file:///tmp/marine_vln_vlm_media/frame.jpg"}}
      ]}
    ],
    "temperature": 0.0
  }'

[QWEN_VL] OpenAI SDK test (python):
python3 - <<'PY'
from openai import OpenAI
client = OpenAI(base_url='http://127.0.0.1:${PORT}/v1', api_key='EMPTY')
resp = client.chat.completions.create(
    model='${SERVED_MODEL_NAME}',
    messages=[{'role':'user','content':'Return JSON: {"ok": true}'}],
    temperature=0.0,
)
print(resp.choices[0].message.content)
PY

[QWEN_VL] IMPORTANT: keep vrx parser model consistent with served model name.
Example:
./launch_marine_vln.sh parser_mode:=hybrid vlm_api_base:=http://127.0.0.1:${PORT}/v1 vlm_model:='${SERVED_MODEL_NAME}'

EOM
