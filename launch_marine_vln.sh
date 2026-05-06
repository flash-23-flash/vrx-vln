#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

unset CONDA_DEFAULT_ENV CONDA_EXE CONDA_PREFIX PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset GZ_SIM_RESOURCE_PATH GZ_GUI_PLUGIN_PATH GZ_SIM_SYSTEM_PLUGIN_PATH
unset IGN_GAZEBO_RESOURCE_PATH IGN_GUI_PLUGIN_PATH
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:$PATH"

source /opt/ros/humble/setup.bash

if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
  echo "[VRX_VLN] install/setup.bash not found. Run ./build_vrx_vln.sh first."
  exit 1
fi

source "$SCRIPT_DIR/install/setup.bash"

append_resource_path_unique() {
  local candidate="$1"
  [ -d "$candidate" ] || return 0
  if [[ -z "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
    GZ_SIM_RESOURCE_PATH="$candidate"
    return 0
  fi
  case ":$GZ_SIM_RESOURCE_PATH:" in
    *":$candidate:"*) ;;
    *) GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:$candidate" ;;
  esac
}

GZ_SIM_RESOURCE_PATH=""
append_resource_path_unique "$SCRIPT_DIR/src/vrx_gz/models"
append_resource_path_unique "$SCRIPT_DIR/install/share/vrx_gz/models"
append_resource_path_unique "$SCRIPT_DIR/src/vrx_urdf"
append_resource_path_unique "$SCRIPT_DIR/install/share"
append_resource_path_unique "$SCRIPT_DIR/src/vrx_vln_bringup/worlds"
append_resource_path_unique "$SCRIPT_DIR/install/share/vrx_vln_bringup/worlds"
export GZ_SIM_RESOURCE_PATH
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"

AUTO_ALIGN_VLM_MODEL="${VRX_VLN_AUTO_ALIGN_VLM_MODEL:-1}"
BASE_URL="${QWEN25_VL_BASE_URL:-http://127.0.0.1:8000/v1}"
HAS_MODEL_ARG=0
HAS_BASE_ARG=0
HAS_HEADLESS_ARG=0
HAS_RVIZ_ARG=0

for arg in "$@"; do
  case "$arg" in
    vlm_model:=*) HAS_MODEL_ARG=1 ;;
    vlm_api_base:=*) HAS_BASE_ARG=1 ;;
    headless:=*) HAS_HEADLESS_ARG=1 ;;
    rviz:=*) HAS_RVIZ_ARG=1 ;;
  esac
done

EXTRA_ARGS=()
if [[ "$AUTO_ALIGN_VLM_MODEL" == "1" && "$HAS_MODEL_ARG" -eq 0 ]]; then
  SERVER_MODEL="$(
    /usr/bin/python3 - "$BASE_URL" <<'PY'
import json
import sys
import urllib.request

base = sys.argv[1].strip().rstrip("/")
if base.endswith("/chat/completions"):
    base = base[: -len("/chat/completions")]
if not base.endswith("/v1"):
    base = f"{base}/v1"
url = f"{base}/models"

try:
    with urllib.request.urlopen(url, timeout=1.5) as resp:
        raw = resp.read().decode("utf-8", errors="replace")
    data = json.loads(raw)
    models = data.get("data", []) if isinstance(data, dict) else []
    if isinstance(models, list) and models:
        first = models[0]
        if isinstance(first, dict):
            model_id = first.get("id", "")
            if isinstance(model_id, str):
                print(model_id.strip())
except Exception:
    pass
PY
  )"
  if [[ -n "$SERVER_MODEL" ]]; then
    EXTRA_ARGS+=("vlm_model:=$SERVER_MODEL")
    if [[ "$HAS_BASE_ARG" -eq 0 ]]; then
      EXTRA_ARGS+=("vlm_api_base:=$BASE_URL")
    fi
    echo "[VRX_VLN] auto-aligned vlm_model from server: $SERVER_MODEL"
  else
    echo "[VRX_VLN] VLM auto-align skipped (server not reachable at $BASE_URL)."
  fi
fi

if [[ "$HAS_HEADLESS_ARG" -eq 0 ]]; then
  EXTRA_ARGS+=("headless:=False")
fi
if [[ "$HAS_RVIZ_ARG" -eq 0 ]]; then
  EXTRA_ARGS+=("rviz:=True")
fi

echo "[VRX_VLN] launch_marine_vln: default UI mode (headless:=False, rviz:=True)."

exec ros2 launch marine_vln_vrx marine_vln_system.launch.py "${EXTRA_ARGS[@]}" "$@"
