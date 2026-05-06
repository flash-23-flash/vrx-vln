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

WORLD_FILE="$SCRIPT_DIR/install/share/vrx_vln_bringup/worlds/vrx_vln_classic_longrange.sdf"
OBJECT_FILE="$SCRIPT_DIR/install/share/marine_vln_vrx/data/scene_objects_classic_longrange.yaml"
BASE_PARAMS_FILE="$SCRIPT_DIR/install/share/marine_vln_vrx/config/marine_vln_params.yaml"

if [ ! -f "$WORLD_FILE" ]; then
  echo "[VRX_VLN] classic world not found: $WORLD_FILE"
  echo "[VRX_VLN] Please run ./build_vrx_vln.sh again."
  exit 1
fi
if [ ! -f "$OBJECT_FILE" ]; then
  echo "[VRX_VLN] classic object file not found: $OBJECT_FILE"
  echo "[VRX_VLN] Please run ./build_vrx_vln.sh again."
  exit 1
fi
if [ ! -f "$BASE_PARAMS_FILE" ]; then
  echo "[VRX_VLN] params file not found: $BASE_PARAMS_FILE"
  echo "[VRX_VLN] Please run ./build_vrx_vln.sh again."
  exit 1
fi

TMP_PARAMS_FILE="$(mktemp /tmp/marine_vln_classic_params_XXXX.yaml)"
/usr/bin/python3 - "$BASE_PARAMS_FILE" "$TMP_PARAMS_FILE" "$OBJECT_FILE" <<'PY'
import sys
from pathlib import Path
import yaml

base_path = Path(sys.argv[1])
out_path = Path(sys.argv[2])
object_file = sys.argv[3]

data = yaml.safe_load(base_path.read_text(encoding="utf-8"))
scene = data.setdefault("scene_parser", {}).setdefault("ros__parameters", {})
scene["object_file"] = object_file

out_path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
PY

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
    echo "[VRX_VLN] classic launch auto-aligned vlm_model from server: $SERVER_MODEL"
  else
    echo "[VRX_VLN] classic launch VLM auto-align skipped (server not reachable at $BASE_URL)."
  fi
fi

if [[ "$HAS_HEADLESS_ARG" -eq 0 ]]; then
  EXTRA_ARGS+=("headless:=False")
fi
if [[ "$HAS_RVIZ_ARG" -eq 0 ]]; then
  EXTRA_ARGS+=("rviz:=True")
fi

echo "[VRX_VLN] launch_marine_vln_classic: default UI mode (headless:=False, rviz:=True)."
echo "[VRX_VLN] pass headless:=True for no Gazebo UI."

exec ros2 launch marine_vln_vrx marine_vln_system.launch.py \
  world:="$WORLD_FILE" \
  params_file:="$TMP_PARAMS_FILE" \
  "${EXTRA_ARGS[@]}" \
  "$@"
