#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[DEMO_GATE] ros2 not found in PATH."
  exit 1
fi

echo "[DEMO_GATE] instruction: 从红色和绿色浮标中间穿过去。"
ros2 topic pub --once /vln/instruction_text std_msgs/msg/String "data: '从红色和绿色浮标中间穿过去。'" >/dev/null
sleep 2

echo "[DEMO_GATE] parser output:"
timeout --signal=INT 6s ros2 topic echo --once /vln/task_json || true

echo "[DEMO_GATE] route summary:"
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
"$ROOT_DIR/scripts/show_vlm_route_status.sh" || true
