#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[DEMO_DOCK] ros2 not found in PATH."
  exit 1
fi

echo "[DEMO_DOCK] instruction: 沿着航道往前走到码头附近。"
ros2 topic pub --once /vln/instruction_text std_msgs/msg/String "data: '沿着航道往前走到码头附近。'" >/dev/null
sleep 2

echo "[DEMO_DOCK] parser output:"
timeout --signal=INT 6s ros2 topic echo --once /vln/task_json || true

echo "[DEMO_DOCK] route summary:"
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
"$ROOT_DIR/scripts/show_vlm_route_status.sh" || true
