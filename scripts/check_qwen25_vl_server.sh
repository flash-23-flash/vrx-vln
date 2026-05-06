#!/usr/bin/env bash
set -euo pipefail

PORT="${QWEN25_VL_PORT:-8000}"
PID_FILE="${QWEN25_VL_PID_FILE:-/tmp/qwen25_vl_server.pid}"
BASE="http://127.0.0.1:${PORT}/v1"

PID=""
if [[ -f "$PID_FILE" ]]; then
  PID="$(cat "$PID_FILE" 2>/dev/null || true)"
fi

if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
  echo "[QWEN_VL] pid file process alive: $PID"
else
  if [[ -n "$PID" ]]; then
    rm -f "$PID_FILE"
    echo "[QWEN_VL] stale pid removed: $PID"
  else
    echo "[QWEN_VL] pid file process not running"
  fi
fi

if curl -fsS "${BASE}/models" >/tmp/qwen_vl_models.json 2>/dev/null; then
  echo "[QWEN_VL] API reachable: ${BASE}"
  echo "[QWEN_VL] models:"
  cat /tmp/qwen_vl_models.json
  exit 0
fi

echo "[QWEN_VL] API not reachable: ${BASE}"
exit 1
