#!/usr/bin/env bash
set -euo pipefail

PID_FILE="${QWEN25_VL_PID_FILE:-/tmp/qwen25_vl_server.pid}"
PORT="${QWEN25_VL_PORT:-8000}"
have_rg=0
if command -v rg >/dev/null 2>&1; then
  have_rg=1
fi

if [[ -f "$PID_FILE" ]]; then
  PID="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
    echo "[QWEN_VL] stopping pid=$PID"
    kill -TERM "$PID" || true
    for _ in $(seq 1 20); do
      if ! kill -0 "$PID" 2>/dev/null; then
        break
      fi
      sleep 0.5
    done
    if kill -0 "$PID" 2>/dev/null; then
      echo "[QWEN_VL] force kill pid=$PID"
      kill -KILL "$PID" || true
    fi
  fi
  rm -f "$PID_FILE"
fi

if (( have_rg == 1 )); then
  EXTRA_PIDS="$(ps -ef | rg "vllm.entrypoints.openai.api_server" | rg ":${PORT}\\b|--port ${PORT}" | awk '{print $2}' || true)"
else
  EXTRA_PIDS="$(ps -ef | grep -E "vllm.entrypoints.openai.api_server" | grep -E "(:${PORT}\\b|--port ${PORT})" | grep -v grep | awk '{print $2}' || true)"
fi
if [[ -n "$EXTRA_PIDS" ]]; then
  echo "[QWEN_VL] stopping residual pids: $EXTRA_PIDS"
  # shellcheck disable=SC2086
  kill -TERM $EXTRA_PIDS || true
fi

echo "[QWEN_VL] stopped."
