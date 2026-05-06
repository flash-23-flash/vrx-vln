#!/usr/bin/env bash
set -euo pipefail

LOG_ROOT="${MARINE_VLN_LOG_ROOT:-/tmp/marine_vln_logs}"
RUN_DIR="${1:-}"
COUNT="${2:-10}"

if [[ -z "$RUN_DIR" ]]; then
  RUN_DIR="$(ls -1dt "$LOG_ROOT"/* 2>/dev/null | head -n 1 || true)"
fi

if [[ -z "$RUN_DIR" ]]; then
  echo "[VLM_ROUTE] no run directory found under $LOG_ROOT"
  exit 1
fi

INST_LOG="$RUN_DIR/instruction_manager.jsonl"
if [[ ! -f "$INST_LOG" ]]; then
  echo "[VLM_ROUTE] instruction log not found: $INST_LOG"
  exit 1
fi

python3 - "$INST_LOG" "$COUNT" <<'PY'
import json
import sys
import urllib.request
from pathlib import Path

log_file = Path(sys.argv[1])
count = max(1, int(sys.argv[2]))

rows = []
requested_models = set()
for line in log_file.read_text(encoding="utf-8").splitlines():
    line = line.strip()
    if not line:
        continue
    try:
        rec = json.loads(line)
    except json.JSONDecodeError:
        continue
    if rec.get("event") != "task_parsed":
        continue
    p = rec.get("payload", {}) if isinstance(rec.get("payload"), dict) else {}
    sem = p.get("semantic_parse", {}) if isinstance(p.get("semantic_parse"), dict) else {}
    model_name = p.get("vlm_model")
    if isinstance(model_name, str) and model_name.strip():
        requested_models.add(model_name.strip())
    rows.append(
        {
            "stamp": int(rec.get("stamp", 0)),
            "route": str(p.get("route", "missing")),
            "fallback": str(p.get("fallback_reason", "")),
            "latency": p.get("parser_latency_ms"),
            "intent": str(sem.get("intent", "unknown")),
            "target": sem.get("target_id"),
            "instruction": str(p.get("instruction", "")),
        }
    )

if not rows:
    print(f"[VLM_ROUTE] no task_parsed event in {log_file}")
    raise SystemExit(1)

rows = rows[-count:]

print(f"[VLM_ROUTE] run_dir: {log_file.parent}")
print(f"[VLM_ROUTE] showing last {len(rows)} task_parsed events")
print("-")

route_counts = {}
for i, row in enumerate(rows, start=1):
    route = row["route"]
    route_counts[route] = route_counts.get(route, 0) + 1
    latency = row["latency"]
    latency_txt = "n/a" if latency is None else f"{float(latency):.1f}ms"
    fallback = row["fallback"] if row["fallback"] else "none"
    target = row["target"] if row["target"] is not None else "null"
    print(
        f"[{i}] route={route:<14} latency={latency_txt:<8} intent={row['intent']:<14} "
        f"target={target:<16} fallback={fallback}"
    )
    print(f"    instruction: {row['instruction']}")

print("-")
summary = ", ".join(f"{k}={v}" for k, v in sorted(route_counts.items()))
print(f"[VLM_ROUTE] route summary: {summary}")
if route_counts.get("vlm", 0) > 0:
    print("[VLM_ROUTE] VLM is taking effect (route=vlm observed).")
else:
    print("[VLM_ROUTE] No route=vlm observed in shown entries.")

if requested_models:
    print(f"[VLM_ROUTE] requested model(s): {', '.join(sorted(requested_models))}")

if any("http_404" in row["fallback"] for row in rows):
    print("[VLM_ROUTE] hint: http_404 usually means vlm_model and served model name are inconsistent.")
    base = "http://127.0.0.1:8000/v1"
    try:
        with urllib.request.urlopen(f"{base}/models", timeout=1.5) as resp:
            data = json.loads(resp.read().decode("utf-8", errors="replace"))
        models = data.get("data", []) if isinstance(data, dict) else []
        served = []
        for item in models:
            if isinstance(item, dict):
                mid = item.get("id")
                if isinstance(mid, str) and mid.strip():
                    served.append(mid.strip())
        if served:
            print(f"[VLM_ROUTE] served model(s): {', '.join(served)}")
            print("[VLM_ROUTE] quick fix launch command:")
            print(f"  ./launch_marine_vln.sh vlm_model:='{served[0]}' vlm_api_base:={base}")
        else:
            print("[VLM_ROUTE] check with: ./scripts/check_qwen25_vl_server.sh")
    except Exception:
        print("[VLM_ROUTE] check with: ./scripts/check_qwen25_vl_server.sh")
PY
