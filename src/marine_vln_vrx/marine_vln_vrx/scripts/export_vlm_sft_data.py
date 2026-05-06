#!/usr/bin/env python3
"""Export instruction_manager logs into VLM SFT jsonl with strict semantic schema labels."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

from marine_vln_vrx.instruction_manager.semantic_schema import (
    default_semantic,
    normalize_semantic_payload,
    validate_semantic_payload,
)


def _iter_task_records(log_file: Path) -> Iterable[Dict[str, Any]]:
    with log_file.open("r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(obj, dict):
                continue
            if obj.get("event") != "task_parsed":
                continue
            payload = obj.get("payload", {})
            if isinstance(payload, dict):
                yield payload


def _find_run_ids(log_root: Path, run_id: Optional[str], latest_only: bool) -> List[Path]:
    if run_id:
        p = log_root / run_id
        return [p] if p.is_dir() else []

    runs = [p for p in log_root.iterdir() if p.is_dir()]
    runs.sort(key=lambda p: p.name)
    if latest_only:
        return runs[-1:] if runs else []
    return runs


def _route_tag(payload: Dict[str, Any]) -> str:
    route = str(payload.get("route", "")).strip()
    if route:
        return route

    parser_name = str(payload.get("parser", "")).strip().lower()
    if parser_name.startswith("rules"):
        return "rules"
    if parser_name.startswith("vlm"):
        return "vlm"
    return parser_name


def _semantic_from_legacy(payload: Dict[str, Any]) -> Dict[str, Any]:
    sequence = payload.get("sequence", [])
    if not isinstance(sequence, list) or not sequence:
        return default_semantic("legacy_empty_sequence")

    step = sequence[0] if isinstance(sequence[0], dict) else {}
    action = str(step.get("action", ""))
    target = step.get("target")

    if action == "go_to":
        out = default_semantic("legacy_go_to")
        out.update(
            {
                "intent": "goto",
                "target_id": target if isinstance(target, str) else None,
                "direction_hint": "toward",
                "confidence": 0.7,
                "use_rule_fallback": False,
            }
        )
        return out

    if action == "pass_between" and isinstance(target, dict):
        out = default_semantic("legacy_pass_between")
        out.update(
            {
                "intent": "go_between",
                "target_id": str(target.get("left")) if target.get("left") else None,
                "secondary_target_id": str(target.get("right")) if target.get("right") else None,
                "direction_hint": "between",
                "target_type": "gate",
                "confidence": 0.7,
                "use_rule_fallback": False,
            }
        )
        return out

    if action == "stop_near":
        out = default_semantic("legacy_stop")
        out.update(
            {
                "intent": "stop",
                "target_id": target if isinstance(target, str) else None,
                "confidence": 0.7,
                "use_rule_fallback": False,
            }
        )
        return out

    return default_semantic("legacy_unmapped")


def _build_user_prompt(instruction: str, ego_state: Dict[str, Any], candidate_objects: List[Dict[str, Any]], local_context_text: str) -> str:
    return (
        "Instruction:\n"
        f"{instruction}\n\n"
        "Ego state:\n"
        f"{json.dumps(ego_state, ensure_ascii=False)}\n\n"
        "Semantic map candidate objects:\n"
        f"{json.dumps(candidate_objects, ensure_ascii=False)}\n\n"
        "Current local navigation context:\n"
        f"{local_context_text}\n\n"
        "Output exactly one JSON object matching the required semantic schema."
    )


def _record_from_payload(run_id: str, payload: Dict[str, Any], route: str) -> Optional[Dict[str, Any]]:
    instruction = str(payload.get("instruction", "")).strip()
    if not instruction:
        return None

    ego_state = payload.get("ego_state") if isinstance(payload.get("ego_state"), dict) else {}
    candidate_objects = payload.get("candidate_objects") if isinstance(payload.get("candidate_objects"), list) else []
    local_context_text = str(payload.get("local_context_text", ""))
    image_path = payload.get("image_path")

    semantic_raw = payload.get("semantic_parse") if isinstance(payload.get("semantic_parse"), dict) else _semantic_from_legacy(payload)
    semantic = normalize_semantic_payload(semantic_raw if isinstance(semantic_raw, dict) else {})
    candidate_ids = []
    for obj in candidate_objects:
        if isinstance(obj, dict):
            cid = obj.get("id")
            if isinstance(cid, str) and cid.strip():
                candidate_ids.append(cid.strip())
    target_json_valid, target_json_errors = validate_semantic_payload(semantic, candidate_ids)

    rec = {
        "instruction": instruction,
        "ego_state": ego_state,
        "candidate_objects": candidate_objects,
        "local_context_text": local_context_text,
        "image_path": image_path,
        "target_json_label": semantic,
        "meta": {
            "run_id": run_id,
            "route": route,
            "target_json_valid": target_json_valid,
            "target_json_errors": target_json_errors,
        },
        "messages": [
            {
                "role": "system",
                "content": "You are a deterministic maritime VLN parser for VRX. Return exactly one JSON object.",
            },
            {
                "role": "user",
                "content": _build_user_prompt(instruction, ego_state, candidate_objects, local_context_text),
            },
            {
                "role": "assistant",
                "content": json.dumps(semantic, ensure_ascii=False),
            },
        ],
    }
    return rec


def main() -> None:
    parser = argparse.ArgumentParser(description="Export parser SFT data from marine_vln logs")
    parser.add_argument("--log-root", default="/tmp/marine_vln_logs", help="Root path of run logs")
    parser.add_argument("--run-id", default=None, help="Specific run_id to export")
    parser.add_argument("--latest-only", action="store_true", help="Only export the newest run")
    parser.add_argument("--output", default="/tmp/marine_vln_parser_sft.jsonl", help="Output JSONL path")
    parser.add_argument(
        "--routes",
        default="vlm,rules,rules_fallback,safe_stop",
        help="Comma-separated task_parsed routes to keep",
    )
    args = parser.parse_args()

    keep_routes = {v.strip() for v in str(args.routes).split(",") if v.strip()}
    log_root = Path(args.log_root).expanduser().resolve()
    runs = _find_run_ids(log_root, args.run_id, bool(args.latest_only))
    if not runs:
        raise SystemExit(f"No run directories found under {log_root}")

    out_path = Path(args.output).expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    count = 0
    with out_path.open("w", encoding="utf-8") as out:
        for run in runs:
            log_file = run / "instruction_manager.jsonl"
            if not log_file.is_file():
                continue
            for payload in _iter_task_records(log_file):
                route = _route_tag(payload)
                if keep_routes and route not in keep_routes:
                    continue

                rec = _record_from_payload(run.name, payload, route)
                if rec is None:
                    continue

                out.write(json.dumps(rec, ensure_ascii=False) + "\n")
                count += 1

    print(f"Exported {count} samples -> {out_path}")


if __name__ == "__main__":
    main()
