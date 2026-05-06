"""Simple JSONL logger for experiment traces."""

from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

from rclpy.node import Node


def _resolve_run_id() -> str:
    run_id = os.environ.get("MARINE_VLN_RUN_ID")
    if run_id:
        return run_id
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    os.environ["MARINE_VLN_RUN_ID"] = run_id
    return run_id


class JsonlLogger:
    """Write per-node JSONL logs under log_root/run_id/."""

    def __init__(self, node: Node, node_name: str, log_root: str) -> None:
        self._node = node
        self._node_name = node_name
        run_id = _resolve_run_id()
        log_dir = Path(log_root).expanduser().resolve() / run_id
        log_dir.mkdir(parents=True, exist_ok=True)
        self._log_path = log_dir / f"{node_name}.jsonl"

    @property
    def log_path(self) -> Path:
        return self._log_path

    def write(self, event: str, payload: Dict[str, Any]) -> None:
        record = {
            "stamp": self._node.get_clock().now().nanoseconds,
            "node": self._node_name,
            "event": event,
            "payload": payload,
        }
        try:
            with self._log_path.open("a", encoding="utf-8") as fp:
                fp.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError as exc:
            self._node.get_logger().error(f"Failed to write log {self._log_path}: {exc}")
