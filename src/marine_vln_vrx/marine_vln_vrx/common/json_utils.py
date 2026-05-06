"""JSON helpers for robust parsing in ROS topics."""

from __future__ import annotations

import json
from typing import Any, Dict

from rclpy.node import Node


def parse_json_or_empty(node: Node, raw_text: str, topic_name: str) -> Dict[str, Any]:
    """Parse JSON text and return empty dict if invalid."""
    try:
        data = json.loads(raw_text)
        if isinstance(data, dict):
            return data
        node.get_logger().warn(f"{topic_name}: JSON payload is not an object.")
    except json.JSONDecodeError as exc:
        node.get_logger().warn(f"{topic_name}: invalid JSON ({exc})")
    return {}
