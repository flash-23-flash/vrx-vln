#!/usr/bin/env python3
"""Semantic mapper node: keep a local semantic cache."""

from __future__ import annotations

import json
import random
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from marine_vln_vrx.common.json_utils import parse_json_or_empty
from marine_vln_vrx.common.log_utils import JsonlLogger


class SemanticMapperNode(Node):
    """Maintain object cache from scene parser and publish semantic map."""

    def __init__(self) -> None:
        super().__init__("semantic_mapper")

        self.declare_parameter("scene_topic", "/vln/scene_state")
        self.declare_parameter("semantic_map_topic", "/vln/semantic_map")
        self.declare_parameter("cache_ttl_sec", 3.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("benchmark_object_drop_rate", 0.0)
        self.declare_parameter("benchmark_pose_noise_std", 0.0)
        self.declare_parameter("benchmark_semantic_corruption_ratio", 0.0)
        self.declare_parameter("benchmark_seed", 12345)
        self.declare_parameter("log_root", "/tmp/marine_vln_logs")

        scene_topic = str(self.get_parameter("scene_topic").value)
        semantic_map_topic = str(self.get_parameter("semantic_map_topic").value)
        self._cache_ttl_sec = float(self.get_parameter("cache_ttl_sec").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._drop_rate = float(self.get_parameter("benchmark_object_drop_rate").value)
        self._pose_noise_std = float(self.get_parameter("benchmark_pose_noise_std").value)
        self._corruption_ratio = float(self.get_parameter("benchmark_semantic_corruption_ratio").value)
        self._rng = random.Random(int(self.get_parameter("benchmark_seed").value))
        log_root = str(self.get_parameter("log_root").value)

        self._logger_file = JsonlLogger(self, "semantic_mapper", log_root)
        self._semantic_pub = self.create_publisher(String, semantic_map_topic, 10)
        self.create_subscription(String, scene_topic, self._scene_callback, 20)
        self.create_timer(max(0.05, 1.0 / max(0.1, publish_rate_hz)), self._publish_semantic_map)

        self._ego: Dict[str, Any] = {}
        self._objects_cache: Dict[str, Dict[str, Any]] = {}

        self.get_logger().info(
            f"semantic_mapper started: scene_topic={scene_topic}, map_topic={semantic_map_topic}, "
            f"drop_rate={self._drop_rate:.2f}, noise_std={self._pose_noise_std:.2f}, "
            f"corruption_ratio={self._corruption_ratio:.2f}, "
            f"log={self._logger_file.log_path}"
        )

    def _scene_callback(self, msg: String) -> None:
        data = parse_json_or_empty(self, msg.data, "scene_topic")
        if not data:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._ego = data.get("ego", {})
        objects = data.get("objects", [])
        if not isinstance(objects, list):
            return

        for obj in objects:
            if not isinstance(obj, dict):
                continue
            name = str(obj.get("name", "unknown"))
            entry = dict(obj)
            entry["last_seen_sec"] = now_sec
            self._objects_cache[name] = entry

    def _publish_semantic_map(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        stale_keys = [
            name
            for name, obj in self._objects_cache.items()
            if now_sec - float(obj.get("last_seen_sec", now_sec)) > self._cache_ttl_sec
        ]
        for key in stale_keys:
            self._objects_cache.pop(key, None)

        objects_raw: List[Dict[str, Any]] = list(self._objects_cache.values())
        objects: List[Dict[str, Any]] = []
        dropped = 0
        corrupted = 0
        for obj in objects_raw:
            if self._drop_rate > 0.0 and self._rng.random() < self._drop_rate:
                dropped += 1
                continue
            this_obj = dict(obj)
            if self._pose_noise_std > 0.0:
                this_obj["x"] = float(this_obj.get("x", 0.0)) + self._rng.gauss(0.0, self._pose_noise_std)
                this_obj["y"] = float(this_obj.get("y", 0.0)) + self._rng.gauss(0.0, self._pose_noise_std)
            if self._corruption_ratio > 0.0 and self._rng.random() < self._corruption_ratio:
                corrupted += 1
                if self._rng.random() < 0.5:
                    this_obj["type"] = "unknown"
                else:
                    this_obj["aliases"] = []
            objects.append(this_obj)

        nearest_obstacle = None
        for obj in objects:
            obj_type = str(obj.get("type", ""))
            if obj_type in {"waypoint", "marker"}:
                continue
            if nearest_obstacle is None or float(obj.get("distance", 1e9)) < float(nearest_obstacle.get("distance", 1e9)):
                nearest_obstacle = obj

        semantic_map = {
            "frame_id": "map",
            "stamp_ns": self.get_clock().now().nanoseconds,
            "ego": self._ego,
            "objects": objects,
            "nearest_obstacle": nearest_obstacle,
        }

        out = String()
        out.data = json.dumps(semantic_map, ensure_ascii=False)
        self._semantic_pub.publish(out)
        self._logger_file.write(
            "semantic_map_published",
            {
                "objects": len(objects),
                "objects_raw": len(objects_raw),
                "dropped": dropped,
                "corrupted": corrupted,
                "nearest_obstacle": nearest_obstacle.get("name") if isinstance(nearest_obstacle, dict) else None,
            },
        )


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SemanticMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            rclpy.shutdown()
