#!/usr/bin/env python3
"""Drive dynamic obstacle models in Gazebo via set_pose service."""

from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any, Dict, List, Set

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from marine_vln_vrx.common.log_utils import JsonlLogger


class DynamicObstacleDriverNode(Node):
    """Update Gazebo model poses for dynamic obstacles."""

    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_driver")

        pkg_share = Path(get_package_share_directory("marine_vln_vrx"))
        default_object_file = str(pkg_share / "data" / "scene_objects_dynamic_challenge.yaml")

        self.declare_parameter("dynamic_set_pose_service", "/world/vrx_vln_dynamic_challenge/set_pose")
        self.declare_parameter("auto_discover_set_pose_service", True)
        self.declare_parameter("dynamic_object_file", default_object_file)
        self.declare_parameter("update_rate_hz", 5.0)
        self.declare_parameter("dynamic_types", ["obstacle_dynamic"])
        self.declare_parameter("dynamic_motion_time_scale", 1.0)
        self.declare_parameter("log_root", "/tmp/marine_vln_logs")

        self._service_name = str(self.get_parameter("dynamic_set_pose_service").value).strip()
        self._auto_discover_service = bool(self.get_parameter("auto_discover_set_pose_service").value)
        self._object_file = str(self.get_parameter("dynamic_object_file").value).strip()
        self._update_rate_hz = max(0.5, float(self.get_parameter("update_rate_hz").value))
        types_raw = self.get_parameter("dynamic_types").value
        if not isinstance(types_raw, list):
            types_raw = ["obstacle_dynamic"]
        self._dynamic_types: Set[str] = {str(x).strip() for x in types_raw if str(x).strip()}
        self._time_scale = max(0.01, float(self.get_parameter("dynamic_motion_time_scale").value))
        log_root = str(self.get_parameter("log_root").value)
        world_match = re.search(r"/world/([^/]+)/", self._service_name)
        self._world_name = world_match.group(1) if world_match else ""

        self._logger_file = JsonlLogger(self, "dynamic_obstacle_driver", log_root)
        self._client = self.create_client(SetEntityPose, self._service_name)

        self._objects = self._load_dynamic_objects(self._object_file)
        self._start_ns = self.get_clock().now().nanoseconds
        self._last_ready_log_ns = 0
        self._last_sent_log_ns = 0
        self._last_discover_log_ns = 0
        self._last_fail_log_ns: Dict[str, int] = {}
        self._resolved_entity_name: Dict[str, str] = {}
        self._sent_count = 0
        self._success_count = 0
        self._fail_count = 0
        self._last_stats_log_ns = 0

        self.create_timer(1.0 / self._update_rate_hz, self._on_timer)

        self.get_logger().info(
            f"dynamic_obstacle_driver started: service={self._service_name}, object_file={self._object_file}, "
            f"dynamic_objects={len(self._objects)}, rate={self._update_rate_hz:.1f}Hz, "
            f"world={self._world_name or 'unknown'}, auto_discover_service={self._auto_discover_service}, "
            f"log={self._logger_file.log_path}"
        )
        self._logger_file.write(
            "driver_started",
            {
                "service": self._service_name,
                "world": self._world_name,
                "auto_discover_service": self._auto_discover_service,
                "dynamic_objects": len(self._objects),
            },
        )

    def _load_dynamic_objects(self, object_file: str) -> List[Dict[str, Any]]:
        path = Path(object_file).expanduser().resolve()
        if not path.exists():
            self.get_logger().warn(f"dynamic object file not found: {path}")
            return []
        try:
            raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (yaml.YAMLError, OSError) as exc:
            self.get_logger().warn(f"failed to load dynamic object file {path}: {exc}")
            return []

        objects = raw.get("objects", []) if isinstance(raw, dict) else []
        out: List[Dict[str, Any]] = []
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            obj_type = str(obj.get("type", "")).strip()
            motion = obj.get("motion", {})
            if obj_type not in self._dynamic_types:
                continue
            pos = obj.get("position", [])
            if not isinstance(pos, list) or len(pos) < 2:
                continue
            if not isinstance(motion, dict):
                continue
            pattern = str(motion.get("pattern", "")).strip().lower()
            if not pattern:
                continue
            out.append(
                {
                    "name": str(obj.get("name", "")).strip(),
                    "x0": float(pos[0]),
                    "y0": float(pos[1]),
                    "z": float(pos[2]) if len(pos) > 2 else 0.25,
                    "pattern": pattern,
                    "period_sec": max(0.5, float(motion.get("period_sec", 20.0))),
                    "phase_sec": float(motion.get("phase_sec", 0.0)),
                    "amplitude_x": float(motion.get("amplitude_x", 0.0)),
                    "amplitude_y": float(motion.get("amplitude_y", 0.0)),
                    "radius": float(motion.get("radius", 0.0)),
                }
            )
        return [o for o in out if o["name"]]

    @staticmethod
    def _triangle_wave(u: float) -> float:
        uu = u % 1.0
        return 1.0 - 4.0 * abs(uu - 0.5)

    def _dynamic_xy(self, obj: Dict[str, Any], elapsed_sec: float) -> tuple[float, float]:
        t = (elapsed_sec + float(obj.get("phase_sec", 0.0))) / max(0.5, float(obj.get("period_sec", 20.0)))
        x0 = float(obj.get("x0", 0.0))
        y0 = float(obj.get("y0", 0.0))
        pattern = str(obj.get("pattern", ""))
        ax = float(obj.get("amplitude_x", 0.0))
        ay = float(obj.get("amplitude_y", 0.0))
        radius = float(obj.get("radius", 0.0))

        if pattern in {"linear_pingpong", "pingpong"}:
            s = self._triangle_wave(t)
            return (x0 + ax * s, y0 + ay * s)
        if pattern in {"circle", "orbit"}:
            rr_x = radius if radius > 0.0 else ax
            rr_y = radius if radius > 0.0 else ay
            ang = 2.0 * math.pi * t
            return (x0 + rr_x * math.cos(ang), y0 + rr_y * math.sin(ang))
        if pattern in {"sine", "sin"}:
            ang = 2.0 * math.pi * t
            return (x0 + ax * math.sin(ang), y0 + ay * math.sin(ang))
        return (x0, y0)

    def _entity_candidates(self, base_name: str) -> List[str]:
        resolved = self._resolved_entity_name.get(base_name, "").strip()
        candidates: List[str] = []
        if resolved:
            candidates.append(resolved)
        if base_name not in candidates:
            candidates.append(base_name)
        if self._world_name:
            scoped = f"{self._world_name}::{base_name}"
            if scoped not in candidates:
                candidates.append(scoped)
        return candidates

    @staticmethod
    def _parse_world_name(service_name: str) -> str:
        m = re.search(r"/world/([^/]+)/", service_name)
        return m.group(1) if m else ""

    def _maybe_auto_discover_service(self, now_ns: int) -> None:
        if not self._auto_discover_service:
            return
        if now_ns - self._last_discover_log_ns < int(2e9):
            return
        self._last_discover_log_ns = now_ns

        candidates: List[str] = []
        for srv_name, srv_types in self.get_service_names_and_types():
            if not srv_name.endswith("/set_pose"):
                continue
            if "ros_gz_interfaces/srv/SetEntityPose" not in srv_types:
                continue
            candidates.append(str(srv_name))

        if not candidates:
            self.get_logger().warn("no SetEntityPose service discovered yet.")
            return

        # Keep current service if available.
        if self._service_name in candidates:
            return

        # Prefer a world-matching candidate if we already know world, otherwise pick first.
        chosen = candidates[0]
        if self._world_name:
            for c in candidates:
                if f"/world/{self._world_name}/" in c:
                    chosen = c
                    break

        old = self._service_name
        self._service_name = chosen
        self._world_name = self._parse_world_name(chosen)
        self._client = self.create_client(SetEntityPose, self._service_name)
        self.get_logger().warn(f"switch set_pose service: {old} -> {self._service_name}")
        self._logger_file.write("service_switched", {"from": old, "to": self._service_name})

    def _on_set_pose_done(self, future: Any, base_name: str, entity_name: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        try:
            response = future.result()
        except BaseException as exc:
            self._fail_count += 1
            last_ns = self._last_fail_log_ns.get(base_name, 0)
            if now_ns - last_ns > int(2e9):
                self._last_fail_log_ns[base_name] = now_ns
                self.get_logger().warn(f"set_pose exception for {base_name} ({entity_name}): {exc}")
            return

        ok = bool(getattr(response, "success", False))
        if ok:
            self._success_count += 1
            if self._resolved_entity_name.get(base_name) != entity_name:
                self._resolved_entity_name[base_name] = entity_name
                self.get_logger().info(f"resolved dynamic entity name: {base_name} -> {entity_name}")
            return

        self._fail_count += 1
        last_ns = self._last_fail_log_ns.get(base_name, 0)
        if now_ns - last_ns > int(2e9):
            self._last_fail_log_ns[base_name] = now_ns
            self.get_logger().warn(f"set_pose rejected for {base_name} ({entity_name})")

    def _send_pose(self, base_name: str, entity_name: str, x: float, y: float, z: float) -> None:
        req = SetEntityPose.Request()
        req.entity.name = entity_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.w = 1.0
        future = self._client.call_async(req)
        self._sent_count += 1
        future.add_done_callback(lambda fut, bn=base_name, en=entity_name: self._on_set_pose_done(fut, bn, en))

    def _on_timer(self) -> None:
        if not self._objects:
            return
        now_ns = self.get_clock().now().nanoseconds
        if not self._client.service_is_ready():
            self._maybe_auto_discover_service(now_ns)
            if now_ns - self._last_ready_log_ns > int(2e9):
                self._last_ready_log_ns = now_ns
                self.get_logger().warn(f"set_pose service not ready yet: {self._service_name}")
            return

        elapsed_sec = max(0.0, (now_ns - self._start_ns) * 1e-9 * self._time_scale)
        sent = 0
        for obj in self._objects:
            x, y = self._dynamic_xy(obj, elapsed_sec)
            base_name = str(obj["name"])
            for entity_name in self._entity_candidates(base_name):
                self._send_pose(base_name, entity_name, x, y, float(obj.get("z", 0.25)))
                sent += 1

        if now_ns - self._last_sent_log_ns > int(1e9):
            self._last_sent_log_ns = now_ns
            self._logger_file.write(
                "dynamic_pose_update",
                {"service": self._service_name, "sent": sent, "elapsed_sec": elapsed_sec},
            )
        if now_ns - self._last_stats_log_ns > int(3e9):
            self._last_stats_log_ns = now_ns
            self._logger_file.write(
                "dynamic_pose_stats",
                {
                    "sent_total": self._sent_count,
                    "success_total": self._success_count,
                    "fail_total": self._fail_count,
                    "resolved_names": self._resolved_entity_name,
                },
            )


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacleDriverNode()
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
