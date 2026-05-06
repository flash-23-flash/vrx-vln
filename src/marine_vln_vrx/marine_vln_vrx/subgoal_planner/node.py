#!/usr/bin/env python3
"""Subgoal planner: task JSON + semantic map -> subgoal sequence."""

from __future__ import annotations

import json
import math
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import Bool, String

from marine_vln_vrx.common.geometry_utils import distance_2d, quaternion_from_yaw
from marine_vln_vrx.common.json_utils import parse_json_or_empty
from marine_vln_vrx.common.log_utils import JsonlLogger


def _norm_name(text: str) -> str:
    return text.strip().lower().replace(" ", "_").replace("-", "_")


class SubgoalPlannerNode(Node):
    """Plan semantic subgoals from structured instructions."""

    def __init__(self) -> None:
        super().__init__("subgoal_planner")

        self.declare_parameter("task_topic", "/vln/task_json")
        self.declare_parameter("semantic_map_topic", "/vln/semantic_map")
        self.declare_parameter("replan_topic", "/vln/replan_required")
        self.declare_parameter("safety_status_topic", "/vln/safety_status_json")
        self.declare_parameter("subgoals_topic", "/vln/subgoals")
        self.declare_parameter("subgoals_json_topic", "/vln/subgoals_json")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("circle_radius", 8.0)
        self.declare_parameter("circle_points", 8)
        self.declare_parameter("stop_near_distance", 4.0)
        self.declare_parameter("avoid_distance", 10.0)
        self.declare_parameter("pass_between_approach_distance", 6.0)
        self.declare_parameter("pass_between_exit_distance", 2.0)
        self.declare_parameter("pass_side_offset_m", 4.0)
        self.declare_parameter("use_topk_hypothesis", True)
        self.declare_parameter("hypothesis_switch_on_replan", True)
        self.declare_parameter("max_hypothesis_switches_per_task", 3)
        self.declare_parameter("hypothesis_switch_high_risk_threshold", 0.72)
        self.declare_parameter("hypothesis_switch_cooldown_sec", 5.0)
        self.declare_parameter("log_root", "/tmp/marine_vln_logs")

        task_topic = str(self.get_parameter("task_topic").value)
        semantic_map_topic = str(self.get_parameter("semantic_map_topic").value)
        replan_topic = str(self.get_parameter("replan_topic").value)
        safety_status_topic = str(self.get_parameter("safety_status_topic").value)
        subgoals_topic = str(self.get_parameter("subgoals_topic").value)
        subgoals_json_topic = str(self.get_parameter("subgoals_json_topic").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._circle_radius = float(self.get_parameter("circle_radius").value)
        self._circle_points = int(self.get_parameter("circle_points").value)
        self._stop_near_distance = float(self.get_parameter("stop_near_distance").value)
        self._avoid_distance = float(self.get_parameter("avoid_distance").value)
        self._pass_approach_distance = float(self.get_parameter("pass_between_approach_distance").value)
        self._pass_exit_distance = float(self.get_parameter("pass_between_exit_distance").value)
        self._pass_side_offset_m = float(self.get_parameter("pass_side_offset_m").value)
        self._use_topk_hypothesis = bool(self.get_parameter("use_topk_hypothesis").value)
        self._hypothesis_switch_on_replan = bool(self.get_parameter("hypothesis_switch_on_replan").value)
        self._max_hypothesis_switches = max(0, int(self.get_parameter("max_hypothesis_switches_per_task").value))
        self._hypothesis_risk_threshold = float(self.get_parameter("hypothesis_switch_high_risk_threshold").value)
        self._hypothesis_switch_cooldown_sec = float(self.get_parameter("hypothesis_switch_cooldown_sec").value)
        log_root = str(self.get_parameter("log_root").value)

        self._logger_file = JsonlLogger(self, "subgoal_planner", log_root)
        self._subgoal_pub = self.create_publisher(PoseArray, subgoals_topic, 10)
        self._subgoal_json_pub = self.create_publisher(String, subgoals_json_topic, 10)
        self.create_subscription(String, task_topic, self._task_callback, 10)
        self.create_subscription(String, semantic_map_topic, self._semantic_map_callback, 20)
        self.create_subscription(Bool, replan_topic, self._replan_callback, 10)
        self.create_subscription(String, safety_status_topic, self._safety_status_callback, 20)
        self.create_timer(max(0.1, 1.0 / max(0.1, self._publish_rate_hz)), self._plan_and_publish_if_needed)

        self._task: Dict[str, Any] = {}
        self._semantic_map: Dict[str, Any] = {}
        self._dirty = False
        self._force_replan = False
        self._force_replan_reason = ""
        self._safety_status: Dict[str, Any] = {}
        self._task_hypotheses: List[Dict[str, Any]] = []
        self._active_hypothesis_idx = 0
        self._active_hypothesis_id = "h0"
        self._last_switch_reason = "none"
        self._switch_count = 0
        self._last_switch_ns = 0

        self.get_logger().info(
            f"subgoal_planner started: task_topic={task_topic}, map_topic={semantic_map_topic}, "
            f"subgoals_topic={subgoals_topic}, log={self._logger_file.log_path}"
        )

    def _task_callback(self, msg: String) -> None:
        data = parse_json_or_empty(self, msg.data, "task_topic")
        if not data:
            return
        self._task = data
        self._load_hypotheses_from_task(data)
        self._dirty = True

    def _semantic_map_callback(self, msg: String) -> None:
        data = parse_json_or_empty(self, msg.data, "semantic_map_topic")
        if not data:
            return
        self._semantic_map = data

    def _replan_callback(self, msg: Bool) -> None:
        # Latch True until planner consumes it, so short pulses are not missed.
        self._force_replan = self._force_replan or bool(msg.data)
        if bool(msg.data):
            self._force_replan_reason = "replan_required"

    def _safety_status_callback(self, msg: String) -> None:
        data = parse_json_or_empty(self, msg.data, "safety_status_topic")
        if not data:
            return
        self._safety_status = data
        risk = float(data.get("risk_score", 0.0))
        if risk >= self._hypothesis_risk_threshold:
            self._force_replan = True
            self._force_replan_reason = "high_risk"

    def _load_hypotheses_from_task(self, task: Dict[str, Any]) -> None:
        raw = task.get("hypotheses", [])
        self._task_hypotheses = []
        if isinstance(raw, list):
            for h in raw:
                if not isinstance(h, dict):
                    continue
                seq = h.get("sequence", [])
                if not isinstance(seq, list):
                    continue
                self._task_hypotheses.append(h)

        active_id = str(task.get("active_hypothesis_id", "h0"))
        idx = 0
        for i, h in enumerate(self._task_hypotheses):
            if str(h.get("id", "")) == active_id:
                idx = i
                break
        self._active_hypothesis_idx = idx
        if self._task_hypotheses:
            self._active_hypothesis_id = str(self._task_hypotheses[idx].get("id", f"h{idx}"))
        else:
            self._active_hypothesis_id = "h0"
        self._last_switch_reason = "none"
        self._switch_count = 0
        self._last_switch_ns = 0

    def _current_task_with_hypothesis(self) -> Dict[str, Any]:
        task_view = dict(self._task)
        if self._use_topk_hypothesis and self._task_hypotheses:
            idx = max(0, min(self._active_hypothesis_idx, len(self._task_hypotheses) - 1))
            h = self._task_hypotheses[idx]
            task_view["sequence"] = h.get("sequence", task_view.get("sequence", []))
            task_view["semantic_parse"] = h.get("semantic_parse", task_view.get("semantic_parse", {}))
            self._active_hypothesis_id = str(h.get("id", f"h{idx}"))
        return task_view

    def _maybe_switch_hypothesis(self, reason: str) -> bool:
        if not self._use_topk_hypothesis:
            return False
        if not self._hypothesis_switch_on_replan:
            return False
        if len(self._task_hypotheses) <= 1:
            return False
        if self._switch_count >= self._max_hypothesis_switches:
            return False

        now_ns = self.get_clock().now().nanoseconds
        if self._last_switch_ns > 0:
            dt = (now_ns - self._last_switch_ns) * 1e-9
            if dt < self._hypothesis_switch_cooldown_sec:
                return False

        next_idx = self._active_hypothesis_idx + 1
        if next_idx >= len(self._task_hypotheses):
            return False
        self._active_hypothesis_idx = next_idx
        self._active_hypothesis_id = str(self._task_hypotheses[next_idx].get("id", f"h{next_idx}"))
        self._last_switch_reason = reason or "hypothesis_switch"
        self._switch_count += 1
        self._last_switch_ns = now_ns
        self._dirty = True
        self._logger_file.write(
            "hypothesis_switched",
            {
                "active_hypothesis_id": self._active_hypothesis_id,
                "active_index": self._active_hypothesis_idx,
                "switch_count": self._switch_count,
                "reason": self._last_switch_reason,
            },
        )
        self.get_logger().warn(
            f"Hypothesis switched to {self._active_hypothesis_id}, reason={self._last_switch_reason}"
        )
        return True

    def _plan_and_publish_if_needed(self) -> None:
        if not self._task or not self._semantic_map:
            return
        if self._force_replan:
            reason = self._force_replan_reason or "replan_required"
            switched = self._maybe_switch_hypothesis(reason)
            if switched:
                self._force_replan = False
                self._force_replan_reason = ""
        if not self._dirty and not self._force_replan:
            return

        task_view = self._current_task_with_hypothesis()
        subgoals, debug_entries = self._build_subgoals(task_view, self._semantic_map)
        if not subgoals:
            return

        pose_array = PoseArray()
        pose_array.header.frame_id = self._frame_id
        pose_array.header.stamp = self.get_clock().now().to_msg()
        for sx, sy in subgoals:
            p = Pose()
            p.position.x = float(sx)
            p.position.y = float(sy)
            p.position.z = 0.0
            p.orientation.w = 1.0
            pose_array.poses.append(p)

        self._subgoal_pub.publish(pose_array)

        debug_msg = String()
        debug_msg.data = json.dumps(
            {
                "frame_id": self._frame_id,
                "stamp_ns": self.get_clock().now().nanoseconds,
                "subgoals": [{"x": s[0], "y": s[1]} for s in subgoals],
                "debug_entries": debug_entries,
                "active_hypothesis_id": self._active_hypothesis_id,
                "candidate_count": len(self._task_hypotheses) if self._task_hypotheses else 1,
                "switch_reason": self._last_switch_reason,
            },
            ensure_ascii=False,
        )
        self._subgoal_json_pub.publish(debug_msg)

        self._logger_file.write(
            "subgoals_published",
            {
                "count": len(subgoals),
                "debug_entries": debug_entries,
                "active_hypothesis_id": self._active_hypothesis_id,
                "candidate_count": len(self._task_hypotheses) if self._task_hypotheses else 1,
                "switch_reason": self._last_switch_reason,
                "safety_status": self._safety_status,
            },
        )
        self._dirty = False
        self._force_replan = False
        self._force_replan_reason = ""
        self.get_logger().info(f"Published {len(subgoals)} subgoal(s).")

    def _objects(self) -> List[Dict[str, Any]]:
        objs = self._semantic_map.get("objects", [])
        return objs if isinstance(objs, list) else []

    def _ego_xy(self) -> Tuple[float, float]:
        ego = self._semantic_map.get("ego", {})
        return (float(ego.get("x", 0.0)), float(ego.get("y", 0.0)))

    def _resolve_object_xy(self, target: Any) -> Optional[Tuple[float, float]]:
        if isinstance(target, dict):
            return None

        entry = self._resolve_object_entry(target)
        if entry is not None:
            return (float(entry.get("x", 0.0)), float(entry.get("y", 0.0)))
        return None

    def _resolve_object_entry(self, target: Any) -> Optional[Dict[str, Any]]:
        if isinstance(target, dict):
            return None

        target_name = _norm_name(str(target))
        if target_name in {"current_position", "当前位置"}:
            ex, ey = self._ego_xy()
            return {"name": "current_position", "x": ex, "y": ey, "risk_radius": 0.0}

        for obj in self._objects():
            name = _norm_name(str(obj.get("name", "")))
            aliases = [_norm_name(str(a)) for a in obj.get("aliases", []) if isinstance(a, str)]
            if target_name == name or target_name in aliases:
                return obj

        # loose fallback: substring match
        for obj in self._objects():
            name = _norm_name(str(obj.get("name", "")))
            if target_name and (target_name in name or name in target_name):
                return obj
        return None

    def _resolve_gate_midpoint(self, gate_target: str) -> Optional[Tuple[float, float]]:
        target = _norm_name(gate_target)
        if target not in {"gate", "the_gate"}:
            return self._resolve_object_xy(gate_target)
        left = self._resolve_object_xy("gate_left")
        right = self._resolve_object_xy("gate_right")
        if left and right:
            return ((left[0] + right[0]) * 0.5, (left[1] + right[1]) * 0.5)
        return None

    def _build_subgoals(self, task: Dict[str, Any], semantic_map: Dict[str, Any]) -> Tuple[List[Tuple[float, float]], List[Dict[str, Any]]]:
        _ = semantic_map  # semantic map is read through self methods
        sequence = task.get("sequence", [])
        if not isinstance(sequence, list):
            return ([], [])

        ego_x, ego_y = self._ego_xy()
        subgoals: List[Tuple[float, float]] = []
        debug_entries: List[Dict[str, Any]] = []

        for step in sequence:
            if not isinstance(step, dict):
                continue
            action = str(step.get("action", "")).strip()
            target = step.get("target")
            relation = str(step.get("spatial_relation", "")).strip()

            if action == "go_to":
                goal = self._resolve_object_xy(target)
                if goal:
                    if relation in {"left_of", "right_of", "pass_left_of", "pass_right_of"}:
                        target_entry = self._resolve_object_entry(target)
                        if target_entry is not None:
                            tx = float(target_entry.get("x", goal[0]))
                            ty = float(target_entry.get("y", goal[1]))
                            rr = float(target_entry.get("risk_radius", 2.0))
                            dx = tx - ego_x
                            dy = ty - ego_y
                            norm = max(1e-6, math.hypot(dx, dy))
                            # left normal of ego->target direction
                            lx = -dy / norm
                            ly = dx / norm
                            side = 1.0 if relation in {"left_of", "pass_left_of"} else -1.0
                            offset = max(1.0, self._pass_side_offset_m + rr)
                            side_pt = (tx + side * offset * lx, ty + side * offset * ly)
                            subgoals.append(side_pt)
                            debug_entries.append(
                                {
                                    "action": action,
                                    "target": target,
                                    "relation": relation,
                                    "subgoal": side_pt,
                                }
                            )
                            continue

                    subgoals.append(goal)
                    debug_entries.append({"action": action, "target": target, "subgoal": goal})

            elif action == "pass_between":
                midpoint: Optional[Tuple[float, float]] = None
                left: Optional[Tuple[float, float]] = None
                right: Optional[Tuple[float, float]] = None
                if isinstance(target, dict):
                    left = self._resolve_object_xy(target.get("left", ""))
                    right = self._resolve_object_xy(target.get("right", ""))
                    if left and right:
                        midpoint = ((left[0] + right[0]) * 0.5, (left[1] + right[1]) * 0.5)
                else:
                    midpoint = self._resolve_gate_midpoint(str(target))
                    left = self._resolve_object_xy("gate_left")
                    right = self._resolve_object_xy("gate_right")
                if midpoint:
                    if left and right:
                        gate_dx = right[0] - left[0]
                        gate_dy = right[1] - left[1]
                        gate_norm = math.hypot(gate_dx, gate_dy)
                        if gate_norm > 1e-6:
                            # Gate normal (perpendicular to buoy-post axis), sign aligned with ego->midpoint direction.
                            nx = -gate_dy / gate_norm
                            ny = gate_dx / gate_norm
                            to_mid_x = midpoint[0] - ego_x
                            to_mid_y = midpoint[1] - ego_y
                            if nx * to_mid_x + ny * to_mid_y < 0.0:
                                nx, ny = -nx, -ny
                            approach = (
                                midpoint[0] - nx * self._pass_approach_distance,
                                midpoint[1] - ny * self._pass_approach_distance,
                            )
                            exit_pt = (
                                midpoint[0] + nx * self._pass_exit_distance,
                                midpoint[1] + ny * self._pass_exit_distance,
                            )
                            subgoals.extend([approach, midpoint, exit_pt])
                            debug_entries.append(
                                {
                                    "action": action,
                                    "target": target,
                                    "subgoals": [approach, midpoint, exit_pt],
                                }
                            )
                            continue
                    subgoals.append(midpoint)
                    debug_entries.append({"action": action, "target": target, "subgoal": midpoint})

            elif action == "circle":
                center = self._resolve_object_xy(target)
                if center:
                    clockwise = relation != "counterclockwise"
                    for i in range(max(3, self._circle_points)):
                        ratio = float(i) / float(max(1, self._circle_points))
                        theta = -2.0 * math.pi * ratio if clockwise else 2.0 * math.pi * ratio
                        sx = center[0] + self._circle_radius * math.cos(theta)
                        sy = center[1] + self._circle_radius * math.sin(theta)
                        subgoals.append((sx, sy))
                    debug_entries.append(
                        {
                            "action": action,
                            "target": target,
                            "direction": "clockwise" if clockwise else "counterclockwise",
                            "count": max(3, self._circle_points),
                        }
                    )

            elif action == "avoid":
                obs = self._resolve_object_xy(target)
                if obs:
                    dx = ego_x - obs[0]
                    dy = ego_y - obs[1]
                    norm = max(1e-6, math.hypot(dx, dy))
                    sx = obs[0] + self._avoid_distance * dx / norm
                    sy = obs[1] + self._avoid_distance * dy / norm
                    subgoals.append((sx, sy))
                    debug_entries.append({"action": action, "target": target, "subgoal": (sx, sy)})

            elif action == "stop_near":
                target_xy = self._resolve_object_xy(target)
                if target_xy is None:
                    target_xy = (ego_x, ego_y)

                dist = distance_2d(ego_x, ego_y, target_xy[0], target_xy[1])
                if dist < self._stop_near_distance:
                    sx, sy = target_xy[0], target_xy[1]
                else:
                    dx = target_xy[0] - ego_x
                    dy = target_xy[1] - ego_y
                    norm = max(1e-6, math.hypot(dx, dy))
                    sx = target_xy[0] - self._stop_near_distance * dx / norm
                    sy = target_xy[1] - self._stop_near_distance * dy / norm
                subgoals.append((sx, sy))
                debug_entries.append({"action": action, "target": target, "subgoal": (sx, sy)})

        if not subgoals:
            subgoals.append((ego_x, ego_y))
            debug_entries.append({"action": "fallback_stop", "subgoal": (ego_x, ego_y)})
        return (subgoals, debug_entries)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SubgoalPlannerNode()
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
