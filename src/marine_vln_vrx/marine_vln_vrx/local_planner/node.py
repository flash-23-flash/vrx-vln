#!/usr/bin/env python3
"""Local planner: semantic subgoals -> collision-aware local path."""

from __future__ import annotations

import json
import math
from typing import Any, Dict, List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from marine_vln_vrx.common.geometry_utils import clamp, quaternion_from_yaw
from marine_vln_vrx.common.json_utils import parse_json_or_empty
from marine_vln_vrx.common.log_utils import JsonlLogger


class LocalPlannerNode(Node):
    """Generate a simple local path through subgoals with obstacle repulsion."""

    def __init__(self) -> None:
        super().__init__("local_planner")

        self.declare_parameter("subgoals_topic", "/vln/subgoals")
        self.declare_parameter("semantic_map_topic", "/vln/semantic_map")
        self.declare_parameter("odom_topic", "/model/wamv/odometry")
        self.declare_parameter("odom_topic_fallback", "/wamv/sensors/position/ground_truth_odometry")
        self.declare_parameter("path_topic", "/vln/local_path")
        self.declare_parameter("planning_rate_hz", 5.0)
        self.declare_parameter("interpolation_step_m", 1.5)
        self.declare_parameter("obstacle_avoid_radius", 4.0)
        self.declare_parameter("obstacle_push_gain", 0.2)
        self.declare_parameter("obstacle_extra_margin", 1.0)
        self.declare_parameter("obstacle_repel_iterations", 2)
        self.declare_parameter("obstacle_push_exponent", 1.8)
        self.declare_parameter("obstacle_max_push_step", 0.55)
        self.declare_parameter("dynamic_obstacle_radius_scale", 1.15)
        self.declare_parameter("dynamic_obstacle_push_scale", 1.35)
        self.declare_parameter("dynamic_push_cap_scale", 1.2)
        self.declare_parameter("dynamic_activation_ratio", 0.82)
        self.declare_parameter("ignore_object_types", ["waypoint", "marker", "gate_post", "buoy"])
        self.declare_parameter("subgoal_reached_distance", 4.0)
        self.declare_parameter("drop_behind_subgoals", True)
        self.declare_parameter("behind_distance_threshold", 6.0)
        self.declare_parameter("max_path_points", 500)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("log_root", "/tmp/marine_vln_logs")

        subgoals_topic = str(self.get_parameter("subgoals_topic").value)
        semantic_map_topic = str(self.get_parameter("semantic_map_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        odom_topic_fallback = str(self.get_parameter("odom_topic_fallback").value)
        path_topic = str(self.get_parameter("path_topic").value)
        planning_rate_hz = float(self.get_parameter("planning_rate_hz").value)
        self._step = float(self.get_parameter("interpolation_step_m").value)
        self._avoid_radius = float(self.get_parameter("obstacle_avoid_radius").value)
        self._push_gain = float(self.get_parameter("obstacle_push_gain").value)
        self._extra_margin = float(self.get_parameter("obstacle_extra_margin").value)
        self._repel_iterations = max(1, int(self.get_parameter("obstacle_repel_iterations").value))
        self._push_exponent = float(self.get_parameter("obstacle_push_exponent").value)
        self._max_push_step = float(self.get_parameter("obstacle_max_push_step").value)
        self._dyn_radius_scale = float(self.get_parameter("dynamic_obstacle_radius_scale").value)
        self._dyn_push_scale = float(self.get_parameter("dynamic_obstacle_push_scale").value)
        self._dyn_push_cap_scale = float(self.get_parameter("dynamic_push_cap_scale").value)
        self._dyn_activation_ratio = float(self.get_parameter("dynamic_activation_ratio").value)
        ignore_raw = self.get_parameter("ignore_object_types").value
        if not isinstance(ignore_raw, list):
            ignore_raw = []
        self._ignore_types = {str(v).strip() for v in ignore_raw}
        self._subgoal_reached_distance = float(self.get_parameter("subgoal_reached_distance").value)
        self._drop_behind_subgoals = bool(self.get_parameter("drop_behind_subgoals").value)
        self._behind_distance_threshold = float(self.get_parameter("behind_distance_threshold").value)
        self._max_path_points = int(self.get_parameter("max_path_points").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        log_root = str(self.get_parameter("log_root").value)

        self._logger_file = JsonlLogger(self, "local_planner", log_root)
        self._path_pub = self.create_publisher(Path, path_topic, 10)
        self.create_subscription(PoseArray, subgoals_topic, self._subgoals_callback, 10)
        self.create_subscription(String, semantic_map_topic, self._semantic_map_callback, 20)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, qos_profile_sensor_data)
        if odom_topic_fallback and odom_topic_fallback != odom_topic:
            self.create_subscription(Odometry, odom_topic_fallback, self._odom_callback, qos_profile_sensor_data)
        self.create_timer(max(0.05, 1.0 / max(0.1, planning_rate_hz)), self._plan_and_publish)

        self._subgoals: List[Tuple[float, float]] = []
        self._semantic_map: Dict[str, Any] = {}
        self._odom: Odometry | None = None

        self.get_logger().info(
            f"local_planner started: subgoals_topic={subgoals_topic}, path_topic={path_topic}, "
            f"ignore_types={sorted(self._ignore_types)}, log={self._logger_file.log_path}"
        )

    def _subgoals_callback(self, msg: PoseArray) -> None:
        self._subgoals = [(float(p.position.x), float(p.position.y)) for p in msg.poses]

    def _semantic_map_callback(self, msg: String) -> None:
        data = parse_json_or_empty(self, msg.data, "semantic_map_topic")
        if data:
            self._semantic_map = data

    def _odom_callback(self, msg: Odometry) -> None:
        self._odom = msg

    def _obstacles(self) -> List[Tuple[float, float, float, bool]]:
        objects = self._semantic_map.get("objects", [])
        if not isinstance(objects, list):
            return []
        result: List[Tuple[float, float, float, bool]] = []
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            obj_type = str(obj.get("type", "unknown"))
            if obj_type in self._ignore_types:
                continue
            ox = float(obj.get("x", 0.0))
            oy = float(obj.get("y", 0.0))
            radius = float(obj.get("risk_radius", 2.0))
            is_dynamic = bool(obj.get("dynamic", False)) or obj_type.endswith("_dynamic")
            result.append((ox, oy, radius, is_dynamic))
        return result

    def _segment(self, start: Tuple[float, float], end: Tuple[float, float]) -> List[Tuple[float, float]]:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return [end]
        n = max(1, int(math.ceil(dist / max(0.1, self._step))))
        return [(start[0] + dx * i / n, start[1] + dy * i / n) for i in range(1, n + 1)]

    def _repel_obstacles(
        self,
        point: Tuple[float, float],
        obstacles: Sequence[Tuple[float, float, float, bool]],
    ) -> Tuple[float, float]:
        px, py = point
        for _ in range(self._repel_iterations):
            moved = False
            for ox, oy, radius, is_dynamic in obstacles:
                rr = radius * (self._dyn_radius_scale if is_dynamic else 1.0)
                threshold = self._avoid_radius + rr + self._extra_margin
                dx = px - ox
                dy = py - oy
                dist = math.hypot(dx, dy)
                if dist < 1e-6:
                    dx = 1.0
                    dy = 0.0
                    dist = 1.0
                if dist >= threshold:
                    continue
                # Dynamic obstacles should not trigger hard detours too early.
                if is_dynamic and dist > threshold * self._dyn_activation_ratio:
                    continue
                gain = self._push_gain * (self._dyn_push_scale if is_dynamic else 1.0)
                penetration = max(0.0, threshold - dist)
                norm = penetration / max(1e-3, threshold)
                push = gain * penetration * math.pow(norm, max(0.0, self._push_exponent))
                push_cap = self._max_push_step * (self._dyn_push_cap_scale if is_dynamic else 1.0)
                push = clamp(push, 0.0, push_cap)
                if push <= 1e-4:
                    continue
                px += (dx / dist) * push
                py += (dy / dist) * push
                moved = True
            if not moved:
                break
        return (px, py)

    def _prune_subgoals(self, sx: float, sy: float, subgoals: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
        active = list(subgoals)
        if not active:
            return active

        while len(active) > 1:
            gx, gy = active[0]
            first_dist = math.hypot(gx - sx, gy - sy)
            nx, ny = active[1]
            second_dist = math.hypot(nx - sx, ny - sy)
            if first_dist <= self._subgoal_reached_distance:
                active.pop(0)
                continue

            # If the next subgoal is already much closer than the current one,
            # treat current as passed to avoid oscillation near gate waypoints.
            if second_dist + 0.25 * self._subgoal_reached_distance < first_dist:
                active.pop(0)
                continue

            if self._drop_behind_subgoals:
                fx, fy = active[-1]
                vx = gx - sx
                vy = gy - sy
                refx = fx - sx
                refy = fy - sy
                ref_norm = math.hypot(refx, refy)
                if ref_norm > 1e-6 and first_dist > self._behind_distance_threshold:
                    if (vx * refx + vy * refy) < 0.0:
                        active.pop(0)
                        continue
            break
        return active

    def _plan_and_publish(self) -> None:
        if self._odom is None or not self._subgoals:
            return

        sx = float(self._odom.pose.pose.position.x)
        sy = float(self._odom.pose.pose.position.y)
        active_subgoals = self._prune_subgoals(sx, sy, self._subgoals)
        if not active_subgoals:
            return
        anchors: List[Tuple[float, float]] = [(sx, sy)] + active_subgoals
        obstacles = self._obstacles()

        path_points: List[Tuple[float, float]] = [(sx, sy)]
        for i in range(len(anchors) - 1):
            seg = self._segment(anchors[i], anchors[i + 1])
            for p in seg:
                path_points.append(self._repel_obstacles(p, obstacles))
                if len(path_points) >= self._max_path_points:
                    break
            if len(path_points) >= self._max_path_points:
                break

        if len(path_points) < 2:
            return

        msg = Path()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        for i, (px, py) in enumerate(path_points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            pose.pose.position.z = 0.0
            next_i = min(i + 1, len(path_points) - 1)
            yaw = math.atan2(path_points[next_i][1] - py, path_points[next_i][0] - px)
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            msg.poses.append(pose)

        self._path_pub.publish(msg)
        self._logger_file.write(
            "path_published",
            {
                "path_points": len(path_points),
                "subgoals_raw": len(self._subgoals),
                "subgoals_active": len(active_subgoals),
                "obstacles": len(obstacles),
            },
        )


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LocalPlannerNode()
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
