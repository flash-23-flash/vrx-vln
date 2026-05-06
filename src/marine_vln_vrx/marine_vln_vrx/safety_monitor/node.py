#!/usr/bin/env python3
"""Safety monitor: risk check and command override before thruster output."""

from __future__ import annotations

import math
import json
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

from marine_vln_vrx.common.geometry_utils import clamp, wrap_to_pi
from marine_vln_vrx.common.json_utils import parse_json_or_empty
from marine_vln_vrx.common.log_utils import JsonlLogger


class SafetyMonitorNode(Node):
    """Scale/stop raw control outputs based on nearest obstacle distance."""

    def __init__(self) -> None:
        super().__init__("safety_monitor")

        self.declare_parameter("semantic_map_topic", "/vln/semantic_map")
        self.declare_parameter("raw_cmd_vel_topic", "/vln/cmd_vel_raw")
        self.declare_parameter("safe_cmd_vel_topic", "/vln/cmd_vel_safe")
        self.declare_parameter("raw_left_thrust_topic", "/vln/raw/left_thrust")
        self.declare_parameter("raw_right_thrust_topic", "/vln/raw/right_thrust")
        self.declare_parameter("raw_left_pos_topic", "/vln/raw/left_pos")
        self.declare_parameter("raw_right_pos_topic", "/vln/raw/right_pos")
        self.declare_parameter("final_left_thrust_topic", "/wamv/thrusters/left/thrust")
        self.declare_parameter("final_right_thrust_topic", "/wamv/thrusters/right/thrust")
        self.declare_parameter("final_left_pos_topic", "/wamv/thrusters/left/pos")
        self.declare_parameter("final_right_pos_topic", "/wamv/thrusters/right/pos")
        self.declare_parameter("replan_topic", "/vln/replan_required")
        self.declare_parameter("safety_status_topic", "/vln/safety_status_json")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("enable_emergency_stop", True)
        self.declare_parameter("enable_replan_trigger", True)
        self.declare_parameter("caution_distance", 8.0)
        self.declare_parameter("stop_distance", 2.8)
        self.declare_parameter("resume_distance", 3.6)
        self.declare_parameter("min_scale", 0.2)
        self.declare_parameter("dynamic_min_scale", 0.22)
        self.declare_parameter("dynamic_stop_distance_scale", 0.45)
        self.declare_parameter("dynamic_hard_stop_distance", 1.2)
        self.declare_parameter("preserve_turning_authority", True)
        self.declare_parameter("turning_authority_min_scale", 0.55)
        self.declare_parameter("turning_authority_exponent", 0.5)
        self.declare_parameter("dynamic_turn_delta_ratio_limit", 0.82)
        self.declare_parameter("max_abs_thrust", 320.0)
        self.declare_parameter("dynamic_emergency_stop", False)
        self.declare_parameter("use_encounter_aware_safety", True)
        self.declare_parameter("ttc_caution_sec", 9.0)
        self.declare_parameter("ttc_stop_sec", 2.0)
        self.declare_parameter("dynamic_margin_base", 1.5)
        self.declare_parameter("dynamic_margin_ttc_gain", 2.0)
        self.declare_parameter("encounter_head_on_penalty", 0.35)
        self.declare_parameter("encounter_crossing_starboard_penalty", 0.30)
        self.declare_parameter("encounter_crossing_port_penalty", 0.18)
        self.declare_parameter("encounter_overtaking_penalty", 0.12)
        self.declare_parameter("encounter_replan_ttc_sec", 4.5)
        self.declare_parameter("risk_replan_threshold", 0.62)
        self.declare_parameter("ignore_object_types", ["waypoint", "marker", "gate_post", "buoy"])
        self.declare_parameter("log_root", "/tmp/marine_vln_logs")

        semantic_map_topic = str(self.get_parameter("semantic_map_topic").value)
        raw_cmd_vel_topic = str(self.get_parameter("raw_cmd_vel_topic").value)
        safe_cmd_vel_topic = str(self.get_parameter("safe_cmd_vel_topic").value)
        raw_left_thrust_topic = str(self.get_parameter("raw_left_thrust_topic").value)
        raw_right_thrust_topic = str(self.get_parameter("raw_right_thrust_topic").value)
        raw_left_pos_topic = str(self.get_parameter("raw_left_pos_topic").value)
        raw_right_pos_topic = str(self.get_parameter("raw_right_pos_topic").value)
        final_left_thrust_topic = str(self.get_parameter("final_left_thrust_topic").value)
        final_right_thrust_topic = str(self.get_parameter("final_right_thrust_topic").value)
        final_left_pos_topic = str(self.get_parameter("final_left_pos_topic").value)
        final_right_pos_topic = str(self.get_parameter("final_right_pos_topic").value)
        replan_topic = str(self.get_parameter("replan_topic").value)
        safety_status_topic = str(self.get_parameter("safety_status_topic").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._enable_emergency_stop = bool(self.get_parameter("enable_emergency_stop").value)
        self._enable_replan_trigger = bool(self.get_parameter("enable_replan_trigger").value)
        self._caution_distance = float(self.get_parameter("caution_distance").value)
        self._stop_distance = float(self.get_parameter("stop_distance").value)
        self._resume_distance = float(self.get_parameter("resume_distance").value)
        self._min_scale = float(self.get_parameter("min_scale").value)
        self._dynamic_min_scale = float(self.get_parameter("dynamic_min_scale").value)
        self._dynamic_stop_distance_scale = float(self.get_parameter("dynamic_stop_distance_scale").value)
        self._dynamic_hard_stop_distance = float(self.get_parameter("dynamic_hard_stop_distance").value)
        self._preserve_turning_authority = bool(self.get_parameter("preserve_turning_authority").value)
        self._turning_min_scale = float(self.get_parameter("turning_authority_min_scale").value)
        self._turning_exponent = float(self.get_parameter("turning_authority_exponent").value)
        self._dynamic_turn_delta_ratio_limit = float(self.get_parameter("dynamic_turn_delta_ratio_limit").value)
        self._max_abs_thrust = float(self.get_parameter("max_abs_thrust").value)
        self._dynamic_emergency_stop = bool(self.get_parameter("dynamic_emergency_stop").value)
        self._use_encounter_aware = bool(self.get_parameter("use_encounter_aware_safety").value)
        self._ttc_caution_sec = float(self.get_parameter("ttc_caution_sec").value)
        self._ttc_stop_sec = float(self.get_parameter("ttc_stop_sec").value)
        self._dynamic_margin_base = float(self.get_parameter("dynamic_margin_base").value)
        self._dynamic_margin_ttc_gain = float(self.get_parameter("dynamic_margin_ttc_gain").value)
        self._encounter_head_on_penalty = float(self.get_parameter("encounter_head_on_penalty").value)
        self._encounter_crossing_starboard_penalty = float(self.get_parameter("encounter_crossing_starboard_penalty").value)
        self._encounter_crossing_port_penalty = float(self.get_parameter("encounter_crossing_port_penalty").value)
        self._encounter_overtaking_penalty = float(self.get_parameter("encounter_overtaking_penalty").value)
        self._encounter_replan_ttc_sec = float(self.get_parameter("encounter_replan_ttc_sec").value)
        self._risk_replan_threshold = float(self.get_parameter("risk_replan_threshold").value)
        ignore_raw = self.get_parameter("ignore_object_types").value
        if not isinstance(ignore_raw, list):
            ignore_raw = []
        self._ignore_types = {str(v).strip() for v in ignore_raw}
        log_root = str(self.get_parameter("log_root").value)
        if self._resume_distance <= self._stop_distance:
            self._resume_distance = self._stop_distance + 0.5
            self.get_logger().warn(
                f"resume_distance <= stop_distance, auto-adjust to {self._resume_distance:.2f}"
            )

        self._logger_file = JsonlLogger(self, "safety_monitor", log_root)

        self._safe_cmd_vel_pub = self.create_publisher(Twist, safe_cmd_vel_topic, 10)
        self._final_left_thrust_pub = self.create_publisher(Float64, final_left_thrust_topic, 10)
        self._final_right_thrust_pub = self.create_publisher(Float64, final_right_thrust_topic, 10)
        self._final_left_pos_pub = self.create_publisher(Float64, final_left_pos_topic, 10)
        self._final_right_pos_pub = self.create_publisher(Float64, final_right_pos_topic, 10)
        self._replan_pub = self.create_publisher(Bool, replan_topic, 10)
        self._safety_status_pub = self.create_publisher(String, safety_status_topic, 10)

        self.create_subscription(String, semantic_map_topic, self._semantic_cb, 20)
        self.create_subscription(Twist, raw_cmd_vel_topic, self._raw_cmd_vel_cb, 20)
        self.create_subscription(Float64, raw_left_thrust_topic, self._raw_left_thrust_cb, 20)
        self.create_subscription(Float64, raw_right_thrust_topic, self._raw_right_thrust_cb, 20)
        self.create_subscription(Float64, raw_left_pos_topic, self._raw_left_pos_cb, 20)
        self.create_subscription(Float64, raw_right_pos_topic, self._raw_right_pos_cb, 20)
        self.create_timer(max(0.02, 1.0 / max(0.1, control_rate_hz)), self._safety_loop)

        self._semantic_map: Dict[str, Any] = {}
        self._raw_cmd_vel = Twist()
        self._raw_left_thrust = 0.0
        self._raw_right_thrust = 0.0
        self._raw_left_pos = 0.0
        self._raw_right_pos = 0.0
        self._last_stop_state = False
        self._last_high_risk_state = False

        self.get_logger().info(
            f"safety_monitor started: final_thruster_topics=({final_left_thrust_topic},{final_right_thrust_topic}), "
            f"caution={self._caution_distance:.2f}, stop={self._stop_distance:.2f}, "
            f"resume={self._resume_distance:.2f}, encounter_aware={self._use_encounter_aware}, "
            f"ignore_types={sorted(self._ignore_types)}, "
            f"log={self._logger_file.log_path}"
        )

    def _semantic_cb(self, msg: String) -> None:
        data = parse_json_or_empty(self, msg.data, "semantic_map_topic")
        if data:
            self._semantic_map = data

    def _raw_cmd_vel_cb(self, msg: Twist) -> None:
        self._raw_cmd_vel = msg

    def _raw_left_thrust_cb(self, msg: Float64) -> None:
        self._raw_left_thrust = float(msg.data)

    def _raw_right_thrust_cb(self, msg: Float64) -> None:
        self._raw_right_thrust = float(msg.data)

    def _raw_left_pos_cb(self, msg: Float64) -> None:
        self._raw_left_pos = float(msg.data)

    def _raw_right_pos_cb(self, msg: Float64) -> None:
        self._raw_right_pos = float(msg.data)

    @staticmethod
    def _is_dynamic_obstacle(obj: Dict[str, Any]) -> bool:
        obj_type = str(obj.get("type", "")).strip().lower()
        name = str(obj.get("name", "")).strip().lower()
        return bool(obj.get("dynamic", False)) or obj_type.endswith("_dynamic") or name.startswith("dyn_")

    def _nearest_obstacle_distance(self, include_dynamic: bool = True) -> Tuple[Optional[float], Optional[str], bool]:
        objs = self._semantic_map.get("objects", [])
        if not isinstance(objs, list):
            return (None, None, False)
        best_dist: Optional[float] = None
        best_name: Optional[str] = None
        best_is_dynamic = False
        for obj in objs:
            if not isinstance(obj, dict):
                continue
            obj_type = str(obj.get("type", "unknown"))
            if obj_type in self._ignore_types:
                continue
            is_dynamic = self._is_dynamic_obstacle(obj)
            if is_dynamic and not include_dynamic:
                continue
            d = float(obj.get("distance", 1e9))
            if best_dist is None or d < best_dist:
                best_dist = d
                best_name = str(obj.get("name", "unknown"))
                best_is_dynamic = is_dynamic
        return (best_dist, best_name, best_is_dynamic)

    def _compute_scale(self, nearest_dist: Optional[float], nearest_is_dynamic: bool) -> float:
        if nearest_dist is None:
            return 1.0
        # When dynamic emergency stop is disabled, dynamic obstacles should slow down
        # the vehicle but not force an immediate zero-speed lock.
        if nearest_is_dynamic and not self._dynamic_emergency_stop:
            if nearest_dist <= self._dynamic_hard_stop_distance:
                return 0.0
            dyn_stop = max(0.3, self._stop_distance * self._dynamic_stop_distance_scale)
            if nearest_dist >= self._caution_distance:
                return 1.0
            ratio = (nearest_dist - dyn_stop) / max(1e-6, self._caution_distance - dyn_stop)
            return clamp(
                self._dynamic_min_scale + ratio * (1.0 - self._dynamic_min_scale),
                self._dynamic_min_scale,
                1.0,
            )

        if nearest_dist <= self._stop_distance:
            return 0.0
        if nearest_dist >= self._caution_distance:
            return 1.0
        ratio = (nearest_dist - self._stop_distance) / max(1e-6, self._caution_distance - self._stop_distance)
        return clamp(self._min_scale + ratio * (1.0 - self._min_scale), self._min_scale, 1.0)

    def _should_stop(self, nearest_dist: Optional[float]) -> bool:
        if nearest_dist is None:
            return False
        # Hysteresis: once stopped, require a larger clearance to resume.
        if self._last_stop_state:
            return nearest_dist < self._resume_distance
        return nearest_dist <= self._stop_distance

    def _ego_state(self) -> Tuple[float, float, float, float, float]:
        ego = self._semantic_map.get("ego", {})
        if not isinstance(ego, dict):
            ego = {}
        ex = float(ego.get("x", 0.0))
        ey = float(ego.get("y", 0.0))
        eyaw = float(ego.get("yaw", 0.0))
        evx = float(ego.get("vx", 0.0))
        evy = float(ego.get("vy", 0.0))
        return (ex, ey, eyaw, evx, evy)

    @staticmethod
    def _encounter_penalty_by_type(encounter_type: str) -> float:
        # placeholder, actual penalties are applied in _encounter_metrics
        _ = encounter_type
        return 0.0

    def _encounter_metrics(self) -> Dict[str, Any]:
        if not self._use_encounter_aware:
            return {
                "risk": 0.0,
                "encounter_type": "none",
                "min_ttc": float("inf"),
                "nearest_dynamic": None,
            }

        ex, ey, eyaw, evx, evy = self._ego_state()
        objects = self._semantic_map.get("objects", [])
        if not isinstance(objects, list):
            objects = []

        best_risk = 0.0
        best_ttc = float("inf")
        best_type = "none"
        best_name: Optional[str] = None

        for obj in objects:
            if not isinstance(obj, dict):
                continue
            obj_type = str(obj.get("type", "unknown")).strip()
            if obj_type in self._ignore_types:
                continue
            is_dynamic = self._is_dynamic_obstacle(obj)
            if not is_dynamic:
                continue

            ox = float(obj.get("x", 0.0))
            oy = float(obj.get("y", 0.0))
            ovx = float(obj.get("vx", 0.0))
            ovy = float(obj.get("vy", 0.0))
            radius = float(obj.get("risk_radius", 2.0))

            rx = ox - ex
            ry = oy - ey
            dist = math.hypot(rx, ry)
            if dist < 1e-6:
                continue
            clearance = max(0.0, dist - radius)
            rvx = ovx - evx
            rvy = ovy - evy
            closing_speed = -((rx * rvx + ry * rvy) / dist)
            ttc = float("inf")
            if closing_speed > 1e-3:
                ttc = clearance / closing_speed

            bearing = wrap_to_pi(math.atan2(ry, rx) - eyaw)
            bearing_deg = math.degrees(bearing)

            encounter_type = "none"
            if abs(bearing_deg) <= 15.0 and closing_speed > 0.05:
                encounter_type = "head_on"
            elif -130.0 <= bearing_deg < -15.0 and closing_speed > 0.05:
                encounter_type = "crossing_starboard"
            elif 15.0 < bearing_deg <= 130.0 and closing_speed > 0.05:
                encounter_type = "crossing_port"
            elif abs(bearing_deg) < 35.0 and math.hypot(evx, evy) > math.hypot(ovx, ovy) + 0.35:
                encounter_type = "overtaking"

            penalty = 0.0
            if encounter_type == "head_on":
                penalty = self._encounter_head_on_penalty
            elif encounter_type == "crossing_starboard":
                penalty = self._encounter_crossing_starboard_penalty
            elif encounter_type == "crossing_port":
                penalty = self._encounter_crossing_port_penalty
            elif encounter_type == "overtaking":
                penalty = self._encounter_overtaking_penalty

            ttc_risk = 0.0
            if math.isfinite(ttc):
                if ttc <= self._ttc_stop_sec:
                    ttc_risk = 1.0
                elif ttc <= self._ttc_caution_sec:
                    ttc_risk = 1.0 - (ttc - self._ttc_stop_sec) / max(1e-3, self._ttc_caution_sec - self._ttc_stop_sec)

            margin = self._dynamic_margin_base
            if math.isfinite(ttc) and ttc < self._ttc_caution_sec:
                margin += self._dynamic_margin_ttc_gain * (1.0 - ttc / max(1e-3, self._ttc_caution_sec))
            clearance_risk = 0.0
            if dist < radius + margin:
                clearance_risk = 1.0 - (dist - radius) / max(1e-3, margin)
                clearance_risk = clamp(clearance_risk, 0.0, 1.0)

            obj_risk = clamp(max(ttc_risk, clearance_risk) + penalty, 0.0, 1.0)
            if obj_risk > best_risk:
                best_risk = obj_risk
                best_type = encounter_type
                best_name = str(obj.get("name", "unknown"))
                best_ttc = ttc
            if ttc < best_ttc:
                best_ttc = ttc

        return {
            "risk": best_risk,
            "encounter_type": best_type,
            "min_ttc": best_ttc,
            "nearest_dynamic": best_name,
        }

    def _safety_loop(self) -> None:
        nearest_dist, nearest_name, nearest_is_dynamic = self._nearest_obstacle_distance(include_dynamic=True)
        stop_ref_dist, stop_ref_name, _ = self._nearest_obstacle_distance(include_dynamic=self._dynamic_emergency_stop)
        encounter = self._encounter_metrics()
        encounter_risk = float(encounter.get("risk", 0.0))
        encounter_type = str(encounter.get("encounter_type", "none"))
        min_ttc = float(encounter.get("min_ttc", float("inf")))

        stop_now = self._should_stop(stop_ref_dist)
        if self._use_encounter_aware and math.isfinite(min_ttc) and min_ttc <= self._ttc_stop_sec:
            stop_now = True
        if not self._enable_emergency_stop:
            stop_now = False

        dist_scale = 0.0 if stop_now else self._compute_scale(nearest_dist, nearest_is_dynamic)
        encounter_scale = clamp(1.0 - encounter_risk, 0.0, 1.0)
        scale = min(dist_scale, encounter_scale) if self._use_encounter_aware else dist_scale
        turn_scale = self._turning_min_scale if stop_now else max(
            self._turning_min_scale,
            math.pow(max(0.0, scale), max(1e-3, self._turning_exponent)),
        )
        turn_scale = clamp(turn_scale, 0.0, 1.0)

        safe_cmd = Twist()
        safe_cmd.linear.x = self._raw_cmd_vel.linear.x * scale
        if self._preserve_turning_authority:
            # Reduce over-steering when linear speed is safety-scaled down.
            safe_cmd.angular.z = self._raw_cmd_vel.angular.z * turn_scale
        else:
            safe_cmd.angular.z = self._raw_cmd_vel.angular.z * scale
        self._safe_cmd_vel_pub.publish(safe_cmd)

        if stop_now:
            left = 0.0
            right = 0.0
        elif self._preserve_turning_authority:
            # Preserve differential steering authority while reducing forward push.
            mean = 0.5 * (self._raw_left_thrust + self._raw_right_thrust)
            delta = 0.5 * (self._raw_right_thrust - self._raw_left_thrust)
            scaled_mean = mean * scale
            scaled_delta = delta * turn_scale
            if nearest_is_dynamic and not self._dynamic_emergency_stop and scale > 0.0:
                # Guard against low-speed pivot lock near dynamic obstacles:
                # keep differential thrust bounded by forward component.
                ratio_limit = clamp(self._dynamic_turn_delta_ratio_limit, 0.0, 1.0)
                max_delta = abs(scaled_mean) * ratio_limit
                scaled_delta = clamp(scaled_delta, -max_delta, max_delta)
            left = scaled_mean - scaled_delta
            right = scaled_mean + scaled_delta
        else:
            left = self._raw_left_thrust * scale
            right = self._raw_right_thrust * scale

        left = clamp(left, -self._max_abs_thrust, self._max_abs_thrust)
        right = clamp(right, -self._max_abs_thrust, self._max_abs_thrust)

        self._final_left_thrust_pub.publish(Float64(data=float(left)))
        self._final_right_thrust_pub.publish(Float64(data=float(right)))
        self._final_left_pos_pub.publish(Float64(data=self._raw_left_pos))
        self._final_right_pos_pub.publish(Float64(data=self._raw_right_pos))

        replan = Bool()
        high_risk = self._use_encounter_aware and (
            encounter_risk >= self._risk_replan_threshold
            or (math.isfinite(min_ttc) and min_ttc <= self._encounter_replan_ttc_sec)
        )
        # Trigger replan on STOP rising edge or encounter-risk rising edge to avoid spam.
        replan.data = self._enable_replan_trigger and (
            (stop_now and not self._last_stop_state) or (high_risk and not self._last_high_risk_state)
        )
        self._replan_pub.publish(replan)

        if stop_now != self._last_stop_state:
            state = "EMERGENCY_STOP" if stop_now else "RESUME"
            if stop_now:
                self.get_logger().warn(f"[{state}] nearest_obstacle={stop_ref_name}, dist={stop_ref_dist}")
            else:
                self.get_logger().warn(f"[{state}] nearest_obstacle={nearest_name}, dist={nearest_dist}")
        self._last_stop_state = stop_now
        self._last_high_risk_state = high_risk

        intervention = "none"
        if stop_now:
            intervention = "emergency_stop"
        elif scale < 0.35:
            intervention = "high"
        elif scale < 0.7:
            intervention = "medium"
        elif scale < 0.98:
            intervention = "low"

        status_payload = {
            "nearest_obstacle": nearest_name,
            "nearest_dist": nearest_dist,
            "nearest_is_dynamic": nearest_is_dynamic,
            "enable_emergency_stop": self._enable_emergency_stop,
            "encounter_type": encounter_type,
            "min_ttc": min_ttc,
            "risk_score": encounter_risk,
            "safety_scale": scale,
            "turn_scale": turn_scale,
            "intervention_level": intervention,
            "emergency_stop_triggered": stop_now,
            "replan_flag": bool(replan.data),
        }
        status_msg = String()
        status_msg.data = json.dumps(status_payload, ensure_ascii=False)
        self._safety_status_pub.publish(status_msg)

        self._logger_file.write(
            "safety_output",
            {
                "nearest_obstacle": nearest_name,
                "nearest_dist": nearest_dist,
                "nearest_is_dynamic": nearest_is_dynamic,
                "stop_ref_obstacle": stop_ref_name,
                "stop_ref_dist": stop_ref_dist,
                "encounter_type": encounter_type,
                "min_ttc": min_ttc,
                "risk_score": encounter_risk,
                "scale": scale,
                "safety_scale": scale,
                "turn_scale": turn_scale,
                "intervention_level": intervention,
                "left_thrust": left,
                "right_thrust": right,
                "emergency_stop": stop_now,
                "emergency_stop_triggered": stop_now,
                "replan_flag": bool(replan.data),
            },
        )


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitorNode()
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
