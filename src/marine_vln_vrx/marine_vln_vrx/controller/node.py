#!/usr/bin/env python3
"""Low-level controller: path tracking -> raw cmd_vel + raw thruster commands."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from marine_vln_vrx.common.geometry_utils import clamp, distance_2d, wrap_to_pi, yaw_from_quaternion
from marine_vln_vrx.common.log_utils import JsonlLogger


class ControllerNode(Node):
    """Pure pursuit + simple speed PI mapped to differential thrust."""

    def __init__(self) -> None:
        super().__init__("controller")

        self.declare_parameter("path_topic", "/vln/local_path")
        self.declare_parameter("odom_topic", "/model/wamv/odometry")
        self.declare_parameter("odom_topic_fallback", "/wamv/sensors/position/ground_truth_odometry")
        self.declare_parameter("cmd_vel_topic", "/vln/cmd_vel_raw")
        self.declare_parameter("left_thrust_topic", "/vln/raw/left_thrust")
        self.declare_parameter("right_thrust_topic", "/vln/raw/right_thrust")
        self.declare_parameter("left_pos_topic", "/vln/raw/left_pos")
        self.declare_parameter("right_pos_topic", "/vln/raw/right_pos")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("lookahead_distance", 3.5)
        self.declare_parameter("goal_tolerance", 1.5)
        self.declare_parameter("desired_speed", 1.2)
        self.declare_parameter("max_omega", 1.0)
        self.declare_parameter("kp_yaw", 1.8)
        self.declare_parameter("min_speed_scale", 0.18)
        self.declare_parameter("hard_turn_heading_rad", 1.05)
        self.declare_parameter("hard_turn_speed_scale", 0.10)
        self.declare_parameter("hard_turn_min_yaw_delta", 70.0)
        self.declare_parameter("use_reverse_for_turning", True)
        self.declare_parameter("kff_thrust", 90.0)
        self.declare_parameter("kp_speed", 40.0)
        self.declare_parameter("ki_speed", 5.0)
        self.declare_parameter("max_thrust", 300.0)
        self.declare_parameter("min_thrust", 0.0)
        self.declare_parameter("yaw_to_thrust_gain", 120.0)
        self.declare_parameter("max_yaw_delta_ratio", 0.65)
        self.declare_parameter("neutral_thruster_pos", 0.0)
        self.declare_parameter("log_root", "/tmp/marine_vln_logs")

        path_topic = str(self.get_parameter("path_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        odom_topic_fallback = str(self.get_parameter("odom_topic_fallback").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        left_thrust_topic = str(self.get_parameter("left_thrust_topic").value)
        right_thrust_topic = str(self.get_parameter("right_thrust_topic").value)
        left_pos_topic = str(self.get_parameter("left_pos_topic").value)
        right_pos_topic = str(self.get_parameter("right_pos_topic").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self._goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self._desired_speed = float(self.get_parameter("desired_speed").value)
        self._max_omega = float(self.get_parameter("max_omega").value)
        self._kp_yaw = float(self.get_parameter("kp_yaw").value)
        self._min_speed_scale = float(self.get_parameter("min_speed_scale").value)
        self._hard_turn_heading_rad = float(self.get_parameter("hard_turn_heading_rad").value)
        self._hard_turn_speed_scale = float(self.get_parameter("hard_turn_speed_scale").value)
        self._hard_turn_min_yaw_delta = float(self.get_parameter("hard_turn_min_yaw_delta").value)
        self._use_reverse_for_turning = bool(self.get_parameter("use_reverse_for_turning").value)
        self._kff_thrust = float(self.get_parameter("kff_thrust").value)
        self._kp_speed = float(self.get_parameter("kp_speed").value)
        self._ki_speed = float(self.get_parameter("ki_speed").value)
        self._max_thrust = float(self.get_parameter("max_thrust").value)
        self._min_thrust = float(self.get_parameter("min_thrust").value)
        self._yaw_to_thrust_gain = float(self.get_parameter("yaw_to_thrust_gain").value)
        self._max_yaw_delta_ratio = float(self.get_parameter("max_yaw_delta_ratio").value)
        self._neutral_pos = float(self.get_parameter("neutral_thruster_pos").value)
        log_root = str(self.get_parameter("log_root").value)

        self._logger_file = JsonlLogger(self, "controller", log_root)
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._left_thrust_pub = self.create_publisher(Float64, left_thrust_topic, 10)
        self._right_thrust_pub = self.create_publisher(Float64, right_thrust_topic, 10)
        self._left_pos_pub = self.create_publisher(Float64, left_pos_topic, 10)
        self._right_pos_pub = self.create_publisher(Float64, right_pos_topic, 10)

        self.create_subscription(Path, path_topic, self._path_callback, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, qos_profile_sensor_data)
        if odom_topic_fallback and odom_topic_fallback != odom_topic:
            self.create_subscription(Odometry, odom_topic_fallback, self._odom_callback, qos_profile_sensor_data)
        self.create_timer(max(0.02, 1.0 / max(0.1, control_rate_hz)), self._control_loop)

        self._path_points: List[Tuple[float, float]] = []
        self._odom: Optional[Odometry] = None
        self._speed_integral: float = 0.0
        self._dt = 1.0 / max(0.1, control_rate_hz)

        self.get_logger().info(
            f"controller started: path_topic={path_topic}, cmd_vel_topic={cmd_vel_topic}, "
            f"thruster_topics=({left_thrust_topic},{right_thrust_topic}), "
            f"lookahead={self._lookahead_distance:.2f}, desired_speed={self._desired_speed:.2f}, "
            f"max_omega={self._max_omega:.2f}, kp_yaw={self._kp_yaw:.2f}, "
            f"hard_turn_rad={self._hard_turn_heading_rad:.2f}, "
            f"hard_turn_speed_scale={self._hard_turn_speed_scale:.2f}, "
            f"reverse_turn={self._use_reverse_for_turning}, "
            f"yaw_to_thrust={self._yaw_to_thrust_gain:.1f}, "
            f"max_yaw_delta_ratio={self._max_yaw_delta_ratio:.2f}, log={self._logger_file.log_path}"
        )

    def _path_callback(self, msg: Path) -> None:
        self._path_points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]

    def _odom_callback(self, msg: Odometry) -> None:
        self._odom = msg

    def _publish_zero(self) -> None:
        self._cmd_vel_pub.publish(Twist())
        self._left_thrust_pub.publish(Float64(data=0.0))
        self._right_thrust_pub.publish(Float64(data=0.0))
        self._left_pos_pub.publish(Float64(data=self._neutral_pos))
        self._right_pos_pub.publish(Float64(data=self._neutral_pos))

    def _nearest_and_lookahead_index(self, x: float, y: float) -> Tuple[int, int]:
        nearest_idx = 0
        nearest_dist = float("inf")
        for i, (px, py) in enumerate(self._path_points):
            dist = distance_2d(x, y, px, py)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_idx = i

        look_idx = nearest_idx
        accum = 0.0
        for i in range(nearest_idx, len(self._path_points) - 1):
            p0 = self._path_points[i]
            p1 = self._path_points[i + 1]
            accum += distance_2d(p0[0], p0[1], p1[0], p1[1])
            look_idx = i + 1
            if accum >= self._lookahead_distance:
                break
        return nearest_idx, look_idx

    def _control_loop(self) -> None:
        if self._odom is None or len(self._path_points) < 2:
            self._publish_zero()
            return

        pose = self._odom.pose.pose
        twist = self._odom.twist.twist
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        speed = math.hypot(float(twist.linear.x), float(twist.linear.y))

        goal_x, goal_y = self._path_points[-1]
        dist_to_goal = distance_2d(x, y, goal_x, goal_y)
        if dist_to_goal <= self._goal_tolerance:
            self._speed_integral = 0.0
            self._publish_zero()
            return

        _, look_idx = self._nearest_and_lookahead_index(x, y)
        tx, ty = self._path_points[look_idx]
        target_yaw = math.atan2(ty - y, tx - x)
        heading_error = wrap_to_pi(target_yaw - yaw)
        omega_cmd = clamp(self._kp_yaw * heading_error, -self._max_omega, self._max_omega)

        abs_heading_error = abs(heading_error)
        speed_scale = 1.0 - min(1.0, abs_heading_error / math.pi)
        speed_scale = max(0.0, min(1.0, speed_scale))
        speed_scale = max(0.05, self._min_speed_scale, speed_scale)
        hard_turn = abs_heading_error >= self._hard_turn_heading_rad
        if hard_turn:
            speed_scale = min(speed_scale, max(0.02, self._hard_turn_speed_scale))
        v_ref = speed_scale * self._desired_speed
        speed_error = v_ref - speed
        self._speed_integral = clamp(self._speed_integral + speed_error * self._dt, -3.0, 3.0)

        base_thrust = (
            self._kff_thrust * v_ref
            + self._kp_speed * speed_error
            + self._ki_speed * self._speed_integral
        )
        yaw_delta = self._yaw_to_thrust_gain * omega_cmd
        max_yaw_delta = max(20.0, abs(base_thrust) * max(0.0, self._max_yaw_delta_ratio))
        yaw_delta = clamp(yaw_delta, -max_yaw_delta, max_yaw_delta)
        if hard_turn and self._hard_turn_min_yaw_delta > 0.0:
            sign = 1.0 if yaw_delta >= 0.0 else -1.0
            yaw_delta = sign * max(abs(yaw_delta), self._hard_turn_min_yaw_delta)

        turn_min_thrust = self._min_thrust if (hard_turn and self._use_reverse_for_turning) else max(0.0, self._min_thrust)
        left_thrust = clamp(base_thrust - yaw_delta, turn_min_thrust, self._max_thrust)
        right_thrust = clamp(base_thrust + yaw_delta, turn_min_thrust, self._max_thrust)

        cmd = Twist()
        cmd.linear.x = float(v_ref)
        cmd.angular.z = float(omega_cmd)
        self._cmd_vel_pub.publish(cmd)
        self._left_thrust_pub.publish(Float64(data=float(left_thrust)))
        self._right_thrust_pub.publish(Float64(data=float(right_thrust)))
        self._left_pos_pub.publish(Float64(data=self._neutral_pos))
        self._right_pos_pub.publish(Float64(data=self._neutral_pos))

        self._logger_file.write(
            "control_output",
            {
                "target_idx": int(look_idx),
                "dist_to_goal": dist_to_goal,
                "v_ref": v_ref,
                "omega_cmd": omega_cmd,
                "heading_error": heading_error,
                "hard_turn": hard_turn,
                "left_thrust": left_thrust,
                "right_thrust": right_thrust,
            },
        )


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
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
