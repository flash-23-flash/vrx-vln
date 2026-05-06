#!/usr/bin/env python3
"""Scene parser node: sensors + simplified object truth -> scene JSON."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
from std_msgs.msg import String

from marine_vln_vrx.common.geometry_utils import distance_2d, yaw_from_quaternion
from marine_vln_vrx.common.log_utils import JsonlLogger


class SceneParserNode(Node):
    """Collect ego state and known objects into a compact scene state."""

    def __init__(self) -> None:
        super().__init__("scene_parser")

        self.declare_parameter("odom_topic", "/model/wamv/odometry")
        self.declare_parameter("odom_topic_fallback", "/wamv/sensors/position/ground_truth_odometry")
        self.declare_parameter("imu_topic", "/wamv/sensors/imu/imu/data")
        self.declare_parameter("gps_topic", "/wamv/sensors/gps/gps/fix")
        self.declare_parameter("camera_topic", "/wamv/sensors/cameras/front_camera_sensor/optical/image_raw")
        self.declare_parameter("lidar_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/points")
        self.declare_parameter("scene_topic", "/vln/scene_state")
        self.declare_parameter("object_file", "")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("enable_dynamic_object_motion", True)
        self.declare_parameter("dynamic_motion_time_scale", 1.0)
        self.declare_parameter("log_root", "/tmp/marine_vln_logs")

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._odom_topic_fallback = str(self.get_parameter("odom_topic_fallback").value)
        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._gps_topic = str(self.get_parameter("gps_topic").value)
        self._camera_topic = str(self.get_parameter("camera_topic").value)
        self._lidar_topic = str(self.get_parameter("lidar_topic").value)
        scene_topic = str(self.get_parameter("scene_topic").value)
        object_file = str(self.get_parameter("object_file").value)
        if not object_file:
            object_file = str(Path(get_package_share_directory("marine_vln_vrx")) / "data" / "scene_objects.yaml")
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._enable_dynamic_motion = bool(self.get_parameter("enable_dynamic_object_motion").value)
        self._dynamic_motion_time_scale = float(self.get_parameter("dynamic_motion_time_scale").value)
        log_root = str(self.get_parameter("log_root").value)

        self._logger_file = JsonlLogger(self, "scene_parser", log_root)
        self._scene_pub = self.create_publisher(String, scene_topic, 10)

        self._latest_odom: Odometry | None = None
        self._latest_imu: Imu | None = None
        self._latest_gps: NavSatFix | None = None
        self._latest_camera_stamp_ns: int | None = None
        self._latest_lidar_stamp_ns: int | None = None
        self._objects: List[Dict[str, Any]] = self._load_objects(object_file)
        self._motion_start_ns = self.get_clock().now().nanoseconds
        self._last_dynamic_sample_log_ns = 0
        self._prev_object_state: Dict[str, tuple[float, float, float]] = {}

        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, qos_profile_sensor_data)
        if self._odom_topic_fallback and self._odom_topic_fallback != self._odom_topic:
            self.create_subscription(Odometry, self._odom_topic_fallback, self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, self._imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, self._gps_topic, self._gps_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self._camera_topic, self._camera_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self._lidar_topic, self._lidar_cb, qos_profile_sensor_data)
        self.create_timer(max(0.05, 1.0 / max(0.1, publish_rate_hz)), self._publish_scene)

        odom_topics = [self._odom_topic]
        if self._odom_topic_fallback and self._odom_topic_fallback != self._odom_topic:
            odom_topics.append(self._odom_topic_fallback)
        self.get_logger().info(
            f"scene_parser started: odom_topics={odom_topics}, scene_topic={scene_topic}, "
            f"object_file={object_file}, objects={len(self._objects)}, "
            f"dynamic_motion={self._enable_dynamic_motion}, log={self._logger_file.log_path}"
        )

    def _load_objects(self, object_file: str) -> List[Dict[str, Any]]:
        if not object_file:
            self.get_logger().warn("object_file is empty, no semantic objects loaded.")
            return []
        file_path = Path(object_file).expanduser().resolve()
        if not file_path.exists():
            self.get_logger().error(f"Object file not found: {file_path}")
            return []

        try:
            raw = yaml.safe_load(file_path.read_text(encoding="utf-8"))
        except (yaml.YAMLError, OSError) as exc:
            self.get_logger().error(f"Failed to load object file {file_path}: {exc}")
            return []

        objects = raw.get("objects", []) if isinstance(raw, dict) else []
        parsed: List[Dict[str, Any]] = []
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            position = obj.get("position", [0.0, 0.0, 0.0])
            if not isinstance(position, list) or len(position) < 2:
                continue
            x0 = float(position[0])
            y0 = float(position[1])
            z0 = float(position[2]) if len(position) > 2 else 0.0

            motion_cfg: Dict[str, Any] = {}
            motion_raw = obj.get("motion", {})
            if isinstance(motion_raw, dict):
                pattern = str(motion_raw.get("pattern", "")).strip().lower()
                if pattern:
                    motion_cfg = {
                        "pattern": pattern,
                        "period_sec": float(motion_raw.get("period_sec", 20.0)),
                        "phase_sec": float(motion_raw.get("phase_sec", 0.0)),
                        "amplitude_x": float(motion_raw.get("amplitude_x", 0.0)),
                        "amplitude_y": float(motion_raw.get("amplitude_y", 0.0)),
                        "radius": float(motion_raw.get("radius", 0.0)),
                    }
            parsed.append(
                {
                    "name": str(obj.get("name", "unknown")),
                    "type": str(obj.get("type", "unknown")),
                    "x": x0,
                    "y": y0,
                    "z": z0,
                    "x0": x0,
                    "y0": y0,
                    "dynamic": bool(motion_cfg),
                    "motion": motion_cfg,
                    "risk_radius": float(obj.get("risk_radius", 2.0)),
                    "aliases": obj.get("aliases", []),
                }
            )
        return parsed

    @staticmethod
    def _triangle_wave(u: float) -> float:
        # [-1, 1], piecewise-linear periodic waveform.
        uu = u % 1.0
        return 1.0 - 4.0 * abs(uu - 0.5)

    def _dynamic_xy(self, obj: Dict[str, Any], elapsed_sec: float) -> tuple[float, float]:
        x0 = float(obj.get("x0", obj.get("x", 0.0)))
        y0 = float(obj.get("y0", obj.get("y", 0.0)))

        if not self._enable_dynamic_motion or not bool(obj.get("dynamic", False)):
            return (x0, y0)

        motion = obj.get("motion", {})
        if not isinstance(motion, dict):
            return (x0, y0)

        pattern = str(motion.get("pattern", "")).strip().lower()
        period = max(0.5, float(motion.get("period_sec", 20.0)))
        phase_sec = float(motion.get("phase_sec", 0.0))
        t = (elapsed_sec + phase_sec) / period

        ax = float(motion.get("amplitude_x", 0.0))
        ay = float(motion.get("amplitude_y", 0.0))
        radius = float(motion.get("radius", 0.0))

        if pattern in {"linear_pingpong", "pingpong"}:
            s = self._triangle_wave(t)
            return (x0 + ax * s, y0 + ay * s)

        if pattern in {"circle", "orbit"}:
            rr_x = radius if radius > 0.0 else ax
            rr_y = radius if radius > 0.0 else ay
            if rr_x <= 0.0 and rr_y <= 0.0:
                return (x0, y0)
            ang = 2.0 * math.pi * t
            return (x0 + rr_x * math.cos(ang), y0 + rr_y * math.sin(ang))

        if pattern in {"sine", "sin"}:
            ang = 2.0 * math.pi * t
            return (x0 + ax * math.sin(ang), y0 + ay * math.sin(ang))

        return (x0, y0)

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _imu_cb(self, msg: Imu) -> None:
        self._latest_imu = msg

    def _gps_cb(self, msg: NavSatFix) -> None:
        self._latest_gps = msg

    def _camera_cb(self, msg: Image) -> None:
        self._latest_camera_stamp_ns = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)

    def _lidar_cb(self, msg: PointCloud2) -> None:
        self._latest_lidar_stamp_ns = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)

    def _publish_scene(self) -> None:
        if self._latest_odom is None:
            return

        pose = self._latest_odom.pose.pose
        twist = self._latest_odom.twist.twist
        ego_x = float(pose.position.x)
        ego_y = float(pose.position.y)
        ego_yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        speed = (twist.linear.x**2 + twist.linear.y**2) ** 0.5
        now_ns = self.get_clock().now().nanoseconds
        elapsed_sec = max(0.0, (now_ns - self._motion_start_ns) * 1e-9 * self._dynamic_motion_time_scale)

        objects_with_distance: List[Dict[str, Any]] = []
        dynamic_count = 0
        for obj in self._objects:
            name = str(obj.get("name", "unknown")).strip()
            ox, oy = self._dynamic_xy(obj, elapsed_sec)
            dist = distance_2d(ego_x, ego_y, ox, oy)
            this_obj = dict(obj)
            this_obj["x"] = ox
            this_obj["y"] = oy
            this_obj["distance"] = dist
            vx = 0.0
            vy = 0.0
            prev = self._prev_object_state.get(name)
            if prev is not None:
                px, py, pt = prev
                dt = elapsed_sec - pt
                if dt > 1e-3:
                    vx = (ox - px) / dt
                    vy = (oy - py) / dt
            this_obj["vx"] = vx
            this_obj["vy"] = vy
            self._prev_object_state[name] = (ox, oy, elapsed_sec)
            if bool(this_obj.get("dynamic", False)):
                dynamic_count += 1
            objects_with_distance.append(this_obj)

        scene = {
            "frame_id": "map",
            "stamp_ns": self.get_clock().now().nanoseconds,
            "ego": {
                "x": ego_x,
                "y": ego_y,
                "yaw": ego_yaw,
                "vx": float(twist.linear.x),
                "vy": float(twist.linear.y),
                "speed": float(speed),
                "lat": float(self._latest_gps.latitude) if self._latest_gps else None,
                "lon": float(self._latest_gps.longitude) if self._latest_gps else None,
            },
            "sensors": {
                "odom_ok": self._latest_odom is not None,
                "imu_ok": self._latest_imu is not None,
                "gps_ok": self._latest_gps is not None,
                "camera_stamp_ns": self._latest_camera_stamp_ns,
                "lidar_stamp_ns": self._latest_lidar_stamp_ns,
            },
            "objects": objects_with_distance,
        }

        msg = String()
        msg.data = json.dumps(scene, ensure_ascii=False)
        self._scene_pub.publish(msg)
        self._logger_file.write(
            "scene_published",
            {"objects": len(objects_with_distance), "dynamic_objects": dynamic_count, "ego": scene["ego"]},
        )

        if now_ns - self._last_dynamic_sample_log_ns > int(1e9):
            self._last_dynamic_sample_log_ns = now_ns
            dynamic_samples: List[Dict[str, Any]] = []
            for obj in objects_with_distance:
                if not bool(obj.get("dynamic", False)):
                    continue
                dynamic_samples.append(
                    {
                        "name": obj.get("name"),
                        "x": float(obj.get("x", 0.0)),
                        "y": float(obj.get("y", 0.0)),
                        "distance": float(obj.get("distance", 0.0)),
                    }
                )
            self._logger_file.write("dynamic_motion_sample", {"samples": dynamic_samples})


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SceneParserNode()
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
