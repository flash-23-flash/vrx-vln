#!/usr/bin/env python3
"""Overlay task/VLM debug and pseudo detection boxes on front camera image."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from marine_vln_vrx.common.json_utils import parse_json_or_empty
from marine_vln_vrx.common.log_utils import JsonlLogger

try:
    import cv2  # type: ignore
    import numpy as np  # type: ignore
except Exception:
    cv2 = None
    np = None


class VLMVisualizerNode(Node):
    """Render a debug overlay for RViz image display."""

    def __init__(self) -> None:
        super().__init__("vlm_visualizer")

        self.declare_parameter("image_topic", "/wamv/sensors/cameras/front_camera_sensor/optical/image_raw")
        self.declare_parameter("task_topic", "/vln/task_json")
        self.declare_parameter("subgoals_json_topic", "/vln/subgoals_json")
        self.declare_parameter("semantic_map_topic", "/vln/semantic_map")
        self.declare_parameter(
            "camera_info_topic", "/wamv/sensors/cameras/front_camera_sensor/optical/camera_info"
        )
        self.declare_parameter("overlay_topic", "/vln/debug/front_camera_overlay")
        self.declare_parameter("pseudo_fov_deg", 90.0)
        self.declare_parameter("camera_mount_x_m", 0.72)
        self.declare_parameter("camera_mount_y_m", 0.30)
        self.declare_parameter("camera_height_m", 1.50)
        self.declare_parameter("camera_yaw_deg", 0.0)
        self.declare_parameter("camera_pitch_deg", 2.0)
        self.declare_parameter("camera_roll_deg", 0.0)
        self.declare_parameter("use_camera_info_intrinsics", True)
        self.declare_parameter("max_objects", 8)
        self.declare_parameter("enable_pseudo_boxes", True)
        self.declare_parameter("publish_if_no_overlay", True)
        self.declare_parameter("log_root", "/tmp/marine_vln_logs")

        image_topic = str(self.get_parameter("image_topic").value)
        task_topic = str(self.get_parameter("task_topic").value)
        subgoals_json_topic = str(self.get_parameter("subgoals_json_topic").value)
        semantic_map_topic = str(self.get_parameter("semantic_map_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        overlay_topic = str(self.get_parameter("overlay_topic").value)
        self._fov_deg = float(self.get_parameter("pseudo_fov_deg").value)
        self._camera_x_m = float(self.get_parameter("camera_mount_x_m").value)
        self._camera_y_m = float(self.get_parameter("camera_mount_y_m").value)
        self._camera_height_m = float(self.get_parameter("camera_height_m").value)
        self._camera_yaw_deg = float(self.get_parameter("camera_yaw_deg").value)
        self._camera_pitch_deg = float(self.get_parameter("camera_pitch_deg").value)
        self._camera_roll_deg = float(self.get_parameter("camera_roll_deg").value)
        self._use_camera_info_intrinsics = bool(self.get_parameter("use_camera_info_intrinsics").value)
        self._max_objects = int(self.get_parameter("max_objects").value)
        self._enable_boxes = bool(self.get_parameter("enable_pseudo_boxes").value)
        self._publish_if_no_overlay = bool(self.get_parameter("publish_if_no_overlay").value)
        log_root = str(self.get_parameter("log_root").value)

        self._logger_file = JsonlLogger(self, "vlm_visualizer", log_root)
        self._overlay_pub = self.create_publisher(Image, overlay_topic, 10)
        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.create_subscription(String, task_topic, self._task_cb, 20)
        self.create_subscription(String, subgoals_json_topic, self._subgoals_status_cb, 20)
        self.create_subscription(String, semantic_map_topic, self._semantic_map_cb, 20)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 20)

        self._task: Dict[str, Any] = {}
        self._subgoals_status: Dict[str, Any] = {}
        self._semantic_map: Dict[str, Any] = {}
        self._cam_fx: Optional[float] = None
        self._cam_fy: Optional[float] = None
        self._cam_cx: Optional[float] = None
        self._cam_cy: Optional[float] = None
        self._cam_w: Optional[int] = None
        self._cam_h: Optional[int] = None
        self._last_log_ns = 0

        if cv2 is None or np is None:
            self.get_logger().warn("cv2/numpy unavailable, vlm_visualizer will republish raw images only.")

        self.get_logger().info(
            f"vlm_visualizer started: image_topic={image_topic}, overlay_topic={overlay_topic}, "
            f"boxes={self._enable_boxes}, fov_deg={self._fov_deg:.1f}, log={self._logger_file.log_path}"
        )

    def _task_cb(self, msg: String) -> None:
        data = parse_json_or_empty(self, msg.data, "task_topic")
        if data:
            self._task = data

    def _subgoals_status_cb(self, msg: String) -> None:
        data = parse_json_or_empty(self, msg.data, "subgoals_json_topic")
        if data:
            self._subgoals_status = data

    def _semantic_map_cb(self, msg: String) -> None:
        data = parse_json_or_empty(self, msg.data, "semantic_map_topic")
        if data:
            self._semantic_map = data

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if len(msg.k) < 9:
            return
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        if fx <= 1.0 or fy <= 1.0:
            return
        self._cam_fx = fx
        self._cam_fy = fy
        self._cam_cx = cx
        self._cam_cy = cy
        self._cam_w = int(msg.width)
        self._cam_h = int(msg.height)

    def _image_cb(self, msg: Image) -> None:
        if cv2 is None or np is None:
            if self._publish_if_no_overlay:
                self._overlay_pub.publish(msg)
            return

        frame = self._to_bgr(msg)
        if frame is None:
            if self._publish_if_no_overlay:
                self._overlay_pub.publish(msg)
            return

        route = str(self._task.get("route", "n/a"))
        fallback = str(self._task.get("fallback_reason", ""))
        semantic = self._task.get("semantic_parse", {})
        intent = str(semantic.get("intent", "unknown")) if isinstance(semantic, dict) else "unknown"
        target = semantic.get("target_id") if isinstance(semantic, dict) else None
        grounding_score = float(self._task.get("grounding_score", 0.0))
        candidate_count = int(self._task.get("hypothesis_count", 1))
        active_hypothesis = str(self._task.get("active_hypothesis_id", "h0"))
        switch_reason = str(self._task.get("hypothesis_switch_reason", "none"))
        if isinstance(self._subgoals_status, dict):
            active_hypothesis = str(self._subgoals_status.get("active_hypothesis_id", active_hypothesis))
            candidate_count = int(self._subgoals_status.get("candidate_count", candidate_count))
            switch_reason = str(self._subgoals_status.get("switch_reason", switch_reason))

        self._draw_header(
            frame,
            route,
            intent,
            target,
            fallback,
            grounding_score,
            active_hypothesis,
            candidate_count,
            switch_reason,
        )
        num_boxes = 0
        if self._enable_boxes:
            num_boxes = self._draw_pseudo_boxes(frame)

        out = self._to_image_msg(frame, msg)
        self._overlay_pub.publish(out)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns > int(1e9):
            self._last_log_ns = now_ns
            self._logger_file.write(
                "overlay_published",
                {
                    "route": route,
                    "intent": intent,
                    "target": target,
                    "fallback": fallback,
                    "pseudo_boxes": num_boxes,
                },
            )

    def _to_bgr(self, msg: Image) -> Optional["np.ndarray"]:
        enc = str(msg.encoding).lower()
        if enc not in {"rgb8", "bgr8"}:
            return None
        if msg.height <= 0 or msg.width <= 0 or msg.step <= 0:
            return None
        channels = 3
        if len(msg.data) < msg.height * msg.step:
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        rows = raw.reshape((msg.height, msg.step))
        frame = rows[:, : msg.width * channels].reshape((msg.height, msg.width, channels)).copy()
        if enc == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    def _to_image_msg(self, frame: "np.ndarray", src: Image) -> Image:
        out = Image()
        out.header = src.header
        out.height = int(frame.shape[0])
        out.width = int(frame.shape[1])
        out.encoding = "bgr8"
        out.is_bigendian = 0
        out.step = int(frame.shape[1] * 3)
        out.data = frame.tobytes()
        return out

    def _draw_header(
        self,
        frame: "np.ndarray",
        route: str,
        intent: str,
        target: Any,
        fallback: str,
        grounding_score: float,
        active_hypothesis: str,
        candidate_count: int,
        switch_reason: str,
    ) -> None:
        h, w = frame.shape[:2]
        panel_h = 92
        cv2.rectangle(frame, (0, 0), (w, panel_h), (25, 25, 25), thickness=-1)
        cv2.putText(
            frame,
            f"VLN route={route} intent={intent} target={target} grounding={grounding_score:.2f}",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (220, 255, 220),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"hypothesis={active_hypothesis} candidates={candidate_count} switch_reason={switch_reason}",
            (10, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 230, 170),
            1,
            cv2.LINE_AA,
        )
        fallback_text = fallback if fallback else "none"
        cv2.putText(
            frame,
            f"fallback={fallback_text}",
            (10, 74),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (180, 220, 255),
            1,
            cv2.LINE_AA,
        )

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def _body_to_camera_frame(self, bx: float, by: float, bz: float) -> Tuple[float, float, float]:
        """Convert body-forward-left-up vector into camera-forward-left-up vector."""
        yaw = math.radians(self._camera_yaw_deg)
        pitch = math.radians(self._camera_pitch_deg)
        roll = math.radians(self._camera_roll_deg)

        # Inverse yaw (around z-up).
        cy = math.cos(-yaw)
        sy = math.sin(-yaw)
        x1 = cy * bx - sy * by
        y1 = sy * bx + cy * by
        z1 = bz

        # Inverse pitch (around y-left).
        cp = math.cos(-pitch)
        sp = math.sin(-pitch)
        x2 = cp * x1 + sp * z1
        y2 = y1
        z2 = -sp * x1 + cp * z1

        # Inverse roll (around x-forward).
        cr = math.cos(-roll)
        sr = math.sin(-roll)
        x3 = x2
        y3 = cr * y2 - sr * z2
        z3 = sr * y2 + cr * z2
        return (x3, y3, z3)

    def _intrinsics_for_frame(self, frame_w: int, frame_h: int) -> Tuple[float, float, float, float]:
        if (
            self._use_camera_info_intrinsics
            and self._cam_fx is not None
            and self._cam_fy is not None
            and self._cam_cx is not None
            and self._cam_cy is not None
        ):
            sx = 1.0
            sy = 1.0
            if self._cam_w and self._cam_w > 0:
                sx = float(frame_w) / float(self._cam_w)
            if self._cam_h and self._cam_h > 0:
                sy = float(frame_h) / float(self._cam_h)
            return (
                float(self._cam_fx) * sx,
                float(self._cam_fy) * sy,
                float(self._cam_cx) * sx,
                float(self._cam_cy) * sy,
            )

        fov = math.radians(self._clamp(self._fov_deg, 30.0, 140.0))
        fx = 0.5 * float(frame_w) / math.tan(0.5 * fov)
        fy = fx
        cx = 0.5 * float(frame_w)
        cy = 0.55 * float(frame_h)
        return (fx, fy, cx, cy)

    def _draw_pseudo_boxes(self, frame: "np.ndarray") -> int:
        ego = self._semantic_map.get("ego", {})
        objects = self._semantic_map.get("objects", [])
        if not isinstance(ego, dict) or not isinstance(objects, list):
            return 0
        if not objects:
            return 0

        ex = float(ego.get("x", 0.0))
        ey = float(ego.get("y", 0.0))
        yaw = float(ego.get("yaw", 0.0))

        h, w = frame.shape[:2]
        fx, fy, cx, cy = self._intrinsics_for_frame(w, h)
        fov_x = 2.0 * math.atan2(0.5 * float(w), max(1.0, fx))
        cam_h = max(0.1, self._camera_height_m)

        drawn = 0
        candidates: List[Tuple[float, Dict[str, Any]]] = []
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            ox = float(obj.get("x", 0.0))
            oy = float(obj.get("y", 0.0))
            dx = ox - ex
            dy = oy - ey
            # World -> ego-forward coordinates (x forward, y left)
            x_fwd = math.cos(yaw) * dx + math.sin(yaw) * dy
            y_left = -math.sin(yaw) * dx + math.cos(yaw) * dy
            z_up = float(obj.get("z", 0.0))
            # Shift from ego origin to camera origin.
            bx = x_fwd - self._camera_x_m
            by = y_left - self._camera_y_m
            bz = z_up - cam_h
            x_cam, y_cam, z_cam = self._body_to_camera_frame(bx, by, bz)
            if x_cam <= 0.6:
                continue
            # Strict horizontal FOV cull so projected boxes stay physically plausible.
            bearing = math.atan2(y_cam, x_cam)
            if abs(bearing) > 0.55 * fov_x:
                continue
            obj_copy = dict(obj)
            obj_copy["_viz_x_cam"] = x_cam
            obj_copy["_viz_y_cam"] = y_cam
            obj_copy["_viz_z_cam"] = z_cam
            obj_copy["_viz_bx"] = bx
            obj_copy["_viz_by"] = by
            obj_copy["_viz_bz"] = bz
            candidates.append((x_cam, obj_copy))

        candidates.sort(key=lambda it: it[0])

        for _, obj in candidates[: max(1, self._max_objects)]:
            x_cam = float(obj.get("_viz_x_cam", 0.0))
            y_cam = float(obj.get("_viz_y_cam", 0.0))
            z_cam = float(obj.get("_viz_z_cam", 0.0))
            bx = float(obj.get("_viz_bx", 0.0))
            by = float(obj.get("_viz_by", 0.0))
            bz = float(obj.get("_viz_bz", 0.0))
            name = str(obj.get("name", "obj"))
            dist = float(obj.get("distance", math.sqrt(x_cam * x_cam + y_cam * y_cam + z_cam * z_cam)))
            rr = float(obj.get("risk_radius", 2.0))
            is_dynamic = bool(obj.get("dynamic", False)) or name.lower().startswith("dyn_")

            u = cx - fx * (y_cam / max(0.5, x_cam))
            v_base = cy - fy * (z_cam / max(0.5, x_cam))
            obj_h = self._nominal_object_height(obj)
            x_top, y_top, z_top = self._body_to_camera_frame(bx, by, bz + obj_h)
            if x_top > 0.5:
                v_top = cy - fy * (z_top / max(0.5, x_top))
            else:
                v_top = v_base - 0.18 * fy / max(1.0, x_cam)
            v = 0.5 * (v_base + v_top)

            if u < -0.1 * w or u > 1.1 * w:
                continue

            base = 0.22 * fx / max(1.0, x_cam)
            if is_dynamic:
                pulse = 1.0 + 0.18 * math.sin(self.get_clock().now().nanoseconds * 1e-9 * 4.0)
                base *= pulse
            bh_proj = abs(v_base - v_top)
            bh = int(self._clamp(max(base * (0.78 + 0.08 * rr), bh_proj), 14.0, 128.0))
            bw = int(self._clamp(max(base * (1.0 + 0.08 * rr), 0.75 * bh), 18.0, 140.0))

            x1 = int(self._clamp(u - 0.5 * bw, 0.0, float(w - 1)))
            x2 = int(self._clamp(u + 0.5 * bw, 0.0, float(w - 1)))
            y1 = int(self._clamp(v - 0.5 * bh, 96.0, float(h - 1)))
            y2 = int(self._clamp(v + 0.5 * bh, 96.0, float(h - 1)))
            if x2 <= x1 or y2 <= y1:
                continue

            color = self._color_for_name(name)
            thickness = 3 if is_dynamic else 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            label = f"{name} {dist:.1f}m"
            if is_dynamic:
                label = f"DYN {label}"
            cv2.putText(
                frame,
                label,
                (x1, max(104, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
            drawn += 1

        return drawn

    @staticmethod
    def _color_for_name(name: str) -> Tuple[int, int, int]:
        n = name.lower()
        if n.startswith("dyn_") or "moving" in n or "dynamic" in n or "动态" in n:
            return (255, 80, 255)
        if "red" in n or "红" in n:
            return (40, 40, 255)
        if "green" in n or "绿" in n:
            return (40, 220, 40)
        if "dock" in n or "码头" in n:
            return (255, 180, 40)
        if "obstacle" in n or "block" in n or "障碍" in n:
            return (80, 160, 255)
        return (220, 220, 220)

    @staticmethod
    def _nominal_object_height(obj: Dict[str, Any]) -> float:
        obj_type = str(obj.get("type", "unknown")).strip().lower()
        if obj_type == "buoy" or obj_type == "gate_post":
            return 1.25
        if obj_type == "dock":
            return 1.6
        if "boat" in obj_type:
            return 1.4
        if "obstacle" in obj_type:
            return 1.2
        return 1.0


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VLMVisualizerNode()
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
