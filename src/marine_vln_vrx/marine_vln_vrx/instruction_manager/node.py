#!/usr/bin/env python3
"""Instruction manager: rule parser + strict VLM semantic parser with hybrid fallback."""

from __future__ import annotations

import json
import math
import os
import random
import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from marine_vln_vrx.common.geometry_utils import wrap_to_pi
from marine_vln_vrx.common.json_utils import parse_json_or_empty
from marine_vln_vrx.common.log_utils import JsonlLogger
from marine_vln_vrx.instruction_manager.semantic_schema import (
    SEMANTIC_GUIDED_JSON,
    default_semantic,
    normalize_semantic_payload,
    semantic_to_sequence,
    validate_semantic_payload,
)
from marine_vln_vrx.instruction_manager.grounding import (
    build_oracle_semantic,
    verify_semantic_with_map,
)
from marine_vln_vrx.instruction_manager.vlm_client import OpenAICompatibleVLMClient


ActionStep = Dict[str, Any]


class _SafeFormatDict(dict):
    def __missing__(self, key: str) -> str:
        return "{" + key + "}"


def _normalize_object_name(raw: str) -> str:
    text = raw.strip().lower().replace("-", "_").replace(" ", "_")
    return re.sub(r"[^a-zA-Z0-9_\u4e00-\u9fff]", "", text)


def _new_step(order: int, action: str, target: Any, spatial_relation: str) -> ActionStep:
    return {
        "order": order,
        "action": action,
        "target": target,
        "spatial_relation": spatial_relation,
    }


def _language_tag(text: str) -> str:
    return "zh" if re.search(r"[\u4e00-\u9fff]", text) else "en"


def parse_instruction_rules(text: str) -> Dict[str, Any]:
    """Parse Chinese/English instruction with a small ruleset."""
    source = text.strip()
    lowered = source.lower().strip()
    compact_zh = re.sub(r"[，。！？；、\s]", "", source)
    is_zh = bool(re.search(r"[\u4e00-\u9fff]", source))

    steps: List[ActionStep] = []

    match = re.fullmatch(r"go to waypoint ([a-z0-9_]+)", lowered)
    if match:
        steps.append(_new_step(1, "go_to", f"waypoint_{_normalize_object_name(match.group(1))}", "at"))

    if not steps:
        match = re.fullmatch(
            r"pass between (the )?([a-z0-9_ ]+) and (the )?([a-z0-9_ ]+) buoys?",
            lowered,
        )
        if match:
            left = _normalize_object_name(match.group(2))
            right = _normalize_object_name(match.group(4))
            steps.append(_new_step(1, "pass_between", {"left": f"{left}_buoy", "right": f"{right}_buoy"}, "between"))

    if not steps:
        match = re.fullmatch(r"circle around (the )?([a-z0-9_ ]+) (clockwise|counterclockwise)", lowered)
        if match:
            steps.append(_new_step(1, "circle", _normalize_object_name(match.group(2)), match.group(3)))

    if not steps:
        match = re.fullmatch(r"avoid (the )?([a-z0-9_ ]+) and stop near (the )?([a-z0-9_ ]+)", lowered)
        if match:
            steps.extend(
                [
                    _new_step(1, "avoid", _normalize_object_name(match.group(2)), "away_from"),
                    _new_step(2, "stop_near", _normalize_object_name(match.group(4)), "near"),
                ]
            )

    if not steps:
        match = re.fullmatch(
            r"first pass (the )?([a-z0-9_ ]+), then go to (the )?([a-z0-9_ ]+), finally stop",
            lowered,
        )
        if match:
            steps.extend(
                [
                    _new_step(1, "pass_between", _normalize_object_name(match.group(2)), "through"),
                    _new_step(2, "go_to", _normalize_object_name(match.group(4)), "at"),
                    _new_step(3, "stop_near", "current_position", "at"),
                ]
            )

    if not steps and compact_zh in {"去航点a", "前往航点a", "去a航点", "gotoa航点"}:
        steps.append(_new_step(1, "go_to", "waypoint_a", "at"))

    if not steps and all(k in compact_zh for k in ("前往", "红", "浮标")):
        steps.append(_new_step(1, "go_to", "red_buoy", "at"))

    if not steps and all(k in compact_zh for k in ("红", "浮标", "左侧")) and ("绕" in compact_zh or "过去" in compact_zh):
        steps.append(_new_step(1, "go_to", "red_buoy", "left_of"))

    if not steps and all(k in compact_zh for k in ("穿过", "红", "绿", "浮标")):
        steps.append(_new_step(1, "pass_between", {"left": "red_buoy", "right": "green_buoy"}, "between"))

    if not steps and all(k in compact_zh for k in ("航道", "码头")):
        steps.extend(
            [
                _new_step(1, "pass_between", {"left": "gate_left", "right": "gate_right"}, "between"),
                _new_step(2, "pass_between", {"left": "gate_left_mid", "right": "gate_right_mid"}, "between"),
                _new_step(3, "pass_between", {"left": "gate_left_far", "right": "gate_right_far"}, "between"),
                _new_step(4, "pass_between", {"left": "gate_left_final", "right": "gate_right_final"}, "between"),
                _new_step(5, "stop_near", "dock", "near"),
            ]
        )

    if not steps and ("小船" in compact_zh and ("观察" in compact_zh or "靠近" in compact_zh)):
        steps.append(_new_step(1, "stop_near", "small_boat", "near"))

    if not steps and all(k in compact_zh for k in ("绕", "海龟")):
        direction = "clockwise" if ("顺时针" in compact_zh or "顺" in compact_zh) else "counterclockwise"
        steps.append(_new_step(1, "circle", "turtle", direction))

    if not steps and all(k in compact_zh for k in ("避开", "鳄鱼", "停", "码头")):
        steps.extend([_new_step(1, "avoid", "crocodile", "away_from"), _new_step(2, "stop_near", "dock", "near")])

    if not steps and all(k in compact_zh for k in ("先", "通过", "再", "最后", "停")):
        steps.extend(
            [
                _new_step(1, "pass_between", "gate", "through"),
                _new_step(2, "go_to", "marker", "at"),
                _new_step(3, "stop_near", "current_position", "at"),
            ]
        )

    if not steps:
        parts = re.split(r", then | then | finally |, finally |，然后|然后|最后", lowered)
        parts = [p.strip() for p in parts if p.strip()]
        for idx, part in enumerate(parts):
            if "go to" in part:
                obj = _normalize_object_name(part.replace("go to", "").replace("the", ""))
                steps.append(_new_step(idx + 1, "go_to", obj, "at"))
            elif "left of" in part and "buoy" in part:
                obj = _normalize_object_name(part.replace("left of", "").replace("the", ""))
                steps.append(_new_step(idx + 1, "go_to", obj, "left_of"))
            elif "pass left of" in part:
                obj = _normalize_object_name(part.replace("pass left of", "").replace("the", ""))
                steps.append(_new_step(idx + 1, "go_to", obj, "left_of"))
            elif "avoid" in part:
                obj = _normalize_object_name(part.replace("avoid", "").replace("the", ""))
                steps.append(_new_step(idx + 1, "avoid", obj, "away_from"))
            elif "follow channel" in part:
                steps.append(_new_step(len(steps) + 1, "pass_between", {"left": "gate_left", "right": "gate_right"}, "between"))
                steps.append(_new_step(len(steps) + 1, "pass_between", {"left": "gate_left_mid", "right": "gate_right_mid"}, "between"))
                steps.append(_new_step(len(steps) + 1, "pass_between", {"left": "gate_left_far", "right": "gate_right_far"}, "between"))
                steps.append(
                    _new_step(len(steps) + 1, "pass_between", {"left": "gate_left_final", "right": "gate_right_final"}, "between")
                )
                if "dock" in lowered or "dock" in part:
                    steps.append(_new_step(len(steps) + 1, "stop_near", "dock", "near"))
            elif "inspect" in part and "boat" in part:
                steps.append(_new_step(idx + 1, "stop_near", "small_boat", "near"))
            elif "stop" in part:
                steps.append(_new_step(idx + 1, "stop_near", "current_position", "at"))

    if not steps:
        steps.append(_new_step(1, "stop_near", "current_position", "at"))

    return {
        "instruction": source,
        "language": "zh" if is_zh else "en",
        "parser": "rules_v1",
        "sequence": steps,
    }


def _rule_to_semantic(rule_task: Dict[str, Any]) -> Dict[str, Any]:
    seq = rule_task.get("sequence", [])
    if not isinstance(seq, list) or not seq:
        return default_semantic("rule_empty")
    step = seq[0] if isinstance(seq[0], dict) else {}
    action = str(step.get("action", ""))
    target = step.get("target")
    relation = str(step.get("spatial_relation", "")).strip().lower()

    if action == "go_to":
        intent = "goto"
        direction_hint = "toward"
        if relation in {"left_of", "pass_left_of"}:
            intent = "pass_left_of"
            direction_hint = "left"
        elif relation in {"right_of", "pass_right_of"}:
            intent = "pass_right_of"
            direction_hint = "right"
        return {
            **default_semantic("from_rules"),
            "intent": intent,
            "target_id": str(target) if isinstance(target, str) else None,
            "target_type": "unknown",
            "direction_hint": direction_hint,
            "confidence": 0.7,
            "use_rule_fallback": False,
            "brief_reason": "parsed by rules",
        }
    if action == "pass_between" and isinstance(target, dict):
        return {
            **default_semantic("from_rules"),
            "intent": "go_between",
            "target_id": str(target.get("left")) if target.get("left") else None,
            "secondary_target_id": str(target.get("right")) if target.get("right") else None,
            "target_type": "gate",
            "direction_hint": "between",
            "confidence": 0.7,
            "use_rule_fallback": False,
            "brief_reason": "parsed by rules",
        }
    if action == "stop_near":
        target_id = str(target) if isinstance(target, str) else None
        intent = "inspect" if target_id and target_id != "current_position" else "stop"
        return {
            **default_semantic("from_rules"),
            "intent": intent,
            "target_id": target_id,
            "target_type": "unknown",
            "direction_hint": "toward" if intent == "inspect" else "none",
            "confidence": 0.7,
            "use_rule_fallback": False,
            "brief_reason": "parsed by rules",
        }

    return default_semantic("rule_no_semantic_mapping")


class InstructionManagerNode(Node):
    """Receive instruction and publish structured task JSON."""

    def __init__(self) -> None:
        super().__init__("instruction_manager")

        pkg_share = Path(get_package_share_directory("marine_vln_vrx"))
        default_system_prompt = str(pkg_share / "config" / "prompts" / "vlm_system_prompt.txt")
        default_user_prompt = str(pkg_share / "config" / "prompts" / "vlm_user_prompt.txt")

        self.declare_parameter("instruction_topic", "/vln/instruction_text")
        self.declare_parameter("task_topic", "/vln/task_json")
        self.declare_parameter("semantic_map_topic", "/vln/semantic_map")
        self.declare_parameter("parser_mode", "hybrid")

        self.declare_parameter("include_scene_context", True)
        self.declare_parameter("use_vlm_image", True)
        self.declare_parameter("camera_compressed_topic", "/wamv/sensors/cameras/front_camera_sensor/optical/image_raw/compressed")
        self.declare_parameter("vlm_api_base", "http://127.0.0.1:8000/v1")
        self.declare_parameter("vlm_model", "Qwen/Qwen2.5-VL-7B-Instruct-AWQ")
        self.declare_parameter("vlm_api_key_env", "OPENAI_API_KEY")
        self.declare_parameter("vlm_timeout_ms", 8000)
        self.declare_parameter("vlm_max_retries", 1)
        self.declare_parameter("vlm_confidence_threshold", 0.55)
        self.declare_parameter("allow_model_rule_fallback", True)
        self.declare_parameter("vlm_temperature", 0.1)
        self.declare_parameter("vlm_max_tokens", 256)
        self.declare_parameter("vlm_media_dir", "/tmp/marine_vln_vlm_media")
        self.declare_parameter("vlm_enable_guided_json", True)
        self.declare_parameter("vlm_save_debug_image", True)
        self.declare_parameter("vlm_log_raw_response", True)

        self.declare_parameter("vlm_system_prompt_file", default_system_prompt)
        self.declare_parameter("vlm_user_prompt_file", default_user_prompt)
        self.declare_parameter("vlm_max_candidates", 12)
        self.declare_parameter("vlm_candidate_max_distance_m", 120.0)
        self.declare_parameter("method_name", "hybrid_current")
        self.declare_parameter("use_hybrid_fallback", True)
        self.declare_parameter("use_semantic_map_authority", True)
        self.declare_parameter("use_grounding_verifier", True)
        self.declare_parameter("grounding_score_threshold", 0.40)
        self.declare_parameter("use_topk_hypothesis", True)
        self.declare_parameter("topk_hypothesis_count", 3)
        self.declare_parameter("use_oracle_parser", False)
        self.declare_parameter("benchmark_mode", False)
        self.declare_parameter("benchmark_vlm_drop_rate", 0.0)
        self.declare_parameter("benchmark_seed", 12345)

        self.declare_parameter("log_root", "/tmp/marine_vln_logs")

        instruction_topic = str(self.get_parameter("instruction_topic").value)
        task_topic = str(self.get_parameter("task_topic").value)
        semantic_map_topic = str(self.get_parameter("semantic_map_topic").value)
        parser_mode = str(self.get_parameter("parser_mode").value).strip().lower()
        if parser_mode not in {"rules", "vlm", "hybrid"}:
            self.get_logger().warn(f"Unknown parser_mode={parser_mode}, fallback to hybrid")
            parser_mode = "hybrid"
        self._parser_mode = parser_mode

        self._include_scene = bool(self.get_parameter("include_scene_context").value)
        self._use_vlm_image = bool(self.get_parameter("use_vlm_image").value)
        camera_compressed_topic = str(self.get_parameter("camera_compressed_topic").value)

        self._vlm_api_base = str(self.get_parameter("vlm_api_base").value).strip()
        self._vlm_model = str(self.get_parameter("vlm_model").value).strip()
        self._vlm_api_key_env = str(self.get_parameter("vlm_api_key_env").value).strip()
        self._vlm_timeout_ms = int(self.get_parameter("vlm_timeout_ms").value)
        self._vlm_max_retries = int(self.get_parameter("vlm_max_retries").value)
        self._vlm_confidence_threshold = float(self.get_parameter("vlm_confidence_threshold").value)
        self._allow_model_rule_fallback = bool(self.get_parameter("allow_model_rule_fallback").value)
        self._vlm_temperature = float(self.get_parameter("vlm_temperature").value)
        self._vlm_max_tokens = int(self.get_parameter("vlm_max_tokens").value)
        self._vlm_media_dir = Path(str(self.get_parameter("vlm_media_dir").value)).expanduser().resolve()
        self._vlm_enable_guided_json = bool(self.get_parameter("vlm_enable_guided_json").value)
        self._vlm_save_debug_image = bool(self.get_parameter("vlm_save_debug_image").value)
        self._vlm_log_raw_response = bool(self.get_parameter("vlm_log_raw_response").value)

        self._system_prompt_file = Path(str(self.get_parameter("vlm_system_prompt_file").value)).expanduser().resolve()
        self._user_prompt_file = Path(str(self.get_parameter("vlm_user_prompt_file").value)).expanduser().resolve()
        self._max_candidates = max(1, int(self.get_parameter("vlm_max_candidates").value))
        self._max_candidate_dist = float(self.get_parameter("vlm_candidate_max_distance_m").value)
        self._method_name = str(self.get_parameter("method_name").value).strip()
        self._use_hybrid_fallback = bool(self.get_parameter("use_hybrid_fallback").value)
        self._use_semantic_map_authority = bool(self.get_parameter("use_semantic_map_authority").value)
        self._use_grounding_verifier = bool(self.get_parameter("use_grounding_verifier").value)
        self._grounding_score_threshold = float(self.get_parameter("grounding_score_threshold").value)
        self._use_topk_hypothesis = bool(self.get_parameter("use_topk_hypothesis").value)
        self._topk_hypothesis_count = max(1, int(self.get_parameter("topk_hypothesis_count").value))
        self._use_oracle_parser = bool(self.get_parameter("use_oracle_parser").value)
        self._benchmark_mode = bool(self.get_parameter("benchmark_mode").value)
        self._benchmark_vlm_drop_rate = float(self.get_parameter("benchmark_vlm_drop_rate").value)
        self._rng = random.Random(int(self.get_parameter("benchmark_seed").value))

        log_root = str(self.get_parameter("log_root").value)

        self._logger_file = JsonlLogger(self, "instruction_manager", log_root)
        self._task_pub = self.create_publisher(String, task_topic, 10)
        self.create_subscription(String, instruction_topic, self._instruction_callback, 10)

        self._semantic_map: Dict[str, Any] = {}
        if self._include_scene or self._parser_mode in {"vlm", "hybrid"}:
            self.create_subscription(String, semantic_map_topic, self._semantic_map_callback, 20)

        self._latest_image_bytes: Optional[bytes] = None
        self._latest_image_ext = ".jpg"
        self._latest_image_stamp_ns: Optional[int] = None
        if self._use_vlm_image:
            self.create_subscription(CompressedImage, camera_compressed_topic, self._image_callback, 5)

        self._system_prompt_template = self._load_prompt_template(
            self._system_prompt_file,
            fallback=(
                "You are a deterministic maritime VLN parser for VRX.\n"
                "Output exactly one JSON object, no markdown, no extra words."
            ),
        )
        self._user_prompt_template = self._load_prompt_template(
            self._user_prompt_file,
            fallback=(
                "Instruction:\n{instruction_text}\n\n"
                "Ego state:\n- position_xy: {ego_xy}\n- heading_deg: {ego_heading_deg}\n- speed_mps: {ego_speed_mps}\n\n"
                "Semantic map candidate objects:\n{candidate_objects_json}\n\n"
                "Current local navigation context:\n{local_context_text}\n\n"
                "Output exactly one JSON object matching the required schema."
            ),
        )

        self._vlm_media_dir.mkdir(parents=True, exist_ok=True)

        self._vlm_client: Optional[OpenAICompatibleVLMClient] = None
        if self._parser_mode in {"vlm", "hybrid"}:
            api_key = os.environ.get(self._vlm_api_key_env, "")
            self._vlm_client = OpenAICompatibleVLMClient(
                api_base=self._vlm_api_base,
                model=self._vlm_model,
                api_key=api_key,
                timeout_ms=self._vlm_timeout_ms,
                max_retries=self._vlm_max_retries,
                temperature=self._vlm_temperature,
                max_tokens=self._vlm_max_tokens,
            )

        self.get_logger().info(
            f"instruction_manager started: mode={self._parser_mode}, vlm_model={self._vlm_model}, "
            f"vlm_base={self._vlm_api_base}, media_dir={self._vlm_media_dir}, "
            f"use_vlm_image={self._use_vlm_image}, method={self._method_name}, "
            f"allow_model_rule_fallback={self._allow_model_rule_fallback}, "
            f"grounding_verifier={self._use_grounding_verifier}, topk={self._topk_hypothesis_count}, "
            f"benchmark_mode={self._benchmark_mode}, vlm_drop_rate={self._benchmark_vlm_drop_rate:.2f}, "
            f"log={self._logger_file.log_path}"
        )

    def _load_prompt_template(self, path: Path, fallback: str) -> str:
        try:
            if path.is_file():
                return path.read_text(encoding="utf-8")
            self.get_logger().warn(f"Prompt file not found: {path}, fallback to built-in template")
        except OSError as exc:
            self.get_logger().warn(f"Failed to read prompt file {path}: {exc}")
        return fallback

    def _semantic_map_callback(self, msg: String) -> None:
        data = parse_json_or_empty(self, msg.data, "semantic_map_topic")
        if data:
            self._semantic_map = data

    def _image_callback(self, msg: CompressedImage) -> None:
        raw = bytes(msg.data)
        if not raw:
            return
        fmt = (msg.format or "").lower()
        ext = ".png" if "png" in fmt else ".jpg"
        self._latest_image_bytes = raw
        self._latest_image_ext = ext
        self._latest_image_stamp_ns = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)

    def _ego_state(self) -> Dict[str, Any]:
        ego = self._semantic_map.get("ego", {})
        if not isinstance(ego, dict):
            ego = {}
        return {
            "x": float(ego.get("x", 0.0)),
            "y": float(ego.get("y", 0.0)),
            "yaw": float(ego.get("yaw", 0.0)),
            "speed": float(ego.get("speed", 0.0)),
        }

    def _candidate_objects(self, instruction_text: str = "") -> List[Dict[str, Any]]:
        raw = self._semantic_map.get("objects", [])
        if not isinstance(raw, list):
            return []

        objs: List[Dict[str, Any]] = []
        for obj in raw:
            if not isinstance(obj, dict):
                continue
            dist = float(obj.get("distance", 1e9))
            if dist > self._max_candidate_dist:
                continue
            name = str(obj.get("name", "")).strip()
            if not name:
                continue
            entry = {
                "id": name,
                "name": name,
                "type": str(obj.get("type", "unknown")),
                "x": float(obj.get("x", 0.0)),
                "y": float(obj.get("y", 0.0)),
                "distance": dist,
                "risk_radius": float(obj.get("risk_radius", 2.0)),
                "aliases": obj.get("aliases", []),
            }
            objs.append(entry)

        objs.sort(key=lambda o: float(o.get("distance", 1e9)))

        # When instruction explicitly asks about buoys, prefer buoy ids and
        # suppress colocated gate_post aliases to reduce ambiguity for VLM.
        lower_text = instruction_text.lower()
        prefer_buoy = ("buoy" in lower_text) or ("浮标" in instruction_text)
        if prefer_buoy:
            buoy_centers = [
                (float(o.get("x", 0.0)), float(o.get("y", 0.0)))
                for o in objs
                if str(o.get("type", "")) == "buoy"
            ]
            filtered: List[Dict[str, Any]] = []
            for o in objs:
                if str(o.get("type", "")) == "gate_post":
                    ox = float(o.get("x", 0.0))
                    oy = float(o.get("y", 0.0))
                    colocated = any(math.hypot(ox - bx, oy - by) < 0.8 for bx, by in buoy_centers)
                    if colocated:
                        continue
                filtered.append(o)
            objs = filtered

        # Keep semantic grounding stable for long-range tasks:
        # 1) always prioritize objects explicitly mentioned in instruction
        # 2) fill remaining slots with nearest objects
        text_lower = instruction_text.lower()
        forced: List[Dict[str, Any]] = []
        for obj in objs:
            oid = str(obj.get("id", "")).strip()
            if not oid:
                continue
            hit = False
            if oid.lower() in text_lower:
                hit = True
            if not hit:
                aliases = obj.get("aliases", [])
                if isinstance(aliases, list):
                    for alias in aliases:
                        if not isinstance(alias, str):
                            continue
                        token = alias.strip()
                        if not token:
                            continue
                        if re.search(r"[\u4e00-\u9fff]", token):
                            if token in instruction_text:
                                hit = True
                                break
                        else:
                            if token.lower() in text_lower:
                                hit = True
                                break
            if hit:
                forced.append(obj)

        selected: List[Dict[str, Any]] = []
        seen: set[str] = set()
        for obj in forced + objs:
            oid = str(obj.get("id", "")).strip()
            if not oid or oid in seen:
                continue
            selected.append(obj)
            seen.add(oid)
            if len(selected) >= self._max_candidates:
                break
        return selected

    def _bearing_relation(self, ego: Dict[str, Any], obj: Dict[str, Any]) -> str:
        dx = float(obj.get("x", 0.0)) - float(ego.get("x", 0.0))
        dy = float(obj.get("y", 0.0)) - float(ego.get("y", 0.0))
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return "front"
        rel = wrap_to_pi(math.atan2(dy, dx) - float(ego.get("yaw", 0.0)))
        if abs(rel) > math.radians(150.0):
            return "behind"
        if abs(rel) < math.radians(30.0):
            return "front"
        return "left" if rel > 0.0 else "right"

    def _local_context_text(self, ego: Dict[str, Any], candidates: List[Dict[str, Any]]) -> str:
        if not candidates:
            return "No nearby semantic objects."

        lines: List[str] = []
        nearest = candidates[0]
        lines.append(
            f"nearest_object={nearest['id']} ({nearest['type']}), dist={float(nearest['distance']):.2f}m"
        )

        channel_like = [o for o in candidates if str(o.get("type", "")) in {"channel", "gate_post", "buoy"}]
        if channel_like:
            lines.append(f"channel_context_objects={len(channel_like)}")

        for obj in candidates[:6]:
            rel = self._bearing_relation(ego, obj)
            lines.append(
                f"object={obj['id']}, type={obj['type']}, rel={rel}, dist={float(obj['distance']):.2f}"
            )
        return "\n".join(lines)

    def _save_image_for_vlm(self) -> Tuple[Optional[str], Optional[str]]:
        if not self._use_vlm_image:
            return (None, None)
        if self._latest_image_bytes is None:
            return (None, None)

        stamp_ns = self.get_clock().now().nanoseconds
        image_name = f"frame_{stamp_ns}{self._latest_image_ext}"
        path = self._vlm_media_dir / image_name
        try:
            path.write_bytes(self._latest_image_bytes)
            return (str(path), None if self._vlm_save_debug_image else str(path))
        except OSError as exc:
            self.get_logger().warn(f"Failed to save VLM image {path}: {exc}")
            return (None, None)

    def _build_prompts(
        self,
        instruction_text: str,
        ego_state: Dict[str, Any],
        candidates: List[Dict[str, Any]],
        local_context_text: str,
        image_available: bool,
    ) -> Tuple[str, str]:
        values = {
            "instruction_text": instruction_text,
            "ego_xy": [round(float(ego_state["x"]), 3), round(float(ego_state["y"]), 3)],
            "ego_heading_deg": round(float(ego_state["yaw"]) * 180.0 / math.pi, 3),
            "ego_speed_mps": round(float(ego_state["speed"]), 3),
            "candidate_objects_json": json.dumps(candidates, ensure_ascii=False),
            "local_context_text": local_context_text,
            "image_status": "available" if image_available else "unavailable",
        }
        system_prompt = self._system_prompt_template.format_map(_SafeFormatDict(values))
        user_prompt = self._user_prompt_template.format_map(_SafeFormatDict(values))
        return (system_prompt, user_prompt)

    def _invoke_vlm(
        self,
        instruction_text: str,
        ego_state: Dict[str, Any],
        candidates: List[Dict[str, Any]],
        local_context_text: str,
    ) -> Dict[str, Any]:
        if self._vlm_client is None:
            return {
                "ok": False,
                "fallback_reason": "vlm_client_unavailable",
                "semantic": default_semantic("vlm_client_unavailable"),
                "latency_ms": 0.0,
                "json_valid": False,
                "raw_response": "",
                "response_text": "",
                "image_path": None,
            }

        if self._benchmark_vlm_drop_rate > 0.0 and self._rng.random() < self._benchmark_vlm_drop_rate:
            return {
                "ok": False,
                "fallback_reason": "vlm_request_failed:simulated_drop",
                "semantic": default_semantic("simulated_vlm_drop"),
                "latency_ms": 0.0,
                "json_valid": False,
                "raw_response": "",
                "response_text": "",
                "image_path": None,
            }

        image_path, cleanup_path = self._save_image_for_vlm()
        system_prompt, user_prompt = self._build_prompts(
            instruction_text,
            ego_state,
            candidates,
            local_context_text,
            image_available=bool(image_path),
        )

        result = self._vlm_client.request_json(
            system_prompt=system_prompt,
            user_prompt=user_prompt,
            image_path=image_path,
            guided_json_schema=SEMANTIC_GUIDED_JSON,
            enable_guided_json=self._vlm_enable_guided_json,
        )

        # If requested model is not served (HTTP 404), auto-discover the served model and retry once.
        if (not bool(result.get("ok", False))) and str(result.get("error", "")) == "http_404":
            served_model = self._vlm_client.discover_first_model()
            if served_model and served_model != self._vlm_model:
                prev_model = self._vlm_model
                self._vlm_model = served_model
                self._vlm_client.set_model(served_model)
                self.get_logger().warn(
                    f"VLM model mismatch detected (requested={prev_model}, served={served_model}). "
                    "Auto-switching to served model and retrying once."
                )
                retry_result = self._vlm_client.request_json(
                    system_prompt=system_prompt,
                    user_prompt=user_prompt,
                    image_path=image_path,
                    guided_json_schema=SEMANTIC_GUIDED_JSON,
                    enable_guided_json=self._vlm_enable_guided_json,
                )
                if bool(retry_result.get("ok", False)):
                    result = retry_result
                else:
                    retry_err = str(retry_result.get("error", "unknown"))
                    retry_result["error"] = f"http_404_auto_switch_failed:{retry_err}"
                    result = retry_result

        if cleanup_path:
            try:
                Path(cleanup_path).unlink()
            except FileNotFoundError:
                pass
            except OSError:
                pass

        if not result.get("ok", False):
            return {
                "ok": False,
                "fallback_reason": f"vlm_request_failed:{result.get('error', 'unknown')}",
                "semantic": default_semantic(str(result.get("error", "vlm_request_failed"))),
                "latency_ms": float(result.get("latency_ms", 0.0)),
                "json_valid": False,
                "raw_response": str(result.get("raw_response", "")),
                "response_text": str(result.get("response_text", "")),
                "image_path": image_path,
            }

        parsed = result.get("parsed_json")
        if not isinstance(parsed, dict):
            return {
                "ok": False,
                "fallback_reason": "invalid_json_object",
                "semantic": default_semantic("invalid_json_object"),
                "latency_ms": float(result.get("latency_ms", 0.0)),
                "json_valid": False,
                "raw_response": str(result.get("raw_response", "")),
                "response_text": str(result.get("response_text", "")),
                "image_path": image_path,
            }

        semantic = normalize_semantic_payload(parsed)
        candidate_ids = [str(c["id"]) for c in candidates]
        valid, errors = validate_semantic_payload(semantic, candidate_ids)
        if not valid:
            return {
                "ok": False,
                "fallback_reason": "schema_validation_failed:" + ",".join(errors),
                "semantic": semantic,
                "latency_ms": float(result.get("latency_ms", 0.0)),
                "json_valid": False,
                "raw_response": str(result.get("raw_response", "")),
                "response_text": str(result.get("response_text", "")),
                "image_path": image_path,
            }

        if bool(semantic.get("use_rule_fallback", False)):
            if self._allow_model_rule_fallback:
                return {
                    "ok": False,
                    "fallback_reason": "use_rule_fallback_true",
                    "semantic": semantic,
                    "latency_ms": float(result.get("latency_ms", 0.0)),
                    "json_valid": True,
                    "raw_response": str(result.get("raw_response", "")),
                    "response_text": str(result.get("response_text", "")),
                    "image_path": image_path,
                }
            # Keep this semantic when benchmarking VLM routing; avoid hard downgrade to rules.
            semantic["use_rule_fallback"] = False

        conf = float(semantic.get("confidence", 0.0))
        if conf < self._vlm_confidence_threshold:
            return {
                "ok": False,
                "fallback_reason": f"low_confidence:{conf:.3f}",
                "semantic": semantic,
                "latency_ms": float(result.get("latency_ms", 0.0)),
                "json_valid": True,
                "raw_response": str(result.get("raw_response", "")),
                "response_text": str(result.get("response_text", "")),
                "image_path": image_path,
            }

        if str(semantic.get("intent", "")) == "unknown":
            return {
                "ok": False,
                "fallback_reason": "intent_unknown",
                "semantic": semantic,
                "latency_ms": float(result.get("latency_ms", 0.0)),
                "json_valid": True,
                "raw_response": str(result.get("raw_response", "")),
                "response_text": str(result.get("response_text", "")),
                "image_path": image_path,
            }

        return {
            "ok": True,
            "fallback_reason": "",
            "semantic": semantic,
            "latency_ms": float(result.get("latency_ms", 0.0)),
            "json_valid": True,
            "raw_response": str(result.get("raw_response", "")),
            "response_text": str(result.get("response_text", "")),
            "image_path": image_path,
        }

    def _candidate_by_id(self, candidates: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
        out: Dict[str, Dict[str, Any]] = {}
        for c in candidates:
            oid = str(c.get("id", "")).strip()
            if oid:
                out[oid] = c
        return out

    def _semantic_goal_xy(self, semantic: Dict[str, Any], candidates_by_id: Dict[str, Dict[str, Any]]) -> Optional[Tuple[float, float]]:
        intent = str(semantic.get("intent", "unknown")).strip().lower()
        target_id = semantic.get("target_id")
        secondary_id = semantic.get("secondary_target_id")
        if intent == "go_between" and isinstance(target_id, str) and isinstance(secondary_id, str):
            a = candidates_by_id.get(target_id)
            b = candidates_by_id.get(secondary_id)
            if a is None or b is None:
                return None
            return (
                0.5 * (float(a.get("x", 0.0)) + float(b.get("x", 0.0))),
                0.5 * (float(a.get("y", 0.0)) + float(b.get("y", 0.0))),
            )
        if isinstance(target_id, str):
            obj = candidates_by_id.get(target_id)
            if obj is not None:
                return (float(obj.get("x", 0.0)), float(obj.get("y", 0.0)))
        return None

    def _nearest_obstacle_to_goal(
        self,
        gx: float,
        gy: float,
        candidates: List[Dict[str, Any]],
    ) -> float:
        nearest = float("inf")
        for c in candidates:
            ctype = str(c.get("type", "unknown")).strip().lower()
            if ctype in {"buoy", "gate_post", "waypoint", "marker"}:
                continue
            d = math.hypot(float(c.get("x", 0.0)) - gx, float(c.get("y", 0.0)) - gy) - float(c.get("risk_radius", 2.0))
            if d < nearest:
                nearest = d
        return nearest

    def _score_hypothesis(
        self,
        semantic: Dict[str, Any],
        grounding_score: float,
        ego_state: Dict[str, Any],
        candidates: List[Dict[str, Any]],
    ) -> Dict[str, float]:
        c_by_id = self._candidate_by_id(candidates)
        goal_xy = self._semantic_goal_xy(semantic, c_by_id)
        if goal_xy is None:
            return {
                "grounding": float(grounding_score),
                "feasibility": 0.05,
                "collision_risk": 0.95,
                "detour_cost": 1.0,
                "progress": 0.0,
                "total": 0.1 * float(grounding_score),
            }

        ex = float(ego_state.get("x", 0.0))
        ey = float(ego_state.get("y", 0.0))
        eyaw = float(ego_state.get("yaw", 0.0))
        dx = float(goal_xy[0]) - ex
        dy = float(goal_xy[1]) - ey
        dist = max(0.0, math.hypot(dx, dy))

        feasibility = 1.0 / (1.0 + dist / 120.0)
        detour_cost = min(1.0, dist / 220.0)
        nearest_clear = self._nearest_obstacle_to_goal(goal_xy[0], goal_xy[1], candidates)
        collision_risk = 1.0 / (1.0 + max(0.0, nearest_clear))
        bearing = wrap_to_pi(math.atan2(dy, dx) - eyaw)
        progress = max(0.0, 1.0 - abs(bearing) / math.pi)

        total = (
            0.42 * float(grounding_score)
            + 0.22 * feasibility
            + 0.16 * (1.0 - collision_risk)
            + 0.10 * (1.0 - detour_cost)
            + 0.10 * progress
        )
        return {
            "grounding": float(grounding_score),
            "feasibility": float(feasibility),
            "collision_risk": float(collision_risk),
            "detour_cost": float(detour_cost),
            "progress": float(progress),
            "total": float(total),
        }

    def _build_hypotheses_from_semantic(
        self,
        semantic: Dict[str, Any],
        instruction_text: str,
        ego_state: Dict[str, Any],
        candidates: List[Dict[str, Any]],
        grounding_score: float,
    ) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
        topk = max(1, self._topk_hypothesis_count if self._use_topk_hypothesis else 1)
        verified = verify_semantic_with_map(
            semantic=semantic,
            instruction_text=instruction_text,
            ego_state=ego_state,
            candidates=candidates,
            topk=topk,
        )
        hypotheses_raw = verified.get("hypotheses", [])
        if not isinstance(hypotheses_raw, list):
            hypotheses_raw = []

        hypotheses_out: List[Dict[str, Any]] = []
        for idx, h in enumerate(hypotheses_raw[:topk]):
            if not isinstance(h, dict):
                continue
            sem = h.get("semantic_parse")
            if not isinstance(sem, dict):
                continue
            h_ground = float(h.get("grounding_score", grounding_score))
            score = self._score_hypothesis(sem, h_ground, ego_state, candidates)
            seq = semantic_to_sequence(sem)
            seq = self._expand_sequence_for_follow_channel(
                instruction_text=instruction_text,
                semantic_parse=sem,
                base_sequence=seq,
            )
            hypotheses_out.append(
                {
                    "id": str(h.get("id", f"h{idx}")),
                    "semantic_parse": sem,
                    "sequence": seq,
                    "grounding_score": h_ground,
                    "map_match_score": float(h.get("map_match_score", verified.get("map_match_score", 0.0))),
                    "relation_score": float(h.get("relation_score", verified.get("relation_score", 0.0))),
                    "attribute_score": float(h.get("attribute_score", verified.get("attribute_score", 0.0))),
                    "score": score,
                }
            )

        if not hypotheses_out:
            seq = semantic_to_sequence(semantic)
            seq = self._expand_sequence_for_follow_channel(
                instruction_text=instruction_text,
                semantic_parse=semantic,
                base_sequence=seq,
            )
            hypotheses_out.append(
                {
                    "id": "h0",
                    "semantic_parse": semantic,
                    "sequence": seq,
                    "grounding_score": grounding_score,
                    "map_match_score": float(verified.get("map_match_score", 0.0)),
                    "relation_score": float(verified.get("relation_score", 0.0)),
                    "attribute_score": float(verified.get("attribute_score", 0.0)),
                    "score": self._score_hypothesis(semantic, grounding_score, ego_state, candidates),
                }
            )

        hypotheses_out.sort(
            key=lambda x: float((x.get("score") or {}).get("total", 0.0)),
            reverse=True,
        )
        return hypotheses_out, verified

    def _instruction_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            self.get_logger().warn("Received empty instruction, ignored.")
            return

        ego_state = self._ego_state()
        candidates = self._candidate_objects(text)
        local_context_text = self._local_context_text(ego_state, candidates)

        rule_task = parse_instruction_rules(text)
        rule_semantic = _rule_to_semantic(rule_task)

        route = "rules"
        fallback_reason = ""
        parser_latency_ms = 0.0
        vlm_json_valid = False
        semantic_parse = rule_semantic
        sequence = list(rule_task.get("sequence", []))
        raw_response = ""
        response_text = ""
        image_path = None
        raw_vlm_json: Dict[str, Any] = {}
        verified_json: Dict[str, Any] = dict(rule_semantic)
        grounding_score = 1.0
        map_match_score = 1.0
        relation_score = 1.0
        attribute_score = 1.0
        hypotheses: List[Dict[str, Any]] = []
        active_hypothesis_id = "h0"
        hypothesis_switch_reason = "none"

        if self._use_oracle_parser:
            oracle_sem = build_oracle_semantic(text, candidates, default_intent="goto")
            semantic_parse = oracle_sem
            route = "oracle"
            sequence = semantic_to_sequence(semantic_parse)
            sequence = self._expand_sequence_for_follow_channel(
                instruction_text=text,
                semantic_parse=semantic_parse,
                base_sequence=sequence,
            )
            hypotheses = [
                {
                    "id": "h0",
                    "semantic_parse": semantic_parse,
                    "sequence": sequence,
                    "grounding_score": 1.0,
                    "map_match_score": 1.0,
                    "relation_score": 1.0,
                    "attribute_score": 1.0,
                    "score": self._score_hypothesis(semantic_parse, 1.0, ego_state, candidates),
                }
            ]

        if (not self._use_oracle_parser) and self._parser_mode in {"vlm", "hybrid"}:
            vlm_result = self._invoke_vlm(text, ego_state, candidates, local_context_text)
            parser_latency_ms = float(vlm_result.get("latency_ms", 0.0))
            fallback_reason = str(vlm_result.get("fallback_reason", ""))
            vlm_json_valid = bool(vlm_result.get("json_valid", False))
            semantic_parse = dict(vlm_result.get("semantic", default_semantic("vlm_result_empty")))
            raw_response = str(vlm_result.get("raw_response", ""))
            response_text = str(vlm_result.get("response_text", ""))
            image_path = vlm_result.get("image_path")
            raw_vlm_json = dict(semantic_parse)

            if bool(vlm_result.get("ok", False)):
                route = "vlm"
                if self._use_grounding_verifier and self._use_semantic_map_authority:
                    hypotheses, verified = self._build_hypotheses_from_semantic(
                        semantic=semantic_parse,
                        instruction_text=text,
                        ego_state=ego_state,
                        candidates=candidates,
                        grounding_score=0.0,
                    )
                    best = hypotheses[0]
                    semantic_parse = dict(best.get("semantic_parse", semantic_parse))
                    sequence = list(best.get("sequence", semantic_to_sequence(semantic_parse)))
                    active_hypothesis_id = str(best.get("id", "h0"))
                    grounding_score = float(best.get("grounding_score", 0.0))
                    map_match_score = float(best.get("map_match_score", 0.0))
                    relation_score = float(best.get("relation_score", 0.0))
                    attribute_score = float(best.get("attribute_score", 0.0))
                    verified_json = dict(semantic_parse)
                    if grounding_score < self._grounding_score_threshold:
                        fallback_reason = f"grounding_low_score:{grounding_score:.3f}"
                        if self._parser_mode == "hybrid" and self._use_hybrid_fallback:
                            route = "rules_fallback"
                            semantic_parse = rule_semantic
                            sequence = list(rule_task.get("sequence", []))
                            hypotheses = []
                            active_hypothesis_id = "h0"
                            hypothesis_switch_reason = "grounding_fallback"
                        else:
                            route = "safe_stop"
                            semantic_parse = default_semantic(fallback_reason)
                            sequence = semantic_to_sequence(semantic_parse)
                    else:
                        route = "vlm"
                else:
                    sequence = semantic_to_sequence(semantic_parse)
                    sequence = self._expand_sequence_for_follow_channel(
                        instruction_text=text,
                        semantic_parse=semantic_parse,
                        base_sequence=sequence,
                    )
                    verified_json = dict(semantic_parse)
                    grounding_score = float(semantic_parse.get("confidence", 0.0))
            else:
                if self._parser_mode == "hybrid" and self._use_hybrid_fallback:
                    route = "rules_fallback"
                    semantic_parse = rule_semantic
                    sequence = list(rule_task.get("sequence", []))
                else:
                    route = "safe_stop"
                    semantic_parse = default_semantic(fallback_reason or "vlm_failed")
                    sequence = semantic_to_sequence(semantic_parse)
                verified_json = dict(semantic_parse)

        if (not self._use_oracle_parser) and self._parser_mode == "rules":
            verified_json = dict(semantic_parse)

        if not isinstance(sequence, list) or not sequence:
            sequence = [_new_step(1, "stop_near", "current_position", "at")]

        if not hypotheses:
            hypotheses = [
                {
                    "id": "h0",
                    "semantic_parse": semantic_parse,
                    "sequence": sequence,
                    "grounding_score": grounding_score,
                    "map_match_score": map_match_score,
                    "relation_score": relation_score,
                    "attribute_score": attribute_score,
                    "score": self._score_hypothesis(semantic_parse, grounding_score, ego_state, candidates),
                }
            ]
            active_hypothesis_id = "h0"

        task_out = {
            "instruction": text,
            "language": _language_tag(text),
            "parser": "vlm_v1" if route == "vlm" else ("rules_v1" if route == "rules" else route),
            "method_name": self._method_name,
            "benchmark_mode": self._benchmark_mode,
            "route": route,
            "fallback_reason": fallback_reason,
            "parser_latency_ms": parser_latency_ms,
            "vlm_json_valid": vlm_json_valid,
            "vlm_model": self._vlm_model,
            "vlm_api_base": self._vlm_api_base,
            "image_used": bool(image_path),
            "raw_vlm_json": raw_vlm_json,
            "verified_json": verified_json,
            "grounding_score": grounding_score,
            "map_match_score": map_match_score,
            "relation_score": relation_score,
            "attribute_score": attribute_score,
            "hypotheses": hypotheses,
            "hypothesis_count": len(hypotheses),
            "active_hypothesis_id": active_hypothesis_id,
            "hypothesis_switch_reason": hypothesis_switch_reason,
            "sequence": sequence,
            "semantic_parse": semantic_parse,
        }

        out_msg = String()
        out_msg.data = json.dumps(task_out, ensure_ascii=False)
        self._task_pub.publish(out_msg)

        log_payload: Dict[str, Any] = {
            "route": route,
            "fallback_reason": fallback_reason,
            "instruction": text,
            "language": _language_tag(text),
            "method_name": self._method_name,
            "sequence": sequence,
            "semantic_parse": semantic_parse,
            "parser_latency_ms": parser_latency_ms,
            "vlm_model": self._vlm_model,
            "vlm_api_base": self._vlm_api_base,
            "vlm_json_valid": vlm_json_valid,
            "raw_vlm_json": raw_vlm_json,
            "verified_json": verified_json,
            "grounding_score": grounding_score,
            "map_match_score": map_match_score,
            "relation_score": relation_score,
            "attribute_score": attribute_score,
            "hypotheses": hypotheses,
            "active_hypothesis_id": active_hypothesis_id,
            "hypothesis_switch_reason": hypothesis_switch_reason,
            "ego_state": ego_state,
            "candidate_objects": candidates,
            "local_context_text": local_context_text,
            "image_path": image_path,
        }
        if self._vlm_log_raw_response:
            log_payload["vlm_raw_response"] = raw_response
            log_payload["vlm_response_text"] = response_text

        self._logger_file.write("task_parsed", log_payload)
        intent = str(semantic_parse.get("intent", "unknown")) if isinstance(semantic_parse, dict) else "unknown"
        target_id = semantic_parse.get("target_id") if isinstance(semantic_parse, dict) else None
        self.get_logger().info(
            f"Parsed instruction route={route}, intent={intent}, target={target_id}, "
            f"steps={len(sequence)}, latency_ms={parser_latency_ms:.1f}, "
            f"fallback={fallback_reason if fallback_reason else 'none'}"
        )

    def _semantic_object_names(self) -> List[str]:
        objs = self._semantic_map.get("objects", [])
        if not isinstance(objs, list):
            return []
        names: List[str] = []
        for obj in objs:
            if not isinstance(obj, dict):
                continue
            name = str(obj.get("name", "")).strip()
            if name:
                names.append(name)
        return names

    def _expand_sequence_for_follow_channel(
        self,
        instruction_text: str,
        semantic_parse: Dict[str, Any],
        base_sequence: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        intent = str(semantic_parse.get("intent", "")).strip().lower()
        if intent != "follow_channel":
            return base_sequence

        names = self._semantic_object_names()
        name_set = set(names)
        if not name_set:
            return base_sequence

        target_id = semantic_parse.get("target_id")
        secondary_id = semantic_parse.get("secondary_target_id")
        target = str(target_id).strip() if isinstance(target_id, str) else ""
        secondary = str(secondary_id).strip() if isinstance(secondary_id, str) else ""

        pair_suffix: List[str] = []
        for name in name_set:
            if not name.startswith("gate_left"):
                continue
            suffix = name[len("gate_left") :]
            right = f"gate_right{suffix}"
            if right in name_set:
                pair_suffix.append(suffix)

        def suffix_rank(s: str) -> Tuple[int, str]:
            if s == "":
                return (0, s)
            if s == "_mid":
                return (1, s)
            if s == "_far":
                return (2, s)
            return (3, s)

        pair_suffix = sorted(set(pair_suffix), key=suffix_rank)
        steps: List[Dict[str, Any]] = []
        order = 1

        if target.startswith("gate_left") and secondary.startswith("gate_right"):
            this_suffix = target[len("gate_left") :]
            if this_suffix in pair_suffix:
                # Ensure selected pair is first while preserving deterministic order.
                pair_suffix = [this_suffix] + [s for s in pair_suffix if s != this_suffix]

        for suffix in pair_suffix:
            left = f"gate_left{suffix}"
            right = f"gate_right{suffix}"
            steps.append(_new_step(order, "pass_between", {"left": left, "right": right}, "between"))
            order += 1

        stop_condition = str(semantic_parse.get("stop_condition", "")).lower()
        need_dock = ("dock" in instruction_text.lower()) or ("码头" in instruction_text) or ("dock" in stop_condition)
        if need_dock and "dock" in name_set:
            steps.append(_new_step(order, "stop_near", "dock", "near"))

        return steps if steps else base_sequence


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InstructionManagerNode()
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
