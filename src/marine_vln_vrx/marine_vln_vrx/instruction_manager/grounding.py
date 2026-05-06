#!/usr/bin/env python3
"""Map-constrained grounding and hypothesis generation for Marine-VLN."""

from __future__ import annotations

import math
import re
from typing import Any, Dict, List, Optional, Sequence, Tuple

from marine_vln_vrx.common.geometry_utils import wrap_to_pi
from marine_vln_vrx.instruction_manager.semantic_schema import default_semantic


def _norm(text: str) -> str:
    return text.strip().lower().replace("-", "_").replace(" ", "_")


def _is_chinese(text: str) -> bool:
    return bool(re.search(r"[\u4e00-\u9fff]", text))


def _obj_color(obj: Dict[str, Any]) -> str:
    name = str(obj.get("name", "")).lower()
    aliases = obj.get("aliases", [])
    alias_text = " ".join([str(x).lower() for x in aliases if isinstance(x, str)])
    text = f"{name} {alias_text}"
    if ("red" in text) or ("红" in text):
        return "red"
    if ("green" in text) or ("绿" in text):
        return "green"
    if ("blue" in text) or ("蓝" in text):
        return "blue"
    if ("yellow" in text) or ("黄" in text):
        return "yellow"
    return "unknown"


def _requested_colors(instruction_text: str) -> List[str]:
    text = instruction_text.lower()
    colors: List[str] = []
    if ("red" in text) or ("红" in instruction_text):
        colors.append("red")
    if ("green" in text) or ("绿" in instruction_text):
        colors.append("green")
    if ("blue" in text) or ("蓝" in instruction_text):
        colors.append("blue")
    if ("yellow" in text) or ("黄" in instruction_text):
        colors.append("yellow")
    return colors


def _requested_direction(instruction_text: str) -> str:
    lower = instruction_text.lower()
    compact = re.sub(r"[，。！？；、\s]", "", instruction_text)
    if "between" in lower or ("中间" in compact) or ("之间" in compact):
        return "between"
    if "left" in lower or "左" in compact:
        return "left"
    if "right" in lower or "右" in compact:
        return "right"
    if "front" in lower or "前" in compact:
        return "front"
    if "behind" in lower or "后" in compact:
        return "behind"
    if "around" in lower or "绕" in compact:
        return "around"
    return "none"


def _bearing_relation(ego: Dict[str, float], obj: Dict[str, Any]) -> str:
    dx = float(obj.get("x", 0.0)) - float(ego.get("x", 0.0))
    dy = float(obj.get("y", 0.0)) - float(ego.get("y", 0.0))
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return "front"
    rel = wrap_to_pi(math.atan2(dy, dx) - float(ego.get("yaw", 0.0)))
    if abs(rel) > math.radians(150.0):
        return "behind"
    if abs(rel) < math.radians(35.0):
        return "front"
    return "left" if rel > 0.0 else "right"


def _lexical_object_score(instruction_text: str, obj: Dict[str, Any]) -> float:
    text_lower = instruction_text.lower()
    name = str(obj.get("name", "")).strip()
    if not name:
        return 0.0
    score = 0.0
    if name.lower() in text_lower:
        score += 0.8
    aliases = obj.get("aliases", [])
    if isinstance(aliases, list):
        for alias in aliases:
            if not isinstance(alias, str):
                continue
            token = alias.strip()
            if not token:
                continue
            if _is_chinese(token):
                if token in instruction_text:
                    score = max(score, 0.8)
                    break
            else:
                if token.lower() in text_lower:
                    score = max(score, 0.8)
                    break
    color_req = _requested_colors(instruction_text)
    if color_req:
        if _obj_color(obj) in color_req:
            score += 0.15
    return min(1.0, score)


def _type_compat_score(intent: str, target_type: str, obj: Dict[str, Any]) -> float:
    obj_type = str(obj.get("type", "unknown")).strip().lower()
    if target_type == "unknown":
        return 1.0
    if target_type == "gate":
        return 1.0 if obj_type in {"gate_post", "buoy", "gate"} else 0.45
    if target_type == "buoy":
        return 1.0 if obj_type == "buoy" else 0.35
    if target_type == "dock":
        return 1.0 if obj_type == "dock" else 0.25
    if target_type == "boat":
        return 1.0 if ("boat" in obj_type) else 0.25
    if intent in {"follow_channel", "go_between"}:
        return 1.0 if obj_type in {"gate_post", "buoy", "channel"} else 0.4
    return 0.75 if obj_type == target_type else 0.45


def _relation_consistency_score(
    instruction_text: str,
    semantic: Dict[str, Any],
    ego: Dict[str, float],
    obj_map: Dict[str, Dict[str, Any]],
) -> float:
    direction_hint = str(semantic.get("direction_hint", "none")).strip().lower()
    req = _requested_direction(instruction_text)
    target_id = semantic.get("target_id")
    if direction_hint in {"none", ""} and req == "none":
        return 1.0
    if direction_hint in {"between"} or req == "between":
        sec = semantic.get("secondary_target_id")
        return 1.0 if (target_id in obj_map and sec in obj_map) else 0.2
    if target_id not in obj_map:
        return 0.2
    actual = _bearing_relation(ego, obj_map[str(target_id)])
    expected = direction_hint if direction_hint != "none" else req
    if expected in {"none", ""}:
        return 0.9
    if expected == actual:
        return 1.0
    if {expected, actual} <= {"front", "left", "right"}:
        return 0.6
    return 0.3


def _select_candidate_for_semantic(
    semantic: Dict[str, Any],
    candidates: Sequence[Dict[str, Any]],
    instruction_text: str,
) -> Optional[str]:
    intent = str(semantic.get("intent", "unknown")).lower()
    target_type = str(semantic.get("target_type", "unknown")).lower()

    scored: List[Tuple[float, str]] = []
    for obj in candidates:
        oid = str(obj.get("id", "")).strip()
        if not oid:
            continue
        lex = _lexical_object_score(instruction_text, obj)
        tscore = _type_compat_score(intent, target_type, obj)
        dist = float(obj.get("distance", 1e9))
        dist_score = 1.0 / (1.0 + max(0.0, dist) / 80.0)
        total = 0.5 * lex + 0.3 * tscore + 0.2 * dist_score
        scored.append((total, oid))
    if not scored:
        return None
    scored.sort(key=lambda x: x[0], reverse=True)
    return scored[0][1]


def _build_go_between_candidates(
    candidates: Sequence[Dict[str, Any]],
    instruction_text: str,
) -> List[Tuple[float, str, str]]:
    left_like = [c for c in candidates if str(c.get("type", "")).lower() in {"gate_post", "buoy", "gate"}]
    out: List[Tuple[float, str, str]] = []
    req_colors = _requested_colors(instruction_text)
    for i in range(len(left_like)):
        for j in range(i + 1, len(left_like)):
            a = left_like[i]
            b = left_like[j]
            aid = str(a.get("id", "")).strip()
            bid = str(b.get("id", "")).strip()
            if not aid or not bid:
                continue
            dy = abs(float(a.get("y", 0.0)) - float(b.get("y", 0.0)))
            dx = abs(float(a.get("x", 0.0)) - float(b.get("x", 0.0)))
            gate_shape = 1.0 / (1.0 + dx / 20.0) * (1.0 / (1.0 + abs(dy - 8.0) / 10.0))
            color_bonus = 0.0
            if req_colors:
                cset = {_obj_color(a), _obj_color(b)}
                if set(req_colors).issubset(cset):
                    color_bonus = 0.3
            score = min(1.0, 0.7 * gate_shape + color_bonus)
            out.append((score, aid, bid))
    out.sort(key=lambda x: x[0], reverse=True)
    return out


def verify_semantic_with_map(
    semantic: Dict[str, Any],
    instruction_text: str,
    ego_state: Dict[str, float],
    candidates: Sequence[Dict[str, Any]],
    topk: int = 3,
) -> Dict[str, Any]:
    """Verify/correct semantic parse against semantic map authority and build top-k hypotheses."""
    norm = dict(semantic)
    obj_map: Dict[str, Dict[str, Any]] = {
        str(c.get("id", "")).strip(): c for c in candidates if str(c.get("id", "")).strip()
    }
    candidate_ids = list(obj_map.keys())
    norm["candidate_target_ids"] = candidate_ids[:8]
    norm["candidate_secondary_target_ids"] = candidate_ids[:8]

    intent = str(norm.get("intent", "unknown")).strip().lower()
    if intent == "":
        intent = "unknown"
        norm["intent"] = "unknown"

    raw_target = norm.get("target_id")
    raw_secondary = norm.get("secondary_target_id")
    target_id = str(raw_target).strip() if isinstance(raw_target, str) and raw_target.strip() else None
    secondary_id = str(raw_secondary).strip() if isinstance(raw_secondary, str) and raw_secondary.strip() else None

    corrections: List[str] = []
    if target_id is not None and target_id not in obj_map:
        corrections.append("target_not_in_map")
        target_id = None
    if secondary_id is not None and secondary_id not in obj_map:
        corrections.append("secondary_not_in_map")
        secondary_id = None

    # Autocorrect missing target(s) with map-constrained ranking.
    if intent == "go_between":
        pair_cands = _build_go_between_candidates(candidates, instruction_text)
        if pair_cands:
            if target_id is None or secondary_id is None:
                _, aid, bid = pair_cands[0]
                target_id = aid
                secondary_id = bid
                corrections.append("autofill_between_pair")
    else:
        if target_id is None:
            best = _select_candidate_for_semantic(norm, candidates, instruction_text)
            if best is not None:
                target_id = best
                corrections.append("autofill_target")

    norm["target_id"] = target_id
    norm["secondary_target_id"] = secondary_id

    vlm_conf = float(norm.get("confidence", 0.0))
    vlm_conf = max(0.0, min(1.0, vlm_conf))

    map_match = 0.0
    if intent == "go_between":
        map_match = 1.0 if (target_id in obj_map and secondary_id in obj_map) else 0.0
    elif intent in {"goto", "pass_left_of", "pass_right_of", "inspect", "follow_channel"}:
        map_match = 1.0 if (target_id in obj_map) else 0.0
    elif intent == "stop":
        map_match = 1.0
    else:
        map_match = 0.2

    if target_id in obj_map:
        map_match *= _type_compat_score(intent, str(norm.get("target_type", "unknown")).lower(), obj_map[target_id])

    relation_score = _relation_consistency_score(instruction_text, norm, ego_state, obj_map)

    attribute_score = 1.0
    req_colors = _requested_colors(instruction_text)
    if req_colors and target_id in obj_map:
        color = _obj_color(obj_map[target_id])
        attribute_score = 1.0 if color in req_colors else 0.45
        if intent == "go_between" and secondary_id in obj_map:
            cset = {color, _obj_color(obj_map[secondary_id])}
            attribute_score = 1.0 if set(req_colors).issubset(cset) else 0.5

    grounding_score = max(0.0, min(1.0, map_match * relation_score * attribute_score))
    grounded_confidence = max(0.0, min(1.0, vlm_conf * grounding_score))
    norm["confidence"] = grounded_confidence

    # Build top-k hypotheses under semantic-map authority.
    hypotheses: List[Dict[str, Any]] = []
    if intent == "go_between":
        pair_cands = _build_go_between_candidates(candidates, instruction_text)[: max(1, topk)]
        for rank, (score, aid, bid) in enumerate(pair_cands):
            h = dict(norm)
            h["target_id"] = aid
            h["secondary_target_id"] = bid
            h["confidence"] = max(0.0, min(1.0, float(h.get("confidence", 0.0)) * max(0.45, score)))
            hypotheses.append(
                {
                    "id": f"h{rank}",
                    "semantic_parse": h,
                    "grounding_score": max(0.0, min(1.0, score * map_match * relation_score)),
                    "map_match_score": map_match,
                    "relation_score": relation_score,
                    "attribute_score": attribute_score,
                }
            )
    else:
        scored_objs: List[Tuple[float, Dict[str, Any]]] = []
        for obj in candidates:
            oid = str(obj.get("id", "")).strip()
            if not oid:
                continue
            lex = _lexical_object_score(instruction_text, obj)
            tscore = _type_compat_score(intent, str(norm.get("target_type", "unknown")).lower(), obj)
            dist_score = 1.0 / (1.0 + float(obj.get("distance", 1e9)) / 100.0)
            s = 0.5 * lex + 0.3 * tscore + 0.2 * dist_score
            scored_objs.append((s, obj))
        scored_objs.sort(key=lambda x: x[0], reverse=True)
        top = scored_objs[: max(1, topk)]
        for rank, (score, obj) in enumerate(top):
            h = dict(norm)
            oid = str(obj.get("id", "")).strip()
            if intent != "stop":
                h["target_id"] = oid
            h["secondary_target_id"] = None
            h["confidence"] = max(0.0, min(1.0, float(h.get("confidence", 0.0)) * max(0.45, score)))
            hypotheses.append(
                {
                    "id": f"h{rank}",
                    "semantic_parse": h,
                    "grounding_score": max(0.0, min(1.0, score * map_match * relation_score)),
                    "map_match_score": map_match,
                    "relation_score": relation_score,
                    "attribute_score": attribute_score,
                }
            )

    if not hypotheses:
        hypotheses = [
            {
                "id": "h0",
                "semantic_parse": dict(norm),
                "grounding_score": grounding_score,
                "map_match_score": map_match,
                "relation_score": relation_score,
                "attribute_score": attribute_score,
            }
        ]

    # Keep first hypothesis consistent with corrected semantic.
    best = hypotheses[0]
    corrected_semantic = dict(best.get("semantic_parse", norm))

    return {
        "raw_semantic": semantic,
        "verified_semantic": corrected_semantic,
        "grounding_score": grounding_score,
        "map_match_score": map_match,
        "relation_score": relation_score,
        "attribute_score": attribute_score,
        "grounded_confidence": grounded_confidence,
        "corrections": corrections,
        "hypotheses": hypotheses,
        "candidate_ids": candidate_ids,
    }


def build_oracle_semantic(
    instruction_text: str,
    candidates: Sequence[Dict[str, Any]],
    default_intent: str = "goto",
) -> Dict[str, Any]:
    """A simple map-authoritative oracle upper-bound parser based on lexical grounding."""
    if not candidates:
        sem = default_semantic("oracle_no_candidates")
        sem["intent"] = "unknown"
        sem["confidence"] = 0.0
        return sem

    if ("between" in instruction_text.lower()) or ("中间" in instruction_text):
        pairs = _build_go_between_candidates(candidates, instruction_text)
        if pairs:
            _, aid, bid = pairs[0]
            sem = default_semantic("oracle_between")
            sem["intent"] = "go_between"
            sem["target_id"] = aid
            sem["secondary_target_id"] = bid
            sem["target_type"] = "gate"
            sem["direction_hint"] = "between"
            sem["confidence"] = 0.99
            sem["use_rule_fallback"] = False
            return sem

    best = None
    best_score = -1.0
    for obj in candidates:
        score = _lexical_object_score(instruction_text, obj)
        if score > best_score:
            best_score = score
            best = obj
    if best is None:
        best = candidates[0]
    sem = default_semantic("oracle_single")
    sem["intent"] = default_intent
    sem["target_id"] = str(best.get("id", ""))
    sem["secondary_target_id"] = None
    sem["target_type"] = "unknown"
    sem["direction_hint"] = _requested_direction(instruction_text)
    if sem["direction_hint"] == "none":
        sem["direction_hint"] = "toward"
    sem["confidence"] = 0.99
    sem["use_rule_fallback"] = False
    return sem
