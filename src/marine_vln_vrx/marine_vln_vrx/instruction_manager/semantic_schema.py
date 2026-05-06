#!/usr/bin/env python3
"""Schema + validation helpers for strict VLM semantic outputs."""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

INTENTS = {
    "goto",
    "pass_left_of",
    "pass_right_of",
    "go_between",
    "follow_channel",
    "inspect",
    "stop",
    "unknown",
}

TARGET_TYPES = {
    "buoy",
    "dock",
    "gate",
    "channel",
    "boat",
    "waypoint",
    "region",
    "unknown",
}

DIRECTION_HINTS = {
    "left",
    "right",
    "front",
    "behind",
    "between",
    "around",
    "toward",
    "none",
}

SEMANTIC_GUIDED_JSON: Dict[str, Any] = {
    "type": "object",
    "properties": {
        "intent": {"type": "string", "enum": sorted(INTENTS)},
        "target_id": {"anyOf": [{"type": "string"}, {"type": "null"}]},
        "secondary_target_id": {"anyOf": [{"type": "string"}, {"type": "null"}]},
        "target_attributes": {
            "type": "object",
            "properties": {
                "color": {"type": "string"},
                "size": {"type": "string"},
                "shape": {"type": "string"},
            },
            "required": ["color", "size", "shape"],
            "additionalProperties": False,
        },
        "candidate_target_ids": {
            "type": "array",
            "items": {"type": "string"},
            "maxItems": 8,
        },
        "candidate_secondary_target_ids": {
            "type": "array",
            "items": {"type": "string"},
            "maxItems": 8,
        },
        "target_type": {"type": "string", "enum": sorted(TARGET_TYPES)},
        "direction_hint": {"type": "string", "enum": sorted(DIRECTION_HINTS)},
        "distance_hint_m": {"type": "number", "minimum": 0.0},
        "speed_scale": {"type": "number", "exclusiveMinimum": 0.0, "maximum": 1.0},
        "stop_condition": {"type": "string"},
        "confidence": {"type": "number", "minimum": 0.0, "maximum": 1.0},
        "use_rule_fallback": {"type": "boolean"},
        "brief_reason": {"type": "string"},
    },
    "required": [
        "intent",
        "target_id",
        "secondary_target_id",
        "target_attributes",
        "candidate_target_ids",
        "candidate_secondary_target_ids",
        "target_type",
        "direction_hint",
        "distance_hint_m",
        "speed_scale",
        "stop_condition",
        "confidence",
        "use_rule_fallback",
        "brief_reason",
    ],
    "additionalProperties": False,
}


def default_semantic(reason: str = "unparsed") -> Dict[str, Any]:
    return {
        "intent": "unknown",
        "target_id": None,
        "secondary_target_id": None,
        "target_attributes": {"color": "unknown", "size": "unknown", "shape": "unknown"},
        "candidate_target_ids": [],
        "candidate_secondary_target_ids": [],
        "target_type": "unknown",
        "direction_hint": "none",
        "distance_hint_m": 0.0,
        "speed_scale": 0.5,
        "stop_condition": "",
        "confidence": 0.0,
        "use_rule_fallback": True,
        "brief_reason": reason,
    }


def _norm_id(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    if not text or text.lower() in {"null", "none", "unknown"}:
        return None
    return text


def normalize_semantic_payload(raw: Dict[str, Any]) -> Dict[str, Any]:
    """Coerce raw model JSON to the expected key set."""
    norm = default_semantic("normalized")

    if "intent" in raw:
        norm["intent"] = str(raw.get("intent", "unknown")).strip().lower()
    norm["target_id"] = _norm_id(raw.get("target_id"))
    norm["secondary_target_id"] = _norm_id(raw.get("secondary_target_id"))
    attrs_raw = raw.get("target_attributes", {})
    if isinstance(attrs_raw, dict):
        norm["target_attributes"] = {
            "color": str(attrs_raw.get("color", "unknown")).strip().lower() or "unknown",
            "size": str(attrs_raw.get("size", "unknown")).strip().lower() or "unknown",
            "shape": str(attrs_raw.get("shape", "unknown")).strip().lower() or "unknown",
        }

    cands = raw.get("candidate_target_ids", [])
    if isinstance(cands, list):
        norm["candidate_target_ids"] = [str(x).strip() for x in cands if str(x).strip()][:8]
    cands2 = raw.get("candidate_secondary_target_ids", [])
    if isinstance(cands2, list):
        norm["candidate_secondary_target_ids"] = [str(x).strip() for x in cands2 if str(x).strip()][:8]

    if "target_type" in raw:
        norm["target_type"] = str(raw.get("target_type", "unknown")).strip().lower()
    elif norm["intent"] == "go_between":
        norm["target_type"] = "gate"
    elif norm["intent"] in {"goto", "pass_left_of", "pass_right_of"} and norm["target_id"] is not None:
        norm["target_type"] = "unknown"
    if "direction_hint" in raw:
        norm["direction_hint"] = str(raw.get("direction_hint", "none")).strip().lower()
    elif norm["intent"] == "go_between":
        norm["direction_hint"] = "between"
    elif norm["intent"] in {"goto", "inspect"}:
        norm["direction_hint"] = "toward"

    try:
        norm["distance_hint_m"] = float(raw.get("distance_hint_m", norm["distance_hint_m"]))
    except (TypeError, ValueError):
        pass

    try:
        norm["speed_scale"] = float(raw.get("speed_scale", norm["speed_scale"]))
    except (TypeError, ValueError):
        pass

    norm["stop_condition"] = str(raw.get("stop_condition", "")).strip()

    if "confidence" in raw:
        try:
            norm["confidence"] = float(raw.get("confidence", norm["confidence"]))
        except (TypeError, ValueError):
            pass
    else:
        # Some VLM outputs omit confidence even when intent/targets are clear.
        # Provide conservative defaults to avoid false fallback on 0.0.
        if not bool(raw.get("use_rule_fallback", False)) and norm["intent"] != "unknown":
            if norm["intent"] == "go_between" and norm["target_id"] is not None and norm["secondary_target_id"] is not None:
                norm["confidence"] = 0.72
            elif norm["target_id"] is not None:
                norm["confidence"] = 0.65
            else:
                norm["confidence"] = 0.58

    norm["use_rule_fallback"] = bool(raw.get("use_rule_fallback", False))
    brief_reason = raw.get("brief_reason", "")
    if (not isinstance(brief_reason, str) or not brief_reason.strip()) and isinstance(raw.get("reason"), str):
        brief_reason = raw.get("reason")
    norm["brief_reason"] = str(brief_reason).strip()

    return norm


def validate_semantic_payload(
    semantic: Dict[str, Any],
    candidate_ids: Sequence[str],
) -> Tuple[bool, List[str]]:
    """Validate semantic payload against strict constraints."""
    errors: List[str] = []

    intent = str(semantic.get("intent", ""))
    if intent not in INTENTS:
        errors.append("intent_invalid")

    target_id = semantic.get("target_id")
    secondary_id = semantic.get("secondary_target_id")
    target_type = str(semantic.get("target_type", ""))
    direction_hint = str(semantic.get("direction_hint", ""))

    if target_type not in TARGET_TYPES:
        errors.append("target_type_invalid")
    if direction_hint not in DIRECTION_HINTS:
        errors.append("direction_hint_invalid")

    if target_id is not None and str(target_id) not in candidate_ids:
        errors.append("target_id_not_in_candidates")
    if secondary_id is not None and str(secondary_id) not in candidate_ids:
        errors.append("secondary_target_id_not_in_candidates")

    for cid in semantic.get("candidate_target_ids", []):
        if str(cid) not in candidate_ids:
            errors.append("candidate_target_ids_not_in_candidates")
            break
    for cid in semantic.get("candidate_secondary_target_ids", []):
        if str(cid) not in candidate_ids:
            errors.append("candidate_secondary_target_ids_not_in_candidates")
            break

    try:
        distance_hint = float(semantic.get("distance_hint_m", 0.0))
        if distance_hint < 0.0:
            errors.append("distance_hint_negative")
    except (TypeError, ValueError):
        errors.append("distance_hint_invalid")

    try:
        speed_scale = float(semantic.get("speed_scale", 0.0))
        if speed_scale <= 0.0 or speed_scale > 1.0:
            errors.append("speed_scale_out_of_range")
    except (TypeError, ValueError):
        errors.append("speed_scale_invalid")

    try:
        confidence = float(semantic.get("confidence", -1.0))
        if confidence < 0.0 or confidence > 1.0:
            errors.append("confidence_out_of_range")
    except (TypeError, ValueError):
        errors.append("confidence_invalid")

    if not isinstance(semantic.get("use_rule_fallback", False), bool):
        errors.append("use_rule_fallback_invalid")

    if intent == "go_between":
        if target_id is None or secondary_id is None:
            errors.append("go_between_missing_targets")

    if intent in {"goto", "pass_left_of", "pass_right_of", "inspect"}:
        if target_id is None:
            errors.append("missing_target_id")

    return (len(errors) == 0, errors)


def semantic_to_sequence(semantic: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Map strict semantic JSON to existing planner-friendly sequence schema."""
    intent = str(semantic.get("intent", "unknown"))
    target_id = _norm_id(semantic.get("target_id"))
    secondary_id = _norm_id(semantic.get("secondary_target_id"))

    if intent == "goto" and target_id:
        return [{"order": 1, "action": "go_to", "target": target_id, "spatial_relation": "at"}]

    if intent == "pass_left_of" and target_id:
        return [{"order": 1, "action": "go_to", "target": target_id, "spatial_relation": "left_of"}]

    if intent == "pass_right_of" and target_id:
        return [{"order": 1, "action": "go_to", "target": target_id, "spatial_relation": "right_of"}]

    if intent == "go_between" and target_id and secondary_id:
        return [
            {
                "order": 1,
                "action": "pass_between",
                "target": {"left": target_id, "right": secondary_id},
                "spatial_relation": "between",
            }
        ]

    if intent == "follow_channel":
        if target_id and secondary_id:
            return [
                {
                    "order": 1,
                    "action": "pass_between",
                    "target": {"left": target_id, "right": secondary_id},
                    "spatial_relation": "between",
                }
            ]
        if target_id:
            return [{"order": 1, "action": "go_to", "target": target_id, "spatial_relation": "toward"}]

    if intent == "inspect" and target_id:
        return [{"order": 1, "action": "stop_near", "target": target_id, "spatial_relation": "near"}]

    if intent == "stop":
        return [{"order": 1, "action": "stop_near", "target": "current_position", "spatial_relation": "at"}]

    return [{"order": 1, "action": "stop_near", "target": "current_position", "spatial_relation": "at"}]
