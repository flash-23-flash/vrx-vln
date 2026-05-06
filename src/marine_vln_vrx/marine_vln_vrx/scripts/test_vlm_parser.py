#!/usr/bin/env python3
"""Quick test tool for OpenAI-compatible VLM parser endpoint."""

from __future__ import annotations

import argparse
import json
import os

from marine_vln_vrx.instruction_manager.semantic_schema import (
    SEMANTIC_GUIDED_JSON,
    normalize_semantic_payload,
    validate_semantic_payload,
)
from marine_vln_vrx.instruction_manager.vlm_client import OpenAICompatibleVLMClient


def main() -> None:
    parser = argparse.ArgumentParser(description="Test VLM endpoint with one instruction")
    parser.add_argument("--api-base", required=True, help="OpenAI-compatible /v1 URL")
    parser.add_argument("--model", required=True, help="Model name")
    parser.add_argument("--api-key-env", default="OPENAI_API_KEY", help="Environment variable for API key")
    parser.add_argument("--instruction", default="Pass between the red and green buoys", help="Instruction text")
    parser.add_argument("--image-path", default="", help="Optional image path")
    parser.add_argument("--timeout-ms", type=int, default=8000)
    parser.add_argument("--max-retries", type=int, default=1)
    parser.add_argument("--temperature", type=float, default=0.1)
    parser.add_argument("--max-tokens", type=int, default=256)
    parser.add_argument("--guided-json", action="store_true", help="Enable guided_json in request")
    parser.add_argument("--candidate-ids", default="", help="Comma-separated candidate ids for validation")
    args = parser.parse_args()

    api_key = os.environ.get(args.api_key_env, "")
    client = OpenAICompatibleVLMClient(
        api_base=args.api_base,
        model=args.model,
        api_key=api_key,
        timeout_ms=args.timeout_ms,
        max_retries=args.max_retries,
        temperature=args.temperature,
        max_tokens=args.max_tokens,
    )

    system_prompt = "You are a deterministic maritime VLN parser for VRX. Output JSON only."
    user_prompt = (
        "Instruction:\n"
        f"{args.instruction}\n\n"
        "Return exactly one JSON object using the required semantic schema."
    )

    image_path = args.image_path.strip() or None
    result = client.request_json(
        system_prompt=system_prompt,
        user_prompt=user_prompt,
        image_path=image_path,
        guided_json_schema=SEMANTIC_GUIDED_JSON,
        enable_guided_json=bool(args.guided_json),
    )

    print("=== Request Result ===")
    print(json.dumps({k: v for k, v in result.items() if k not in {"raw_response", "response_text"}}, ensure_ascii=False, indent=2))

    if not result.get("ok", False):
        print("\n=== Raw Response ===")
        print(result.get("raw_response", ""))
        raise SystemExit(2)

    parsed = result.get("parsed_json", {})
    semantic = normalize_semantic_payload(parsed if isinstance(parsed, dict) else {})

    candidate_ids = [x.strip() for x in args.candidate_ids.split(",") if x.strip()]
    valid, errors = validate_semantic_payload(semantic, candidate_ids)

    print("\n=== Normalized Semantic JSON ===")
    print(json.dumps(semantic, ensure_ascii=False, indent=2))
    print("\n=== Validation ===")
    print(json.dumps({"valid": valid, "errors": errors, "candidate_ids": candidate_ids}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
