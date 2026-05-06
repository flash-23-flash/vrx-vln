#!/usr/bin/env python3
"""OpenAI-compatible VLM client for instruction parsing."""

from __future__ import annotations

import json
import re
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional


def extract_first_json_object(raw_text: str) -> Optional[Dict[str, Any]]:
    """Extract the first JSON object from model text."""
    text = raw_text.strip()
    if not text:
        return None

    try:
        obj = json.loads(text)
        if isinstance(obj, dict):
            return obj
    except json.JSONDecodeError:
        pass

    fenced = re.search(r"```json\s*(\{.*?\})\s*```", text, flags=re.DOTALL | re.IGNORECASE)
    if fenced:
        try:
            obj = json.loads(fenced.group(1))
            if isinstance(obj, dict):
                return obj
        except json.JSONDecodeError:
            pass

    lb = text.find("{")
    rb = text.rfind("}")
    if lb >= 0 and rb > lb:
        try:
            obj = json.loads(text[lb : rb + 1])
            if isinstance(obj, dict):
                return obj
        except json.JSONDecodeError:
            return None
    return None


class OpenAICompatibleVLMClient:
    """Minimal HTTP client for OpenAI-compatible chat completions."""

    def __init__(
        self,
        api_base: str,
        model: str,
        api_key: str,
        timeout_ms: int,
        max_retries: int,
        temperature: float,
        max_tokens: int,
    ) -> None:
        self._api_base = api_base.strip().rstrip("/")
        self._model = model.strip()
        self._api_key = api_key.strip()
        self._timeout_ms = int(timeout_ms)
        self._max_retries = max(0, int(max_retries))
        self._temperature = float(temperature)
        self._max_tokens = int(max_tokens)

    @property
    def model(self) -> str:
        return self._model

    def set_model(self, model: str) -> None:
        self._model = model.strip()

    @property
    def chat_url(self) -> str:
        if self._api_base.endswith("/chat/completions"):
            return self._api_base
        if self._api_base.endswith("/v1"):
            return f"{self._api_base}/chat/completions"
        if self._api_base.endswith("/v1/"):
            return f"{self._api_base}chat/completions"
        return f"{self._api_base}/chat/completions"

    @property
    def models_url(self) -> str:
        base = self._api_base
        if base.endswith("/chat/completions"):
            base = base[: -len("/chat/completions")]
        if base.endswith("/v1"):
            return f"{base}/models"
        if base.endswith("/v1/"):
            return f"{base}models"
        return f"{base}/models"

    def discover_first_model(self) -> Optional[str]:
        req = urllib.request.Request(self.models_url, method="GET")
        try:
            with urllib.request.urlopen(req, timeout=max(0.1, self._timeout_ms / 1000.0)) as resp:
                raw = resp.read().decode("utf-8", errors="replace")
        except (urllib.error.URLError, TimeoutError):
            return None
        except urllib.error.HTTPError:
            return None

        try:
            obj = json.loads(raw)
        except json.JSONDecodeError:
            return None

        data = obj.get("data")
        if not isinstance(data, list) or not data:
            return None
        first = data[0]
        if not isinstance(first, dict):
            return None
        model_id = first.get("id")
        if not isinstance(model_id, str):
            return None
        model_id = model_id.strip()
        return model_id or None

    def request_json(
        self,
        system_prompt: str,
        user_prompt: str,
        image_path: Optional[str],
        guided_json_schema: Optional[Dict[str, Any]],
        enable_guided_json: bool,
    ) -> Dict[str, Any]:
        """Issue one request (with retries) and return response details."""
        if not self._api_base:
            return {
                "ok": False,
                "error": "api_base_empty",
                "latency_ms": 0.0,
                "raw_response": "",
                "response_text": "",
                "parsed_json": None,
            }

        user_content: List[Dict[str, Any]] = [{"type": "text", "text": user_prompt}]
        if image_path:
            p = Path(image_path).expanduser().resolve()
            user_content.append({"type": "image_url", "image_url": {"url": f"file://{p}"}})

        payload: Dict[str, Any] = {
            "model": self._model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_content},
            ],
            "temperature": self._temperature,
            "max_tokens": self._max_tokens,
        }
        if enable_guided_json and guided_json_schema:
            payload["guided_json"] = guided_json_schema

        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if self._api_key:
            headers["Authorization"] = f"Bearer {self._api_key}"

        last_error = "request_failed"
        last_raw_response = ""
        for attempt in range(self._max_retries + 1):
            t0 = time.perf_counter()
            req = urllib.request.Request(self.chat_url, data=body, headers=headers, method="POST")
            try:
                with urllib.request.urlopen(req, timeout=max(0.1, self._timeout_ms / 1000.0)) as resp:
                    raw = resp.read().decode("utf-8", errors="replace")
            except urllib.error.HTTPError as exc:
                latency_ms = (time.perf_counter() - t0) * 1000.0
                err_text = exc.read().decode("utf-8", errors="replace")
                last_error = f"http_{exc.code}"
                last_raw_response = err_text
                if attempt < self._max_retries:
                    continue
                return {
                    "ok": False,
                    "error": last_error,
                    "latency_ms": latency_ms,
                    "raw_response": err_text,
                    "response_text": "",
                    "parsed_json": None,
                }
            except urllib.error.URLError as exc:
                latency_ms = (time.perf_counter() - t0) * 1000.0
                last_error = f"url_error:{exc}"
                if attempt < self._max_retries:
                    continue
                return {
                    "ok": False,
                    "error": last_error,
                    "latency_ms": latency_ms,
                    "raw_response": "",
                    "response_text": "",
                    "parsed_json": None,
                }
            except TimeoutError:
                latency_ms = (time.perf_counter() - t0) * 1000.0
                last_error = "timeout"
                if attempt < self._max_retries:
                    continue
                return {
                    "ok": False,
                    "error": last_error,
                    "latency_ms": latency_ms,
                    "raw_response": "",
                    "response_text": "",
                    "parsed_json": None,
                }

            latency_ms = (time.perf_counter() - t0) * 1000.0
            response_text = self._extract_response_text(raw)
            parsed = extract_first_json_object(response_text)
            if parsed is None:
                last_error = "json_not_found"
                last_raw_response = raw
                if attempt < self._max_retries:
                    continue
                return {
                    "ok": False,
                    "error": last_error,
                    "latency_ms": latency_ms,
                    "raw_response": raw,
                    "response_text": response_text,
                    "parsed_json": None,
                }

            return {
                "ok": True,
                "error": "",
                "latency_ms": latency_ms,
                "raw_response": raw,
                "response_text": response_text,
                "parsed_json": parsed,
                "used_model": self._model,
            }

        return {
            "ok": False,
            "error": last_error,
            "latency_ms": 0.0,
            "raw_response": last_raw_response,
            "response_text": "",
            "parsed_json": None,
        }

    @staticmethod
    def _extract_response_text(raw_response_json: str) -> str:
        try:
            obj = json.loads(raw_response_json)
        except json.JSONDecodeError:
            return raw_response_json

        choices = obj.get("choices", [])
        if not isinstance(choices, list) or not choices:
            return raw_response_json

        msg = choices[0].get("message", {})
        content = msg.get("content", "")

        if isinstance(content, str):
            return content
        if isinstance(content, list):
            texts: List[str] = []
            for part in content:
                if not isinstance(part, dict):
                    continue
                part_type = str(part.get("type", ""))
                if part_type in {"text", "output_text"}:
                    txt = part.get("text", "")
                    if isinstance(txt, str) and txt:
                        texts.append(txt)
            return "\n".join(texts).strip()

        return raw_response_json
