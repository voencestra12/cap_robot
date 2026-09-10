"""역할: 제한된 HTTP LLM 호출. 인터페이스: OllamaClient.generate(prompt, context)."""

from __future__ import annotations

import json
import math
import time
from typing import Any

import requests


def strict_json(text: str, *, max_bytes: int = 131072) -> dict:
    """[변경] 코드 실행·부분 JSON 추출 없이 단일 유한 JSON 객체만 허용한다."""
    if not isinstance(text, str) or len(text.encode("utf-8")) > max_bytes:
        raise ValueError(f"LLM response exceeds {max_bytes} bytes or is not text")

    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    def reject_constant(value):
        raise ValueError(f"non-finite JSON number: {value}")

    result = json.loads(text, object_pairs_hook=pairs, parse_constant=reject_constant)
    if not isinstance(result, dict):
        raise ValueError("LLM response must be a single JSON object")

    # [변경] 1e999도 JSON decoder를 통과하므로 재귀적으로 유한성을 확인한다.
    def check(value):
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError("non-finite number")
        if isinstance(value, dict):
            for item in value.values():
                check(item)
        if isinstance(value, list):
            for item in value:
                check(item)

    check(result)
    return result


class OllamaClient:
    """No runtime mock or scripted fallback; callers run generate in a worker."""

    def __init__(self, config: dict):
        self.url = str(config.get("url", "http://localhost:11434/api/generate"))
        self.model = str(config.get("model", "")).strip()
        self.timeout = float(config.get("timeout", 45.0))
        if not self.url.startswith(("http://", "https://")):
            raise ValueError("LLM URL must be HTTP(S)")
        if not math.isfinite(self.timeout) or not 1 <= self.timeout <= 300:
            raise ValueError("LLM timeout must be between 1 and 300 seconds")
        self.last_latency_s = 0.0

    def generate(self, system_prompt: str, context: dict[str, Any]) -> dict:
        if not self.model:
            raise ValueError("Configure an installed Ollama model; automatic fallback is disabled")
        user = json.dumps(context, ensure_ascii=False, allow_nan=False)
        payload = {
            "model": self.model,
            "stream": False,
            "format": "json",
            "options": {"temperature": 0.1, "num_ctx": 16384, "num_predict": 8192},
        }
        if self.url.rstrip("/").endswith("/api/chat"):
            payload["messages"] = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user},
            ]
        else:
            payload.update(system=system_prompt, prompt=user)
        started = time.monotonic()
        try:
            # [변경] 무제한 응답 버퍼를 피하고, 실패도 호출자가 계측할 수 있게 지연을 남긴다.
            with requests.post(
                self.url, json=payload, timeout=(5, self.timeout), stream=True
            ) as response:
                response.raise_for_status()
                raw = bytearray()
                for chunk in response.iter_content(8192):
                    if time.monotonic() - started > self.timeout:
                        raise TimeoutError("LLM total response deadline exceeded")
                    raw.extend(chunk)
                    if len(raw) > 1048576:
                        raise ValueError("LLM HTTP response exceeds 1 MiB")
                # [변경] 큰 HTTP 외피도 중복 키·비유한 수를 거부하며 계획은 128 KiB로 유지한다.
                envelope = strict_json(raw.decode("utf-8"), max_bytes=1048576)
                content = envelope.get("response")
                if content is None:
                    content = envelope.get("message", {}).get("content")
                return strict_json(content)
        finally:
            self.last_latency_s = time.monotonic() - started
