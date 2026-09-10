"""역할: Ollama 응답 외피·계획 파싱 회귀 시험. 인터페이스: pytest, HTTP·장치 연결 없음."""

import json

import pytest

from cap_robot.llm import OllamaClient, strict_json


@pytest.fixture
def generate_response(monkeypatch):
    """실제 generate의 스트리밍·외피·계획 파싱을 메모리 응답으로 검증한다."""

    def generate(text, endpoint="generate"):
        raw = text.encode("utf-8")

        class Response:
            def __enter__(self):
                return self

            def __exit__(self, *_):
                pass

            def raise_for_status(self):
                pass

            def iter_content(self, chunk_size):
                for start in range(0, len(raw), chunk_size):
                    yield raw[start : start + chunk_size]

        monkeypatch.setattr("cap_robot.llm.requests.post", lambda *_, **__: Response())
        client = OllamaClient({"url": f"http://unused/api/{endpoint}", "model": "test"})
        return client.generate("Return a plan", {})

    return generate


@pytest.mark.parametrize("endpoint", ["generate", "chat"])
def test_large_valid_envelope_keeps_both_ollama_interfaces(generate_response, endpoint):
    plan = {"steps": [], "reason": "done"}
    envelope = {"padding": "x" * 131072}
    if endpoint == "chat":
        envelope["message"] = {"role": "assistant", "content": json.dumps(plan)}
    else:
        envelope["response"] = json.dumps(plan)
    assert generate_response(json.dumps(envelope), endpoint) == plan


@pytest.mark.parametrize(
    "invalid_member",
    [
        '"response":"{}"',
        '"metadata":{"count":1,"count":2}',
        '"metadata":{"value":NaN}',
        '"metadata":{"value":Infinity}',
        '"metadata":{"value":-Infinity}',
        '"metadata":{"values":[1e999]}',
    ],
)
def test_large_envelope_rejects_duplicates_and_nonfinite_metadata(
    generate_response, invalid_member
):
    # [변경] 정상 계획을 포함해도 큰 외피의 불명확하거나 비유한 JSON을 수용하지 않는다.
    text = '{"response":"{}","padding":' + json.dumps("x" * 131072)
    text += "," + invalid_member + "}"
    assert 131072 < len(text.encode("utf-8")) < 1048576
    with pytest.raises(ValueError, match="duplicate JSON key|non-finite"):
        generate_response(text)


def test_large_envelope_does_not_raise_plan_size_limit(generate_response):
    plan = json.dumps({"steps": [], "padding": "x" * 131072})
    with pytest.raises(ValueError, match="131072 bytes"):
        generate_response(json.dumps({"response": plan}))


def test_http_response_limit_remains_one_mib(generate_response):
    envelope = json.dumps({"response": "{}", "padding": "x" * 1048576})
    with pytest.raises(ValueError, match="HTTP response exceeds 1 MiB"):
        generate_response(envelope)


def test_strict_json_default_limit_is_still_128_kib():
    with pytest.raises(ValueError, match="131072 bytes"):
        strict_json(json.dumps({"padding": "x" * 131072}))
