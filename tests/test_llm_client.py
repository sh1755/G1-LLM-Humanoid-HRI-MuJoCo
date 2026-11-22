import json
import urllib.error

import pytest

from unitree_g1_social_hri.action_catalog import load_catalog
from unitree_g1_social_hri.llm_client import CommandInterpreter, action_schema, extract_json

CAT = load_catalog()


def test_schema_enum_is_the_allowlist():
    s = action_schema(CAT)
    assert s["properties"]["actions"]["items"]["enum"] == CAT.allowed
    assert s["additionalProperties"] is False and set(s["required"]) == {"actions", "speech"}


def test_extract_json_variants():
    assert extract_json('{"actions":["bow"],"speech":"x"}')["actions"] == ["bow"]
    assert extract_json('```json\n{"action":"bow","speech":"x"}\n```')["actions"] == ["bow"]
    assert extract_json('Sure! {"actions":["none"],"speech":"hi"} hope that helps')["speech"] == "hi"
    with pytest.raises(ValueError):
        extract_json("no json here")


def test_openai_request_and_parse(monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "sk-test-not-real")
    seen = {}

    def fake_post(url, payload, headers, timeout):
        seen.update(url=url, payload=payload, headers=headers)
        return {"output": [{"type": "message", "content": [
            {"type": "output_text", "text": json.dumps({"actions": ["wave_right_hand"],
                                                       "speech": "Hello! I am Unitree G1."})}]}]}

    ci = CommandInterpreter(CAT, "openai", http_post=fake_post)
    r = ci.interpret("introduce yourself and wave")
    assert r.backend == "openai" and r.request["actions"] == ["wave_right_hand"]
    assert seen["url"].endswith("/v1/responses")
    assert seen["payload"]["text"]["format"]["type"] == "json_schema"
    assert seen["payload"]["text"]["format"]["strict"] is True
    assert seen["headers"]["Authorization"] == "Bearer sk-test-not-real"


def test_ollama_request_and_parse(monkeypatch):
    def fake_post(url, payload, headers, timeout):
        assert url.endswith("/api/chat") and payload["stream"] is False
        assert payload["format"]["properties"]["actions"]["items"]["enum"] == CAT.allowed
        return {"message": {"role": "assistant", "content": '{"actions":["bow"],"speech":"Thanks"}'}}

    r = CommandInterpreter(CAT, "ollama", http_post=fake_post).interpret("bow please")
    assert r.backend == "ollama" and r.request["actions"] == ["bow"]


def test_fallback_without_key(monkeypatch):
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    r = CommandInterpreter(CAT, "openai").interpret("wave your right hand")
    assert r.backend == "rule" and "OPENAI_API_KEY" in r.fallback_reason
    assert r.request["actions"] == ["wave_right_hand"]


def test_fallback_on_network_error_hides_key(monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "sk-secret-123")

    def boom(url, payload, headers, timeout):
        raise urllib.error.HTTPError(url, 401, "Unauthorized", {"Authorization": headers["Authorization"]}, None)

    r = CommandInterpreter(CAT, "openai", http_post=boom).interpret("raise both arms")
    assert r.backend == "rule" and r.request["actions"] == ["raise_both_arms"]
    assert "sk-secret" not in r.fallback_reason


def test_fallback_on_bad_json():
    def junk(url, payload, headers, timeout):
        return {"message": {"content": "I would love to wave!"}}

    r = CommandInterpreter(CAT, "ollama", http_post=junk).interpret("wave")
    assert r.backend == "rule"


def test_no_api_key_in_source_tree():
    import pathlib, re
    root = pathlib.Path(__file__).resolve().parents[1]
    for p in root.rglob("*.py"):
        if "tests" in p.parts:
            continue
        assert not re.search(r"sk-[A-Za-z0-9]{20,}", p.read_text(errors="ignore")), p
