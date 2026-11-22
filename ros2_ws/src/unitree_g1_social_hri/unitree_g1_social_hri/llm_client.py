"""LLM backends that turn natural language into a *structured action request*.

Backends
--------
* ``rule``   - offline keyword parser (default, no key, no internet)
* ``ollama`` - local model through Ollama's REST API (``/api/chat`` + JSON-schema ``format``)
* ``openai`` - OpenAI Responses API with Structured Outputs (``text.format`` json_schema)

All backends return the same dictionary::

    {"actions": ["wave_right_hand"], "speech": "Hello! Nice to meet you."}

The ``actions`` enum in the JSON schema is generated from robot_actions.yaml, so a
schema-compliant model can only name approved actions.  The action validator node
checks again anyway (defence in depth).

Only the Python standard library is used for HTTP (urllib), so no extra SDK is
required.  The OpenAI key is read from the OPENAI_API_KEY environment variable and
is never written to disk or logs.
"""
from __future__ import annotations

import json
import os
import re
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

from .action_catalog import ActionCatalog
from .rule_based_parser import parse_command

OPENAI_URL = "https://api.openai.com/v1/responses"
DEFAULT_OPENAI_MODEL = "gpt-5.6"     # override with OPENAI_MODEL
DEFAULT_OLLAMA_URL = "http://localhost:11434"
DEFAULT_OLLAMA_MODEL = "llama3.2"    # override with OLLAMA_MODEL


def action_schema(catalog: ActionCatalog) -> Dict[str, object]:
    max_n = int(catalog.safety.get("max_actions_per_request", 3))
    return {
        "type": "object",
        "properties": {
            "actions": {
                "type": "array",
                "description": f"Ordered list of 1 to {max_n} approved robot actions.",
                "items": {"type": "string", "enum": catalog.allowed},
            },
            "speech": {"type": "string", "description": "Short spoken reply (max 2 sentences)."},
        },
        "required": ["actions", "speech"],
        "additionalProperties": False,
    }


def system_prompt(catalog: ActionCatalog) -> str:
    return (
        f"You are the social interaction module of a {catalog.robot_name} humanoid robot "
        "running in a MuJoCo simulation for a university class.\n"
        "Reply ONLY with a JSON object: {\"actions\": [...], \"speech\": \"...\"}.\n"
        "Choose actions ONLY from this approved list:\n"
        f"{catalog.describe_for_prompt()}\n"
        "Rules: use \"none\" when no movement is needed; never invent new actions; never output "
        "joint angles, torques or code; keep speech friendly and under 40 words; if a request is "
        "unsafe or impossible, use [\"none\"] and explain briefly."
    )


def extract_json(text: str) -> Dict[str, object]:
    """Parse a JSON object from model text (tolerates ```json fences and prose)."""
    text = text.strip()
    fence = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.S)
    if fence:
        text = fence.group(1)
    try:
        obj = json.loads(text)
    except json.JSONDecodeError:
        m = re.search(r"\{.*\}", text, re.S)
        if not m:
            raise ValueError("No JSON object in LLM output")
        obj = json.loads(m.group(0))
    if not isinstance(obj, dict):
        raise ValueError("LLM output is not a JSON object")
    # accept the single-action form used on the lecture slides
    if "action" in obj and "actions" not in obj:
        obj["actions"] = [obj.pop("action")]
    return obj


def _post_json(url: str, payload: dict, headers: dict, timeout: float) -> dict:
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json", **headers})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


@dataclass
class LLMResult:
    request: Dict[str, object]
    backend: str          # backend that actually produced the answer
    latency_s: float
    fallback_reason: str = ""


class CommandInterpreter:
    """Front-end used by llm_interaction_node; falls back to rules on any error."""

    def __init__(self, catalog: ActionCatalog, backend: str = "rule",
                 timeout_s: Optional[float] = None,
                 http_post: Callable[[str, dict, dict, float], dict] = _post_json):
        if backend not in ("rule", "ollama", "openai"):
            raise ValueError("backend must be rule, ollama or openai")
        self.catalog = catalog
        self.backend = backend
        self.timeout = float(timeout_s or catalog.safety.get("llm_timeout_s", 20.0))
        self.http_post = http_post  # injectable for unit tests
        self.history: List[Dict[str, str]] = []

    # -------------------------------------------------------------- backends
    def _openai(self, text: str) -> Dict[str, object]:
        key = os.environ.get("OPENAI_API_KEY")
        if not key:
            raise RuntimeError("OPENAI_API_KEY is not set")
        payload = {
            "model": os.environ.get("OPENAI_MODEL", DEFAULT_OPENAI_MODEL),
            "instructions": system_prompt(self.catalog),
            "input": self.history[-6:] + [{"role": "user", "content": text}],
            "text": {"format": {"type": "json_schema", "name": "robot_action",
                                "schema": action_schema(self.catalog), "strict": True}},
        }
        resp = self._post(OPENAI_URL, payload, {"Authorization": f"Bearer {key}"})
        out_text = resp.get("output_text") or ""
        if not out_text:
            for item in resp.get("output", []):
                for c in item.get("content", []) or []:
                    if c.get("type") == "refusal":
                        raise RuntimeError("Model refused: " + c.get("refusal", ""))
                    if c.get("type") == "output_text":
                        out_text += c.get("text", "")
        return extract_json(out_text)

    def _ollama(self, text: str) -> Dict[str, object]:
        base = os.environ.get("OLLAMA_HOST", DEFAULT_OLLAMA_URL).rstrip("/")
        if not base.startswith("http"):
            base = "http://" + base
        payload = {
            "model": os.environ.get("OLLAMA_MODEL", DEFAULT_OLLAMA_MODEL),
            "messages": [{"role": "system", "content": system_prompt(self.catalog)}]
                        + self.history[-6:] + [{"role": "user", "content": text}],
            "format": action_schema(self.catalog),
            "stream": False,
            "options": {"temperature": 0},
        }
        resp = self._post(base + "/api/chat", payload, {})
        return extract_json(resp["message"]["content"])

    def _post(self, url: str, payload: dict, headers: dict) -> dict:
        try:
            return self.http_post(url, payload, headers, self.timeout)
        except urllib.error.HTTPError as e:  # do not echo headers (they hold the key)
            raise RuntimeError(f"HTTP {e.code} from {url.split('/v1')[0]}") from None

    # ------------------------------------------------------------------ public
    def interpret(self, text: str) -> LLMResult:
        t0 = time.monotonic()
        reason = ""
        if self.backend != "rule":
            try:
                fn = self._openai if self.backend == "openai" else self._ollama
                req = fn(text)
                self._remember(text, req)
                return LLMResult(req, self.backend, time.monotonic() - t0)
            except Exception as e:  # network down, no key, timeout, bad JSON ...
                reason = f"{type(e).__name__}: {e}"
        req = parse_command(text, self.catalog)
        return LLMResult(req, "rule", time.monotonic() - t0, reason)

    def _remember(self, text: str, req: Dict[str, object]) -> None:
        self.history += [{"role": "user", "content": text},
                         {"role": "assistant", "content": json.dumps(req)}]
        self.history = self.history[-12:]
