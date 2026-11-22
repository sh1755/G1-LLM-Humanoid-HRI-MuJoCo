"""Safety gate between the language layer and the robot controller.

Every request coming from an LLM (or the rule parser) is treated as untrusted
input.  It is approved only if it passes ALL of these checks:

1. it is a JSON object with an ``actions`` list and a ``speech`` string;
2. it contains no low-level fields (joint angles, torques, gains, code ...);
3. every action is on the allowlist in robot_actions.yaml;
4. the number of actions is within ``max_actions_per_request``;
5. the emergency stop is not active;
6. requests are not arriving faster than ``min_seconds_between_requests``.

Speech is truncated to ``max_speech_chars``.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .action_catalog import ActionCatalog

FORBIDDEN_KEYS = {"joint", "joints", "joint_positions", "q", "qpos", "torque", "torques", "tau",
                  "kp", "kd", "velocity", "trajectory", "code", "command", "motor_cmd", "lowcmd"}


@dataclass
class ValidationResult:
    approved: bool
    actions: List[str] = field(default_factory=list)
    speech: str = ""
    reasons: List[str] = field(default_factory=list)


class ActionValidator:
    def __init__(self, catalog: ActionCatalog, clock=time.monotonic):
        self.catalog = catalog
        self.clock = clock
        self.estop_active = False
        self._last_request_t: Optional[float] = None

    def validate(self, request: object) -> ValidationResult:
        s = self.catalog.safety
        reasons: List[str] = []
        if not isinstance(request, dict):
            return ValidationResult(False, reasons=["request is not a JSON object"])

        bad = sorted(k for k in request if k.lower() in FORBIDDEN_KEYS)
        if bad:
            reasons.append(f"low-level fields are not allowed: {bad}")

        actions = request.get("actions")
        if isinstance(actions, str):
            actions = [actions]
        if not isinstance(actions, list) or not actions:
            reasons.append("'actions' must be a non-empty list")
            actions = []
        actions = [a for a in actions]
        for a in actions:
            if not isinstance(a, str) or not self.catalog.is_allowed(a):
                reasons.append(f"action '{a}' is not on the approved list")
        max_n = int(s.get("max_actions_per_request", 3))
        if len(actions) > max_n:
            reasons.append(f"too many actions ({len(actions)} > {max_n})")

        speech = request.get("speech", "")
        if not isinstance(speech, str):
            reasons.append("'speech' must be a string")
            speech = ""
        max_chars = int(s.get("max_speech_chars", 300))
        if len(speech) > max_chars:
            speech = speech[: max_chars - 3].rstrip() + "..."

        motion = [a for a in actions if isinstance(a, str) and a != "none"]
        if motion and self.estop_active:
            reasons.append("emergency stop is active")

        now = self.clock()
        min_gap = float(s.get("min_seconds_between_requests", 0.0))
        if motion and self._last_request_t is not None and now - self._last_request_t < min_gap:
            reasons.append("requests are arriving too quickly")

        if reasons:
            return ValidationResult(False, [], speech, reasons)
        if motion:
            self._last_request_t = now
        return ValidationResult(True, motion, speech, [])
