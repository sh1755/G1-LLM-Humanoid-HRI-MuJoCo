"""Tiny JSON message helpers shared by all nodes.

All /hri/* topics use std_msgs/String carrying a JSON object, so students can
read every message with ``ros2 topic echo`` and no custom .msg build is needed.
Each interaction gets an ``id`` and wall-clock timestamps (``t_*``) so the
monitor can measure end-to-end latency on a single machine.
"""
from __future__ import annotations

import json
import time
import uuid
from typing import Any, Dict

TOPICS = {
    "speech_text": "/hri/speech_text",       # user utterance (typed or STT)
    "llm_response": "/hri/llm_response",     # structured action request (untrusted)
    "robot_action": "/hri/robot_action",     # validated action(s)
    "robot_status": "/hri/robot_status",     # simulator / validator status
    "speech_output": "/hri/speech_output",   # text the robot should say
    "emergency_stop": "/hri/emergency_stop", # std_msgs/Bool
}


def new_id() -> str:
    return uuid.uuid4().hex[:8]


def now() -> float:
    return time.time()


def encode(payload: Dict[str, Any]) -> str:
    return json.dumps(payload, separators=(",", ":"))


def decode(text: str) -> Dict[str, Any]:
    try:
        obj = json.loads(text)
        return obj if isinstance(obj, dict) else {"text": text}
    except json.JSONDecodeError:
        return {"text": text}  # plain text from `ros2 topic pub` is accepted too
