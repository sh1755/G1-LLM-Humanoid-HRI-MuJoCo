"""Offline, deterministic command interpreter (no LLM, no internet, no API key).

It returns exactly the same structure as the LLM backends:

    {"actions": ["wave_right_hand"], "speech": "Hello! Nice to meet you."}

so the rest of the pipeline cannot tell which interpreter was used.
"""
from __future__ import annotations

import re
from typing import Dict, List

from .action_catalog import ActionCatalog

GREETING_WORDS = ("hello", "hi ", "hi,", "hey", "good morning", "good afternoon", "greetings")
INTRO_PHRASES = ("introduce yourself", "who are you", "your name", "what are you")
STOP_WORDS = ("stop", "freeze", "halt", "emergency")
EXPLAIN_TOPICS = {
    "humanoid": ("Humanoid robots have a human-like body: a head, torso, two arms and two legs. "
                 "That lets them use spaces and tools designed for people and makes interaction "
                 "with humans feel natural."),
    "ros": ("ROS 2 is the Robot Operating System. Programs called nodes exchange messages on "
            "topics, so speech, AI and motor control can run as separate parts."),
    "llm": ("A large language model understands text. In this demo it only chooses from a list of "
            "approved actions; it never sends motor commands directly."),
}
CLAUSE_SPLIT = re.compile(r"\s*(?:,|;|\band then\b|\bthen\b|\band\b|\bafter that\b)\s*")


def _match_action(clause: str, catalog: ActionCatalog) -> str | None:
    """Return the action whose longest keyword appears in the clause."""
    best, best_len = None, 0
    for name, spec in catalog.actions.items():
        if spec.type != "motion":
            continue
        for kw in spec.keywords:
            if re.search(r"\b" + re.escape(kw) + r"\b", clause) and len(kw) > best_len:
                best, best_len = name, len(kw)
    return best


def parse_command(text: str, catalog: ActionCatalog) -> Dict[str, object]:
    t = " " + text.lower().strip() + " "
    t = re.sub(r"[!?.]", " ", t)
    actions: List[str] = []
    speech_parts: List[str] = []

    # The emergency stop itself is raised locally by speech_input_node (it must not
    # depend on any language model); here we only make sure no motion is requested.
    if any(re.search(r"\b" + w + r"\b", t) for w in STOP_WORDS):
        return {"actions": ["none"], "speech": "Stopping now."}

    if any(w in t for w in GREETING_WORDS):
        speech_parts.append(f"Hello! I am {catalog.robot_name}.")
    if any(p in t for p in INTRO_PHRASES):
        speech_parts.append(f"I am {catalog.robot_name}, a humanoid robot running in a MuJoCo "
                            "simulation and controlled through ROS 2.")
    if "tell me about" in t or "explain" in t or "what is" in t:
        for key, answer in EXPLAIN_TOPICS.items():
            if key in t:
                speech_parts.append(answer)
                break

    for clause in CLAUSE_SPLIT.split(t):
        name = _match_action(" " + clause.strip() + " ", catalog)
        if name and (not actions or actions[-1] != name):
            actions.append(name)

    if actions and not speech_parts:
        speech_parts.append(catalog.actions[actions[0]].speech)
    if not actions and not speech_parts:
        speech_parts.append("Sorry, I did not understand. You can ask me to wave, raise my arms, "
                            "point, bow or return to neutral.")
    return {"actions": actions or ["none"], "speech": " ".join(p for p in speech_parts if p)}
