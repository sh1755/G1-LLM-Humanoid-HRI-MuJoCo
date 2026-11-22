"""Loads config/robot_actions.yaml: the single source of truth for approved actions."""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml


@dataclass
class ActionSpec:
    name: str
    type: str
    description: str
    keywords: List[str] = field(default_factory=list)
    speech: str = ""


@dataclass
class ActionCatalog:
    robot_name: str
    actions: Dict[str, ActionSpec]
    safety: Dict[str, object]

    @property
    def allowed(self) -> List[str]:
        return list(self.actions.keys())

    @property
    def motion_actions(self) -> List[str]:
        return [n for n, a in self.actions.items() if a.type == "motion"]

    def is_allowed(self, name: str) -> bool:
        return name in self.actions

    def describe_for_prompt(self) -> str:
        return "\n".join(f"- {n}: {a.description}" for n, a in self.actions.items())


def default_config_path() -> Path:
    """Find robot_actions.yaml in the installed share dir or the source tree."""
    env = os.environ.get("G1_HRI_ACTIONS")
    if env:
        return Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("unitree_g1_social_hri")) / "config" / "robot_actions.yaml"
        if p.is_file():
            return p
    except Exception:  # ament not available (unit tests, no-ROS demo)
        pass
    return Path(__file__).resolve().parent.parent / "config" / "robot_actions.yaml"


def load_catalog(path: Optional[str] = None) -> ActionCatalog:
    cfg_path = Path(path) if path else default_config_path()
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    actions = {}
    for name, spec in (cfg.get("actions") or {}).items():
        actions[name] = ActionSpec(
            name=name,
            type=spec.get("type", "motion"),
            description=spec.get("description", ""),
            keywords=[k.lower() for k in spec.get("keywords", [])],
            speech=spec.get("speech", ""),
        )
    safety = cfg.get("safety", {})
    if safety.get("physical_robot_enabled", False):
        raise RuntimeError("physical_robot_enabled must stay false in this teaching package")
    return ActionCatalog(robot_name=cfg.get("robot_name", "Unitree G1"), actions=actions, safety=safety)
