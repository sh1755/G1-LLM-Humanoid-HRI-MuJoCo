"""Predefined, safe upper-body motion primitives for the Unitree G1 (29-DoF model).

This module is deliberately ROS-free and MuJoCo-free so it can be unit-tested
anywhere.  It contains:

* the G1 29-DoF joint order (identical to the Unitree ``unitree_hg`` LowCmd
  motor index order documented in unitree_mujoco/unitree_robots/g1/
  g1_joint_index_dds.md),
* joint limits copied from the official ``g1_29dof.xml`` MuJoCo model,
* keyframe definitions for every approved action,
* minimum-jerk (quintic) interpolation with joint-limit clamping and
  velocity-based time scaling.

An LLM never produces joint angles.  It may only *name* one of the actions in
``PRIMITIVES``; the numbers below are written and reviewed by humans.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Mapping, Sequence, Tuple

import numpy as np

# --------------------------------------------------------------------------
# Joint order: index i here == motor index i in unitree_hg LowCmd_.motor_cmd
# == actuator i in the official g1_29dof.xml.
# --------------------------------------------------------------------------
G1_JOINT_NAMES: Tuple[str, ...] = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
NUM_JOINTS = len(G1_JOINT_NAMES)  # 29
JOINT_INDEX: Dict[str, int] = {n: i for i, n in enumerate(G1_JOINT_NAMES)}

# Joints that a social action may move.  Legs are never commanded.
UPPER_BODY_JOINTS: Tuple[str, ...] = G1_JOINT_NAMES[12:]
LEG_JOINTS: Tuple[str, ...] = G1_JOINT_NAMES[:12]

# (lower, upper) in radians, copied from unitree_mujoco g1_29dof.xml
G1_JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    "left_hip_pitch_joint": (-2.5307, 2.8798),
    "left_hip_roll_joint": (-0.5236, 2.9671),
    "left_hip_yaw_joint": (-2.7576, 2.7576),
    "left_knee_joint": (-0.087267, 2.8798),
    "left_ankle_pitch_joint": (-0.87267, 0.5236),
    "left_ankle_roll_joint": (-0.2618, 0.2618),
    "right_hip_pitch_joint": (-2.5307, 2.8798),
    "right_hip_roll_joint": (-2.9671, 0.5236),
    "right_hip_yaw_joint": (-2.7576, 2.7576),
    "right_knee_joint": (-0.087267, 2.8798),
    "right_ankle_pitch_joint": (-0.87267, 0.5236),
    "right_ankle_roll_joint": (-0.2618, 0.2618),
    "waist_yaw_joint": (-2.618, 2.618),
    "waist_roll_joint": (-0.52, 0.52),
    "waist_pitch_joint": (-0.52, 0.52),
    "left_shoulder_pitch_joint": (-3.0892, 2.6704),
    "left_shoulder_roll_joint": (-1.5882, 2.2515),
    "left_shoulder_yaw_joint": (-2.618, 2.618),
    "left_elbow_joint": (-1.0472, 2.0944),
    "left_wrist_roll_joint": (-1.97222, 1.97222),
    "left_wrist_pitch_joint": (-1.61443, 1.61443),
    "left_wrist_yaw_joint": (-1.61443, 1.61443),
    "right_shoulder_pitch_joint": (-3.0892, 2.6704),
    "right_shoulder_roll_joint": (-2.2515, 1.5882),
    "right_shoulder_yaw_joint": (-2.618, 2.618),
    "right_elbow_joint": (-1.0472, 2.0944),
    "right_wrist_roll_joint": (-1.97222, 1.97222),
    "right_wrist_pitch_joint": (-1.61443, 1.61443),
    "right_wrist_yaw_joint": (-1.61443, 1.61443),
}

LIMIT_MARGIN = 0.05          # rad kept away from every hard limit
MAX_JOINT_VELOCITY = 2.0     # rad/s, conservative classroom limit

# Relaxed standing posture for the arms (all other joints 0).
# G1 convention (checked with forward kinematics on g1_29dof.xml):
#   elbow 0 rad = forearm bent 90 deg forward, elbow ~1.5 rad = straight arm;
#   shoulder pitch < 0 lifts the arm forwards/upwards;
#   right shoulder roll < 0 (left > 0) lifts the arm sideways.
NEUTRAL_POSE: Dict[str, float] = {
    "left_shoulder_pitch_joint": 0.20, "left_shoulder_roll_joint": 0.20,
    "left_elbow_joint": 1.10,
    "right_shoulder_pitch_joint": 0.20, "right_shoulder_roll_joint": -0.20,
    "right_elbow_joint": 1.10,
}


@dataclass(frozen=True)
class Keyframe:
    """Target pose reached ``duration`` seconds after the previous keyframe."""

    duration: float
    pose: Mapping[str, float]


def _pose(**overrides: float) -> Dict[str, float]:
    """Neutral pose with some joints overridden (keyword = short joint name)."""
    p = dict(NEUTRAL_POSE)
    for short, value in overrides.items():
        p[short + "_joint"] = value
    return p


def _arm(side: str, pitch: float, roll: float, yaw: float, elbow: float) -> Dict[str, float]:
    """Arm pose given as RIGHT-arm values; roll and yaw are mirrored for the left arm."""
    m = 1.0 if side == "right" else -1.0
    return {f"{side}_shoulder_pitch": pitch, f"{side}_shoulder_roll": m * roll,
            f"{side}_shoulder_yaw": m * yaw, f"{side}_elbow": elbow}


def _wave(side: str) -> List[Keyframe]:
    """Upper arm out to the side, forearm vertical, forearm swings 3 times."""
    up = _arm(side, 0.0, -1.30, -1.57, 0.0)
    swing_a = dict(up, **{f"{side}_elbow": 0.40})
    swing_b = dict(up, **{f"{side}_elbow": -0.40})
    frames = [Keyframe(1.2, _pose(**up))]
    for _ in range(3):
        frames.append(Keyframe(0.40, _pose(**swing_a)))
        frames.append(Keyframe(0.40, _pose(**swing_b)))
    frames.append(Keyframe(0.40, _pose(**up)))
    frames.append(Keyframe(1.2, _pose()))
    return frames


def _hold(duration_up: float, hold: float, duration_down: float, **pose: float) -> List[Keyframe]:
    target = _pose(**pose)
    return [Keyframe(duration_up, target), Keyframe(hold, target), Keyframe(duration_down, _pose())]


_R_UP = dict(right_shoulder_pitch=-2.80, right_shoulder_roll=-0.60, right_elbow=1.40)
_L_UP = dict(left_shoulder_pitch=-2.80, left_shoulder_roll=0.60, left_elbow=1.40)

PRIMITIVES: Dict[str, List[Keyframe]] = {
    "neutral": [Keyframe(1.5, _pose())],
    "wave_right_hand": _wave("right"),
    "wave_left_hand": _wave("left"),
    "raise_both_arms": _hold(1.8, 1.5, 1.8, **_R_UP, **_L_UP),
    "raise_right_arm": _hold(1.5, 1.2, 1.5, **_R_UP),
    "raise_left_arm": _hold(1.5, 1.2, 1.5, **_L_UP),
    "point_forward": _hold(1.3, 1.2, 1.3, right_shoulder_pitch=-1.50,
                           right_shoulder_roll=0.0, right_elbow=1.50),
    "bow": _hold(1.5, 0.8, 1.5, waist_pitch=0.40),
}


# --------------------------------------------------------------------------
# Trajectory generation
# --------------------------------------------------------------------------
def pose_to_vector(pose: Mapping[str, float], base: Sequence[float] | None = None) -> np.ndarray:
    """Convert a {joint_name: angle} dict to a 29-vector (missing joints -> base or 0)."""
    q = np.zeros(NUM_JOINTS) if base is None else np.array(base, dtype=float).copy()
    for name, value in pose.items():
        if name not in JOINT_INDEX:
            raise KeyError(f"Unknown G1 joint '{name}'")
        q[JOINT_INDEX[name]] = float(value)
    return q


def neutral_vector() -> np.ndarray:
    return pose_to_vector(NEUTRAL_POSE)


def clamp_to_limits(q: np.ndarray, margin: float = LIMIT_MARGIN) -> np.ndarray:
    """Clamp a 29-vector to the official joint limits minus a safety margin."""
    out = np.array(q, dtype=float).copy()
    for i, name in enumerate(G1_JOINT_NAMES):
        lo, hi = G1_JOINT_LIMITS[name]
        out[i] = min(max(out[i], lo + margin), hi - margin)
    return out


def min_jerk(s: np.ndarray) -> np.ndarray:
    """Quintic blend 10s^3 - 15s^4 + 6s^5 (zero velocity/acceleration at ends)."""
    s = np.clip(s, 0.0, 1.0)
    return s ** 3 * (10.0 - 15.0 * s + 6.0 * s ** 2)


MIN_JERK_PEAK_VELOCITY_FACTOR = 1.875  # max d/ds of the quintic blend


def build_trajectory(action: str, q_start: Sequence[float], dt: float = 0.01,
                     max_velocity: float = MAX_JOINT_VELOCITY) -> Tuple[np.ndarray, np.ndarray]:
    """Return (times, positions[N, 29]) for an approved action.

    * starts from the robot's *measured* pose ``q_start`` (no jumps),
    * every keyframe is clamped to the joint limits minus ``LIMIT_MARGIN``,
    * leg joints are held at their start values (fixed-base upper-body demo),
    * a segment is slowed down if its peak joint velocity would exceed
      ``max_velocity``.
    """
    if action not in PRIMITIVES:
        raise KeyError(f"'{action}' is not an approved motion primitive")
    q0 = np.array(q_start, dtype=float)
    if q0.shape != (NUM_JOINTS,):
        raise ValueError(f"q_start must have {NUM_JOINTS} elements")

    times: List[float] = [0.0]
    points: List[np.ndarray] = [q0.copy()]
    t = 0.0
    prev = q0.copy()
    for kf in PRIMITIVES[action]:
        target = pose_to_vector(kf.pose, base=np.zeros(NUM_JOINTS))
        target[:12] = q0[:12]                 # never move the legs
        target = clamp_to_limits(target)
        dq = np.max(np.abs(target - prev))
        duration = max(kf.duration, MIN_JERK_PEAK_VELOCITY_FACTOR * dq / max_velocity, dt)
        n = max(int(round(duration / dt)), 1)
        s = min_jerk(np.arange(1, n + 1) / n)
        for k in range(n):
            points.append(prev + (target - prev) * s[k])
            times.append(t + (k + 1) * duration / n)
        t += duration
        prev = target
    return np.asarray(times), np.vstack(points)


def trajectory_duration(action: str) -> float:
    """Nominal duration (s) of an action starting from the neutral pose."""
    times, _ = build_trajectory(action, neutral_vector())
    return float(times[-1])
