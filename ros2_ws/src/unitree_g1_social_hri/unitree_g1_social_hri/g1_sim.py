"""ROS-free MuJoCo wrapper for the official Unitree G1 29-DoF model.

Pathway:  approved action -> motion_primitives.build_trajectory() ->
          joint PD controller (tau = Kp (q* - q) - Kd dq) -> clip to the
          actuator torque range of g1_29dof.xml -> MuJoCo physics.

The official model uses torque (``<motor>``) actuators, exactly like the
real robot's low-level interface (LowCmd carries q, dq, kp, kd, tau per
motor).  We therefore run the PD loop ourselves instead of assuming any
Unitree high-level API exists in simulation (it does not).

Fixed base: the model's ``floating_base_joint`` is removed at load time, so
the pelvis is welded to the world.  This is a safe classroom set-up for
upper-body social gestures; it is NOT a balance or walking controller.
"""
from __future__ import annotations

import os
import re
import threading
from pathlib import Path
from typing import Dict, Optional, Sequence

import numpy as np

from . import motion_primitives as mp

# PD gains (N m / rad, N m s / rad).  Arm values follow the Unitree
# g1_dual_arm_example (kp 40, kd 1); legs/waist are stiffer to hold posture.
DEFAULT_KP = {"leg": 150.0, "waist": 150.0, "arm": 40.0, "wrist": 15.0}
DEFAULT_KD = {"leg": 4.0, "waist": 4.0, "arm": 1.0, "wrist": 0.5}
TRACKING_ERROR_ABORT = 0.6  # rad; abort an action if tracking error exceeds this


def joint_group(name: str) -> str:
    if "wrist" in name:
        return "wrist"
    if "shoulder" in name or "elbow" in name:
        return "arm"
    if "waist" in name:
        return "waist"
    return "leg"


def find_model_dir(explicit: Optional[str] = None) -> Path:
    """Locate the official G1 model folder (contains g1_29dof.xml + meshes/)."""
    candidates = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    if os.environ.get("G1_MODEL_DIR"):
        candidates.append(Path(os.environ["G1_MODEL_DIR"]).expanduser())
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidates.append(parent / "models" / "unitree_g1")
    candidates.append(Path.home() / "RO603_Week8_Unitree_G1_Speech_LLM_Control_ROS2" / "models" / "unitree_g1")
    for c in candidates:
        if (c / "g1_29dof.xml").is_file():
            return c
    raise FileNotFoundError(
        "Official Unitree G1 model not found. Run scripts/download_models.sh or set "
        "G1_MODEL_DIR to the folder containing g1_29dof.xml")


def make_fixed_base_scene(model_dir: Path) -> Path:
    """Write g1_29dof_fixed_base.xml + scene next to the official files.

    Only one line is changed: the free joint of the pelvis is removed.
    Everything else (masses, meshes, limits, actuators) is the official model.
    """
    src = (model_dir / "g1_29dof.xml").read_text()
    fixed, n = re.subn(r'\s*<joint\s+name="floating_base_joint"[^>]*/>', "", src)
    if n == 0:
        fixed, n = re.subn(r"\s*<freejoint[^>]*/>", "", src)
    if n != 1:
        raise RuntimeError("Could not find the G1 floating base joint to remove")
    fixed = fixed.replace('model="g1_29dof"', 'model="g1_29dof_fixed_base"')
    robot = model_dir / "g1_29dof_fixed_base.xml"
    robot.write_text(fixed)
    scene_src = (model_dir / "scene_29dof.xml").read_text()
    scene = model_dir / "scene_29dof_fixed_base.xml"
    scene.write_text(scene_src.replace('file="g1_29dof.xml"', 'file="g1_29dof_fixed_base.xml"'))
    return scene


class G1Simulator:
    """Fixed-base G1 in MuJoCo with joint-space PD control of all 29 motors."""

    def __init__(self, model_dir: Optional[str] = None, fixed_base: bool = True):
        import mujoco  # imported here so the rest of the package works without it

        self.mujoco = mujoco
        self.model_dir = find_model_dir(model_dir)
        scene = make_fixed_base_scene(self.model_dir) if fixed_base else self.model_dir / "scene_29dof.xml"
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        self.lock = threading.Lock()

        # Low-level actuator mapping: our index i -> MuJoCo actuator / qpos / dof
        self.act_id = np.zeros(mp.NUM_JOINTS, dtype=int)
        self.qpos_adr = np.zeros(mp.NUM_JOINTS, dtype=int)
        self.dof_adr = np.zeros(mp.NUM_JOINTS, dtype=int)
        self.torque_limit = np.zeros(mp.NUM_JOINTS)
        self.model_limits = np.zeros((mp.NUM_JOINTS, 2))
        for i, name in enumerate(mp.G1_JOINT_NAMES):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError(f"Joint {name} missing from model")
            aid = next(a for a in range(self.model.nu) if self.model.actuator_trnid[a, 0] == jid)
            self.act_id[i] = aid
            self.qpos_adr[i] = self.model.jnt_qposadr[jid]
            self.dof_adr[i] = self.model.jnt_dofadr[jid]
            self.torque_limit[i] = self.model.actuator_ctrlrange[aid, 1]
            self.model_limits[i] = self.model.jnt_range[jid]
        groups = [joint_group(n) for n in mp.G1_JOINT_NAMES]
        self.kp = np.array([DEFAULT_KP[g] for g in groups])
        self.kd = np.array([DEFAULT_KD[g] for g in groups])

        self.q_target = mp.neutral_vector()
        self.set_joint_positions(self.q_target)
        self._traj_t: Optional[np.ndarray] = None
        self._traj_q: Optional[np.ndarray] = None
        self._traj_start = 0.0
        self.current_action: Optional[str] = None
        self.estop = False
        self.last_error = 0.0

    # ------------------------------------------------------------------ state
    @property
    def time(self) -> float:
        return float(self.data.time)

    def joint_positions(self) -> np.ndarray:
        return self.data.qpos[self.qpos_adr].copy()

    def joint_velocities(self) -> np.ndarray:
        return self.data.qvel[self.dof_adr].copy()

    def set_joint_positions(self, q: Sequence[float]) -> None:
        self.data.qpos[self.qpos_adr] = np.asarray(q, dtype=float)
        self.data.qvel[:] = 0.0
        self.mujoco.mj_forward(self.model, self.data)

    def body_position(self, body: str) -> np.ndarray:
        bid = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_BODY, body)
        return self.data.xpos[bid].copy()

    # ---------------------------------------------------------------- actions
    def start_action(self, action: str) -> float:
        """Begin an approved action; returns its duration in seconds."""
        if self.estop:
            raise RuntimeError("Emergency stop is active")
        t, q = mp.build_trajectory(action, self.joint_positions())
        with self.lock:
            self._traj_t, self._traj_q = t, q
            self._traj_start = self.time
            self.current_action = action
        return float(t[-1])

    def is_busy(self) -> bool:
        return self.current_action is not None

    def emergency_stop(self) -> None:
        """Cancel motion and hold the current measured pose."""
        with self.lock:
            self.estop = True
            self._traj_t = self._traj_q = None
            self.current_action = None
            self.q_target = self.joint_positions()

    def reset_emergency_stop(self) -> None:
        self.estop = False

    # ------------------------------------------------------------------ physics
    def _update_target(self) -> Optional[str]:
        """Advance the trajectory; return 'done' or 'aborted' when it ends."""
        if self._traj_t is None:
            return None
        elapsed = self.time - self._traj_start
        idx = int(np.searchsorted(self._traj_t, elapsed))
        if idx >= len(self._traj_t):
            self.q_target = self._traj_q[-1].copy()
            self._traj_t = self._traj_q = None
            self.current_action = None
            return "done"
        self.q_target = self._traj_q[idx].copy()
        return None

    def compute_torques(self) -> np.ndarray:
        q, dq = self.joint_positions(), self.joint_velocities()
        tau = self.kp * (self.q_target - q) - self.kd * dq
        return np.clip(tau, -self.torque_limit, self.torque_limit)

    def step(self, n: int = 1) -> Optional[str]:
        """Run n physics steps. Returns 'done'/'aborted' if an action ended."""
        event = None
        for _ in range(n):
            with self.lock:
                ev = self._update_target()
                event = ev or event
                tau = self.compute_torques()
                self.data.ctrl[self.act_id] = tau
                self.mujoco.mj_step(self.model, self.data)
                err = float(np.max(np.abs(self.q_target - self.joint_positions())[12:]))
                self.last_error = err
                if self.current_action and err > TRACKING_ERROR_ABORT:
                    self._traj_t = self._traj_q = None
                    self.current_action = None
                    self.q_target = self.joint_positions()
                    event = "aborted"
        return event

    def run_action_blocking(self, action: str, record_every: int = 0):
        """Execute an action to completion (used by tests and offline demos)."""
        duration = self.start_action(action)
        log = []
        steps = int((duration + 0.5) / self.dt)
        result = "timeout"
        for k in range(steps):
            ev = self.step()
            if record_every and k % record_every == 0:
                log.append((self.time, self.q_target.copy(), self.joint_positions()))
            if ev:
                result = ev
                break
        return result, log

    # ------------------------------------------------------------ Unitree bridge
    def lowcmd_dict(self) -> Dict[str, list]:
        """What a unitree_hg/LowCmd would contain for the current target.

        Educational dry-run only: index i matches motor_cmd[i] on the robot.
        Nothing in this package publishes to /lowcmd.
        """
        return {"q": self.q_target.tolist(), "dq": [0.0] * mp.NUM_JOINTS,
                "kp": self.kp.tolist(), "kd": self.kd.tolist(), "tau": [0.0] * mp.NUM_JOINTS}
