"""Project D (optional): a SMALL reinforcement-learning example on the official G1 model.

Task   : move the right hand of the fixed-base G1 to a random target in front of it.
Agent  : PPO (Stable-Baselines3) outputs changes to 4 right-arm joint targets.
Control: the same joint PD controller, joint limits and torque limits as the
         ROS 2 demo (unitree_g1_social_hri.g1_sim).

This is an educational extension that shows the RL loop
    observation -> policy -> action -> simulation -> reward -> policy update.
It is NOT a walking or balance controller; whole-body humanoid locomotion is
usually trained with thousands of parallel environments on a GPU (e.g. Isaac
Lab or unitree_rl_gym) and is outside the scope of this practical.
"""
from __future__ import annotations

import sys
from pathlib import Path

import gymnasium as gym
import numpy as np
from gymnasium import spaces

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "unitree_g1_social_hri"))
from unitree_g1_social_hri import motion_primitives as mp  # noqa: E402
from unitree_g1_social_hri.g1_sim import G1Simulator  # noqa: E402

ARM = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint",
       "right_shoulder_yaw_joint", "right_elbow_joint"]
HAND_BODY = "right_wrist_yaw_link"


class G1ReachEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 25}

    def __init__(self, episode_steps: int = 100, frame_skip: int = 10, render_mode=None):
        super().__init__()
        self.sim = G1Simulator()
        self.idx = np.array([mp.JOINT_INDEX[j] for j in ARM])
        self.episode_steps, self.frame_skip = episode_steps, frame_skip
        self.render_mode = render_mode
        self._renderer = None
        self.action_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)
        # arm q (4), arm dq (4), hand xyz (3), target xyz (3), hand-target (3)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(17,), dtype=np.float32)
        self.max_delta = 0.05  # rad change of the joint target per step (safety: rate limit)

    def _obs(self):
        q, dq = self.sim.joint_positions()[self.idx], self.sim.joint_velocities()[self.idx]
        hand = self.sim.body_position(HAND_BODY)
        return np.concatenate([q, 0.1 * dq, hand, self.target, self.target - hand]).astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.sim.set_joint_positions(mp.neutral_vector())
        self.sim.q_target = mp.neutral_vector()
        # random target in front of the right shoulder (reachable box)
        self.target = np.array([self.np_random.uniform(0.25, 0.45),
                                self.np_random.uniform(-0.40, -0.10),
                                self.np_random.uniform(0.95, 1.35)])
        self.t = 0
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=float), -1, 1)
        q_t = self.sim.q_target.copy()
        q_t[self.idx] += self.max_delta * a
        self.sim.q_target = mp.clamp_to_limits(q_t)   # same joint-limit guard as the demo
        for _ in range(self.frame_skip):
            tau = self.sim.compute_torques()
            self.sim.data.ctrl[self.sim.act_id] = tau
            self.sim.mujoco.mj_step(self.sim.model, self.sim.data)
        self.t += 1
        dist = float(np.linalg.norm(self.sim.body_position(HAND_BODY) - self.target))
        reward = -dist - 0.01 * float(np.sum(a ** 2)) + (0.5 if dist < 0.05 else 0.0)
        truncated = self.t >= self.episode_steps
        return self._obs(), reward, False, truncated, {"distance": dist}

    def render(self):
        import mujoco
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.sim.model, 360, 360)
            self._cam = mujoco.MjvCamera()
            self._cam.lookat[:] = [0.1, -0.15, 1.05]
            self._cam.distance, self._cam.azimuth, self._cam.elevation = 1.9, 150, -10
        self._renderer.update_scene(self.sim.data, self._cam)
        # draw the target as a small red sphere
        scn = self._renderer.scene
        if scn.ngeom < scn.maxgeom:
            mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                np.array([0.03, 0, 0]), self.target, np.eye(3).flatten(),
                                np.array([0.9, 0.1, 0.1, 1.0], dtype=np.float32))
            scn.ngeom += 1
        return self._renderer.render()
