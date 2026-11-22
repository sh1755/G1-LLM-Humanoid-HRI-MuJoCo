"""Train / evaluate PPO on the G1 right-hand reaching task (Project D, optional).

    pip install "stable-baselines3>=2.3" "gymnasium>=0.29"
    python3 rl_extension/train_ppo_reach.py --steps 300000         # ~7 min here (2 CPU cores); 5-20 min on a laptop
    python3 rl_extension/train_ppo_reach.py --evaluate              # compare with random policy

Outputs (in rl_extension/runs/): ppo_g1_reach.zip, learning_curve.csv,
learning_curve.png and eval_results.json.
"""
from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor

from g1_reach_env import G1ReachEnv

RUNS = Path(__file__).resolve().parent / "runs"


class CurveLogger(BaseCallback):
    """Store (timesteps, mean episode return, mean final distance)."""

    def __init__(self):
        super().__init__()
        self.rows, self.returns, self.dists = [], [], []

    def _on_step(self) -> bool:
        for info in self.locals["infos"]:
            if "episode" in info:
                self.returns.append(info["episode"]["r"])
                self.dists.append(info["distance"])
        return True

    def _on_rollout_end(self) -> None:
        if self.returns:
            self.rows.append((self.num_timesteps, float(np.mean(self.returns[-20:])),
                              float(np.mean(self.dists[-20:]))))
            print(f"  steps={self.num_timesteps:6d}  return={self.rows[-1][1]:7.2f}  "
                  f"final hand-target distance={self.rows[-1][2] * 100:5.1f} cm")


def evaluate(policy_fn, episodes=20, seed=123):
    env = G1ReachEnv()
    finals = []
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        done = False
        while not done:
            obs, r, term, trunc, info = env.step(policy_fn(obs))
            done = term or trunc
        finals.append(info["distance"])
    return float(np.mean(finals)), float(np.mean(np.array(finals) < 0.05))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=300000)
    ap.add_argument("--evaluate", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    RUNS.mkdir(exist_ok=True)
    model_path = RUNS / "ppo_g1_reach.zip"

    if not args.evaluate:
        env = Monitor(G1ReachEnv())
        model = PPO("MlpPolicy", env, n_steps=1000, batch_size=250, learning_rate=3e-4,
                    gamma=0.98, seed=args.seed, verbose=0)
        cb = CurveLogger()
        t0 = time.time()
        model.learn(total_timesteps=args.steps, callback=cb)
        print(f"Training took {time.time() - t0:.0f} s")
        model.save(model_path)
        with open(RUNS / "learning_curve.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timesteps", "mean_return", "mean_final_distance_m"])
            w.writerows(cb.rows)
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            r = np.array(cb.rows)
            fig, ax = plt.subplots(figsize=(6, 3.2))
            ax.plot(r[:, 0], r[:, 2] * 100, color="#1f5fbf", lw=2)
            ax.set_xlabel("environment steps")
            ax.set_ylabel("final hand-target distance (cm)")
            ax.set_title("PPO on G1 right-hand reaching (MuJoCo)")
            ax.grid(alpha=0.3)
            fig.tight_layout()
            fig.savefig(RUNS / "learning_curve.png", dpi=150)
        except ImportError:
            pass

    model = PPO.load(model_path)
    trained = evaluate(lambda o: model.predict(o, deterministic=True)[0])
    rng = np.random.default_rng(0)
    random = evaluate(lambda o: rng.uniform(-1, 1, 4))
    res = {"trained_mean_final_distance_m": trained[0], "trained_success_rate_<5cm": trained[1],
           "random_mean_final_distance_m": random[0], "random_success_rate_<5cm": random[1]}
    (RUNS / "eval_results.json").write_text(json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
