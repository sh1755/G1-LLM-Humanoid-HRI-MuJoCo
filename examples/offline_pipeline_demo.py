"""Same pipeline as the ROS 2 system, in ONE Python process and WITHOUT ROS 2.

    text -> CommandInterpreter (rule / ollama / openai) -> ActionValidator
         -> motion primitive -> PD control of the official G1 model in MuJoCo

Use it to (a) check MuJoCo + the G1 model work before building ROS 2, (b) try
the LLM prompt quickly, (c) record GIFs of each action.  The ROS 2 nodes import
exactly these classes, so behaviour is identical; only the transport differs.

    python3 examples/offline_pipeline_demo.py                         # scripted commands
    python3 examples/offline_pipeline_demo.py --interactive           # type commands
    python3 examples/offline_pipeline_demo.py --backend ollama
    python3 examples/offline_pipeline_demo.py --gif assets/generated  # save GIFs (headless)
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "unitree_g1_social_hri"))
os.environ.setdefault("G1_MODEL_DIR", str(ROOT / "models" / "unitree_g1"))

from unitree_g1_social_hri.action_catalog import load_catalog  # noqa: E402
from unitree_g1_social_hri.action_validator import ActionValidator  # noqa: E402
from unitree_g1_social_hri.g1_sim import G1Simulator  # noqa: E402
from unitree_g1_social_hri.llm_client import CommandInterpreter  # noqa: E402

SCRIPT = [
    "Hello robot!",
    "Can you wave your hand?",
    "Raise both arms",
    "Return to neutral",
    "Tell me about humanoid robots",
    "G1, introduce yourself and then wave at me",
    "Do a backflip",
]


def run(text, interp, validator, sim, gif_dir=None, renderer=None, cam=None):
    t0 = time.perf_counter()
    res = interp.interpret(text)
    t1 = time.perf_counter()
    val = validator.validate(res.request)
    t2 = time.perf_counter()
    print(f'\nyou> {text}')
    print(f'  [{res.backend:6s}] request   {res.request}  ({(t1 - t0) * 1000:.1f} ms)'
          + (f'  fallback: {res.fallback_reason}' if res.fallback_reason else ''))
    if not val.approved:
        print(f'  [valid.] REJECTED: {"; ".join(val.reasons)}')
        print('  G1 says: "Sorry, I cannot do that safely."')
        return
    print(f'  [valid.] approved  {val.actions or ["(speech only)"]}  ({(t2 - t1) * 1000:.2f} ms)')
    if val.speech:
        print(f'  G1 says: "{val.speech}"')
    for action in val.actions:
        frames = []
        t3 = time.perf_counter()
        t_sim0 = sim.time
        duration = sim.start_action(action)
        start_latency = (time.perf_counter() - t3) * 1000
        k, result = 0, "timeout"
        while k * sim.dt < duration + 0.5:
            ev = sim.step()
            if renderer is not None and k % 20 == 0:       # 25 fps
                renderer.update_scene(sim.data, cam)
                frames.append(renderer.render())
            k += 1
            if ev:
                result = ev
                break
        print(f'  [sim   ] {action}: {result} after {sim.time - t_sim0:.2f} s, '
              f'trajectory start {start_latency:.1f} ms, last tracking error {sim.last_error:.3f} rad')
        if frames and gif_dir:
            import imageio.v2 as imageio
            path = Path(gif_dir) / f"g1_{action}.gif"
            imageio.mimsave(path, frames, duration=0.04, loop=0)
            print(f"           saved {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="rule", choices=["rule", "ollama", "openai"])
    ap.add_argument("--interactive", action="store_true")
    ap.add_argument("--gif", default="", help="folder for GIFs (enables offscreen rendering)")
    args = ap.parse_args()

    catalog = load_catalog()
    interp = CommandInterpreter(catalog, args.backend)
    validator = ActionValidator(catalog)
    validator.catalog.safety["min_seconds_between_requests"] = 0.0  # scripted demo
    sim = G1Simulator()
    renderer = cam = None
    if args.gif:
        import mujoco
        Path(args.gif).mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(sim.model, 400, 360)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0, 0, 0.95]
        cam.distance, cam.azimuth, cam.elevation = 2.5, 160, -5
    print(f"G1 loaded: {sim.model_dir}  (fixed base, {sim.model.nu} motors, dt={sim.dt * 1000:.0f} ms)")
    print(f"Interpreter backend: {args.backend}")

    if args.interactive:
        while True:
            try:
                text = input("\nyou> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if text in ("/quit", "quit", "exit"):
                break
            run(text, interp, validator, sim, args.gif, renderer, cam)
    else:
        for text in SCRIPT:
            run(text, interp, validator, sim, args.gif, renderer, cam)
        print("\n--- What if a (misbehaving) LLM returns something unsafe? ---")
        for bad in ({"actions": ["backflip"], "speech": "Watch this!"},
                    {"actions": ["wave_right_hand"], "speech": "ok", "joint_positions": [3.1] * 29},
                    {"actions": ["bow", "bow", "bow", "bow"], "speech": "bowing"}):
            v = validator.validate(bad)
            print(f"  LLM output {bad if len(str(bad)) < 90 else str(bad)[:87] + '...'}")
            print(f"    -> {'APPROVED' if v.approved else 'REJECTED: ' + '; '.join(v.reasons)}")


if __name__ == "__main__":
    main()
