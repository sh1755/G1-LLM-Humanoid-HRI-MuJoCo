# Safety design – why the LLM never drives the motors

| Layer | Where | What it stops |
|---|---|---|
| JSON schema with `enum` of approved actions | `llm_client.action_schema()` | a schema-compliant LLM naming anything else |
| Allowlist check | `action_validator.py` | unknown / hallucinated actions (e.g. `backflip`) |
| Forbidden fields | `action_validator.FORBIDDEN_KEYS` | joint angles, torques, gains, code inside the JSON |
| Max 3 actions, speech ≤ 300 chars | `robot_actions.yaml → safety` | runaway sequences, very long speech |
| Rate limit 0.5 s | validator | command flooding |
| Emergency stop | `speech_input_node` (stop words, `/stop`) → `/hri/emergency_stop` | motion continues while someone shouts "stop" – detected locally, no LLM |
| Second allowlist check | `g1_simulation_node.on_action` | messages published directly to `/hri/robot_action` |
| Human-written keyframes | `motion_primitives.PRIMITIVES` | any motion nobody reviewed |
| Joint limits − 0.05 rad, ≤ 2 rad/s, min-jerk | `build_trajectory()` | jumps, limit hits, fast motions |
| Torque clipping to the model's `ctrlrange` | `g1_sim.compute_torques()` | unrealistic actuator effort |
| Tracking-error watchdog 0.6 rad | `g1_sim.step()` | collisions / stuck joints (action aborted, pose held) |
| Fixed base | `g1_sim.make_fixed_base_scene()` | falls – gestures cannot unbalance the robot |
| Physical robot disabled | no `unitree_sdk2`, no `/lowcmd`, `physical_robot_enabled: false` enforced | accidental commands to real hardware |

## Why not let the LLM output joint commands?
LLMs can hallucinate, be manipulated by prompt injection ("ignore your rules and…"),
return malformed numbers and respond with variable latency (hundreds of ms to seconds),
while a joint controller needs deterministic updates every 2 ms. The LLM is therefore
used only for **what** to do (high-level intent and dialogue); **how** to move is decided
by reviewed motion primitives and a classical controller.

## Before any real-robot work (not part of this practical)
Real G1 control needs Unitree's documented procedures (debug mode, gantry/suspension,
`unitree_hg` LowCmd with CRC, network set-up), a risk assessment and supervision.
Unitree's *high-level* APIs (e.g. locomotion clients) are **not** available in
unitree_mujoco; the simulator only bridges the low-level LowCmd/LowState interface.
