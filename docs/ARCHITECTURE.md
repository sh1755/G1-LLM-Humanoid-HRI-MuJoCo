# Architecture and message formats

## Data flow
1. **speech_input_node** – keyboard (`input_mode:=text`) or microphone (`voice`, Vosk / Whisper).
   Publishes `{"id","text","source","t_input"}` on `/hri/speech_text`. Stop words → `/hri/emergency_stop = true`.
2. **llm_interaction_node** – `CommandInterpreter` (rule | ollama | openai, automatic rule fallback).
   Publishes `{"id","text","request":{"actions":[...],"speech":"..."},"backend","llm_latency_s","t_input","t_llm"}`.
3. **action_validator_node** – `ActionValidator`. Approved → `/hri/robot_action {"id","actions","t_input","t_llm","t_validated"}`
   and `/hri/speech_output {"id","text"}`; rejected → `/hri/robot_status {"stage":"validator","state":"rejected","reasons"}`.
4. **g1_simulation_node** – queue (max 3) → `build_trajectory()` → PD torque control at 500 Hz →
   `/hri/robot_status` (`executing`, `done`, `aborted`, `stopped`) and `/joint_states` at 50 Hz.
5. **speech_output_node** – print / espeak-ng / Piper / pyttsx3.
6. **hri_monitor_node** – joins all messages by `id`, writes CSV with
   `llm_ms, validation_ms, dispatch_ms, input_to_motion_ms, input_to_done_ms`.

## Controller
For every joint i (29 motors, same order as `unitree_hg` LowCmd `motor_cmd[i]`):

    tau_i = Kp_i (q*_i(t) - q_i) - Kd_i dq_i ,   clipped to the actuator range of g1_29dof.xml

Kp/Kd: legs & waist 150/4, arms 40/1 (Unitree g1_dual_arm_example values), wrists 15/0.5.
`q*(t)` comes from minimum-jerk interpolation between reviewed keyframes:
`s(τ) = 10τ³ − 15τ⁴ + 6τ⁵`, `q*(t) = q_a + (q_b − q_a)·s(t/T)`, with T increased if
`1.875·|q_b − q_a|/T > 2 rad/s`.

## Low-level vs high-level control on the G1
* **Low level** (`unitree_hg/LowCmd` on `/lowcmd`): per motor `q, dq, kp, kd, tau`, 500 Hz;
  supported by unitree_ros2, unitree_sdk2 and the unitree_mujoco bridge. `G1Simulator.lowcmd_dict()`
  shows what such a message would contain – it is never sent.
* **High level** (locomotion / arm action clients in unitree_sdk2): run on the robot's own
  controller; they are **not** reproduced by unitree_mujoco.

## Alternative pathway (advanced)
`unitree_mujoco/simulate_python` + `unitree_sdk2_python` simulates the robot behind the
same DDS LowCmd/LowState interface as the hardware (set `ROBOT="g1"`, domain 1, interface `lo`).
A bridge node could translate `/hri/robot_action` into LowCmd messages for that simulator.
This package uses direct MuJoCo control instead because it needs no extra build and is
easier to inspect in class.
