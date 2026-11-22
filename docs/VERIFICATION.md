# Verification record (what was actually executed)

Build environment: Ubuntu 24.04 container, Python 3.13, 2 CPU cores, no GPU, no display,
no microphone, no ROS 2 (installer blocked), outbound internet limited to package
registries and GitHub. Date: 22 Nov 2026.

## Executed and passing
| Item | Result |
|---|---|
| `pytest tests` with MuJoCo 4.0.1 | 74 passed |
| `pytest tests` with MuJoCo 3.17.0 (the version pip installs for Python 3.10) | 74 passed |
| Official G1 `g1_29dof.xml` loads; fixed-base conversion; 29 actuators map 1:1 to unitree_hg motor order | yes |
| Every approved action executed in MuJoCo physics, max upper-body tracking error < 0.25 rad (typically ~0.03 rad at the end), base fixed, torques within limits | yes |
| Emergency stop holds the pose and blocks new actions until released | yes |
| Six node files run together on an in-process rclpy stand-in (`tests/fake_ros`) – typed command → motion → CSV log; 2-action sequence; rejection of an LLM-invented action; stop word e-stop | yes |
| `examples/offline_pipeline_demo.py` (output in `assets/generated/offline_demo_output.txt`) | yes |
| `scripts/download_models.sh` – G1 clone, checkout of pinned commit, model check | yes |
| All shell scripts: `bash -n` and `shellcheck -S warning` | clean |
| RL extension: `gymnasium` `check_env`; PPO 300 k steps in 553 s; mean final hand–target distance 5.4 cm (45 % < 5 cm) vs 55.7 cm for a random policy over 20 evaluation episodes (`rl_extension/runs/`) | yes |
| Pose signs (elbow 0 = 90° bend, ≈1.5 rad = straight arm; shoulder pitch < 0 lifts forward) checked by forward kinematics and renders | yes |
| OpenAI Responses API and Ollama `/api/chat` payloads | checked against current official docs; exercised with mocked HTTP only |

## NOT executed here – check on a lab PC before teaching
| Item | Why not | How to check (≈15 min) |
|---|---|---|
| `colcon build`, `ros2 launch`, real DDS/CycloneDDS, `ros2 topic` CLI | ROS 2 could not be installed in the build environment | `./scripts/build_ros2.sh && ./scripts/run_text_demo.sh`, type "wave your right hand" |
| MuJoCo interactive viewer window (`mujoco.viewer.launch_passive`) | no display | same as above, window should open |
| Microphone + Vosk / Whisper STT | no microphone; Vosk model host not reachable | `./scripts/run_voice_demo.sh`, say "raise both arms" |
| espeak-ng / Piper / pyttsx3 audio | no audio device | `tts_backend:=espeak` |
| Live OpenAI and Ollama calls | no key / no Ollama server | `llm_backend:=ollama` then `openai` |
| `scripts/install.sh` apt steps on Ubuntu 22.04 / Python 3.10 | different OS | run once on a lab image |
| The ROS 2 terminal outputs shown in the student guide | labelled *illustrative* there | compare with your run and update screenshots |

## Known behaviour (documented, not a bug)
* Pure PD control leaves a ~0.13 rad steady-state error on the raised shoulder roll during
  `wave_right_hand` (gravity). Peak upper-body tracking error during any action stays < 0.25 rad
  (tested). Gravity compensation is left as a student exercise (guide §15).
* Monitor CSV durations from the fake-rclpy test are short because that test runs physics
  faster than real time (`realtime:=false`).
