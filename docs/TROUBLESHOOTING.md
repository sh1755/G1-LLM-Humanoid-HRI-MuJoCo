# Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Official Unitree G1 model not found` | model folder missing / moved | `./scripts/download_models.sh` or `export G1_MODEL_DIR=/path/to/folder/with/g1_29dof.xml` |
| `Package 'unitree_g1_social_hri' not found` | workspace not built or not sourced | `./scripts/build_ros2.sh` then `source ros2_ws/install/setup.bash` |
| MuJoCo window does not open / `GLFW` error | no display (SSH, WSL without GUI) | `./scripts/run_simulation.sh headless:=true`; check `echo $DISPLAY`; WSL2: use WSLg |
| `mujoco` import fails / very old version | pip went to a different Python | `python3 -m pip install --user "mujoco>=3.2"`; `python3 -c "import mujoco;print(mujoco.__version__)"` |
| numpy errors in ROS 2 messages | pip installed numpy 2.x | `python3 -m pip install --user "numpy<2"` |
| Nodes cannot see each other | different `ROS_DOMAIN_ID` / RMW in the terminals | open terminals via the scripts (they source `scripts/common.sh`), or export the same values |
| `rmw_cyclonedds_cpp` not found | package missing | `sudo apt install ros-humble-rmw-cyclonedds-cpp` or `unset RMW_IMPLEMENTATION` |
| Another student's robot moves | shared network, same domain | set a unique `ROS_DOMAIN_ID` in `.env`; keep `ROS_LOCALHOST_ONLY=1` |
| `Voice input failed … PortAudio` | no microphone / ALSA | `sudo apt install libportaudio2`; `python3 -m sounddevice`; pick device in Settings → Sound |
| `vosk_model_path parameter is empty` | Vosk model not downloaded | `./scripts/download_models.sh` (needs internet once) |
| STT hears nothing / wrong words | noise, distance, threshold | speak close to the mic; Whisper: lower `energy_threshold`; use text mode |
| `openai failed -> rule fallback (RuntimeError: OPENAI_API_KEY is not set)` | no key | add the key to `.env`, open a new terminal |
| `HTTP 401` / `HTTP 429` from OpenAI | wrong key / quota | check the key and billing; the demo keeps working via the fallback |
| `ollama failed -> rule fallback (URLError …)` | Ollama not running | `ollama serve` (or `systemctl status ollama`), `ollama pull llama3.2` |
| LLM answers slowly (> 5 s) | large local model on CPU | use a smaller model (`OLLAMA_MODEL=llama3.2:1b`) or the rule backend |
| `REJECTED … not on the approved list` | LLM invented an action | expected behaviour – improve the prompt or add a primitive (see guide §15) |
| `REJECTED … emergency stop is active` | e-stop latched | `/resume` in the text terminal or `ros2 topic pub --once /hri/emergency_stop std_msgs/msg/Bool "{data: false}"` |
| `REJECTED … requests are arriving too quickly` | rate limit | wait 0.5 s between commands |
| Robot does nothing, no error | command not understood | check `ros2 topic echo /hri/llm_response`; say e.g. "wave your right hand" |
| No audio from TTS | `tts_backend:=print` (default) | `tts_backend:=espeak` (`sudo apt install espeak-ng`) |
