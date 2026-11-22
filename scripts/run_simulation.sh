#!/usr/bin/env bash
# Start the whole system (simulator + validator + interpreter + TTS + monitor).
#   ./scripts/run_simulation.sh                          # rule-based, MuJoCo viewer
#   ./scripts/run_simulation.sh llm_backend:=ollama
#   ./scripts/run_simulation.sh llm_backend:=openai tts_backend:=espeak
#   ./scripts/run_simulation.sh headless:=true           # no window (SSH / lab PCs)
# Then send commands from a second terminal: ./scripts/run_text_demo.sh --input-only
set -eo pipefail
. "$(dirname "$0")/common.sh"
require_model; source_ros; require_build
exec ros2 launch unitree_g1_social_hri social_hri.launch.py model_dir:="$G1_MODEL_DIR" "$@"
