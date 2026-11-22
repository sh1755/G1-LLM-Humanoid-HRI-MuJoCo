#!/usr/bin/env bash
# Project A - text-controlled humanoid (no microphone, no LLM, no internet).
#   ./scripts/run_text_demo.sh                 # starts the system in the background + keyboard here
#   ./scripts/run_text_demo.sh --input-only    # keyboard only (system already running elsewhere)
#   ./scripts/run_text_demo.sh llm_backend:=ollama
set -eo pipefail
. "$(dirname "$0")/common.sh"
require_model; source_ros; require_build
if [ "${1:-}" = "--input-only" ]; then
  exec ros2 run unitree_g1_social_hri speech_input_node --ros-args -p input_mode:=text
fi
LOG="$PROJECT_ROOT/assets/last_launch.log"
ros2 launch unitree_g1_social_hri social_hri.launch.py model_dir:="$G1_MODEL_DIR" "$@" > "$LOG" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null' EXIT
echo "System starting (log: $LOG). Waiting for nodes..."
for _ in $(seq 1 30); do
  ros2 node list 2>/dev/null | grep -q g1_simulation_node && break; sleep 1
done
ros2 node list
ros2 run unitree_g1_social_hri speech_input_node --ros-args -p input_mode:=text
