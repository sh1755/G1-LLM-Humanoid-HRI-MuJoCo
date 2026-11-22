#!/usr/bin/env bash
# Project B/C - microphone + offline STT (+ optional LLM and TTS).
#   ./scripts/run_voice_demo.sh                                  # Vosk + rule-based + espeak
#   ./scripts/run_voice_demo.sh llm_backend:=ollama
#   ./scripts/run_voice_demo.sh stt_engine:=whisper llm_backend:=openai
set -eo pipefail
. "$(dirname "$0")/common.sh"
require_model; source_ros; require_build
python3 -c "import sounddevice as sd; print('Default input device:', sd.query_devices(kind='input')['name'])" \
  || { echo "No microphone found. Use ./scripts/run_text_demo.sh instead."; exit 1; }
exec ros2 launch unitree_g1_social_hri social_hri.launch.py model_dir:="$G1_MODEL_DIR" \
  input_mode:=voice vosk_model_path:="$VOSK_MODEL_DIR" tts_backend:=espeak "$@"
