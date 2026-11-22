#!/usr/bin/env bash
# Download the OFFICIAL Unitree G1 MuJoCo model (+ optional speech models).
#   ./scripts/download_models.sh            # G1 model + Vosk small English model
#   ./scripts/download_models.sh --piper    # also a Piper TTS voice
#   ./scripts/download_models.sh --whisper  # also pre-download Whisper base.en
set -euo pipefail
. "$(dirname "$0")/common.sh"
MODELS="$PROJECT_ROOT/models"; mkdir -p "$MODELS"
UNITREE_MUJOCO_COMMIT="${UNITREE_MUJOCO_COMMIT:-1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d}"

if [ -f "$G1_MODEL_DIR/g1_29dof.xml" ]; then
  echo "==> G1 model already present in $G1_MODEL_DIR"
else
  echo "==> Cloning unitreerobotics/unitree_mujoco (BSD-3-Clause) at $UNITREE_MUJOCO_COMMIT"
  TMP=$(mktemp -d)
  git clone https://github.com/unitreerobotics/unitree_mujoco.git "$TMP/unitree_mujoco"
  git -C "$TMP/unitree_mujoco" checkout -q "$UNITREE_MUJOCO_COMMIT" || \
    echo "   (commit not found - using latest; re-run tests/ if anything changed)"
  mkdir -p "$G1_MODEL_DIR"
  cp -r "$TMP/unitree_mujoco/unitree_robots/g1/." "$G1_MODEL_DIR/"
  cp "$TMP/unitree_mujoco/LICENSE" "$G1_MODEL_DIR/LICENSE_unitree_mujoco_BSD3"
  git -C "$TMP/unitree_mujoco" rev-parse HEAD > "$G1_MODEL_DIR/SOURCE_COMMIT.txt"
  rm -rf "$TMP"
fi
python3 - <<PY
import mujoco
m = mujoco.MjModel.from_xml_path("$G1_MODEL_DIR/scene_29dof.xml")
print(f"   G1 model OK: {m.nu} actuators, {m.njnt} joints, timestep {m.opt.timestep}s")
PY

if [ ! -d "$VOSK_MODEL_DIR" ]; then
  echo "==> Vosk small English STT model (~40 MB)"
  wget -q --show-progress -O /tmp/vosk.zip https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip \
    && unzip -q /tmp/vosk.zip -d "$MODELS" && rm /tmp/vosk.zip \
    || echo "   Vosk download failed - voice mode needs it, text mode does not."
fi

for a in "$@"; do
  case "$a" in
    --piper)
      echo "==> Piper voice en_US-lessac-medium"
      mkdir -p "$MODELS/piper" && (cd "$MODELS/piper" && python3 -m piper.download_voices en_US-lessac-medium) ;;
    --whisper)
      python3 -c "import whisper; whisper.load_model('base.en'); print('Whisper base.en cached')" ;;
  esac
done
echo "Models are in $MODELS"
