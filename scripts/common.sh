#!/usr/bin/env bash
# Shared settings for all RO603 Week 8 scripts (sourced, not executed).
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$PROJECT_ROOT/ros2_ws"
export G1_MODEL_DIR="${G1_MODEL_DIR:-$PROJECT_ROOT/models/unitree_g1}"
export VOSK_MODEL_DIR="${VOSK_MODEL_DIR:-$PROJECT_ROOT/models/vosk-model-small-en-us-0.15}"
# Keep all class traffic on this computer and on one DDS domain
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-8}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
# CycloneDDS is the DDS used by Unitree robots; fall back silently if not installed
if [ -d /opt/ros/humble/lib ] && ls /opt/ros/humble/lib/librmw_cyclonedds_cpp.so >/dev/null 2>&1; then
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
fi
# Load OPENAI_API_KEY / OLLAMA_MODEL etc. from .env if present (never committed)
if [ -f "$PROJECT_ROOT/.env" ]; then set -a; . "$PROJECT_ROOT/.env"; set +a; fi

source_ros() {
  if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: ROS 2 Humble not found in /opt/ros/humble. Run scripts/install.sh --with-ros" >&2
    exit 1
  fi
  # shellcheck disable=SC1091
  set +u; source /opt/ros/humble/setup.bash
  if [ -f "$WS/install/setup.bash" ]; then source "$WS/install/setup.bash"; fi; set -u
}

require_build() {
  if [ ! -f "$WS/install/setup.bash" ]; then
    echo "ERROR: workspace not built. Run scripts/build_ros2.sh first." >&2
    exit 1
  fi
}

require_model() {
  if [ ! -f "$G1_MODEL_DIR/g1_29dof.xml" ]; then
    echo "ERROR: G1 model not found in $G1_MODEL_DIR. Run scripts/download_models.sh" >&2
    exit 1
  fi
}
