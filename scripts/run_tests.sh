#!/usr/bin/env bash
# Run the automated tests (no ROS 2 needed for most of them).
set -eo pipefail
. "$(dirname "$0")/common.sh"
cd "$PROJECT_ROOT"
MUJOCO_GL="${MUJOCO_GL:-egl}" python3 -m pytest -v tests "$@"
