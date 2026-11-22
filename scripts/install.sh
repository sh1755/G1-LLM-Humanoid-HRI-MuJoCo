#!/usr/bin/env bash
# Install everything needed for the RO603 Week 8 practical on Ubuntu 22.04.
#   ./scripts/install.sh              # system + Python packages (ROS 2 already installed)
#   ./scripts/install.sh --with-ros   # also install ROS 2 Humble Desktop from packages.ros.org
#   ./scripts/install.sh --with-rl    # also install Stable-Baselines3 for Project D
set -euo pipefail
. "$(dirname "$0")/common.sh"

WITH_ROS=0; WITH_RL=0
for a in "$@"; do
  case "$a" in
    --with-ros) WITH_ROS=1 ;;
    --with-rl) WITH_RL=1 ;;
    *) echo "unknown option $a"; exit 1 ;;
  esac
done

. /etc/os-release
if [ "${VERSION_ID:-}" != "22.04" ]; then
  echo "WARNING: this practical is tested for Ubuntu 22.04 (found $PRETTY_NAME)."
fi

if [ "$WITH_ROS" = 1 ] && [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "==> Installing ROS 2 Humble (official instructions: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)"
  sudo apt update && sudo apt install -y software-properties-common curl
  sudo add-apt-repository -y universe
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt update && sudo apt install -y ros-humble-desktop
fi

echo "==> System packages (CycloneDDS RMW, colcon, audio, TTS)"
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions python3-yaml \
  ros-humble-rmw-cyclonedds-cpp \
  portaudio19-dev libportaudio2 alsa-utils espeak-ng \
  libgl1 libglfw3 libegl1 git wget unzip || {
    echo "Some apt packages failed. If ROS 2 is not installed yet, re-run with --with-ros"; exit 1; }

echo "==> Python packages (user site, Python 3.10)"
python3 -m pip install --user --upgrade pip
python3 -m pip install --user -r "$PROJECT_ROOT/requirements.txt"
if [ "$WITH_RL" = 1 ]; then
  python3 -m pip install --user -r "$PROJECT_ROOT/rl_extension/requirements-rl.txt"
fi

echo "==> Checking"
python3 -c "import mujoco, numpy, yaml; print('mujoco', mujoco.__version__, '| numpy', numpy.__version__)"
python3 -c "import sounddevice; print('sounddevice OK (microphone support)')" || echo "sounddevice not usable - voice mode disabled, text mode still works"
if [ ! -f "$PROJECT_ROOT/.env" ]; then cp "$PROJECT_ROOT/.env.example" "$PROJECT_ROOT/.env"; echo "Created .env (edit it to add an API key - optional)"; fi
echo "Done. Next: ./scripts/download_models.sh && ./scripts/build_ros2.sh"
