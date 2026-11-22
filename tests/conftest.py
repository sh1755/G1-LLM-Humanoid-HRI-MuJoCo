import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG_SRC = ROOT / "ros2_ws" / "src" / "unitree_g1_social_hri"
sys.path.insert(0, str(PKG_SRC))
os.environ.setdefault("G1_MODEL_DIR", str(ROOT / "models" / "unitree_g1"))
os.environ.setdefault("MUJOCO_GL", "egl")   # headless rendering if a test renders
