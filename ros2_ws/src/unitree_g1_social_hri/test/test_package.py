"""Run with `colcon test --packages-select unitree_g1_social_hri` (full suite: ../../tests)."""
from unitree_g1_social_hri import motion_primitives as mp
from unitree_g1_social_hri.action_catalog import load_catalog


def test_catalog_and_primitives_agree():
    cat = load_catalog()
    assert all(a in mp.PRIMITIVES for a in cat.motion_actions)
    assert cat.safety["physical_robot_enabled"] is False
