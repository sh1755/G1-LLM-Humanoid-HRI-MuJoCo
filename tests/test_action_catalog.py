import pytest
import yaml

from unitree_g1_social_hri import motion_primitives as mp
from unitree_g1_social_hri.action_catalog import default_config_path, load_catalog


def test_every_motion_action_has_a_primitive():
    cat = load_catalog()
    for name in cat.motion_actions:
        assert name in mp.PRIMITIVES, name
    assert "none" in cat.allowed


def test_physical_robot_flag_must_stay_false(tmp_path):
    cfg = yaml.safe_load(default_config_path().read_text())
    cfg["safety"]["physical_robot_enabled"] = True
    p = tmp_path / "bad.yaml"
    p.write_text(yaml.safe_dump(cfg))
    with pytest.raises(RuntimeError):
        load_catalog(str(p))
