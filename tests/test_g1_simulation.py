"""Physics tests on the OFFICIAL Unitree G1 29-DoF MuJoCo model (fixed base)."""
import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from unitree_g1_social_hri import motion_primitives as mp  # noqa: E402
from unitree_g1_social_hri.g1_sim import G1Simulator  # noqa: E402

try:
    SIM = G1Simulator()
except FileNotFoundError as e:  # model not downloaded
    pytest.skip(str(e), allow_module_level=True)


@pytest.fixture
def sim():
    s = G1Simulator()
    for _ in range(250):  # settle 0.5 s
        s.step()
    return s


def test_model_matches_joint_table():
    assert SIM.model.nu == 29
    assert SIM.model.nq == 29          # free joint removed -> fixed base
    for i, name in enumerate(mp.G1_JOINT_NAMES):
        assert SIM.act_id[i] == i      # actuator order == unitree_hg motor order
        lo, hi = mp.G1_JOINT_LIMITS[name]
        assert np.allclose(SIM.model_limits[i], [lo, hi], atol=1e-4)


@pytest.mark.parametrize("action", sorted(mp.PRIMITIVES))
def test_action_executes_and_tracks(sim, action):
    pelvis0 = sim.body_position("pelvis")
    result, log = sim.run_action_blocking(action, record_every=10)
    assert result == "done"
    err = max(np.max(np.abs(tgt[12:] - q[12:])) for _, tgt, q in log)
    assert err < 0.25, f"max upper-body tracking error {err:.3f} rad"
    assert np.allclose(sim.body_position("pelvis"), pelvis0)        # base fixed
    q = sim.joint_positions()
    for i, name in enumerate(mp.G1_JOINT_NAMES):
        lo, hi = mp.G1_JOINT_LIMITS[name]
        assert lo - 0.02 <= q[i] <= hi + 0.02


def test_wave_lifts_right_hand_above_shoulder(sim):
    sim.start_action("wave_right_hand")
    top = -1.0
    for _ in range(int(3.0 / sim.dt)):
        sim.step()
        top = max(top, sim.body_position("right_wrist_yaw_link")[2])
    assert top > sim.body_position("right_shoulder_pitch_link")[2] + 0.1


def test_torques_within_actuator_limits(sim):
    sim.start_action("raise_both_arms")
    for _ in range(500):
        sim.step()
        assert np.all(np.abs(sim.data.ctrl) <= sim.model.actuator_ctrlrange[:, 1] + 1e-9)


def test_emergency_stop_freezes_and_blocks(sim):
    sim.start_action("raise_both_arms")
    for _ in range(400):
        sim.step()
    sim.emergency_stop()
    q_stop = sim.joint_positions()
    for _ in range(500):
        sim.step()
    assert np.max(np.abs(sim.joint_positions()[12:] - q_stop[12:])) < 0.05
    with pytest.raises(RuntimeError):
        sim.start_action("bow")
    sim.reset_emergency_stop()
    assert sim.start_action("neutral") > 0


def test_lowcmd_dry_run_has_29_motors(sim):
    cmd = sim.lowcmd_dict()
    assert all(len(v) == 29 for v in cmd.values())
