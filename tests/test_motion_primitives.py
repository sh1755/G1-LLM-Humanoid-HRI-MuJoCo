import numpy as np
import pytest

from unitree_g1_social_hri import motion_primitives as mp


def test_joint_table_is_complete():
    assert mp.NUM_JOINTS == 29
    assert set(mp.G1_JOINT_LIMITS) == set(mp.G1_JOINT_NAMES)
    assert mp.G1_JOINT_NAMES[15] == "left_shoulder_pitch_joint"   # unitree_hg motor 15
    assert mp.G1_JOINT_NAMES[22] == "right_shoulder_pitch_joint"  # unitree_hg motor 22


@pytest.mark.parametrize("action", sorted(mp.PRIMITIVES))
def test_trajectory_respects_limits_and_velocity(action):
    dt = 0.01
    t, q = mp.build_trajectory(action, mp.neutral_vector(), dt=dt)
    assert np.all(np.diff(t) > 0)
    for i, name in enumerate(mp.G1_JOINT_NAMES):
        lo, hi = mp.G1_JOINT_LIMITS[name]
        assert q[:, i].min() >= lo + mp.LIMIT_MARGIN - 1e-9
        assert q[:, i].max() <= hi - mp.LIMIT_MARGIN + 1e-9
    vel = np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None]
    assert vel.max() <= mp.MAX_JOINT_VELOCITY * 1.02
    # legs never move
    assert np.allclose(q[:, :12], q[0, :12])


@pytest.mark.parametrize("action", sorted(mp.PRIMITIVES))
def test_every_action_returns_to_neutral(action):
    _, q = mp.build_trajectory(action, mp.neutral_vector())
    assert np.allclose(q[-1], mp.clamp_to_limits(mp.neutral_vector()), atol=1e-9)


def test_trajectory_starts_from_measured_pose():
    q0 = mp.neutral_vector()
    q0[mp.JOINT_INDEX["right_elbow_joint"]] = 0.3
    _, q = mp.build_trajectory("wave_right_hand", q0)
    assert np.allclose(q[0], q0)
    assert np.max(np.abs(q[1] - q[0])) < 0.05  # no jump


def test_min_jerk_endpoints():
    s = mp.min_jerk(np.array([0.0, 0.5, 1.0]))
    assert s[0] == 0 and s[2] == 1 and abs(s[1] - 0.5) < 1e-12


def test_unknown_action_rejected():
    with pytest.raises(KeyError):
        mp.build_trajectory("backflip", mp.neutral_vector())


def test_slow_down_when_too_fast():
    t, _ = mp.build_trajectory("raise_both_arms", mp.neutral_vector(), max_velocity=0.5)
    t_fast, _ = mp.build_trajectory("raise_both_arms", mp.neutral_vector())
    assert t[-1] > t_fast[-1]
