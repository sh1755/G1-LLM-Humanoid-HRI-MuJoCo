from unitree_g1_social_hri.action_catalog import load_catalog
from unitree_g1_social_hri.action_validator import ActionValidator

CAT = load_catalog()


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def make():
    c = Clock()
    return ActionValidator(CAT, clock=c), c


def test_approves_allowlisted_action():
    v, _ = make()
    r = v.validate({"actions": ["wave_right_hand"], "speech": "Hi"})
    assert r.approved and r.actions == ["wave_right_hand"]


def test_single_action_form_from_slides():
    v, _ = make()
    assert v.validate({"actions": "wave_right_hand", "speech": "Hi"}).approved


def test_rejects_unknown_action():
    v, _ = make()
    r = v.validate({"actions": ["backflip"], "speech": ""})
    assert not r.approved and "not on the approved list" in r.reasons[0]


def test_rejects_low_level_fields():
    v, _ = make()
    r = v.validate({"actions": ["neutral"], "speech": "", "joint_positions": [0.1] * 29})
    assert not r.approved


def test_rejects_non_dict_and_missing_actions():
    v, _ = make()
    assert not v.validate("wave").approved
    assert not v.validate({"speech": "hi"}).approved


def test_too_many_actions():
    v, _ = make()
    assert not v.validate({"actions": ["bow"] * 4, "speech": ""}).approved


def test_estop_blocks_motion_but_not_speech():
    v, _ = make()
    v.estop_active = True
    assert not v.validate({"actions": ["bow"], "speech": ""}).approved
    r = v.validate({"actions": ["none"], "speech": "I am stopped."})
    assert r.approved and r.actions == []


def test_rate_limit():
    v, c = make()
    assert v.validate({"actions": ["bow"], "speech": ""}).approved
    c.t = 0.1
    assert not v.validate({"actions": ["bow"], "speech": ""}).approved
    c.t = 1.0
    assert v.validate({"actions": ["bow"], "speech": ""}).approved


def test_speech_truncated():
    v, _ = make()
    r = v.validate({"actions": ["none"], "speech": "x" * 1000})
    assert r.approved and len(r.speech) <= CAT.safety["max_speech_chars"]
