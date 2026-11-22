import pytest

from unitree_g1_social_hri.action_catalog import load_catalog
from unitree_g1_social_hri.rule_based_parser import parse_command

CAT = load_catalog()


@pytest.mark.parametrize("text,expected", [
    ("Hello G1, please wave your right hand.", ["wave_right_hand"]),
    ("Can you wave your hand?", ["wave_right_hand"]),
    ("wave your left hand", ["wave_left_hand"]),
    ("Raise both arms", ["raise_both_arms"]),
    ("raise your right arm", ["raise_right_arm"]),
    ("Return to neutral", ["neutral"]),
    ("point forward", ["point_forward"]),
    ("please take a bow", ["bow"]),
    ("G1, introduce yourself and then wave at me", ["wave_right_hand"]),
    ("raise both arms and then bow", ["raise_both_arms", "bow"]),
    ("Hello robot!", ["none"]),
    ("Tell me about humanoid robots", ["none"]),
    ("make me a sandwich", ["none"]),
    ("STOP!", ["none"]),
])
def test_actions(text, expected):
    assert parse_command(text, CAT)["actions"] == expected


def test_greeting_and_intro_speech():
    r = parse_command("Hello robot!", CAT)
    assert "Unitree G1" in r["speech"]
    r = parse_command("G1, introduce yourself and then wave at me", CAT)
    assert "humanoid robot" in r["speech"]


def test_explanation_speech():
    assert "human-like" in parse_command("Tell me about humanoid robots", CAT)["speech"]


def test_unknown_gives_help():
    assert "did not understand" in parse_command("make me a sandwich", CAT)["speech"]
