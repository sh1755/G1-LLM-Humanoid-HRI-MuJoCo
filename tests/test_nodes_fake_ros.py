"""Runs the six REAL node source files together on an in-process fake rclpy.

This checks node logic and wiring (topics, JSON fields, e-stop, monitor CSV)
on machines without ROS 2.  It is NOT a substitute for running the launch file
on Ubuntu 22.04 + ROS 2 Humble: there is no DDS, QoS or executor here.
"""
import csv
import importlib
import json
import sys
import time
from pathlib import Path

import pytest

try:
    import rclpy as _real  # noqa: F401
    if not getattr(_real, "__file__", "").endswith(str(Path("fake_ros/rclpy/__init__.py"))):
        pytest.skip("real rclpy available: use colcon test / the launch file instead",
                    allow_module_level=True)
except ImportError:
    pass

sys.path.insert(0, str(Path(__file__).parent / "fake_ros"))
pytest.importorskip("mujoco")
import rclpy  # noqa: E402  (fake)
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

NODES = ["speech_input_node", "llm_interaction_node", "action_validator_node",
         "g1_simulation_node", "speech_output_node", "hri_monitor_node"]


@pytest.fixture
def system(tmp_path):
    rclpy.init()
    rclpy.LOG.clear()
    Node.OVERRIDES = {
        "speech_input_node": {"input_mode": "none"},
        "g1_simulation_node": {"headless": True, "realtime": False},
        "hri_monitor_node": {"log_dir": str(tmp_path)},
    }
    received = {t: [] for t in ["/hri/llm_response", "/hri/robot_action", "/hri/robot_status",
                                "/hri/speech_output", "/joint_states"]}
    for topic in received:
        rclpy._BUS.setdefault(topic, []).append(received[topic].append)
    mods = {n: importlib.import_module(f"unitree_g1_social_hri.{n}") for n in NODES}
    nodes = {
        "input": mods["speech_input_node"].SpeechInputNode(),
        "llm": mods["llm_interaction_node"].LLMInteractionNode(),
        "validator": mods["action_validator_node"].ActionValidatorNode(),
        "sim": mods["g1_simulation_node"].G1SimulationNode(),
        "tts": mods["speech_output_node"].SpeechOutputNode(),
        "monitor": mods["hri_monitor_node"].HRIMonitorNode(),
    }
    yield nodes, received, tmp_path
    for n in nodes.values():
        n.destroy_node()
    rclpy.shutdown()


def wait_for(pred, timeout=30.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.05)
    return False


def statuses(received):
    return [json.loads(m.data) for m in received["/hri/robot_status"]]


def test_typed_command_moves_robot(system):
    nodes, rx, log_dir = system
    nodes["input"].publish_text("Hello G1, please wave your right hand.", "keyboard")
    assert wait_for(lambda: any(s.get("state") == "done" for s in statuses(rx)))
    llm = json.loads(rx["/hri/llm_response"][0].data)
    assert llm["request"]["actions"] == ["wave_right_hand"] and llm["backend"] == "rule"
    act = json.loads(rx["/hri/robot_action"][0].data)
    assert act["actions"] == ["wave_right_hand"] and act["id"] == llm["id"]
    states = [s["state"] for s in statuses(rx)]
    assert states[:2] == ["executing", "done"]
    assert any("Unitree G1" in json.loads(m.data)["text"] for m in rx["/hri/speech_output"])
    js = rx["/joint_states"][-1]
    assert len(js.name) == 29 and len(js.position) == 29
    # monitor wrote a CSV row with latencies
    assert wait_for(lambda: any(log_dir.glob("hri_log_*.csv")) and
                    len(list(csv.DictReader(open(next(log_dir.glob("hri_log_*.csv")))))) >= 1, 5)
    row = list(csv.DictReader(open(next(log_dir.glob("hri_log_*.csv")))))[0]
    assert row["outcome"] == "done" and float(row["input_to_motion_ms"]) >= 0


def test_sequence_of_two_actions(system):
    nodes, rx, _ = system
    nodes["input"].publish_text("raise both arms and then bow", "keyboard")
    assert wait_for(lambda: [s["state"] for s in statuses(rx)].count("done") == 2)
    assert [s["action"] for s in statuses(rx) if s["state"] == "executing"] == ["raise_both_arms", "bow"]


def test_unknown_llm_action_is_rejected(system):
    nodes, rx, _ = system
    bad = {"id": "evil01", "request": {"actions": ["backflip"], "speech": "watch this"},
           "t_input": time.time(), "t_llm": time.time()}
    rclpy.publish("/hri/llm_response", String(data=json.dumps(bad)))
    assert wait_for(lambda: any(s.get("state") == "rejected" for s in statuses(rx)), 5)
    assert rx["/hri/robot_action"] == []
    assert "cannot do that safely" in json.loads(rx["/hri/speech_output"][-1].data)["text"]


def test_stop_word_triggers_estop(system):
    nodes, rx, _ = system
    nodes["input"].publish_text("raise both arms", "keyboard")
    assert wait_for(lambda: any(s.get("state") == "executing" for s in statuses(rx)))
    nodes["input"].publish_text("stop!", "voice")
    assert wait_for(lambda: any(s.get("state") == "stopped" for s in statuses(rx)), 5)
    assert nodes["sim"].sim.estop and nodes["validator"].validator.estop_active
    nodes["input"].publish_text("wave", "keyboard")   # blocked while stopped
    assert wait_for(lambda: any(s.get("state") == "rejected" for s in statuses(rx)), 5)
    rclpy.publish("/hri/emergency_stop", Bool(data=False))  # operator releases
    time.sleep(0.6)
    nodes["input"].publish_text("return to neutral", "keyboard")
    assert wait_for(lambda: [s["state"] for s in statuses(rx)].count("done") >= 1)
