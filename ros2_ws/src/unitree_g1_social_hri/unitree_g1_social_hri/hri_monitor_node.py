"""hri_monitor_node - records every interaction and its latency to CSV.

For each interaction id it collects the time stamps written by the other nodes:

  t_input        speech_input_node   (text typed / transcribed)
  t_llm          llm_interaction_node (structured request ready)
  t_validated    action_validator_node
  t_motion_start g1_simulation_node   (trajectory started)
  t_done         g1_simulation_node   (trajectory finished)

and writes one CSV row per finished interaction to
``<log_dir>/hri_log_<date>.csv`` (default ~/ro603_hri_logs).
Actions that do not finish within ``action_timeout_s`` are flagged.
"""
from __future__ import annotations

import collections
import csv
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from . import hri_messages as hm
from .action_catalog import load_catalog

FIELDS = ["id", "text", "backend", "actions", "outcome", "llm_ms", "validation_ms",
          "dispatch_ms", "input_to_motion_ms", "input_to_done_ms", "notes"]


def _ms(a, b):
    return "" if a is None or b is None else round((b - a) * 1000.0, 1)


class HRIMonitorNode(Node):
    def __init__(self):
        super().__init__("hri_monitor_node")
        self.declare_parameter("log_dir", str(Path.home() / "ro603_hri_logs"))
        self.declare_parameter("actions_config", "")
        cat = load_catalog(self.get_parameter("actions_config").value or None)
        self.timeout = float(cat.safety.get("action_timeout_s", 15.0))
        log_dir = Path(os.path.expanduser(self.get_parameter("log_dir").value))
        log_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = log_dir / f"hri_log_{time.strftime('%Y%m%d')}.csv"
        new = not self.csv_path.exists()
        self.csv_file = open(self.csv_path, "a", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=FIELDS)
        if new:
            self.writer.writeheader()
        self.records: dict[str, dict] = {}
        self.finished: "collections.deque[str]" = collections.deque(maxlen=500)
        for key, cb in (("speech_text", self.on_text), ("llm_response", self.on_llm),
                        ("robot_action", self.on_action), ("robot_status", self.on_status)):
            self.create_subscription(String, hm.TOPICS[key], cb, 10)
        self.create_subscription(Bool, hm.TOPICS["emergency_stop"], self.on_estop, 10)
        self.create_timer(1.0, self.check_timeouts)
        self.get_logger().info(f"Logging interactions to {self.csv_path}")

    def _rec(self, rid):
        if rid in self.finished:          # late message for a logged interaction
            return {"id": rid, "notes": "", "_late": True}
        return self.records.setdefault(rid, {"id": rid, "notes": ""})

    def on_text(self, msg):
        d = hm.decode(msg.data)
        if "id" in d:
            r = self._rec(d["id"])
            r.update(text=d.get("text", ""), t_input=d.get("t_input"))

    def on_llm(self, msg):
        d = hm.decode(msg.data)
        r = self._rec(d.get("id"))
        r.update(t_input=r.get("t_input") or d.get("t_input"), t_llm=d.get("t_llm"),
                 backend=d.get("backend"), text=r.get("text") or d.get("text", ""))
        acts = (d.get("request") or {}).get("actions", [])
        r["actions"] = " ".join(a for a in acts if isinstance(a, str))
        if acts in (["none"], []):
            r["t_validated_pending_speech"] = True

    def on_action(self, msg):
        d = hm.decode(msg.data)
        r = self._rec(d.get("id"))
        r.update(t_validated=d.get("t_validated"), pending=len(d.get("actions", [])))
        r.setdefault("t_input", d.get("t_input"))
        r.pop("t_validated_pending_speech", None)

    def on_status(self, msg):
        d = hm.decode(msg.data)
        r = self._rec(d.get("id"))
        state = d.get("state")
        if state == "rejected":
            r["t_validated"] = d.get("t_validated") or d.get("t")
            r["text"] = r.get("text") or d.get("text", "")
            r["backend"] = r.get("backend") or d.get("backend", "")
            self._finish(r, "rejected", "; ".join(d.get("reasons", [])) or d.get("note", ""))
        elif state == "executing":
            r.setdefault("t_motion_start", d.get("t_motion_start"))
        elif state in ("done", "aborted", "stopped"):
            r["pending"] = r.get("pending", 1) - 1
            r["t_done"] = d.get("t_done") or d.get("t")
            if state != "done" or r["pending"] <= 0:
                self._finish(r, state)

    def on_estop(self, msg):
        self.get_logger().warn(f"[monitor] emergency stop {'ACTIVE' if msg.data else 'released'}")

    def check_timeouts(self):
        now = hm.now()
        for r in list(self.records.values()):
            if r.get("t_validated") and r.get("pending") and now - r["t_validated"] > self.timeout:
                self._finish(r, "timeout", f"no 'done' within {self.timeout:.0f} s")
            elif r.pop("t_validated_pending_speech", False) and r.get("t_llm"):
                self._finish(r, "speech_only")

    def _finish(self, r, outcome, note=""):
        if r.get("_late"):
            return
        row = {"id": r["id"], "text": r.get("text", ""), "backend": r.get("backend", ""),
               "actions": r.get("actions", ""), "outcome": outcome,
               "llm_ms": _ms(r.get("t_input"), r.get("t_llm")),
               "validation_ms": _ms(r.get("t_llm"), r.get("t_validated")),
               "dispatch_ms": _ms(r.get("t_validated"), r.get("t_motion_start")),
               "input_to_motion_ms": _ms(r.get("t_input"), r.get("t_motion_start")),
               "input_to_done_ms": _ms(r.get("t_input"), r.get("t_done")),
               "notes": note}
        self.writer.writerow(row)
        self.csv_file.flush()
        self.records.pop(r["id"], None)
        self.finished.append(r["id"])
        self.get_logger().info(
            f"[{outcome}] \"{row['text']}\" -> {row['actions'] or '-'} | LLM {row['llm_ms']} ms | "
            f"input->motion {row['input_to_motion_ms']} ms | input->done {row['input_to_done_ms']} ms")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HRIMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
