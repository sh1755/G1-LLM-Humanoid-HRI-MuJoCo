"""llm_interaction_node - text -> structured action request on /hri/llm_response.

Parameter ``llm_backend``: rule (default, offline) | ollama | openai.
If the chosen backend fails (no key, no internet, timeout, invalid JSON) the
node automatically falls back to the rule-based parser, so the demo never stalls.

The output is UNTRUSTED: it goes to action_validator_node, never to the robot.
"""
from __future__ import annotations

import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from . import hri_messages as hm
from .action_catalog import load_catalog
from .llm_client import CommandInterpreter


class LLMInteractionNode(Node):
    def __init__(self):
        super().__init__("llm_interaction_node")
        self.declare_parameter("llm_backend", "rule")
        self.declare_parameter("actions_config", "")
        self.declare_parameter("timeout_s", 0.0)
        cfg = self.get_parameter("actions_config").value or None
        self.catalog = load_catalog(cfg)
        backend = self.get_parameter("llm_backend").value
        timeout = self.get_parameter("timeout_s").value or None
        self.interpreter = CommandInterpreter(self.catalog, backend, timeout)
        self.pub = self.create_publisher(String, hm.TOPICS["llm_response"], 10)
        self.create_subscription(String, hm.TOPICS["speech_text"], self.on_text, 10)
        # LLM calls can take seconds: do them in a worker so ROS callbacks stay responsive
        self.jobs: "queue.Queue[dict]" = queue.Queue(maxsize=5)
        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info(f"llm_interaction_node ready (backend={backend})")

    def on_text(self, msg: String) -> None:
        data = hm.decode(msg.data)
        if "id" not in data:                      # plain `ros2 topic pub` text
            data = {"id": hm.new_id(), "text": data.get("text", msg.data), "t_input": hm.now()}
        try:
            self.jobs.put_nowait(data)
        except queue.Full:
            self.get_logger().warn("Interpreter busy - command dropped")

    def _worker(self) -> None:
        while rclpy.ok():
            try:
                data = self.jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            self.process(data)

    def process(self, data: dict) -> None:
        result = self.interpreter.interpret(data.get("text", ""))
        if result.fallback_reason:
            self.get_logger().warn(f"{self.interpreter.backend} failed -> rule fallback "
                                   f"({result.fallback_reason})")
        out = {"id": data["id"], "text": data.get("text", ""), "request": result.request,
               "backend": result.backend, "llm_latency_s": round(result.latency_s, 3),
               "t_input": data.get("t_input", hm.now()), "t_llm": hm.now()}
        self.pub.publish(String(data=hm.encode(out)))
        self.get_logger().info(f"[{result.backend}] {hm.encode(result.request)}")


def main(args=None):
    rclpy.init(args=args)
    node = LLMInteractionNode()
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
