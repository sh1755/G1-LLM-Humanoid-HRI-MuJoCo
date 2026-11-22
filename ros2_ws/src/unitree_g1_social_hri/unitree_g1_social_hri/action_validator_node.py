"""action_validator_node - the safety gate.

/hri/llm_response (untrusted) --[allowlist, schema, e-stop, rate limit]-->
    /hri/robot_action   (approved actions only)
    /hri/speech_output  (what the robot says)
    /hri/robot_status   (rejections, with reasons)
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from . import hri_messages as hm
from .action_catalog import load_catalog
from .action_validator import ActionValidator


class ActionValidatorNode(Node):
    def __init__(self):
        super().__init__("action_validator_node")
        self.declare_parameter("actions_config", "")
        self.catalog = load_catalog(self.get_parameter("actions_config").value or None)
        self.validator = ActionValidator(self.catalog)
        self.pub_action = self.create_publisher(String, hm.TOPICS["robot_action"], 10)
        self.pub_speech = self.create_publisher(String, hm.TOPICS["speech_output"], 10)
        self.pub_status = self.create_publisher(String, hm.TOPICS["robot_status"], 10)
        self.create_subscription(String, hm.TOPICS["llm_response"], self.on_request, 10)
        self.create_subscription(Bool, hm.TOPICS["emergency_stop"], self.on_estop, 10)
        self.get_logger().info("action_validator_node ready. Allowlist: "
                               + ", ".join(self.catalog.allowed))

    def on_estop(self, msg: Bool) -> None:
        self.validator.estop_active = bool(msg.data)
        self.get_logger().warn(f"Emergency stop {'ACTIVE' if msg.data else 'released'}")

    def on_request(self, msg: String) -> None:
        data = hm.decode(msg.data)
        rid = data.get("id", hm.new_id())
        result = self.validator.validate(data.get("request"))
        base = {"id": rid, "t_input": data.get("t_input"), "t_llm": data.get("t_llm"),
                "t_validated": hm.now(), "backend": data.get("backend", "?"),
                "text": data.get("text", "")}
        if result.approved:
            if result.actions:
                self.pub_action.publish(String(data=hm.encode({**base, "actions": result.actions})))
            self.get_logger().info(f"APPROVED {result.actions or ['(speech only)']} (id={rid})")
            speech = result.speech
        else:
            self.pub_status.publish(String(data=hm.encode(
                {**base, "stage": "validator", "state": "rejected", "reasons": result.reasons})))
            self.get_logger().warn(f"REJECTED (id={rid}): {'; '.join(result.reasons)}")
            speech = "Sorry, I cannot do that safely."
        if speech:
            self.pub_speech.publish(String(data=hm.encode({"id": rid, "text": speech})))


def main(args=None):
    rclpy.init(args=args)
    node = ActionValidatorNode()
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
