"""g1_simulation_node - executes APPROVED actions on the official G1 model in MuJoCo.

Subscribes  /hri/robot_action   (validated action list)
            /hri/emergency_stop (std_msgs/Bool, latched in the simulator)
Publishes   /joint_states       (sensor_msgs/JointState, 29 joints, 50 Hz)
            /hri/robot_status   (executing / done / aborted / stopped)

Physics runs in its own thread at the model time step (2 ms) in real time; the
MuJoCo viewer is updated at ~50 Hz unless ``headless:=true``.
Physical robot control is disabled: this node never imports unitree_sdk2 and
never publishes /lowcmd.
"""
from __future__ import annotations

import collections
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from . import hri_messages as hm
from . import motion_primitives as mp
from .g1_sim import G1Simulator

MAX_QUEUE = 3


class G1SimulationNode(Node):
    def __init__(self):
        super().__init__("g1_simulation_node")
        self.declare_parameter("model_dir", "")
        self.declare_parameter("headless", False)
        self.declare_parameter("realtime", True)
        self.declare_parameter("joint_state_rate", 50.0)
        self.declare_parameter("use_physical_robot", False)
        if self.get_parameter("use_physical_robot").value:
            raise RuntimeError("Physical robot control is disabled in this teaching package.")

        self.sim = G1Simulator(self.get_parameter("model_dir").value or None)
        self.headless = bool(self.get_parameter("headless").value)
        self.realtime = bool(self.get_parameter("realtime").value)
        self.queue: "collections.deque[tuple[str, dict]]" = collections.deque()
        self.active_meta: dict | None = None

        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_status = self.create_publisher(String, hm.TOPICS["robot_status"], 10)
        self.create_subscription(String, hm.TOPICS["robot_action"], self.on_action, 10)
        self.create_subscription(Bool, hm.TOPICS["emergency_stop"], self.on_estop, 10)
        rate = float(self.get_parameter("joint_state_rate").value)
        self.create_timer(1.0 / rate, self.publish_joint_states)

        self._running = True
        self.physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
        self.physics_thread.start()
        self.get_logger().info(
            f"G1 loaded from {self.sim.model_dir} (fixed base, {mp.NUM_JOINTS} actuated joints, "
            f"dt={self.sim.dt * 1000:.1f} ms, headless={self.headless})")

    # ------------------------------------------------------------- callbacks
    def on_action(self, msg: String) -> None:
        data = hm.decode(msg.data)
        actions = data.get("actions", [])
        for a in actions:
            if a not in mp.PRIMITIVES:  # second line of defence
                self.get_logger().error(f"Unknown primitive '{a}' ignored")
                continue
            if len(self.queue) >= MAX_QUEUE:
                self._status(data, a, "rejected", note="action queue full")
                continue
            self.queue.append((a, data))

    def on_estop(self, msg: Bool) -> None:
        if msg.data:
            self.queue.clear()
            self.sim.emergency_stop()
            meta = self.active_meta or {}
            self._status(meta, meta.get("_action"), "stopped")
            self.active_meta = None
            self.get_logger().warn("EMERGENCY STOP: motion cancelled, holding pose")
        else:
            self.sim.reset_emergency_stop()
            self.get_logger().info("Emergency stop released")

    def _status(self, meta: dict, action, state: str, **extra) -> None:
        out = {"id": meta.get("id"), "stage": "simulator", "action": action, "state": state,
               "t_input": meta.get("t_input"), "t_validated": meta.get("t_validated"),
               "t": hm.now(), **extra}
        self.pub_status.publish(String(data=hm.encode(out)))

    def publish_joint_states(self) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(mp.G1_JOINT_NAMES)
        js.position = self.sim.joint_positions().tolist()
        js.velocity = self.sim.joint_velocities().tolist()
        self.pub_js.publish(js)

    # --------------------------------------------------------------- physics
    def _next_action(self) -> None:
        if self.sim.is_busy() or self.sim.estop or not self.queue:
            return
        action, meta = self.queue.popleft()
        duration = self.sim.start_action(action)
        self.active_meta = dict(meta, _action=action)
        self._status(meta, action, "executing", t_motion_start=hm.now(),
                     expected_duration_s=round(duration, 2))
        self.get_logger().info(f"Executing '{action}' ({duration:.1f} s)")

    def _physics_loop(self) -> None:
        viewer = None
        if not self.headless:
            try:
                import mujoco.viewer
                viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
            except Exception as e:
                self.get_logger().warn(f"Viewer unavailable ({e}); running headless")
        steps_per_sync = max(1, int(0.02 / self.sim.dt))
        wall0, sim0 = time.monotonic(), self.sim.time
        while self._running and rclpy.ok():
            self._next_action()
            event = self.sim.step(steps_per_sync)
            if event:
                meta = self.active_meta or {}
                self._status(meta, meta.get("_action", "?"), event, t_done=hm.now(),
                             tracking_error=round(self.sim.last_error, 3))
                self.get_logger().info(f"Action {event}")
                self.active_meta = None
            if viewer is not None:
                if not viewer.is_running():
                    break
                with viewer.lock():
                    viewer.sync()
            if self.realtime:
                lag = (self.sim.time - sim0) - (time.monotonic() - wall0)
                if lag > 0:
                    time.sleep(lag)
        if viewer is not None:
            viewer.close()

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = G1SimulationNode()
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
