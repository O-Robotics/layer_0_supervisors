#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage


class WorldPoseRelay(Node):
    def __init__(self) -> None:
        super().__init__("world_pose_to_sim_pose")

        input_topic = self.declare_parameter(
            "input_topic",
            "/world/empty1/pose/info",
        ).value
        output_topic = self.declare_parameter(
            "output_topic",
            "/amr_sweeper/simulation/pose/info",
        ).value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._publisher = self.create_publisher(TFMessage, output_topic, qos)
        self._subscription = self.create_subscription(
            TFMessage,
            input_topic,
            self._handle_message,
            qos,
        )

    def _handle_message(self, msg: TFMessage) -> None:
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = WorldPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()
