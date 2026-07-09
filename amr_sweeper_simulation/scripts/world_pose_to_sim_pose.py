#!/usr/bin/env python3

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage


_shutdown_requested = False


def _request_shutdown(_signum, _frame) -> None:
    global _shutdown_requested
    _shutdown_requested = True


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
        if _shutdown_requested:
            return
        self._publisher.publish(msg)


def main() -> None:
    global _shutdown_requested

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    rclpy.init()
    node = WorldPoseRelay()
    try:
        while rclpy.ok() and not _shutdown_requested:
            try:
                rclpy.spin_once(node, timeout_sec=0.5)
            except KeyboardInterrupt:
                _shutdown_requested = True
            except ExternalShutdownException:
                if _shutdown_requested or not rclpy.ok():
                    break
                raise
            except RuntimeError as exc:
                if _shutdown_requested or not rclpy.ok():
                    node.get_logger().debug(
                        f"Suppressing shutdown race while relaying world pose: {exc}"
                    )
                    break
                raise
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown_requested = True
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