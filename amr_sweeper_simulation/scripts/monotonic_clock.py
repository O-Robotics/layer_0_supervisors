#!/usr/bin/env python3

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


_shutdown_requested = False


def _request_shutdown(_signum, _frame) -> None:
    global _shutdown_requested
    _shutdown_requested = True


def _to_nanoseconds(clock: Clock) -> int:
    return (int(clock.clock.sec) * 1_000_000_000) + int(clock.clock.nanosec)


def _from_nanoseconds(nanoseconds: int) -> Clock:
    msg = Clock()
    msg.clock.sec = nanoseconds // 1_000_000_000
    msg.clock.nanosec = nanoseconds % 1_000_000_000
    return msg


class MonotonicClock(Node):
    def __init__(self) -> None:
        super().__init__("monotonic_clock")

        input_topic = self.declare_parameter(
            "input_topic",
            "/amr_sweeper/simulation/raw_clock",
        ).value
        output_topic = self.declare_parameter("output_topic", "/clock").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._last_published_ns: int | None = None
        self._clamped_count = 0
        self._publisher = self.create_publisher(Clock, output_topic, qos)
        self._subscription = self.create_subscription(
            Clock,
            input_topic,
            self._handle_clock,
            qos,
        )

    def _handle_clock(self, msg: Clock) -> None:
        if _shutdown_requested:
            return

        next_ns = _to_nanoseconds(msg)
        if self._last_published_ns is not None and next_ns < self._last_published_ns:
            self._clamped_count += 1
            if self._clamped_count <= 5 or self._clamped_count % 100 == 0:
                jump_s = (self._last_published_ns - next_ns) / 1_000_000_000.0
                self.get_logger().warn(
                    "Clamped backward simulation clock jump "
                    f"of {jump_s:.6f}s (count={self._clamped_count})"
                )
            next_ns = self._last_published_ns

        self._last_published_ns = next_ns
        self._publisher.publish(_from_nanoseconds(next_ns))


def main() -> None:
    global _shutdown_requested

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    rclpy.init()
    node = MonotonicClock()
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
                        f"Suppressing shutdown race while publishing clock: {exc}"
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
