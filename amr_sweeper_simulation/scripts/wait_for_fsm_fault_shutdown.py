#!/usr/bin/env python3

import argparse

import rclpy
from amr_sweeper_fsm.msg import FSMState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class FaultWatcher(Node):
    def __init__(self, topic: str, target_state: str, target_profile: int):
        super().__init__('fsm_fault_watcher')
        self._target_state = target_state
        self._target_profile = target_profile
        self._matched = False
        self._interrupted = False
        self.create_subscription(FSMState, topic, self._on_state, 10)

    @property
    def matched(self) -> bool:
        return self._matched

    @property
    def interrupted(self) -> bool:
        return self._interrupted

    def mark_interrupted(self) -> None:
        self._interrupted = True

    def _on_state(self, msg: FSMState) -> None:
        if msg.current_state == self._target_state and int(msg.current_profile) == self._target_profile:
            self.get_logger().warning(
                f'Detected FSM state={msg.current_state} profile={msg.current_profile}; requesting simulation shutdown'
            )
            self._matched = True
            self.destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/amr_sweeper/fsm/supervisor_node/fsm_state')
    parser.add_argument('--state', default='FAULT')
    parser.add_argument('--profile', type=int, default=400)
    args = parser.parse_args()

    rclpy.init()
    node = FaultWatcher(args.topic, args.state, args.profile)
    try:
        while rclpy.ok() and not node.matched:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        node.mark_interrupted()
    except ExternalShutdownException:
        node.mark_interrupted()
    finally:
        matched = node.matched
        interrupted = node.interrupted
        if not matched:
            try:
                node.destroy_node()
            except (KeyboardInterrupt, RuntimeError):
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except RuntimeError:
            pass
    return 42 if matched else 0 if interrupted else 1


if __name__ == '__main__':
    raise SystemExit(main())
