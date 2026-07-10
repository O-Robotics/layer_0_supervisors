#!/usr/bin/env python3

import argparse

import rclpy
from amr_sweeper_fsm.msg import FSMState
from amr_sweeper_mission_executor.srv import EndMission
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class FaultWatcher(Node):
    def __init__(self, topic: str, target_state: str, target_profile: int, end_mission_service: str):
        super().__init__('fsm_fault_watcher')
        self._target_state = target_state
        self._target_profile = target_profile
        self._matched = False
        self._interrupted = False
        self._fault_state = ''
        self._fault_profile = 0
        self._end_mission_client = self.create_client(EndMission, end_mission_service)
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
            self._fault_state = msg.current_state
            self._fault_profile = int(msg.current_profile)
            self._matched = True

    def finalize_faulted_mission(self) -> None:
        # Without this, a FAULT-triggered shutdown tears down the launch tree
        # without ever finalizing the run's context.json (runtime_status stays
        # "STARTED" forever, mission_outcome/end_reason/duration never get written).
        if not self._end_mission_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(
                'end_mission service unavailable; mission context.json will not be finalized for this fault'
            )
            return

        request = EndMission.Request()
        request.reason = (
            f'FSM entered {self._fault_state}/{self._fault_profile}; aborted by fsm_fault_watcher'
        )
        request.outcome = 'aborted'
        request.requester = 'fsm_fault_watcher'
        request.force = True
        request.request_idling = True

        future = self._end_mission_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done():
            self.get_logger().warning('Timed out waiting for end_mission response after fault')
            return

        response = future.result()
        if response is None:
            self.get_logger().warning('end_mission call failed after fault')
        elif not response.success:
            self.get_logger().warning(f'end_mission reported failure after fault: {response.message}')
        else:
            self.get_logger().info(f'Mission finalized after fault: {response.message}')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/amr_sweeper/fsm/supervisor_node/fsm_state')
    parser.add_argument('--state', default='FAULT')
    parser.add_argument('--profile', type=int, default=400)
    parser.add_argument('--end-mission-service', default='/amr_sweeper/end_mission')
    args = parser.parse_args()

    rclpy.init()
    node = FaultWatcher(args.topic, args.state, args.profile, args.end_mission_service)
    try:
        while rclpy.ok() and not node.matched:
            rclpy.spin_once(node, timeout_sec=0.5)
        if node.matched:
            node.finalize_faulted_mission()
    except KeyboardInterrupt:
        node.mark_interrupted()
    except ExternalShutdownException:
        node.mark_interrupted()
    finally:
        matched = node.matched
        interrupted = node.interrupted
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
