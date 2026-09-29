"""Command-line client for the nav2_agent ExecuteCommand action.

Usage: ros2 run nav2_agent send "Move 2 meters forward"
"""

import argparse
import signal
import sys
import threading
import time
from typing import Any, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from nav2_agent_msgs.action import ExecuteCommand
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

ACTION_NAME = '/nav2_agent/execute_command'
CANCEL_TIMEOUT = 10.0
PHASES = {
    ExecuteCommand.Feedback.PHASE_PLANNING: 'planning',
    ExecuteCommand.Feedback.PHASE_EXECUTING: 'executing',
    ExecuteCommand.Feedback.PHASE_REPORTING: 'reporting',
}


def format_feedback(feedback: ExecuteCommand.Feedback) -> str:
    phase = PHASES.get(feedback.phase, 'unknown')
    if feedback.phase == ExecuteCommand.Feedback.PHASE_EXECUTING and feedback.distance_remaining > 0.0:
        return (f'[{phase}] {feedback.distance_remaining:.2f} m remaining, '
                f'{feedback.number_of_recoveries} recoveries')
    return f'[{phase}] {feedback.detail}'.rstrip()


def format_result(result: ExecuteCommand.Result) -> str:
    lines = []
    if result.nav2_action:
        lines.append(f'{result.nav2_action}: {result.nav2_status or "UNKNOWN"}, '
                     f'{result.navigation_time:.1f} s, {result.number_of_recoveries} recoveries')
    if result.error_code:
        lines.append(f'error {result.error_code}: {result.error_msg}')
    if result.report:
        lines.append(result.report)
    return '\n'.join(lines)


class CommandSender(Node):
    def __init__(self) -> None:
        super().__init__('nav2_agent_send')
        self._client = ActionClient(self, ExecuteCommand, ACTION_NAME)
        self._last_line = ''
        self._cancel_requested = threading.Event()

    def request_cancel(self, *_: object) -> None:
        self._cancel_requested.set()

    def send(self, command: str, server_timeout: float) -> int:
        if not self._client.wait_for_server(timeout_sec=server_timeout):
            print(f'Action server {ACTION_NAME} not available.', file=sys.stderr)
            return 2

        goal = ExecuteCommand.Goal()
        goal.command = command
        goal_handle = self._wait(self._client.send_goal_async(goal, feedback_callback=self._on_feedback))
        if goal_handle is None or not goal_handle.accepted:
            print('Command rejected.', file=sys.stderr)
            return 2

        result_future = goal_handle.get_result_async()
        cancel_deadline = None
        while not result_future.done():
            if self._cancel_requested.is_set() and cancel_deadline is None:
                print('Canceling command...')
                goal_handle.cancel_goal_async()
                cancel_deadline = time.monotonic() + CANCEL_TIMEOUT
            if cancel_deadline is not None and time.monotonic() > cancel_deadline:
                print('Cancel not confirmed.', file=sys.stderr)
                return 130
            time.sleep(0.05)

        response = result_future.result()
        print(format_result(response.result))
        return 0 if response.status == GoalStatus.STATUS_SUCCEEDED else 1

    def _wait(self, future: Any) -> Any:
        while not future.done():
            if self._cancel_requested.is_set():
                return None
            time.sleep(0.05)
        return future.result()

    def _on_feedback(self, message: ExecuteCommand.Impl.FeedbackMessage) -> None:
        line = format_feedback(message.feedback)
        if line != self._last_line:
            print(line, flush=True)
            self._last_line = line


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    parser = argparse.ArgumentParser(description='Send a natural-language command to nav2_agent.')
    parser.add_argument('command', nargs='+', help='Command text, for example "Move 2 meters forward"')
    parser.add_argument('--server-timeout', type=float, default=5.0, help='Seconds to wait for the action server')
    parsed = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])

    node = CommandSender()
    signal.signal(signal.SIGINT, node.request_cancel)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        code = node.send(' '.join(parsed.command), parsed.server_timeout)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
