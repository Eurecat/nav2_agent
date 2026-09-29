import unittest

try:
    from nav2_agent_msgs.action import ExecuteCommand

    from nav2_agent.send_command import format_feedback, format_result
except ImportError:  # nav2_agent_msgs is only available in a built workspace
    ExecuteCommand = None


@unittest.skipIf(ExecuteCommand is None, 'nav2_agent_msgs is not built')
class TestSendCommandFormatting(unittest.TestCase):
    def test_planning_feedback_shows_step(self):
        feedback = ExecuteCommand.Feedback(phase=ExecuteCommand.Feedback.PHASE_PLANNING,
                                           detail='tool_make_relative_translation')
        self.assertEqual('[planning] tool_make_relative_translation', format_feedback(feedback))

    def test_executing_feedback_shows_distance(self):
        feedback = ExecuteCommand.Feedback(phase=ExecuteCommand.Feedback.PHASE_EXECUTING,
                                           distance_remaining=2.5, number_of_recoveries=1)
        self.assertEqual('[executing] 2.50 m remaining, 1 recoveries', format_feedback(feedback))

    def test_result_includes_status_and_report(self):
        result = ExecuteCommand.Result(nav2_action='navigate_to_pose', nav2_status='SUCCEEDED',
                                       navigation_time=11.1, number_of_recoveries=0, report='Goal reached.')
        self.assertEqual('navigate_to_pose: SUCCEEDED, 11.1 s, 0 recoveries\nGoal reached.', format_result(result))

    def test_result_includes_error(self):
        result = ExecuteCommand.Result(nav2_action='navigate_to_pose', nav2_status='ABORTED',
                                       error_code=104, error_msg='No valid path')
        self.assertIn('error 104: No valid path', format_result(result))


if __name__ == '__main__':
    unittest.main()
