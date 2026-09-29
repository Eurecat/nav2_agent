import unittest

try:
    from nav2_agent.conversation import Conversation, Turn
    from nav2_agent.models import TargetPose
except ImportError:  # pydantic is not installed
    Conversation = None


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


@unittest.skipIf(Conversation is None, 'pydantic is not installed')
class TestConversation(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.conversation = Conversation(max_turns=2, timeout_sec=60.0, clock=self.clock)
        self.origin = TargetPose(frame_id='map', x=0.0, y=0.0)
        self.shelves = TargetPose(frame_id='map', x=12.5, y=-0.8)

    def test_previous_start_pose(self):
        self.conversation.add(Turn('go to the shelves', 'navigate_to_pose, SUCCEEDED', self.origin, self.shelves))

        self.assertEqual(self.origin, self.conversation.previous_start_pose(1))
        self.assertIsNone(self.conversation.previous_start_pose(2))

    def test_context_lists_turns_with_commands_ago(self):
        self.conversation.add(Turn('go to the shelves', 'navigate_to_pose, SUCCEEDED', self.origin, self.shelves))

        context = self.conversation.format_context()
        self.assertIn('[1] "go to the shelves" -> navigate_to_pose, SUCCEEDED.', context)
        self.assertIn('End: map x=12.50 y=-0.80', context)

    def test_keeps_the_most_recent_turns(self):
        for command in ('a', 'b', 'c'):
            self.conversation.add(Turn(command, 'reply', self.origin, self.origin))

        self.assertEqual(['b', 'c'], [turn.command for turn in self.conversation.turns()])

    def test_expires_after_timeout(self):
        self.conversation.add(Turn('a', 'reply', self.origin, self.origin))
        self.clock.now = 61.0
        self.conversation.expire_if_idle()

        self.assertEqual([], self.conversation.turns())


if __name__ == '__main__':
    unittest.main()
