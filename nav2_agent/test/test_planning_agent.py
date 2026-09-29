import asyncio
import logging
import unittest
from pathlib import Path

try:
    from pydantic_ai.messages import ModelResponse, RetryPromptPart, ToolCallPart
    from pydantic_ai.models.function import FunctionModel

    from nav2_agent.bt_catalog import load_bt_catalog
    from nav2_agent.pydantic_agent import AgentDependencies, PlanningComplete, create_nav2_agent
except ImportError:  # pydantic-ai is not installed
    FunctionModel = None

CATALOG_PATH = Path(__file__).resolve().parents[1] / 'config' / 'bt_catalog.yaml'

VALID_TREE = (
    '<root main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree"><Sequence>'
    '<ComputePathToPose goal="{goal}" path="{path}"/><FollowPath path="{path}"/>'
    '</Sequence></BehaviorTree></root>'
)
INVALID_TREE = (
    '<root main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree"><Sequence>'
    '<FollowPath path="{path}"/><ComputePathToPose goal="{goal}" path="{path}"/>'
    '</Sequence></BehaviorTree></root>'
)


def scripted_model(tool_calls, retries):
    """Model that returns one scripted tool call per request and records retry prompts."""
    calls = iter(tool_calls)

    def respond(messages, info):
        retries.extend(part for part in messages[-1].parts if isinstance(part, RetryPromptPart))
        return ModelResponse(parts=[next(calls)])

    return FunctionModel(respond)


@unittest.skipIf(FunctionModel is None, 'pydantic-ai is not installed')
class TestPlanningAgent(unittest.TestCase):
    def plan(self, tool_calls, robot_base_frame='base_link'):
        catalog = load_bt_catalog(CATALOG_PATH)
        agent = create_nav2_agent('test', 'http://localhost:1/v1', catalog, robot_base_frame=robot_base_frame)
        deps = AgentDependencies(bt_catalog=catalog, command='test', logger=logging.getLogger(), trace=[])
        retries = []

        async def run():
            with agent.override(model=scripted_model(tool_calls, retries)):
                try:
                    await agent.run('test', deps=deps)
                except PlanningComplete as completed:
                    return completed.plan
            return None

        return asyncio.run(run()), retries

    def test_relative_pose_uses_robot_base_frame(self):
        plan, _ = self.plan([
            ToolCallPart('tool_select_navigation_action', {'action': 'navigate_to_pose', 'expected_pose_count': 1}),
            ToolCallPart('tool_make_relative_translation', {'direction': 'forward', 'distance_m': 2.0}),
            ToolCallPart('tool_create_behavior_tree', {'filename': 'bt.xml', 'xml': VALID_TREE, 'reasoning': 'r'}),
        ], robot_base_frame='pelvis')

        self.assertEqual('pelvis', plan.target_pose.frame_id)
        self.assertEqual(2.0, plan.target_pose.x)

    def test_invalid_tree_is_returned_to_the_model(self):
        plan, retries = self.plan([
            ToolCallPart('tool_select_navigation_action', {'action': 'navigate_to_pose', 'expected_pose_count': 1}),
            ToolCallPart('tool_make_relative_turn', {'direction': 'left', 'angle_rad': 1.5708}),
            ToolCallPart('tool_create_behavior_tree', {'filename': 'bt.xml', 'xml': INVALID_TREE, 'reasoning': 'r'}),
            ToolCallPart('tool_create_behavior_tree', {'filename': 'bt.xml', 'xml': VALID_TREE, 'reasoning': 'r'}),
        ])

        self.assertEqual(1, len(retries))
        self.assertIn('consumes {path} before it is available', str(retries[0].content))
        self.assertAlmostEqual(1.5708, plan.target_pose.theta)


if __name__ == '__main__':
    unittest.main()
