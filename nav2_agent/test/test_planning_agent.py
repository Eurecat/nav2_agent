import asyncio
import logging
import unittest
from pathlib import Path

try:
    from pydantic_ai.messages import ModelResponse, RetryPromptPart, ToolCallPart
    from pydantic_ai.models.function import FunctionModel

    from nav2_agent.bt_catalog import load_bt_catalog
    from nav2_agent.locations import load_locations
    from nav2_agent.pydantic_agent import AgentDependencies, PlanningComplete, create_nav2_agent
except ImportError:  # pydantic-ai is not installed
    FunctionModel = None

CATALOG_PATH = Path(__file__).resolve().parents[1] / 'config' / 'bt_catalog.yaml'
LOCATIONS_PATH = Path(__file__).resolve().parents[1] / 'config' / 'locations.yaml'

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
        agent = create_nav2_agent('test', 'http://localhost:1/v1', catalog, robot_base_frame=robot_base_frame,
                                  locations=load_locations(LOCATIONS_PATH))
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

    def test_named_location(self):
        plan, retries = self.plan([
            ToolCallPart('tool_select_navigation_action', {'action': 'navigate_to_pose', 'expected_pose_count': 1}),
            ToolCallPart('tool_get_location', {'name': 'Loading Area'}),
            ToolCallPart('tool_create_behavior_tree', {'filename': 'bt.xml', 'xml': VALID_TREE, 'reasoning': 'r'}),
        ])

        self.assertEqual([], retries)
        self.assertEqual(('map', 20.3, 1.2), (plan.target_pose.frame_id, plan.target_pose.x, plan.target_pose.y))

    def test_unknown_location_is_returned_to_the_model(self):
        plan, retries = self.plan([
            ToolCallPart('tool_select_navigation_action', {'action': 'navigate_to_pose', 'expected_pose_count': 1}),
            ToolCallPart('tool_get_location', {'name': 'kitchen'}),
            ToolCallPart('tool_get_location', {'name': 'shelves'}),
            ToolCallPart('tool_create_behavior_tree', {'filename': 'bt.xml', 'xml': VALID_TREE, 'reasoning': 'r'}),
        ])

        self.assertIn("Unknown location 'kitchen'", str(retries[0].content))
        self.assertEqual(12.5, plan.target_pose.x)

    def test_relative_motion_after_location_is_rejected(self):
        _, retries = self.plan([
            ToolCallPart('tool_select_navigation_action', {'action': 'navigate_through_poses', 'expected_pose_count': 2}),
            ToolCallPart('tool_get_location', {'name': 'shelves'}),
            ToolCallPart('tool_make_relative_translation', {'direction': 'forward', 'distance_m': 1.0}),
            ToolCallPart('tool_get_location', {'name': 'loading_area'}),
            ToolCallPart('tool_create_behavior_tree', {'filename': 'bt.xml', 'xml': VALID_TREE.replace(
                'ComputePathToPose goal="{goal}"', 'ComputePathThroughPoses goals="{goals}"'), 'reasoning': 'r'}),
        ])

        self.assertIn('Relative motion is only supported', str(retries[0].content))


if __name__ == '__main__':
    unittest.main()
