from pathlib import Path
import unittest

from nav2_agent.bt_catalog import (
    behavior_tree_mermaid,
    behavior_tree_node_summary,
  format_behavior_tree_xml,
    format_catalog_reference,
    load_bt_catalog,
    validate_behavior_tree,
)

CATALOG_PATH = Path(__file__).resolve().parents[1] / 'config' / 'bt_catalog.yaml'


class TestBehaviorTreeCatalog(unittest.TestCase):
    def setUp(self):
        self.catalog = load_bt_catalog(CATALOG_PATH)

    def test_load_catalog_contains_nodes_and_action_contracts(self):
        self.assertEqual({'nodes', 'actions'}, set(self.catalog))
        self.assertIn('ComputePathToPose', self.catalog['nodes'])
        self.assertIn('FollowPath', self.catalog['nodes'])
        self.assertEqual(['goal'], self.catalog['actions']['navigate_to_pose']['initial_blackboard'])

    def test_format_catalog_reference_includes_nodes_and_actions(self):
        reference = format_catalog_reference(self.catalog)

        self.assertIn('Available Behavior Tree nodes:', reference)
        self.assertIn('Action contracts:', reference)
        self.assertIn('ComputePathToPose', reference)
        self.assertIn('navigate_to_pose', reference)

    def test_validate_minimal_navigate_to_pose_tree(self):
        validate_behavior_tree(
            '''<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <ComputePathToPose goal="{goal}" path="{path}"/>
      <FollowPath path="{path}"/>
    </Sequence>
  </BehaviorTree>
</root>''',
            self.catalog,
            action='navigate_to_pose',
            command='Move 1 meter forward.',
        )

    def test_validate_minimal_navigate_through_poses_tree(self):
        validate_behavior_tree(
            '''<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <ComputePathThroughPoses goals="{goals}" path="{path}"/>
      <FollowPath path="{path}"/>
    </Sequence>
  </BehaviorTree>
</root>''',
            self.catalog,
            action='navigate_through_poses',
            command='Move through these two waypoints.',
        )

    def test_unknown_node_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'nodes not present in catalog: MagicNavigate'):
            validate_behavior_tree(
                '''<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <MagicNavigate goal="{goal}" path="{path}"/>
      <FollowPath path="{path}"/>
    </Sequence>
  </BehaviorTree>
</root>''',
                self.catalog,
                action='navigate_to_pose',
                command='Move 1 meter forward.',
            )

    def test_missing_required_attribute_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "<FollowPath> is missing required attribute 'path'"):
            validate_behavior_tree(
                '''<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <ComputePathToPose goal="{goal}" path="{path}"/>
      <FollowPath/>
    </Sequence>
  </BehaviorTree>
</root>''',
                self.catalog,
                action='navigate_to_pose',
                command='Move 1 meter forward.',
            )

    def test_blackboard_consume_before_produce_is_rejected(self):
        with self.assertRaisesRegex(ValueError, r'<FollowPath> consumes \{path\} before it is available'):
            validate_behavior_tree(
                '''<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <FollowPath path="{path}"/>
      <ComputePathToPose goal="{goal}" path="{path}"/>
    </Sequence>
  </BehaviorTree>
</root>''',
                self.catalog,
                action='navigate_to_pose',
                command='Move 1 meter forward.',
            )

    def test_action_contract_rejects_wrong_path_computation_node(self):
        with self.assertRaisesRegex(
            ValueError,
            r"Action 'navigate_to_pose' forbids node\(s\): <ComputePathThroughPoses>",
        ):
            validate_behavior_tree(
                '''<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <ComputePathThroughPoses goals="{goals}" path="{path}"/>
      <FollowPath path="{path}"/>
    </Sequence>
  </BehaviorTree>
</root>''',
                self.catalog,
                action='navigate_to_pose',
                command='Move 1 meter forward.',
            )

    def test_recovery_node_requires_explicit_request(self):
        with self.assertRaisesRegex(ValueError, '<RecoveryNode> requires an explicit user request'):
            validate_behavior_tree(
                '''<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="1">
      <Sequence>
        <ComputePathToPose goal="{goal}" path="{path}"/>
        <FollowPath path="{path}"/>
      </Sequence>
      <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
    </RecoveryNode>
  </BehaviorTree>
</root>''',
                self.catalog,
                action='navigate_to_pose',
                command='Move 1 meter forward.',
            )

    def test_recovery_node_is_allowed_when_explicitly_requested(self):
        validate_behavior_tree(
            '''<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="1">
      <Sequence>
        <ComputePathToPose goal="{goal}" path="{path}"/>
        <FollowPath path="{path}"/>
      </Sequence>
      <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
    </RecoveryNode>
  </BehaviorTree>
</root>''',
            self.catalog,
            action='navigate_to_pose',
            command='Move 1 meter forward and recover if it fails by clearing the costmap.',
        )

    def test_blackboard_ports_must_use_braced_keys(self):
        with self.assertRaisesRegex(ValueError, 'must reference a blackboard key'):
            validate_behavior_tree(
                '''<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <ComputePathToPose goal="goal" path="{path}"/>
      <FollowPath path="{path}"/>
    </Sequence>
  </BehaviorTree>
</root>''',
                self.catalog,
                action='navigate_to_pose',
                command='Move 1 meter forward.',
            )

    def test_behavior_tree_rendering_helpers(self):
        xml = '<root main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree"><Sequence name="Main"><ComputePathToPose goal="{goal}" path="{path}"/><FollowPath path="{path}"/></Sequence></BehaviorTree></root>'

        self.assertEqual(['Sequence', 'ComputePathToPose', 'FollowPath'], behavior_tree_node_summary(xml))
        formatted_xml = format_behavior_tree_xml(xml)
        self.assertIn('\n  <BehaviorTree ID="MainTree">', formatted_xml)
        self.assertIn('\n      <ComputePathToPose', formatted_xml)
        mermaid = behavior_tree_mermaid(xml)
        self.assertIn('graph TD', mermaid)
        self.assertIn('Sequence: Main', mermaid)
        self.assertIn('ComputePathToPose', mermaid)


if __name__ == '__main__':
    unittest.main()
