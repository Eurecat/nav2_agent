"""Asynchronous bridge between the agent and Nav2 action interfaces."""

import asyncio
import logging
import math
from typing import Any, List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_agent.models import NavigationOutcome, TargetPose


BT_XML_PREFIX = '/home/user/workspace/src/navigation/behavior_trees/'

GOAL_STATUS_LABELS = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class Nav2Bridge:
    """Boundary between validated navigation plans and Nav2 ROS 2 actions."""

    def __init__(
        self,
        node: Node,
        logger: Optional[logging.Logger] = None,
        navigate_to_pose_action: str = '/navigate_to_pose',
        navigate_through_poses_action: str = '/navigate_through_poses',
        action_server_timeout_sec: float = 5.0,
        dry_run_nav2: bool = False,
    ) -> None:
        self._node = node
        self._logger = logger or logging.getLogger(__name__)
        self._navigate_to_pose_action = navigate_to_pose_action
        self._navigate_through_poses_action = navigate_through_poses_action
        self._action_server_timeout_sec = action_server_timeout_sec
        self._dry_run_nav2 = dry_run_nav2
        self._navigate_to_pose_client = ActionClient(self._node, NavigateToPose, self._navigate_to_pose_action)
        self._navigate_through_poses_client = ActionClient(
            self._node,
            NavigateThroughPoses,
            self._navigate_through_poses_action,
        )

    async def send_navigate_to_pose(self, target: TargetPose, bt_xml: Optional[str] = None) -> NavigationOutcome:
        """Send a NavigateToPose goal to Nav2."""
        goal = NavigateToPose.Goal()
        goal.pose = self._target_pose_to_pose_stamped(target)
        goal.behavior_tree = self._behavior_tree_path(bt_xml)
        goal_details = self._navigate_to_pose_goal_to_dict(goal)

        if self._dry_run_nav2:
            self._logger.debug(
                'Dry-run NavigateToPose goal: action=%s goal=%s',
                self._navigate_to_pose_action,
                goal_details,
            )
            return NavigationOutcome(succeeded=True, status='DRY_RUN')

        await self._wait_for_action_server(self._navigate_to_pose_client, self._navigate_to_pose_action)

        self._logger.info(
            'Sending NavigateToPose goal: target=%s bt_xml=%s action=%s',
            target.model_dump(),
            bt_xml or 'default',
            self._navigate_to_pose_action,
        )
        return await self._send_goal_and_wait_for_result(self._navigate_to_pose_client, goal, 'NavigateToPose')

    async def send_navigate_through_poses(self, targets: List[TargetPose], bt_xml: Optional[str] = None) -> NavigationOutcome:
        """Send a NavigateThroughPoses goal to Nav2."""
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._target_pose_to_pose_stamped(target) for target in targets]
        goal.behavior_tree = self._behavior_tree_path(bt_xml)
        goal_details = self._navigate_through_poses_goal_to_dict(goal)

        if self._dry_run_nav2:
            self._logger.debug(
                'Dry-run NavigateThroughPoses goal: action=%s goal=%s',
                self._navigate_through_poses_action,
                goal_details,
            )
            return NavigationOutcome(succeeded=True, status='DRY_RUN')

        await self._wait_for_action_server(self._navigate_through_poses_client, self._navigate_through_poses_action)

        self._logger.info(
            'Sending NavigateThroughPoses goal: targets=%s bt_xml=%s action=%s',
            [target.model_dump() for target in targets],
            bt_xml or 'default',
            self._navigate_through_poses_action,
        )
        return await self._send_goal_and_wait_for_result(
            self._navigate_through_poses_client,
            goal,
            'NavigateThroughPoses',
        )

    def destroy(self) -> None:
        """Release action client resources owned by the bridge."""
        self._navigate_to_pose_client.destroy()
        self._navigate_through_poses_client.destroy()

    def describe_navigate_to_pose_goal(self, target: TargetPose, bt_xml: Optional[str] = None) -> dict[str, Any]:
        goal = NavigateToPose.Goal()
        goal.pose = self._target_pose_to_pose_stamped(target)
        goal.behavior_tree = self._behavior_tree_path(bt_xml)
        return self._navigate_to_pose_goal_to_dict(goal)

    def describe_navigate_through_poses_goal(
        self,
        targets: List[TargetPose],
        bt_xml: Optional[str] = None,
    ) -> dict[str, Any]:
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._target_pose_to_pose_stamped(target) for target in targets]
        goal.behavior_tree = self._behavior_tree_path(bt_xml)
        return self._navigate_through_poses_goal_to_dict(goal)

    def _behavior_tree_path(self, bt_xml: Optional[str]) -> str:
        if not bt_xml:
            return ''
        if bt_xml.startswith('/'):
            return bt_xml
        return BT_XML_PREFIX + bt_xml

    async def _wait_for_action_server(self, client: ActionClient, action_name: str) -> None:
        available = await asyncio.to_thread(client.wait_for_server, timeout_sec=self._action_server_timeout_sec)
        if not available:
            raise TimeoutError(
                f'Nav2 action server {action_name!r} was not available within '
                f'{self._action_server_timeout_sec:.1f} seconds.'
            )

    async def _send_goal_and_wait_for_result(
        self,
        client: ActionClient,
        goal: Any,
        action_label: str,
    ) -> NavigationOutcome:
        last_feedback: dict[str, Any] = {}

        def _feedback_callback(feedback_msg: Any) -> None:
            last_feedback['feedback'] = feedback_msg.feedback

        goal_handle = await self._await_ros_future(
            client.send_goal_async(goal, feedback_callback=_feedback_callback)
        )
        if not goal_handle.accepted:
            self._logger.warning('%s goal was rejected by Nav2.', action_label)
            return NavigationOutcome(succeeded=False, status='REJECTED')

        self._logger.info('%s goal accepted by Nav2.', action_label)
        result_response = await self._await_ros_future(goal_handle.get_result_async())
        outcome = self._navigation_outcome(result_response, last_feedback.get('feedback'))
        if outcome.succeeded:
            self._logger.info('%s goal succeeded: %s', action_label, outcome.model_dump())
        else:
            self._logger.warning('%s goal finished without success: %s', action_label, outcome.model_dump())
        return outcome

    def _navigation_outcome(self, result_response: Any, feedback: Optional[Any]) -> NavigationOutcome:
        result = result_response.result
        outcome = NavigationOutcome(
            succeeded=result_response.status == GoalStatus.STATUS_SUCCEEDED,
            status=GOAL_STATUS_LABELS.get(result_response.status, str(result_response.status)),
            error_code=int(getattr(result, 'error_code', 0) or 0),
            error_msg=str(getattr(result, 'error_msg', '') or ''),
        )
        if feedback is None:
            return outcome

        outcome.number_of_recoveries = int(getattr(feedback, 'number_of_recoveries', 0))
        outcome.distance_remaining = float(getattr(feedback, 'distance_remaining', 0.0))
        navigation_time = getattr(feedback, 'navigation_time', None)
        if navigation_time is not None:
            outcome.navigation_time_sec = navigation_time.sec + navigation_time.nanosec * 1e-9
        current_pose = getattr(feedback, 'current_pose', None)
        if current_pose is not None:
            outcome.last_pose = self._pose_stamped_to_target_pose(current_pose)
        return outcome

    def _pose_stamped_to_target_pose(self, pose: PoseStamped) -> TargetPose:
        orientation = pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        return TargetPose(
            frame_id=pose.header.frame_id,
            x=pose.pose.position.x,
            y=pose.pose.position.y,
            theta=yaw,
        )

    async def _await_ros_future(self, ros_future: Any) -> Any:
        loop = asyncio.get_running_loop()
        asyncio_future = loop.create_future()

        def _complete(done_future: Any) -> None:
            try:
                result = done_future.result()
            except Exception as exc:
                loop.call_soon_threadsafe(asyncio_future.set_exception, exc)
            else:
                loop.call_soon_threadsafe(asyncio_future.set_result, result)

        ros_future.add_done_callback(_complete)
        return await asyncio_future

    def _target_pose_to_pose_stamped(self, target: TargetPose) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = target.frame_id
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = target.x
        pose.pose.position.y = target.y
        pose.pose.position.z = 0.0

        half_yaw = target.theta * 0.5
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return pose

    def _navigate_to_pose_goal_to_dict(self, goal: NavigateToPose.Goal) -> dict[str, Any]:
        return {
            'ros_goal_type': 'nav2_msgs/action/NavigateToPose.Goal',
            'pose': self._pose_stamped_to_dict(goal.pose),
            'behavior_tree': goal.behavior_tree,
        }

    def _navigate_through_poses_goal_to_dict(self, goal: NavigateThroughPoses.Goal) -> dict[str, Any]:
        return {
            'ros_goal_type': 'nav2_msgs/action/NavigateThroughPoses.Goal',
            'poses': [self._pose_stamped_to_dict(pose) for pose in goal.poses],
            'behavior_tree': goal.behavior_tree,
        }

    def _pose_stamped_to_dict(self, pose: PoseStamped) -> dict[str, Any]:
        return {
            'header': {
                'frame_id': pose.header.frame_id,
                'stamp': {
                    'sec': pose.header.stamp.sec,
                    'nanosec': pose.header.stamp.nanosec,
                },
            },
            'pose': {
                'position': {
                    'x': pose.pose.position.x,
                    'y': pose.pose.position.y,
                    'z': pose.pose.position.z,
                },
                'orientation': {
                    'x': pose.pose.orientation.x,
                    'y': pose.pose.orientation.y,
                    'z': pose.pose.orientation.z,
                    'w': pose.pose.orientation.w,
                },
            },
        }
