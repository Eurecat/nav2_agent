"""Asynchronous bridge between the agent and Nav2 action interfaces."""

import asyncio
import hashlib
import logging
import math
from typing import Any, List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_agent.models import TargetPose


class Nav2Bridge:
    """Boundary between agent tools and Nav2 ROS 2 actions.

    Spatial lookup and recovery are still mocked. Navigation methods use real
    ActionClient instances for Nav2 NavigateToPose and NavigateThroughPoses.
    """

    def __init__(
        self,
        node: Node,
        logger: Optional[logging.Logger] = None,
        navigate_to_pose_action: str = '/navigate_to_pose',
        navigate_through_poses_action: str = '/navigate_through_poses',
        action_server_timeout_sec: float = 5.0,
        mock_latency_seconds: float = 0.05,
    ) -> None:
        self._node = node
        self._logger = logger or logging.getLogger(__name__)
        self._navigate_to_pose_action = navigate_to_pose_action
        self._navigate_through_poses_action = navigate_through_poses_action
        self._action_server_timeout_sec = action_server_timeout_sec
        self._mock_latency_seconds = mock_latency_seconds
        self._navigate_to_pose_client = ActionClient(self._node, NavigateToPose, self._navigate_to_pose_action)
        self._navigate_through_poses_client = ActionClient(
            self._node,
            NavigateThroughPoses,
            self._navigate_through_poses_action,
        )

    async def extract_frame_coordinates(self, entity_name: str, reference_frame: str = 'map') -> TargetPose:
        """Simulate extracting coordinates for a detected or referenced entity."""
        await asyncio.sleep(self._mock_latency_seconds)
        digest = hashlib.sha256(f'{entity_name}:{reference_frame}'.encode('utf-8')).digest()
        x = round((digest[0] / 255.0) * 8.0 - 4.0, 2)
        y = round((digest[1] / 255.0) * 8.0 - 4.0, 2)
        theta = round((digest[2] / 255.0) * 6.283185307179586 - 3.141592653589793, 2)
        pose = TargetPose(frame_id=reference_frame, x=x, y=y, theta=theta)
        self._logger.info(
            'Mock frame extraction: entity=%s reference_frame=%s pose=%s',
            entity_name,
            reference_frame,
            pose.model_dump(),
        )
        return pose

    async def send_navigate_to_pose(self, target: TargetPose, bt_xml: Optional[str] = None) -> bool:
        """Send a NavigateToPose goal to Nav2."""
        await self._wait_for_action_server(self._navigate_to_pose_client, self._navigate_to_pose_action)

        goal = NavigateToPose.Goal()
        goal.pose = self._target_pose_to_pose_stamped(target)
        goal.behavior_tree = bt_xml or ''

        self._logger.info(
            'Sending NavigateToPose goal: target=%s bt_xml=%s action=%s',
            target.model_dump(),
            bt_xml or 'default',
            self._navigate_to_pose_action,
        )
        return await self._send_goal_and_wait_for_result(self._navigate_to_pose_client, goal, 'NavigateToPose')

    async def send_navigate_through_poses(self, targets: List[TargetPose], bt_xml: Optional[str] = None) -> bool:
        """Send a NavigateThroughPoses goal to Nav2."""
        await self._wait_for_action_server(self._navigate_through_poses_client, self._navigate_through_poses_action)

        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._target_pose_to_pose_stamped(target) for target in targets]
        goal.behavior_tree = bt_xml or ''

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

    async def execute_recovery(self, recovery_type: str) -> bool:
        """Simulate invoking a recovery behavior such as spin or wait."""
        await asyncio.sleep(self._mock_latency_seconds)
        normalized_recovery = recovery_type.strip().lower()
        if normalized_recovery not in {'spin', 'wait'}:
            self._logger.warning('Mock recovery rejected: unsupported recovery_type=%s', recovery_type)
            return False

        self._logger.info('Mock recovery behavior executed: recovery_type=%s', normalized_recovery)
        return True

    def destroy(self) -> None:
        """Release action client resources owned by the bridge."""
        self._navigate_to_pose_client.destroy()
        self._navigate_through_poses_client.destroy()

    async def _wait_for_action_server(self, client: ActionClient, action_name: str) -> None:
        available = await asyncio.to_thread(client.wait_for_server, timeout_sec=self._action_server_timeout_sec)
        if not available:
            raise TimeoutError(
                f'Nav2 action server {action_name!r} was not available within '
                f'{self._action_server_timeout_sec:.1f} seconds.'
            )

    async def _send_goal_and_wait_for_result(self, client: ActionClient, goal: Any, action_label: str) -> bool:
        goal_handle = await self._await_ros_future(client.send_goal_async(goal))
        if not goal_handle.accepted:
            self._logger.warning('%s goal was rejected by Nav2.', action_label)
            return False

        self._logger.info('%s goal accepted by Nav2.', action_label)
        result_response = await self._await_ros_future(goal_handle.get_result_async())
        succeeded = result_response.status == GoalStatus.STATUS_SUCCEEDED
        if succeeded:
            self._logger.info('%s goal succeeded.', action_label)
        else:
            self._logger.warning('%s goal finished with status=%s.', action_label, result_response.status)
        return succeeded

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
