"""ROS 2 node that bridges natural-language commands to the pydantic-ai Nav2 agent."""

import asyncio
import json
import logging
import threading
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from nav2_agent.models import AgentResponse
from nav2_agent.nav2_bridge import Nav2Bridge
from nav2_agent.pydantic_agent import AgentDependencies, create_nav2_agent


class Nav2AgentNode(Node):
    """Main ROS 2 node for natural-language navigation commands."""

    def __init__(self) -> None:
        super().__init__('nav2_agent_node')

        self.declare_parameter('vllm_model_name', 'Qwen/Qwen2.5-7B-Instruct')
        self.declare_parameter('vllm_api_base', 'http://localhost:8000/v1')
        self.declare_parameter('system_prompt', '')
        self.declare_parameter('bt_catalog_path', '')
        self.declare_parameter('default_trigger_command', '')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('navigate_through_poses_action', '/navigate_through_poses')
        self.declare_parameter('action_server_timeout_sec', 5.0)

        self._status_pub = self.create_publisher(String, '/nav2_agent/status', 10)
        self._voice_sub = self.create_subscription(String, '/voice_command', self._voice_command_callback, 10)
        self._trigger_srv = self.create_service(Trigger, '/nav2_agent/trigger_command', self._trigger_callback)

        self._last_voice_command: Optional[str] = None
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._run_async_loop, name='nav2_agent_asyncio', daemon=True)
        self._loop_thread.start()

        bt_catalog_path = self._resolve_bt_catalog_path(self.get_parameter('bt_catalog_path').value)
        self._bt_catalog = self._load_bt_catalog(bt_catalog_path)

        self._bridge = Nav2Bridge(
            node=self,
            logger=logging.getLogger('nav2_agent.nav2_bridge'),
            navigate_to_pose_action=str(self.get_parameter('navigate_to_pose_action').value),
            navigate_through_poses_action=str(self.get_parameter('navigate_through_poses_action').value),
            action_server_timeout_sec=float(self.get_parameter('action_server_timeout_sec').value),
        )
        self._deps = AgentDependencies(
            bridge=self._bridge,
            bt_catalog=self._bt_catalog,
            logger=logging.getLogger('nav2_agent.pydantic_agent'),
        )
        self._agent = create_nav2_agent(
            model_name=str(self.get_parameter('vllm_model_name').value),
            api_base=str(self.get_parameter('vllm_api_base').value),
            bt_catalog=self._bt_catalog,
            system_prompt=str(self.get_parameter('system_prompt').value or ''),
        )

        self._publish_status('ready', 'Nav2 agent node initialized.')
        self.get_logger().info('Nav2 agent node initialized with BT catalog: %s' % sorted(self._bt_catalog.keys()))

    def destroy_node(self) -> bool:
        """Stop the asyncio loop before tearing down the ROS node."""
        self._bridge.destroy()
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._loop_thread.is_alive():
            self._loop_thread.join(timeout=2.0)
        self._loop.close()
        return super().destroy_node()

    def _run_async_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _voice_command_callback(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            self._publish_status('ignored', 'Received an empty voice command.')
            return

        self._last_voice_command = command
        self._schedule_agent_command(command=command, source='topic:/voice_command')

    def _trigger_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        command = self._last_voice_command or str(self.get_parameter('default_trigger_command').value or '').strip()
        if not command:
            response.success = False
            response.message = (
                'No command available. Publish a std_msgs/String on /voice_command or set default_trigger_command.'
            )
            self._publish_status('rejected', response.message)
            return response

        self._schedule_agent_command(command=command, source='service:/nav2_agent/trigger_command')
        response.success = True
        response.message = f'Command accepted for asynchronous execution: {command}'
        return response

    def _schedule_agent_command(self, command: str, source: str) -> None:
        self._publish_status('accepted', f'Command accepted from {source}: {command}')
        future = asyncio.run_coroutine_threadsafe(self._run_agent(command=command, source=source), self._loop)
        future.add_done_callback(lambda completed: self._agent_done_callback(completed, command, source))

    async def _run_agent(self, command: str, source: str) -> AgentResponse:
        del source
        result = await self._agent.run(command, deps=self._deps)
        output = getattr(result, 'output', None)
        if output is None:
            output = getattr(result, 'data', None)
        if isinstance(output, AgentResponse):
            return output
        if isinstance(output, dict):
            return AgentResponse.model_validate(output)
        return AgentResponse(success=True, message=str(output), actions_executed=[])

    def _agent_done_callback(self, future: Any, command: str, source: str) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - depends on external LLM server runtime
            self.get_logger().error('Agent execution failed for command %r from %s: %s' % (command, source, exc))
            self._publish_status('failed', f'Agent execution failed: {exc}')
            return

        state = 'succeeded' if response.success else 'failed'
        self._publish_status(
            state,
            response.message,
            actions_executed=response.actions_executed,
            command=command,
            source=source,
        )

    def _publish_status(self, state: str, message: str, **extra: Any) -> None:
        payload = {'state': state, 'message': message, **extra}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self._status_pub.publish(msg)

    def _resolve_bt_catalog_path(self, configured_path: Any) -> Path:
        if configured_path:
            path = Path(str(configured_path)).expanduser()
            if path.is_file():
                return path
            self.get_logger().warning('Configured bt_catalog_path does not exist: %s' % path)

        share_dir = Path(get_package_share_directory('nav2_agent'))
        return share_dir / 'config' / 'bt_catalog.yaml'

    def _load_bt_catalog(self, path: Path) -> Dict[str, Dict[str, Any]]:
        if not path.is_file():
            raise FileNotFoundError(f'Behavior Tree catalog file not found: {path}')

        with path.open('r', encoding='utf-8') as yaml_file:
            raw_catalog = yaml.safe_load(yaml_file) or {}

        behavior_trees = raw_catalog.get('behavior_trees', raw_catalog)
        if isinstance(behavior_trees, list):
            catalog = {str(item['id']): dict(item) for item in behavior_trees if 'id' in item}
        elif isinstance(behavior_trees, dict):
            catalog = {str(bt_id): dict(metadata or {}) for bt_id, metadata in behavior_trees.items()}
        else:
            raise ValueError(f'Invalid Behavior Tree catalog format in {path}')

        if not catalog:
            raise ValueError(f'Behavior Tree catalog is empty: {path}')
        return catalog


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Nav2AgentNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
