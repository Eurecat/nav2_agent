"""ROS 2 node that bridges natural-language commands to the pydantic-ai Nav2 agent."""

import asyncio
import json
import logging
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from nav2_agent.bt_catalog import (
    behavior_tree_mermaid,
    behavior_tree_node_summary,
    format_catalog_reference,
    indent_text,
    load_bt_catalog,
)
from nav2_agent.models import AgentResponse, NavigationPlan
from nav2_agent.nav2_bridge import Nav2Bridge
from nav2_agent.pydantic_agent import DEFAULT_SYSTEM_PROMPT, AgentDependencies, PlanningComplete, create_nav2_agent

try:
    from pydantic_ai import capture_run_messages
except ImportError:  # pragma: no cover - depends on runtime pydantic-ai version
    capture_run_messages = None  # type: ignore[assignment]


class Nav2AgentNode(Node):
    """Main ROS 2 node for natural-language navigation commands."""

    def __init__(self) -> None:
        super().__init__('nav2_agent_node')

        self.declare_parameter('vllm_model_name', 'gemma-4-e4b')
        self.declare_parameter('vllm_api_base', 'http://localhost:8000/v1')
        self.declare_parameter('vllm_api_key', 'EMPTY')
        self.declare_parameter('system_prompt', '')
        self.declare_parameter('bt_catalog_path', '')
        self.declare_parameter('generated_bt_dir', '/tmp/nav2_agent/behavior_trees')
        self.declare_parameter('default_trigger_command', '')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('navigate_through_poses_action', '/navigate_through_poses')
        self.declare_parameter('action_server_timeout_sec', 5.0)
        self.declare_parameter('agent_run_timeout_sec', 90.0)
        self.declare_parameter('dry_run_nav2', False)

        self._status_pub = self.create_publisher(String, '/nav2_agent/status', 10)
        self._user_command_sub = self.create_subscription(String, '/user_command', self._user_command_callback, 10)
        self._trigger_srv = self.create_service(Trigger, '/nav2_agent/trigger_command', self._trigger_callback)

        self._vllm_model_name = str(self.get_parameter('vllm_model_name').value)
        self._vllm_api_base = str(self.get_parameter('vllm_api_base').value)
        self._vllm_api_key = str(self.get_parameter('vllm_api_key').value)
        self._system_prompt = str(self.get_parameter('system_prompt').value or '')
        self._navigate_to_pose_action = str(self.get_parameter('navigate_to_pose_action').value)
        self._navigate_through_poses_action = str(self.get_parameter('navigate_through_poses_action').value)
        self._action_server_timeout_sec = float(self.get_parameter('action_server_timeout_sec').value)
        self._agent_run_timeout_sec = float(self.get_parameter('agent_run_timeout_sec').value)
        self._dry_run_nav2 = bool(self.get_parameter('dry_run_nav2').value)
        self._default_trigger_command = str(self.get_parameter('default_trigger_command').value or '').strip()
        self._generated_bt_dir = Path(str(self.get_parameter('generated_bt_dir').value)).expanduser()
        self._last_user_command: Optional[str] = None
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._run_async_loop, name='nav2_agent_asyncio', daemon=True)
        self._loop_thread.start()

        bt_catalog_path = self._resolve_bt_catalog_path(self.get_parameter('bt_catalog_path').value)
        self._bt_catalog = load_bt_catalog(bt_catalog_path)

        self._bridge = Nav2Bridge(
            node=self,
            logger=logging.getLogger('nav2_agent.nav2_bridge'),
            navigate_to_pose_action=self._navigate_to_pose_action,
            navigate_through_poses_action=self._navigate_through_poses_action,
            action_server_timeout_sec=self._action_server_timeout_sec,
            dry_run_nav2=self._dry_run_nav2,
        )
        self._agent = create_nav2_agent(
            model_name=self._vllm_model_name,
            api_base=self._vllm_api_base,
            bt_catalog=self._bt_catalog,
            api_key=self._vllm_api_key,
            system_prompt=self._system_prompt,
        )

        self._publish_status('ready', 'Nav2 agent node initialized.')
        self._log_startup_banner()
        if capture_run_messages is None:
            self.get_logger().debug('pydantic-ai capture_run_messages is unavailable; raw agent traces disabled.')

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

    def _log_startup_banner(self) -> None:
        default_command = self._default_trigger_command or '<none>'
        banner = f'''

============================================================
 nav2_agent_node started
============================================================
 Agent runtime
   model:                 {self._vllm_model_name}
   api_base:              {self._vllm_api_base}
     agent run timeout:     {self._agent_run_timeout_sec:.1f}s

 Navigation boundary
   dry_run_nav2:          {self._dry_run_nav2}
   NavigateToPose:        {self._navigate_to_pose_action}
   NavigateThroughPoses:  {self._navigate_through_poses_action}
   action timeout:        {self._action_server_timeout_sec:.1f}s

 Command input
    topic:                 /user_command
   trigger service:       /nav2_agent/trigger_command
   status topic:          /nav2_agent/status
   default command:       {default_command}

 Motion contract
   map:                   explicit global x/y/theta commands
   base_link:             relative forward/back/left/right/turn commands
   axes:                  x forward, y left, theta yaw radians
    yaw sign:              right negative, left positive

 Behavior Trees
    catalog nodes:         {', '.join(sorted(self._bt_catalog['nodes'].keys()))}
    generated dir:         {self._generated_bt_dir}
============================================================
'''
        self.get_logger().info(banner)

    def _user_command_callback(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            self._publish_status('ignored', 'Received an empty user command.')
            return

        self._last_user_command = command
        self.get_logger().info('Received user command: %s' % command)
        self._schedule_agent_command(command=command, source='topic:/user_command')

    def _trigger_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        command = self._last_user_command or str(self.get_parameter('default_trigger_command').value or '').strip()
        if not command:
            response.success = False
            response.message = (
                'No command available. Publish a std_msgs/String on /user_command or set default_trigger_command.'
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
        trace = [{'step': 'input', 'command': command}]
        deps = AgentDependencies(
            bt_catalog=self._bt_catalog,
            command=command,
            logger=logging.getLogger('nav2_agent.pydantic_agent'),
            trace=trace,
            debug_log=self.get_logger().debug,
        )
        plan = await self._extract_navigation_plan(command, deps)

        return await self._execute_navigation_plan(plan=plan, trace=trace)

    async def _extract_navigation_plan(self, command: str, deps: AgentDependencies) -> NavigationPlan:
        run_messages = None
        if capture_run_messages is not None:
            try:
                with capture_run_messages() as captured_messages:
                    result = await asyncio.wait_for(
                        self._agent.run(command, deps=deps),
                        timeout=self._agent_run_timeout_sec,
                    )
            except PlanningComplete as completed:
                self._log_agent_messages(captured_messages)
                self.get_logger().debug(
                    'PlanningComplete received; executing validated NavigationPlan:\n%s'
                    % self._format_navigation_plan(completed.plan)
                )
                return completed.plan
            except asyncio.TimeoutError as exc:
                self._log_agent_messages(captured_messages)
                self.get_logger().warning('Planning trace before timeout: %s' % json.dumps(deps.trace, default=str))
                if deps.proposed_plan is not None:
                    self.get_logger().warning('Using NavigationPlan validated by planning tool before timeout.')
                    return deps.proposed_plan
                if self._debug_logging_enabled():
                    await self._debug_raw_chat_completion(command)
                raise TimeoutError(
                    f'Navigation plan extraction timed out after {self._agent_run_timeout_sec:.1f} seconds.'
                ) from exc
            except Exception:
                self._log_agent_messages(captured_messages)
                if deps.proposed_plan is not None:
                    self.get_logger().warning('Using NavigationPlan validated by planning tool after agent finalization failed.')
                    return deps.proposed_plan
                if self._debug_logging_enabled():
                    await self._debug_raw_chat_completion(command)
                raise
            run_messages = captured_messages
        else:
            try:
                result = await asyncio.wait_for(
                    self._agent.run(command, deps=deps),
                    timeout=self._agent_run_timeout_sec,
                )
            except PlanningComplete as completed:
                self.get_logger().debug(
                    'PlanningComplete received; executing validated NavigationPlan:\n%s'
                    % self._format_navigation_plan(completed.plan)
                )
                return completed.plan
            except asyncio.TimeoutError as exc:
                if deps.proposed_plan is not None:
                    return deps.proposed_plan
                raise TimeoutError(
                    f'Navigation plan extraction timed out after {self._agent_run_timeout_sec:.1f} seconds.'
                ) from exc
            except Exception:
                if deps.proposed_plan is not None:
                    return deps.proposed_plan
                raise

        if run_messages is not None:
            self._log_agent_messages(run_messages)

        output = getattr(result, 'output', None)
        if output is None:
            output = getattr(result, 'data', None)
        if isinstance(output, NavigationPlan):
            plan = output
        elif isinstance(output, dict):
            plan = NavigationPlan.model_validate(output)
        else:
            raise TypeError(f'Agent returned unsupported navigation plan output: {output!r}')

        self.get_logger().debug('Validated NavigationPlan:\n%s' % self._format_navigation_plan(plan))
        return plan

    async def _execute_navigation_plan(self, plan: NavigationPlan, trace: list[Dict[str, Any]]) -> AgentResponse:
        bt_artifacts = self._write_generated_behavior_tree(plan)
        bt_xml = str(bt_artifacts['xml_path'])

        trace.append(
            {
                'step': 'navigation_plan',
                'plan': plan.model_dump(),
                'behavior_tree': bt_artifacts,
            }
        )
        self.get_logger().info('NavigationPlan ready: %s' % self._navigation_plan_summary(plan, bt_artifacts))
        self.get_logger().debug('Generated NavigationPlan:\n%s' % self._format_navigation_plan(plan))

        if plan.action == 'navigate_to_pose':
            if plan.target_pose is None:
                raise ValueError('Navigation plan did not include target_pose for navigate_to_pose.')
            goal = self._bridge.describe_navigate_to_pose_goal(target=plan.target_pose, bt_xml=bt_xml)
            trace.append({'step': 'build_nav2_goal', 'action': plan.action, 'goal': goal})
            self._log_generated_nav2_goal('NavigateToPose', goal)
            result = await self._bridge.send_navigate_to_pose(target=plan.target_pose, bt_xml=bt_xml)
            trace.append({'step': 'send_nav2_goal', 'action': plan.action, 'result': result})
            self.get_logger().debug('NavigateToPose bridge result: %s' % result)
            action_name = 'NavigateToPose'
        else:
            goal = self._bridge.describe_navigate_through_poses_goal(targets=plan.target_poses, bt_xml=bt_xml)
            trace.append({'step': 'build_nav2_goal', 'action': plan.action, 'goal': goal})
            self._log_generated_nav2_goal('NavigateThroughPoses', goal)
            result = await self._bridge.send_navigate_through_poses(targets=plan.target_poses, bt_xml=bt_xml)
            trace.append({'step': 'send_nav2_goal', 'action': plan.action, 'result': result})
            self.get_logger().debug('NavigateThroughPoses bridge result: %s' % result)
            action_name = 'NavigateThroughPoses'

        planning_tools = [entry['tool'] for entry in trace if entry.get('step') == 'planning_tool']
        actions_executed = ['extract_navigation_plan', *planning_tools, 'build_nav2_goal', plan.action]
        message = f'{action_name} goal accepted by navigation bridge.' if result else f'{action_name} goal failed.'
        return AgentResponse(
            success=result,
            message=message,
            actions_executed=actions_executed,
            trace=trace,
        )

    def _write_generated_behavior_tree(self, plan: NavigationPlan) -> Dict[str, Any]:
        self._generated_bt_dir.mkdir(parents=True, exist_ok=True)
        source_filename = Path(plan.behavior_tree.filename)
        bt_path = self._generated_bt_dir / f'{source_filename.stem}_{time.time_ns()}{source_filename.suffix}'
        mermaid_path = bt_path.with_suffix('.mmd')
        node_summary = behavior_tree_node_summary(plan.behavior_tree.xml)
        mermaid = behavior_tree_mermaid(plan.behavior_tree.xml)
        bt_path.write_text(plan.behavior_tree.xml + '\n', encoding='utf-8')
        mermaid_path.write_text(mermaid + '\n', encoding='utf-8')
        self.get_logger().debug(
            'Generated Behavior Tree XML written to %s. Mermaid preview: %s. Nodes: %s. Reasoning: %s'
            % (bt_path, mermaid_path, node_summary, plan.behavior_tree.reasoning)
        )
        return {
            'xml_path': str(bt_path),
            'mermaid_path': str(mermaid_path),
            'node_summary': node_summary,
            'mermaid': mermaid,
        }

    def _log_generated_nav2_goal(self, action_name: str, goal: Dict[str, Any]) -> None:
        formatted_goal = json.dumps(goal, ensure_ascii=True, indent=2)
        self.get_logger().debug('Generated %s goal message for Nav2:\n%s' % (action_name, formatted_goal))

    def _format_navigation_plan(self, plan: NavigationPlan) -> str:
        return json.dumps(plan.model_dump(), ensure_ascii=True, indent=2)

    def _navigation_plan_summary(self, plan: NavigationPlan, bt_artifacts: Dict[str, Any]) -> str:
        if plan.action == 'navigate_to_pose' and plan.target_pose is not None:
            target = self._target_pose_summary(plan.target_pose)
        else:
            targets = [self._target_pose_summary(target_pose) for target_pose in plan.target_poses]
            target = '[' + '; '.join(targets) + ']'

        return (
            '\n'
            '  Action: %s\n'
            '  Target: %s\n'
            '  Behavior tree: %s\n'
            '  Nodes: %s\n'
            '  XML: %s\n'
            '  Mermaid: %s\n'
            '  BT XML:\n%s'
            % (
                plan.action,
                target,
                plan.behavior_tree.filename,
                ' > '.join(bt_artifacts['node_summary']),
                bt_artifacts['xml_path'],
                bt_artifacts['mermaid_path'],
                indent_text(plan.behavior_tree.xml, spaces=4),
            )
        )

    def _target_pose_summary(self, target_pose: Any) -> str:
        return (
            '%s(x=%.3f, y=%.3f, theta=%.3f)'
            % (target_pose.frame_id, target_pose.x, target_pose.y, target_pose.theta)
        )

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
            trace=response.trace,
            command=command,
            source=source,
        )

    def _publish_status(self, state: str, message: str, **extra: Any) -> None:
        payload = {'state': state, 'message': message, **extra}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self._status_pub.publish(msg)

    def _debug_logging_enabled(self) -> bool:
        logger = self.get_logger()
        try:
            from rclpy.logging import LoggingSeverity

            is_enabled_for = getattr(logger, 'is_enabled_for', None)
            if callable(is_enabled_for):
                return bool(is_enabled_for(LoggingSeverity.DEBUG))

            get_effective_level = getattr(logger, 'get_effective_level', None)
            if callable(get_effective_level):
                return int(get_effective_level()) <= int(LoggingSeverity.DEBUG)
        except Exception:
            return False
        return False

    async def _debug_raw_chat_completion(self, command: str) -> None:
        try:
            raw_response = await asyncio.to_thread(self._request_raw_chat_completion, command)
        except Exception as exc:  # pragma: no cover - diagnostic path depends on external server
            self.get_logger().warning('Agent raw vLLM debug request failed: %s' % exc)
            return

        self.get_logger().debug('Agent raw vLLM chat/completions response: %s' % self._truncate_debug_text(raw_response))
        self._log_raw_navigation_plan_diagnostic(raw_response)

    def _log_raw_navigation_plan_diagnostic(self, raw_response: str) -> None:
        try:
            content = self._raw_chat_message_content(raw_response)
            plan_payload = self._extract_json_object(content)
            plan = NavigationPlan.model_validate(plan_payload)
        except Exception as exc:
            self.get_logger().warning('Raw vLLM diagnostic did not produce a valid NavigationPlan: %s' % exc)
            return

        self.get_logger().debug('Raw vLLM diagnostic NavigationPlan validated: %s' % plan.model_dump_json())

    def _request_raw_chat_completion(self, command: str) -> str:
        url = self._vllm_api_base.rstrip('/') + '/chat/completions'
        payload = {
            'model': self._vllm_model_name,
            'messages': [
                {'role': 'system', 'content': self._raw_navigation_plan_prompt()},
                {'role': 'user', 'content': command},
            ],
            'response_format': {'type': 'json_object'},
            'max_tokens': 768,
            'temperature': 0.0,
        }
        request = urllib.request.Request(
            url,
            data=json.dumps(payload).encode('utf-8'),
            headers={
                'Authorization': f'Bearer {self._vllm_api_key}',
                'Content-Type': 'application/json',
            },
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=60.0) as response:
                return response.read().decode('utf-8', errors='replace')
        except urllib.error.HTTPError as exc:
            body = exc.read().decode('utf-8', errors='replace')
            return f'HTTP {exc.code}: {body}'

    def _raw_navigation_plan_prompt(self) -> str:
        prompt = self._system_prompt.strip() or DEFAULT_SYSTEM_PROMPT
        return f'''{prompt}

Behavior Tree authoring reference:
{self._catalog_summary()}

Return exactly one compact JSON object and no extra text.
This raw diagnostic request does not expose planning tools; return the JSON directly.
The JSON object must use this shape:
{{
    "action": "navigate_to_pose" | "navigate_through_poses",
  "target_pose": {{"frame_id": "map" | "base_link", "x": number, "y": number, "theta": number}} | null,
  "target_poses": [{{"frame_id": "map" | "base_link", "x": number, "y": number, "theta": number}}],
    "behavior_tree": {{"filename": string ending in .xml, "reasoning": string, "xml": complete Nav2 Behavior Tree XML string}},
  "message": string
}}
For navigate_to_pose, target_pose is required and target_poses must be [].
For navigate_through_poses, target_pose must be null and target_poses must contain the ordered poses.
The behavior_tree.xml value must be a Nav2 Behavior Tree composed only from the available node catalog.
For navigate_to_pose, include ComputePathToPose and FollowPath. For navigate_through_poses, include ComputePathThroughPoses and FollowPath.
Use recovery, retry, replanning, wait, spin, or backup nodes only when they are useful for the command.
For relative base_link movement with multiple poses, output accumulated waypoints relative to the initial base_link frame.
Right turns use negative theta. Left turns use positive theta.
If the command combines translation and rotation, use navigate_through_poses with one pose for the translation and a following pose for the rotation.
If the command lacks metric pose information, do not invent coordinates.'''

    def _catalog_summary(self) -> str:
        return format_catalog_reference(self._bt_catalog)

    def _raw_chat_message_content(self, raw_response: str) -> str:
        try:
            payload = json.loads(raw_response)
        except json.JSONDecodeError as exc:
            raise ValueError(f'vLLM returned invalid JSON response envelope: {raw_response[:500]}') from exc

        choices = payload.get('choices') or []
        if not choices:
            raise ValueError(f'vLLM returned no choices: {raw_response[:500]}')
        message = choices[0].get('message') or {}
        content = message.get('content')
        if not isinstance(content, str) or not content.strip():
            raise ValueError(f'vLLM returned no textual NavigationPlan content: {raw_response[:500]}')
        return content

    def _extract_json_object(self, text: str) -> dict[str, Any]:
        start = text.find('{')
        if start < 0:
            raise ValueError(f'NavigationPlan output did not contain a JSON object: {text[:500]}')

        decoder = json.JSONDecoder()
        candidate = text[start:].strip()
        try:
            payload, _ = decoder.raw_decode(candidate)
        except json.JSONDecodeError:
            payload = self._parse_with_balanced_closing_braces(candidate)

        if not isinstance(payload, dict):
            raise ValueError(f'NavigationPlan JSON must be an object, got {type(payload).__name__}.')
        return payload

    def _parse_with_balanced_closing_braces(self, candidate: str) -> dict[str, Any]:
        sanitized = candidate.rstrip()
        open_braces = sanitized.count('{') - sanitized.count('}')
        if open_braces <= 0:
            raise ValueError(f'NavigationPlan output was not valid JSON: {candidate[:500]}')

        repaired = sanitized + ('}' * open_braces)
        try:
            payload, _ = json.JSONDecoder().raw_decode(repaired)
        except json.JSONDecodeError as exc:
            raise ValueError(f'NavigationPlan output was not valid JSON: {candidate[:500]}') from exc

        self.get_logger().debug('Repaired NavigationPlan JSON by appending %d closing brace(s).' % open_braces)
        return payload

    def _debug_navigation_plan_response_format(self) -> dict[str, Any]:
        pose_schema = {
            'type': 'object',
            'properties': {
                'frame_id': {'type': 'string'},
                'x': {'type': 'number'},
                'y': {'type': 'number'},
                'theta': {'type': 'number'},
            },
            'required': ['frame_id', 'x', 'y', 'theta'],
        }
        return {
            'type': 'json_schema',
            'json_schema': {
                'name': 'NavigationPlan',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'action': {'type': 'string', 'enum': ['navigate_to_pose', 'navigate_through_poses']},
                        'target_pose': {'anyOf': [pose_schema, {'type': 'null'}]},
                        'target_poses': {'type': 'array', 'items': pose_schema},
                        'behavior_tree': {
                            'type': 'object',
                            'properties': {
                                'filename': {'type': 'string'},
                                'reasoning': {'type': 'string'},
                                'xml': {'type': 'string'},
                            },
                            'required': ['filename', 'reasoning', 'xml'],
                        },
                        'message': {'type': 'string'},
                    },
                    'required': ['action', 'target_pose', 'target_poses', 'behavior_tree', 'message'],
                },
            },
        }

    def _log_agent_messages(self, run_messages: Any) -> None:
        messages = list(run_messages or [])
        request_count = sum(1 for message in messages if message.__class__.__name__ == 'ModelRequest')
        response_count = sum(1 for message in messages if message.__class__.__name__ == 'ModelResponse')
        retry_parts = []
        tool_calls = []
        model_requests = []
        model_responses = []

        for message_index, message in enumerate(messages):
            if message.__class__.__name__ == 'ModelRequest':
                model_requests.append((message_index, message))
            if message.__class__.__name__ == 'ModelResponse':
                model_responses.append((message_index, message))
            for part in getattr(message, 'parts', []) or []:
                part_name = part.__class__.__name__
                part_kind = str(getattr(part, 'part_kind', ''))
                if 'retry' in part_name.lower() or 'retry' in part_kind.lower():
                    retry_parts.append((message_index, part))
                if part_name == 'ToolCallPart' or 'tool-call' in part_kind:
                    tool_calls.append((message_index, part))

        self.get_logger().debug(
            'Agent debug messages: total=%d model_requests=%d model_responses=%d retries=%d tool_calls=%d'
            % (len(messages), request_count, response_count, len(retry_parts), len(tool_calls))
        )

        for message_index, message in model_requests:
            self.get_logger().debug(
                'Agent raw model request at message[%d]: %s'
                % (message_index, self._format_agent_message(message))
            )

        for message_index, message in model_responses:
            parts = getattr(message, 'parts', []) or []
            finish_reason = str(getattr(message, 'finish_reason', ''))
            provider_details = getattr(message, 'provider_details', {}) or {}
            provider_finish_reason = str(provider_details.get('finish_reason', ''))
            if not parts and 'tool' in f'{finish_reason} {provider_finish_reason}'.lower():
                self.get_logger().warning(
                    'Agent model response requested a tool call, but pydantic-ai parsed zero response parts. '
                    'This usually indicates a model/server tool-call parser mismatch.'
                )
            self.get_logger().debug(
                'Agent raw model response at message[%d]: %s'
                % (message_index, self._format_agent_message(message))
            )

        for message_index, part in retry_parts:
            self.get_logger().warning(
                'Agent retry requested at message[%d]: %s'
                % (message_index, self._format_agent_message_part(part))
            )

        for message_index, part in tool_calls:
            self.get_logger().debug(
                'Agent tool call at message[%d]: %s'
                % (message_index, self._format_agent_message_part(part))
            )

    def _format_agent_message(self, message: Any) -> str:
        if hasattr(message, 'model_dump'):
            payload = message.model_dump(mode='json')
        else:
            payload = getattr(message, '__dict__', repr(message))
        text = json.dumps(payload, ensure_ascii=True, default=str)
        return self._truncate_debug_text(text)

    def _format_agent_message_part(self, part: Any) -> str:
        if hasattr(part, 'model_dump'):
            payload = part.model_dump(mode='json')
        else:
            payload = getattr(part, '__dict__', repr(part))
        text = json.dumps(payload, ensure_ascii=True, default=str)
        return self._truncate_debug_text(text, limit=2000)

    def _truncate_debug_text(self, text: str, limit: int = 12000) -> str:
        if len(text) > limit:
            return text[:limit] + '...<truncated>'
        return text

    def _resolve_bt_catalog_path(self, configured_path: Any) -> Path:
        if configured_path:
            path = Path(str(configured_path)).expanduser()
            if path.is_file():
                return path
            self.get_logger().warning('Configured bt_catalog_path does not exist: %s' % path)

        share_dir = Path(get_package_share_directory('nav2_agent'))
        return share_dir / 'config' / 'bt_catalog.yaml'

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
