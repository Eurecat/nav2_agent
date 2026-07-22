"""PydanticAI agent definition and tool registration for navigation orchestration."""

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from pydantic_ai import Agent, RunContext

try:
    from pydantic_ai.models.openai import OpenAIChatModel as OpenAICompatibleModel
except ImportError:  # pragma: no cover - compatibility with older pydantic-ai releases
    from pydantic_ai.models.openai import OpenAIModel as OpenAICompatibleModel  # type: ignore[attr-defined]

try:
    from pydantic_ai.providers.openai import OpenAIProvider
except ImportError:  # pragma: no cover - compatibility with older pydantic-ai releases
    OpenAIProvider = None  # type: ignore[assignment]

from nav2_agent.models import AgentResponse, BTSelection, TargetPose
from nav2_agent.nav2_bridge import Nav2Bridge

DEFAULT_SYSTEM_PROMPT = """You are a robotic navigation command orchestrator for a ROS 2 robot using Nav2.
You receive natural-language commands and must turn them into safe, explicit navigation actions.

Operational policy:
1. If a command references an object, place, person, pallet, table, room, marker, or other environmental entity instead of explicit metric coordinates, first call tool_extract_coordinates_and_frame to resolve the entity into a TargetPose.
2. Evaluate the command and context. If the command mentions people nearby, narrow aisles, fragile cargo, low confidence perception, unknown obstacles, or careful motion, select a cautious Behavior Tree. If the command emphasizes speed and the environment is clear, select a faster Behavior Tree. Otherwise select the default Behavior Tree.
3. Validate the selected Behavior Tree with tool_select_behavior_tree before using it.
4. Execute exactly one navigation command for the main request: tool_navigate_to_pose for a single destination or tool_navigate_through_poses for a route with multiple waypoints.
5. If visual detection, coordinate extraction, or navigation fails, call tool_recovery with spin or wait when appropriate, then report the outcome.

Always return an AgentResponse. Include the relevant tool actions in actions_executed and keep the message concise and operational."""


@dataclass
class AgentDependencies:
    """Runtime dependencies injected into pydantic-ai tools."""

    bridge: Nav2Bridge
    bt_catalog: Dict[str, Dict[str, Any]]
    logger: logging.Logger


def _build_openai_model(model_name: str, api_base: str, api_key: str = 'EMPTY') -> OpenAICompatibleModel:
    """Create an OpenAI-compatible model client for vLLM-backed pydantic-ai runs."""
    if OpenAIProvider is not None:
        return OpenAICompatibleModel(
            model_name,
            provider=OpenAIProvider(base_url=api_base, api_key=api_key),
        )

    return OpenAICompatibleModel(model_name, base_url=api_base, api_key=api_key)


def _catalog_summary(bt_catalog: Dict[str, Dict[str, Any]]) -> str:
    lines = []
    for bt_id, metadata in sorted(bt_catalog.items()):
        description = metadata.get('description', 'No description provided.')
        use_when = metadata.get('use_when', 'No usage guidance provided.')
        lines.append(f'- {bt_id}: {description} Use when: {use_when}')
    return '\n'.join(lines) if lines else '- No Behavior Trees are available.'


def create_nav2_agent(
    model_name: str,
    api_base: str,
    bt_catalog: Dict[str, Dict[str, Any]],
    system_prompt: Optional[str] = None,
) -> Agent[AgentDependencies, AgentResponse]:
    """Create the pydantic-ai agent and register navigation tools."""
    prompt = system_prompt or DEFAULT_SYSTEM_PROMPT
    prompt = f'{prompt}\n\nAvailable Behavior Trees:\n{_catalog_summary(bt_catalog)}'
    model = _build_openai_model(model_name=model_name, api_base=api_base)
    agent = Agent(
        model,
        deps_type=AgentDependencies,
        output_type=AgentResponse,
        system_prompt=prompt,
    )

    @agent.tool
    async def tool_extract_coordinates_and_frame(
        ctx: RunContext[AgentDependencies],
        entity_name: str,
        reference_frame: str = 'map',
    ) -> TargetPose:
        """Resolve an entity or object name into a TargetPose in the requested frame."""
        pose = await ctx.deps.bridge.extract_frame_coordinates(entity_name, reference_frame)
        ctx.deps.logger.info('Tool extracted coordinates for %s: %s', entity_name, pose.model_dump())
        return pose

    @agent.tool
    async def tool_select_behavior_tree(
        ctx: RunContext[AgentDependencies],
        selection: BTSelection,
    ) -> BTSelection:
        """Validate a requested Behavior Tree selection against the configured catalog."""
        if selection.bt_id not in ctx.deps.bt_catalog:
            available = ', '.join(sorted(ctx.deps.bt_catalog.keys())) or 'none'
            raise ValueError(f'Unknown Behavior Tree {selection.bt_id!r}. Available Behavior Trees: {available}')

        ctx.deps.logger.info(
            'Tool selected Behavior Tree: bt_id=%s reasoning=%s',
            selection.bt_id,
            selection.reasoning,
        )
        return selection

    @agent.tool
    async def tool_navigate_to_pose(
        ctx: RunContext[AgentDependencies],
        target: TargetPose,
        bt_xml: Optional[str] = None,
    ) -> bool:
        """Send a single-pose navigation command through the Nav2 bridge."""
        if bt_xml is not None and bt_xml not in ctx.deps.bt_catalog:
            available = ', '.join(sorted(ctx.deps.bt_catalog.keys())) or 'none'
            raise ValueError(f'Unknown Behavior Tree {bt_xml!r}. Available Behavior Trees: {available}')

        return await ctx.deps.bridge.send_navigate_to_pose(target=target, bt_xml=bt_xml)

    @agent.tool
    async def tool_navigate_through_poses(
        ctx: RunContext[AgentDependencies],
        targets: List[TargetPose],
        bt_xml: Optional[str] = None,
    ) -> bool:
        """Send a multi-waypoint navigation command through the Nav2 bridge."""
        if not targets:
            raise ValueError('At least one target pose is required.')
        if bt_xml is not None and bt_xml not in ctx.deps.bt_catalog:
            available = ', '.join(sorted(ctx.deps.bt_catalog.keys())) or 'none'
            raise ValueError(f'Unknown Behavior Tree {bt_xml!r}. Available Behavior Trees: {available}')

        return await ctx.deps.bridge.send_navigate_through_poses(targets=targets, bt_xml=bt_xml)

    @agent.tool
    async def tool_recovery(ctx: RunContext[AgentDependencies], recovery_type: str) -> bool:
        """Invoke a mock recovery behavior when perception or navigation cannot proceed."""
        return await ctx.deps.bridge.execute_recovery(recovery_type=recovery_type)

    return agent
