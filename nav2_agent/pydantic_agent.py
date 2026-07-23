"""PydanticAI planning agent definition for navigation orchestration."""

import logging
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Literal, Optional

from pydantic_ai import Agent, RunContext

try:
    from pydantic_ai.models.openai import OpenAIChatModel as OpenAICompatibleModel
except ImportError:  # pragma: no cover - compatibility with older pydantic-ai releases
    from pydantic_ai.models.openai import OpenAIModel as OpenAICompatibleModel  # type: ignore[attr-defined]

try:
    from pydantic_ai.providers.openai import OpenAIProvider
except ImportError:  # pragma: no cover - compatibility with older pydantic-ai releases
    OpenAIProvider = None  # type: ignore[assignment]

from nav2_agent.models import BTSelection, NavigationPlan, TargetPose

DEFAULT_SYSTEM_PROMPT = """You are a robotic navigation command orchestrator for a ROS 2 robot using Nav2.
You receive natural-language commands and must extract one explicit NavigationPlan.

Operational policy:
1. Interpret metric motion commands that provide coordinates, distances, lateral offsets, or yaw rotations.
2. Select action='navigate_to_pose' for one target pose, or action='navigate_through_poses' for multiple ordered poses.
3. Frame rules: explicit global coordinates use frame_id='map' unless the user names another frame. Relative robot motion such as forward, backward, left, right, lateral movement, or turn/rotate uses frame_id='base_link' unless the user names another frame.
4. ROS planar convention: x is forward, y is left, right is negative y, backward is negative x, and theta is yaw in radians. Right turns use negative theta. Left turns use positive theta. If the user gives degrees, convert degrees to radians.
5. Put the selected single pose in target_pose for navigate_to_pose.
6. Put ordered poses in target_poses for navigate_through_poses.
7. For relative base_link movement with multiple poses, output accumulated waypoints relative to the initial base_link frame, not per-step deltas.
8. If the command combines translation and rotation, use navigate_through_poses with one pose for the translation and a following pose for the rotation accumulated at the translated point.
9. Use tool_select_navigation_action once to choose the action and number of poses needed.
10. Use tool_make_target_pose once for each pose you need in the plan.
11. Use tool_select_behavior_tree once to choose the Behavior Tree.
12. Stop after the planning tools have selected action, poses, and Behavior Tree. Do not execute navigation and do not describe ROS messages yourself.

Always return the requested structured NavigationPlan. If the command does not contain enough metric navigation information, do not invent a target."""


class PlanningComplete(BaseException):
    """Internal signal used to stop the agent after the planning tools complete."""

    def __init__(self, plan: NavigationPlan) -> None:
        self.plan = plan
        super().__init__('Navigation plan completed by planning tools.')


@dataclass
class AgentDependencies:
    """Runtime dependencies injected into the pydantic-ai planning agent."""

    bt_catalog: Dict[str, Dict[str, Any]]
    logger: logging.Logger
    trace: List[Dict[str, Any]]
    proposed_plan: Optional[NavigationPlan] = None
    selected_action: Optional[Literal['navigate_to_pose', 'navigate_through_poses']] = None
    expected_pose_count: int = 0
    planned_poses: List[TargetPose] = field(default_factory=list)
    bt_selection: Optional[BTSelection] = None
    debug_log: Optional[Callable[[str], None]] = None


def _complete_plan_if_ready(ctx: RunContext[AgentDependencies]) -> None:
    if ctx.deps.selected_action is None or ctx.deps.bt_selection is None:
        return
    if ctx.deps.expected_pose_count <= 0:
        return
    if len(ctx.deps.planned_poses) < ctx.deps.expected_pose_count:
        return

    selected_poses = ctx.deps.planned_poses[: ctx.deps.expected_pose_count]
    if ctx.deps.selected_action == 'navigate_to_pose':
        plan = NavigationPlan(
            action='navigate_to_pose',
            target_pose=selected_poses[0],
            target_poses=[],
            bt_selection=ctx.deps.bt_selection,
        )
    else:
        plan = NavigationPlan(
            action='navigate_through_poses',
            target_pose=None,
            target_poses=selected_poses,
            bt_selection=ctx.deps.bt_selection,
        )

    ctx.deps.proposed_plan = plan
    ctx.deps.trace.append(
        {
            'step': 'planning_complete',
            'plan': plan.model_dump(),
        }
    )
    if ctx.deps.debug_log is not None:
        ctx.deps.debug_log('Planning completed: %s' % plan.model_dump())
    raise PlanningComplete(plan)


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
    api_key: str = 'EMPTY',
    system_prompt: Optional[str] = None,
) -> Agent[AgentDependencies, NavigationPlan]:
    """Create the pydantic-ai agent that extracts a validated navigation plan."""
    prompt = system_prompt or DEFAULT_SYSTEM_PROMPT
    prompt = f'{prompt}\n\nAvailable Behavior Trees:\n{_catalog_summary(bt_catalog)}'
    model = _build_openai_model(model_name=model_name, api_base=api_base, api_key=api_key)
    agent = Agent(
        model,
        deps_type=AgentDependencies,
        output_type=NavigationPlan,
        system_prompt=prompt,
        retries=2,
    )

    @agent.tool
    async def tool_select_navigation_action(
        ctx: RunContext[AgentDependencies],
        action: Literal['navigate_to_pose', 'navigate_through_poses'],
        expected_pose_count: int,
    ) -> Dict[str, Any]:
        """Select the Nav2 action type and how many poses the plan must contain."""
        if expected_pose_count <= 0:
            raise ValueError('expected_pose_count must be greater than zero.')
        if action == 'navigate_to_pose' and expected_pose_count != 1:
            raise ValueError('navigate_to_pose requires expected_pose_count=1.')

        ctx.deps.selected_action = action
        ctx.deps.expected_pose_count = expected_pose_count
        selection = {'action': action, 'expected_pose_count': expected_pose_count}
        ctx.deps.trace.append(
            {
                'step': 'planning_tool',
                'tool': 'tool_select_navigation_action',
                'selection': selection,
            }
        )
        if ctx.deps.debug_log is not None:
            ctx.deps.debug_log('Planning tool_select_navigation_action returned: %s' % selection)
        ctx.deps.logger.debug('Planning tool_select_navigation_action returned: %s', selection)
        _complete_plan_if_ready(ctx)
        return selection

    @agent.tool
    async def tool_make_target_pose(
        ctx: RunContext[AgentDependencies],
        frame_id: str,
        x: float,
        y: float,
        theta: float = 0.0,
    ) -> TargetPose:
        """Create one validated target pose for the NavigationPlan without executing navigation."""
        pose = TargetPose(frame_id=frame_id, x=x, y=y, theta=theta)
        ctx.deps.planned_poses.append(pose)
        ctx.deps.trace.append(
            {
                'step': 'planning_tool',
                'tool': 'tool_make_target_pose',
                'pose': pose.model_dump(),
            }
        )
        if ctx.deps.debug_log is not None:
            ctx.deps.debug_log('Planning tool_make_target_pose returned: %s' % pose.model_dump())
        ctx.deps.logger.debug('Planning tool_make_target_pose returned: %s', pose.model_dump())
        _complete_plan_if_ready(ctx)
        return pose

    @agent.tool
    async def tool_select_behavior_tree(
        ctx: RunContext[AgentDependencies],
        bt_id: str,
        reasoning: str,
    ) -> BTSelection:
        """Select and validate one Behavior Tree from the configured catalog without executing navigation."""
        if bt_id not in ctx.deps.bt_catalog:
            available = ', '.join(sorted(ctx.deps.bt_catalog.keys())) or 'none'
            raise ValueError(f'Unknown Behavior Tree {bt_id!r}. Available Behavior Trees: {available}')

        selection = BTSelection(bt_id=bt_id, reasoning=reasoning)
        ctx.deps.bt_selection = selection
        ctx.deps.trace.append(
            {
                'step': 'planning_tool',
                'tool': 'tool_select_behavior_tree',
                'selection': selection.model_dump(),
                'bt_metadata': ctx.deps.bt_catalog[bt_id],
            }
        )
        if ctx.deps.debug_log is not None:
            ctx.deps.debug_log('Planning tool_select_behavior_tree returned: %s' % selection.model_dump())
        ctx.deps.logger.debug('Planning tool_select_behavior_tree returned: %s', selection.model_dump())
        _complete_plan_if_ready(ctx)
        return selection

    return agent
