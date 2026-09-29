"""PydanticAI planning agent definition for navigation orchestration."""

import logging
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Literal, Optional

from pydantic_ai import Agent, ModelRetry, RunContext

try:
    from pydantic_ai.models.openai import OpenAIChatModel as OpenAICompatibleModel
except ImportError:  # pragma: no cover
    from pydantic_ai.models.openai import OpenAIModel as OpenAICompatibleModel  # type: ignore[attr-defined]

try:
    from pydantic_ai.providers.openai import OpenAIProvider
except ImportError:  # pragma: no cover
    OpenAIProvider = None  # type: ignore[assignment]

from nav2_agent.bt_catalog import format_catalog_reference, validate_behavior_tree
from nav2_agent.models import BehaviorTreeSpec, NavigationOutcome, NavigationPlan, TargetPose

DEFAULT_SYSTEM_PROMPT = """You are a robotic navigation command orchestrator for a ROS 2 robot using Nav2.
You receive natural-language commands and must extract one explicit NavigationPlan.

Operational policy:
1. Interpret metric motion commands that provide coordinates, distances, lateral offsets, or yaw rotations.
2. Select action='navigate_to_pose' for one target pose, or action='navigate_through_poses' for multiple ordered poses.
3. Frame rules: explicit global coordinates use frame_id='{global_frame}' unless the user names another frame. Relative robot motion such as forward, backward, left, right, lateral movement, or turn/rotate uses frame_id='{robot_base_frame}' unless the user names another frame.
4. ROS planar convention: x is forward, y is left, right is negative y, backward is negative x, and theta is yaw in radians. Right turns use negative theta. Left turns use positive theta. If the user gives degrees, convert degrees to radians.
5. Put the selected single pose in target_pose for navigate_to_pose.
6. Put ordered poses in target_poses for navigate_through_poses.
7. For relative {robot_base_frame} movement with multiple poses, output accumulated waypoints relative to the initial {robot_base_frame} frame, not per-step deltas.
8. If the command combines translation and rotation, use navigate_through_poses with one pose for the translation and a following pose for the rotation accumulated at the translated point.
9. Use tool_select_navigation_action once to choose the action and number of poses needed.
10. Use tool_make_relative_translation for relative forward/backward/left/right translations. Pass positive distances; the tool applies ROS axis signs.
11. Use tool_make_relative_turn for relative left/right rotations. Pass positive radians; the tool applies yaw signs.
12. Use tool_make_target_pose only for explicit coordinates or when a command already specifies signed x/y/theta values.
13. Use tool_create_behavior_tree once to design a complete Nav2 Behavior Tree XML document for the selected action. Compose the tree from the available node catalog; do not merely choose a prewritten template.
14. The generated tree must satisfy the selected action contract, required node attributes, child-count rules, and blackboard data flow declared by the catalog.
15. If tool_create_behavior_tree reports validation errors, repair the XML using the catalog and call the tool again.
16. Stop after the planning tools have selected action, poses, and generated Behavior Tree XML. Do not execute navigation and do not describe ROS messages yourself.

Always return the requested structured NavigationPlan. If the command does not contain enough metric navigation information, do not invent a target."""


DEFAULT_BT_AUTHORING_GUIDE = """Nav2 Behavior Tree XML authoring reference:
- Return one complete XML document with <root main_tree_to_execute="MainTree"> and <BehaviorTree ID="MainTree">.
- Use BehaviorTree.CPP/Nav2 XML tags, not ROS launch XML.
- Compose a tree from the available node catalog. The catalog is a toolbox, not a list of complete behaviors.
- Use only node tags present in the catalog. If a useful node is missing from the catalog, do not invent it.
- For NavigateToPose goals, use blackboard keys {goal} and {path}; include ComputePathToPose followed by FollowPath.
- For NavigateThroughPoses goals, use blackboard keys {goals} and {path}; include ComputePathThroughPoses followed by FollowPath.
- You decide how robust the tree must be. Recovery, retry, and replanning structure is your design choice, not something the user must ask for.
- Nodes listed under child_categories restrict which node categories may appear inside each child branch.
- Attributes listed as consumes or produces must reference blackboard keys with braces, for example path="{path}".
- A consumed blackboard key must be available from the selected action or produced by an earlier ticked node.
- Balance simplicity and robustness for the command, and explain the chosen structure in reasoning.
- Do not include comments, markdown fences, YAML, or explanatory text inside the XML string.
"""


class PlanningComplete(BaseException):
    """Internal signal used to stop the agent after the planning tools complete."""

    def __init__(self, plan: NavigationPlan) -> None:
        self.plan = plan
        super().__init__('Navigation plan completed by planning tools.')


@dataclass
class AgentDependencies:
    """Runtime dependencies injected into the pydantic-ai planning agent."""

    bt_catalog: Dict[str, Any]
    command: str
    logger: logging.Logger
    trace: List[Dict[str, Any]]
    proposed_plan: Optional[NavigationPlan] = None
    selected_action: Optional[Literal['navigate_to_pose', 'navigate_through_poses']] = None
    expected_pose_count: int = 0
    planned_poses: List[TargetPose] = field(default_factory=list)
    behavior_tree: Optional[BehaviorTreeSpec] = None
    debug_log: Optional[Callable[[str], None]] = None
    on_step: Optional[Callable[[str], None]] = None


def _notify_step(ctx: RunContext[AgentDependencies], step: str) -> None:
    if ctx.deps.on_step is not None:
        ctx.deps.on_step(step)


def _complete_plan_if_ready(ctx: RunContext[AgentDependencies]) -> None:
    if ctx.deps.selected_action is None or ctx.deps.behavior_tree is None:
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
            behavior_tree=ctx.deps.behavior_tree,
        )
    else:
        plan = NavigationPlan(
            action='navigate_through_poses',
            target_pose=None,
            target_poses=selected_poses,
            behavior_tree=ctx.deps.behavior_tree,
        )

    try:
        validate_behavior_tree(
            plan.behavior_tree.xml,
            ctx.deps.bt_catalog,
            action=plan.action,
        )
    except ValueError as exc:
        ctx.deps.behavior_tree = None
        raise ModelRetry(f'Behavior Tree does not match the selected action: {exc} Create it again.') from exc

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
    """Create an OpenAI-compatible model client."""
    if OpenAIProvider is not None:
        return OpenAICompatibleModel(
            model_name,
            provider=OpenAIProvider(base_url=api_base, api_key=api_key),
        )

    return OpenAICompatibleModel(model_name, base_url=api_base, api_key=api_key)


def _authoring_reference(bt_catalog: Dict[str, Any]) -> str:
    return f'{DEFAULT_BT_AUTHORING_GUIDE}\n\n{format_catalog_reference(bt_catalog)}'


def _relative_origin(planned_poses: List[TargetPose], robot_base_frame: str) -> tuple[float, float]:
    if planned_poses and planned_poses[-1].frame_id == robot_base_frame:
        return planned_poses[-1].x, planned_poses[-1].y
    return 0.0, 0.0


def _record_target_pose(ctx: RunContext[AgentDependencies], tool_name: str, pose: TargetPose) -> None:
    ctx.deps.planned_poses.append(pose)
    ctx.deps.trace.append(
        {
            'step': 'planning_tool',
            'tool': tool_name,
            'pose': pose.model_dump(),
        }
    )
    if ctx.deps.debug_log is not None:
        ctx.deps.debug_log('Planning %s returned: %s' % (tool_name, pose.model_dump()))
    _notify_step(ctx, tool_name)
    ctx.deps.logger.debug('Planning %s returned: %s', tool_name, pose.model_dump())


def create_nav2_agent(
    model_name: str,
    api_base: str,
    bt_catalog: Dict[str, Any],
    api_key: str = 'EMPTY',
    system_prompt: Optional[str] = None,
    global_frame: str = 'map',
    robot_base_frame: str = 'base_link',
) -> Agent[AgentDependencies, NavigationPlan]:
    """Create the pydantic-ai agent that extracts a validated navigation plan."""
    prompt = (system_prompt or DEFAULT_SYSTEM_PROMPT).replace('{global_frame}', global_frame)
    prompt = prompt.replace('{robot_base_frame}', robot_base_frame)
    prompt = f'{prompt}\n\nBehavior Tree authoring reference:\n{_authoring_reference(bt_catalog)}'
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
            raise ModelRetry('expected_pose_count must be greater than zero.')
        if action == 'navigate_to_pose' and expected_pose_count != 1:
            raise ModelRetry('navigate_to_pose requires expected_pose_count=1.')

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
        _notify_step(ctx, 'tool_select_navigation_action')
        ctx.deps.logger.debug('Planning tool_select_navigation_action returned: %s', selection)
        _complete_plan_if_ready(ctx)
        return selection

    @agent.tool
    async def tool_make_relative_translation(
        ctx: RunContext[AgentDependencies],
        direction: Literal['forward', 'backward', 'left', 'right'],
        distance_m: float,
        theta: float = 0.0,
    ) -> TargetPose:
        """Create an accumulated target pose in the robot base frame for a relative cardinal translation."""
        if distance_m < 0.0:
            raise ModelRetry('distance_m must be non-negative; choose direction to express sign.')

        x, y = _relative_origin(ctx.deps.planned_poses, robot_base_frame)
        if direction == 'forward':
            x += distance_m
        elif direction == 'backward':
            x -= distance_m
        elif direction == 'left':
            y += distance_m
        else:
            y -= distance_m

        pose = TargetPose(frame_id=robot_base_frame, x=x, y=y, theta=theta)
        _record_target_pose(ctx, 'tool_make_relative_translation', pose)
        _complete_plan_if_ready(ctx)
        return pose

    @agent.tool
    async def tool_make_relative_turn(
        ctx: RunContext[AgentDependencies],
        direction: Literal['left', 'right'],
        angle_rad: float,
    ) -> TargetPose:
        """Create an accumulated target pose in the robot base frame for a relative yaw rotation."""
        if angle_rad < 0.0:
            raise ModelRetry('angle_rad must be non-negative; choose direction to express sign.')

        x, y = _relative_origin(ctx.deps.planned_poses, robot_base_frame)
        theta = angle_rad if direction == 'left' else -angle_rad
        pose = TargetPose(frame_id=robot_base_frame, x=x, y=y, theta=theta)
        _record_target_pose(ctx, 'tool_make_relative_turn', pose)
        _complete_plan_if_ready(ctx)
        return pose

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
        _record_target_pose(ctx, 'tool_make_target_pose', pose)
        _complete_plan_if_ready(ctx)
        return pose

    @agent.tool
    async def tool_create_behavior_tree(
        ctx: RunContext[AgentDependencies],
        filename: str,
        xml: str,
        reasoning: str,
    ) -> BehaviorTreeSpec:
        """Create and validate one complete Nav2 Behavior Tree XML document without executing navigation."""
        if ctx.deps.selected_action is None:
            raise ModelRetry('Select the navigation action before creating a Behavior Tree.')
        try:
            behavior_tree = BehaviorTreeSpec(filename=filename, xml=xml, reasoning=reasoning)
            validate_behavior_tree(
                behavior_tree.xml,
                ctx.deps.bt_catalog,
                action=ctx.deps.selected_action,
            )
        except ValueError as exc:
            # Send validation errors back to the model so it can repair the XML.
            ctx.deps.trace.append({'step': 'planning_tool_retry', 'tool': 'tool_create_behavior_tree', 'error': str(exc)})
            _notify_step(ctx, 'tool_create_behavior_tree rejected')
            raise ModelRetry(f'Behavior Tree rejected: {exc} Repair the XML and call tool_create_behavior_tree again.') from exc
        ctx.deps.behavior_tree = behavior_tree
        ctx.deps.trace.append(
            {
                'step': 'planning_tool',
                'tool': 'tool_create_behavior_tree',
                'behavior_tree': behavior_tree.model_dump(),
            }
        )
        if ctx.deps.debug_log is not None:
            ctx.deps.debug_log('Planning tool_create_behavior_tree returned: %s' % behavior_tree.model_dump())
        _notify_step(ctx, 'tool_create_behavior_tree')
        ctx.deps.logger.debug('Planning tool_create_behavior_tree returned: %s', behavior_tree.model_dump())
        _complete_plan_if_ready(ctx)
        return behavior_tree

    return agent


DEFAULT_REPORT_PROMPT = """You are the same navigation agent that planned and sent a Nav2 goal for a ROS 2 robot.
Navigation has finished and you now receive the real execution outcome reported by Nav2.
Write a short report for the operator in the language of the original command:
- State whether the robot reached the goal.
- Interpret the Nav2 status, error_code, error_msg, recoveries, and distance remaining yourself.
- Relate the outcome to the Behavior Tree you designed, for example whether its recovery structure was exercised.
- If the command failed, give your best diagnosis and what you would change next time.
Do not invent data that is not present in the outcome. Answer in plain text, at most four sentences."""


def create_report_agent(
    model_name: str,
    api_base: str,
    api_key: str = 'EMPTY',
    system_prompt: Optional[str] = None,
) -> Agent[None, str]:
    """Create the pydantic-ai agent that interprets the Nav2 execution outcome."""
    model = _build_openai_model(model_name=model_name, api_base=api_base, api_key=api_key)
    return Agent(model, output_type=str, system_prompt=system_prompt or DEFAULT_REPORT_PROMPT)


def format_outcome_report_request(command: str, plan: NavigationPlan, outcome: NavigationOutcome) -> str:
    """Build the report agent input from the original command, executed plan, and Nav2 outcome."""
    return (
        f'Original command:\n{command}\n\n'
        f'Executed plan:\n{plan.model_dump_json(indent=2)}\n\n'
        f'Nav2 outcome:\n{outcome.model_dump_json(indent=2)}'
    )
