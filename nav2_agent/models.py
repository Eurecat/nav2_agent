"""Structured data models used by the navigation agent."""

from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, Field, model_validator


class TargetPose(BaseModel):
    """Navigation target expressed as a planar pose in a ROS frame."""

    frame_id: str = Field(
        default='map',
        description='Reference frame for the coordinates, for example map, odom, or base_link.',
    )
    x: float = Field(description='X coordinate in meters.')
    y: float = Field(description='Y coordinate in meters.')
    theta: float = Field(
        default=0.0,
        description='Yaw orientation in radians.',
    )


class BTSelection(BaseModel):
    """Behavior Tree selected for a navigation command."""

    bt_id: str = Field(
        description='Identifier or XML filename of the Behavior Tree to use, for example default_nav.xml or cautious_slow.xml.',
    )
    reasoning: str = Field(
        description='Agent justification for selecting this Behavior Tree from the command and environment context.',
    )


class NavigationPlan(BaseModel):
    """Structured navigation intent extracted from a natural-language command."""

    action: Literal['navigate_to_pose', 'navigate_through_poses'] = Field(
        description='Nav2 action type selected for the command.',
    )
    target_pose: Optional[TargetPose] = Field(
        default=None,
        description='Single target pose when action is navigate_to_pose.',
    )
    target_poses: List[TargetPose] = Field(
        default_factory=list,
        description='Ordered target poses when action is navigate_through_poses.',
    )
    bt_selection: BTSelection = Field(description='Behavior Tree selected for the command.')
    message: str = Field(
        default='',
        description='Concise explanation of the extracted plan.',
    )

    @model_validator(mode='after')
    def validate_action_targets(self) -> 'NavigationPlan':
        if self.action == 'navigate_to_pose':
            if self.target_pose is None:
                raise ValueError('target_pose is required for navigate_to_pose.')
            if self.target_poses:
                raise ValueError('target_poses must be empty for navigate_to_pose.')
        elif self.action == 'navigate_through_poses':
            if self.target_pose is not None:
                raise ValueError('target_pose must be null for navigate_through_poses.')
            if not self.target_poses:
                raise ValueError('target_poses must contain at least one pose for navigate_through_poses.')
        return self


class AgentResponse(BaseModel):
    """Final structured response returned by the navigation agent."""

    success: bool = Field(description='Whether the requested navigation command was accepted and executed successfully.')
    message: str = Field(description='Human-readable execution summary.')
    actions_executed: List[str] = Field(
        default_factory=list,
        description='Ordered list of tool calls or navigation actions executed by the agent.',
    )
    trace: List[Dict[str, Any]] = Field(
        default_factory=list,
        description='Deterministic runtime trace captured by nav2_agent while executing tools.',
    )
