"""Structured data models used by the navigation agent."""

from typing import List

from pydantic import BaseModel, Field


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


class FrameLookupRequest(BaseModel):
    """Request to resolve an entity or object into navigation coordinates."""

    target_entity: str = Field(
        description='Entity or object to find or reference, for example pallet or table_1.',
    )
    source_frame: str = Field(
        default='map',
        description='Source frame requested for the transform or coordinate extraction.',
    )


class BTSelection(BaseModel):
    """Behavior Tree selected for a navigation command."""

    bt_id: str = Field(
        description='Identifier or XML filename of the Behavior Tree to use, for example default_nav.xml or cautious_slow.xml.',
    )
    reasoning: str = Field(
        description='Agent justification for selecting this Behavior Tree from the command and environment context.',
    )


class AgentResponse(BaseModel):
    """Final structured response returned by the navigation agent."""

    success: bool = Field(description='Whether the requested navigation command was accepted and executed successfully.')
    message: str = Field(description='Human-readable execution summary.')
    actions_executed: List[str] = Field(
        default_factory=list,
        description='Ordered list of tool calls or navigation actions executed by the agent.',
    )
