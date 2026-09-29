"""Structured data models used by the navigation agent."""

import re
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, Field, field_validator, model_validator


class TargetPose(BaseModel):
    """Navigation target expressed as a planar pose in a ROS frame."""

    frame_id: str = Field(
        default='map',
        description='Reference frame of the coordinates.',
    )
    x: float = Field(description='X coordinate in meters.')
    y: float = Field(description='Y coordinate in meters.')
    theta: float = Field(
        default=0.0,
        description='Yaw orientation in radians.',
    )


class Location(BaseModel):
    """Named pose in the global frame."""

    name: str = Field(description='Location name.')
    x: float = Field(description='X coordinate in meters.')
    y: float = Field(description='Y coordinate in meters.')
    theta: float = Field(default=0.0, description='Yaw orientation in radians.')
    description: str = Field(default='', description='Short description of the location.')


class BehaviorTreeSpec(BaseModel):
    """Behavior Tree XML generated for a navigation command."""

    filename: str = Field(
        default='generated_nav2_bt.xml',
        description='Safe XML filename to write before sending the Nav2 goal.',
    )
    reasoning: str = Field(
        description='Agent justification for the generated Behavior Tree structure.',
    )
    xml: str = Field(
        description='Complete Nav2 Behavior Tree XML document to send through the Nav2 action goal behavior_tree field.',
    )

    @field_validator('filename')
    @classmethod
    def validate_filename(cls, filename: str) -> str:
        candidate = filename.strip() or 'generated_nav2_bt.xml'
        if '/' in candidate or '\\' in candidate:
            raise ValueError('Behavior Tree filename must not include path separators.')
        if not candidate.endswith('.xml'):
            raise ValueError('Behavior Tree filename must end with .xml.')
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', candidate):
            raise ValueError('Behavior Tree filename may only contain letters, numbers, underscore, dash, and dot.')
        return candidate

    @field_validator('xml')
    @classmethod
    def validate_xml(cls, xml_text: str) -> str:
        candidate = xml_text.strip()
        if not candidate:
            raise ValueError('Behavior Tree XML must not be empty.')

        try:
            root = ET.fromstring(candidate)
        except ET.ParseError as exc:
            raise ValueError(f'Behavior Tree XML is not well-formed: {exc}') from exc

        if root.tag != 'root':
            raise ValueError('Behavior Tree XML root element must be <root>.')
        if 'main_tree_to_execute' not in root.attrib:
            raise ValueError('Behavior Tree XML <root> must include main_tree_to_execute.')
        if root.find('BehaviorTree') is None:
            raise ValueError('Behavior Tree XML must include a <BehaviorTree> element.')
        return candidate


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
    behavior_tree: BehaviorTreeSpec = Field(description='Behavior Tree generated for the command.')
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


class NavigationOutcome(BaseModel):
    """Execution outcome of one Nav2 goal, reported back to the agent."""

    succeeded: bool = Field(description='Whether Nav2 reported STATUS_SUCCEEDED.')
    status: str = Field(description='Final goal status, for example SUCCEEDED, ABORTED, CANCELED, REJECTED or DRY_RUN.')
    error_code: int = Field(default=0, description='Nav2 action result error_code, 0 when unavailable.')
    error_msg: str = Field(default='', description='Nav2 action result error_msg, empty when unavailable.')
    number_of_recoveries: Optional[int] = Field(default=None, description='Recoveries executed by the Behavior Tree.')
    distance_remaining: Optional[float] = Field(default=None, description='Last reported distance to goal in meters.')
    navigation_time_sec: Optional[float] = Field(default=None, description='Last reported navigation time in seconds.')
    last_pose: Optional[TargetPose] = Field(default=None, description='Last robot pose reported by Nav2 feedback.')


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
    outcome: Optional[NavigationOutcome] = Field(default=None, description='Nav2 execution outcome.')
    report: str = Field(default='', description='Agent interpretation of the Nav2 execution outcome.')
