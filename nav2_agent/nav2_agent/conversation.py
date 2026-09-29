"""Recent commands, used as context for follow-up commands."""

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, List, Optional

from nav2_agent.models import TargetPose


@dataclass
class Turn:
    command: str
    summary: str
    start_pose: Optional[TargetPose]
    end_pose: Optional[TargetPose]


def format_pose(pose: Optional[TargetPose]) -> str:
    if pose is None:
        return 'unknown'
    return f'{pose.frame_id} x={pose.x:.2f} y={pose.y:.2f} yaw={math.degrees(pose.theta):.0f} deg'


class Conversation:
    """Bounded history of commands that expires after a period without commands."""

    def __init__(self, max_turns: int, timeout_sec: float, clock: Callable[[], float] = time.monotonic) -> None:
        self._turns: Deque[Turn] = deque(maxlen=max(1, max_turns))
        self._timeout_sec = timeout_sec
        self._clock = clock
        self._last_update = clock()

    def reset(self) -> None:
        self._turns.clear()

    def expire_if_idle(self) -> None:
        if self._timeout_sec > 0 and self._clock() - self._last_update > self._timeout_sec:
            self.reset()

    def add(self, turn: Turn) -> None:
        self._turns.append(turn)
        self._last_update = self._clock()

    def turns(self) -> List[Turn]:
        return list(self._turns)

    def previous_start_pose(self, commands_ago: int) -> Optional[TargetPose]:
        """Robot pose at the start of a previous command; 1 is the most recent one."""
        if commands_ago < 1 or commands_ago > len(self._turns):
            return None
        return self._turns[-commands_ago].start_pose

    def format_context(self) -> str:
        if not self._turns:
            return 'Previous commands: none.'
        lines = ['Previous commands, most recent last (commands_ago in brackets):']
        count = len(self._turns)
        for index, turn in enumerate(self._turns):
            lines.append(
                f'[{count - index}] "{turn.command}" -> {turn.summary}. '
                f'Start: {format_pose(turn.start_pose)}. End: {format_pose(turn.end_pose)}.'
            )
        return '\n'.join(lines)
