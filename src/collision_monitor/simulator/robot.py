"""Deterministic one-node-per-tick simulated robot behaviour."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from collision_monitor.models import Action, Pose


@dataclass(frozen=True, slots=True)
class RobotSimulationSummary:
    """End-of-run counters and goal status for one robot."""

    robot_id: str
    pauses: int
    resumes: int
    nodes_advanced: int
    goal_reached: bool
    goal_tick: int | None


@dataclass(slots=True)
class SimulatedRobot:
    """A robot that advances at most one remaining path node per tick."""

    robot_id: str
    pose: Pose
    future_path: tuple[Pose, ...]
    loaded: bool
    battery_level: float
    deadline_ms: int
    latest_action: Action = Action.PAUSE
    action_received: bool = False
    pauses: int = 0
    resumes: int = 0
    nodes_advanced: int = 0
    goal_tick: int | None = None
    _remaining_path: list[Pose] = field(init=False, repr=False)
    _terminal_state_published: bool = field(default=False, init=False, repr=False)

    def __post_init__(self) -> None:
        if not self.robot_id:
            raise ValueError("simulated robot ID must not be empty")
        if not 0 <= self.battery_level <= 100:
            raise ValueError("simulated battery level must be in [0, 100]")
        if self.deadline_ms < 0:
            raise ValueError("simulated deadline must not be negative")
        self._remaining_path = [self.pose, *self.future_path]

    @property
    def at_goal(self) -> bool:
        """Return whether no future path node remains."""
        return len(self._remaining_path) <= 1

    @property
    def remaining_path(self) -> tuple[Pose, ...]:
        """Return the current pose followed by every future node."""
        return tuple(self._remaining_path)

    @property
    def should_publish_state(self) -> bool:
        """Publish continuously while moving and once after reaching the goal."""
        return not self.at_goal or not self._terminal_state_published

    def advance_for_tick(self, tick: int) -> None:
        """Apply the latest action, advancing exactly one node on Resume."""
        if tick < 1:
            raise ValueError("simulation tick must be positive")
        if self.at_goal or self.latest_action is Action.PAUSE:
            return
        self._remaining_path.pop(0)
        self.pose = self._remaining_path[0]
        self.nodes_advanced += 1
        if self.at_goal and self.goal_tick is None:
            self.goal_tick = tick

    def receive_action(self, action: Action) -> None:
        """Remember the latest action; Pause is the implicit initial default."""
        self.latest_action = action
        self.action_received = True
        if action is Action.RESUME:
            self.resumes += 1
        else:
            self.pauses += 1

    def state_payload(self, *, timestamp_ms: int) -> dict[str, Any]:
        """Build assignment JSON with current pose first in a non-empty path."""
        if timestamp_ms < 0:
            raise ValueError("state timestamp must not be negative")
        path = (
            []
            if self.at_goal
            else [{"x": pose.x, "y": pose.y, "theta": pose.theta} for pose in self._remaining_path]
        )
        return {
            "device_id": self.robot_id,
            "timestamp": timestamp_ms,
            "x": self.pose.x,
            "y": self.pose.y,
            "theta": self.pose.theta,
            "battery_level": self.battery_level,
            "loaded": self.loaded,
            "deadline": self.deadline_ms,
            "path": path,
        }

    def mark_state_published(self) -> None:
        """Record the one terminal publication after goal completion."""
        if self.at_goal:
            self._terminal_state_published = True

    def summary(self) -> RobotSimulationSummary:
        """Return immutable counters for the end-of-run report."""
        return RobotSimulationSummary(
            robot_id=self.robot_id,
            pauses=self.pauses,
            resumes=self.resumes,
            nodes_advanced=self.nodes_advanced,
            goal_reached=self.at_goal,
            goal_tick=self.goal_tick,
        )
