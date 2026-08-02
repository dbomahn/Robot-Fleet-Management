from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Annotated, Any

from pydantic import BaseModel, ConfigDict, Field


@dataclass(frozen=True, slots=True)
class Pose:
    """A position in metres and an orientation in radians."""

    x: float
    y: float
    theta: float


class PathNode(BaseModel):
    """A validated path node received at the input boundary."""

    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, frozen=True)

    x: float
    y: float
    theta: float = Field(description="Orientation in radians")

    def to_pose(self) -> Pose:
        """Convert the boundary model to an internal pose."""
        return Pose(x=self.x, y=self.y, theta=self.theta)


class RobotState(BaseModel):
    """A robot state matching the assignment input JSON."""

    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, frozen=True)

    device_id: str = Field(min_length=1)
    timestamp: Annotated[int, Field(ge=0, description="Milliseconds since epoch")]
    x: float
    y: float
    theta: float = Field(description="Orientation in radians")
    battery_level: Annotated[float, Field(ge=0, le=100)]
    loaded: bool
    deadline: Annotated[int, Field(ge=0, description="Milliseconds since epoch")]
    path: list[PathNode] = Field(default_factory=list)

    def current_pose(self) -> Pose:
        """Return the current pose as an internal value object."""
        return Pose(x=self.x, y=self.y, theta=self.theta)


class Action(StrEnum):
    """An action emitted for a robot."""

    PAUSE = "Pause"
    RESUME = "Resume"


class DecisionSource(StrEnum):
    """The mechanism that produced a robot action."""

    CP_SAT = "cp_sat"
    HEURISTIC = "heuristic"
    REPAIR = "repair"
    FAIL_SAFE = "fail_safe"
    POLICY = "policy"


@dataclass(frozen=True, slots=True)
class RobotSnapshot:
    """A parsed state enriched with monitor-local observations."""

    state: RobotState
    received_at_ms: int
    stale: bool
    next_pose: Pose
    at_goal: bool

    @classmethod
    def from_state(
        cls,
        state: RobotState,
        *,
        received_at_ms: int,
        stale: bool,
        tolerance: float,
    ) -> RobotSnapshot:
        """Build a snapshot and derive its next pose deterministically."""
        next_pose, at_goal = derive_next_pose(state, tolerance=tolerance)
        return cls(
            state=state,
            received_at_ms=received_at_ms,
            stale=stale,
            next_pose=next_pose,
            at_goal=at_goal,
        )


@dataclass(frozen=True, slots=True)
class RobotDecision:
    """A traceable action selected for one robot."""

    robot_id: str
    action: Action
    reason_codes: tuple[str, ...]
    tick_id: str
    decision_source: DecisionSource
    diagnostic_metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class FleetDecision:
    """All robot actions and diagnostic metadata for one decision tick."""

    decisions: tuple[RobotDecision, ...]
    tick_id: str
    tick_metadata: Mapping[str, Any] = field(default_factory=dict)


def poses_equal(left: Pose, right: Pose, *, tolerance: float) -> bool:
    """Return whether two poses match, accounting for wrapped angles."""
    if tolerance < 0:
        raise ValueError("tolerance must not be negative")

    angle_delta = math.atan2(
        math.sin(left.theta - right.theta),
        math.cos(left.theta - right.theta),
    )
    return (
        abs(left.x - right.x) <= tolerance
        and abs(left.y - right.y) <= tolerance
        and abs(angle_delta) <= tolerance
    )


def derive_next_pose(state: RobotState, *, tolerance: float) -> tuple[Pose, bool]:
    """Select the next path pose and report whether the path is exhausted."""
    current_pose = state.current_pose()
    if not state.path:
        return current_pose, True

    first_pose = state.path[0].to_pose()
    if len(state.path) > 1 and poses_equal(current_pose, first_pose, tolerance=tolerance):
        return state.path[1].to_pose(), False
    return first_pose, False


def validate_unique_device_ids(states: Sequence[RobotState]) -> None:
    """Reject duplicate robot identifiers within an explicit input batch."""
    seen: set[str] = set()
    duplicates: set[str] = set()
    for state in states:
        if state.device_id in seen:
            duplicates.add(state.device_id)
        seen.add(state.device_id)

    if duplicates:
        identifiers = ", ".join(sorted(duplicates))
        raise ValueError(f"duplicate device IDs in batch: {identifiers}")


def parse_robot_state_batch(payloads: Sequence[Mapping[str, Any]]) -> tuple[RobotState, ...]:
    """Parse an explicit batch and enforce identifier uniqueness."""
    states = tuple(RobotState.model_validate(payload) for payload in payloads)
    validate_unique_device_ids(states)
    return states
