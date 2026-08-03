"""In-memory latest-state storage with monotonic receive-time tracking."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from collision_monitor.models import RobotSnapshot, RobotState


@dataclass(frozen=True, slots=True)
class StoredRobotState:
    """One latest validated state and its local monotonic receive time."""

    state: RobotState
    received_monotonic_seconds: float

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.received_monotonic_seconds)
            or self.received_monotonic_seconds < 0
        ):
            raise ValueError("received monotonic time must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class StateUpdateResult:
    """Report whether a received state became the stored latest state."""

    accepted: bool
    robot_id: str
    reason: str


@dataclass(frozen=True, slots=True)
class FleetStateSnapshot:
    """An immutable tick-local view of every known robot state."""

    snapshots: tuple[RobotSnapshot, ...]
    source_state_ages_ms: Mapping[str, int]
    captured_epoch_ms: int
    captured_monotonic_seconds: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "source_state_ages_ms",
            MappingProxyType(dict(self.source_state_ages_ms)),
        )
        if self.captured_epoch_ms < 0:
            raise ValueError("captured epoch time must not be negative")
        if (
            not math.isfinite(self.captured_monotonic_seconds)
            or self.captured_monotonic_seconds < 0
        ):
            raise ValueError("captured monotonic time must be finite and non-negative")
        snapshot_ids = tuple(snapshot.state.device_id for snapshot in self.snapshots)
        if snapshot_ids != tuple(sorted(snapshot_ids)):
            raise ValueError("fleet snapshots must use deterministic robot-ID order")
        if set(snapshot_ids) != set(self.source_state_ages_ms):
            raise ValueError("every fleet snapshot must have one source-state age")


class LatestStateStore:
    """Keep at most one latest valid state for each device ID."""

    def __init__(self) -> None:
        self._states: dict[str, StoredRobotState] = {}

    def __len__(self) -> int:
        return len(self._states)

    @property
    def robot_ids(self) -> tuple[str, ...]:
        """Return all known robot IDs in stable order."""
        return tuple(sorted(self._states))

    def update(
        self,
        state: RobotState,
        *,
        received_monotonic_seconds: float,
    ) -> StateUpdateResult:
        """Apply the documented per-robot source-timestamp ordering policy.

        A state older than the stored source timestamp is rejected and cannot
        refresh receive time. Equal timestamps are accepted in receive order,
        so a corrected retransmission deterministically replaces the previous
        value and refreshes receive time. A newer timestamp always replaces the
        stored value. This policy is independent for each robot.
        """
        candidate = StoredRobotState(
            state=state,
            received_monotonic_seconds=received_monotonic_seconds,
        )
        existing = self._states.get(state.device_id)
        if existing is not None and state.timestamp < existing.state.timestamp:
            return StateUpdateResult(
                accepted=False,
                robot_id=state.device_id,
                reason="older_source_timestamp",
            )
        self._states[state.device_id] = candidate
        return StateUpdateResult(
            accepted=True,
            robot_id=state.device_id,
            reason=("replaced_latest_state" if existing is not None else "new_robot_state"),
        )

    def latest(self, robot_id: str) -> StoredRobotState:
        """Return one stored state or raise KeyError for an unknown robot."""
        return self._states[robot_id]

    def snapshot(
        self,
        *,
        captured_monotonic_seconds: float,
        captured_epoch_ms: int,
        stale_timeout_seconds: float,
        pose_tolerance: float,
    ) -> FleetStateSnapshot:
        """Build immutable engine snapshots from one coherent store view."""
        if not math.isfinite(captured_monotonic_seconds) or captured_monotonic_seconds < 0:
            raise ValueError("captured monotonic time must be finite and non-negative")
        if captured_epoch_ms < 0:
            raise ValueError("captured epoch time must not be negative")
        if stale_timeout_seconds <= 0 or not math.isfinite(stale_timeout_seconds):
            raise ValueError("stale timeout must be finite and greater than zero")
        if pose_tolerance <= 0 or not math.isfinite(pose_tolerance):
            raise ValueError("pose tolerance must be finite and greater than zero")

        snapshots: list[RobotSnapshot] = []
        ages_ms: dict[str, int] = {}
        for robot_id in sorted(self._states):
            stored = self._states[robot_id]
            age_seconds = max(
                captured_monotonic_seconds - stored.received_monotonic_seconds,
                0.0,
            )
            age_ms = int(age_seconds * 1_000)
            ages_ms[robot_id] = age_ms
            tick_local_received_epoch_ms = max(captured_epoch_ms - age_ms, 0)
            snapshots.append(
                RobotSnapshot.from_state(
                    stored.state,
                    received_at_ms=tick_local_received_epoch_ms,
                    stale=age_seconds >= stale_timeout_seconds,
                    tolerance=pose_tolerance,
                )
            )
        return FleetStateSnapshot(
            snapshots=tuple(snapshots),
            source_state_ages_ms=ages_ms,
            captured_epoch_ms=captured_epoch_ms,
            captured_monotonic_seconds=captured_monotonic_seconds,
        )
