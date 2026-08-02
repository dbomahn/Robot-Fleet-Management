"""Persistent right-of-way grants, fairness and liveness safeguards.

Safety is always a hard override: a grant becomes a hard Resume assignment
only while it belongs to a set of retained grants that admits a complete,
independently validated action assignment. Business-priority changes cannot
pre-empt such a grant.

Liveness is conditional, not absolute. Progress is guaranteed only when at
least one safe Resume action, or an implication-closed safe Resume group,
exists under the available Pause/Resume controls. Otherwise the manager emits
``UNRESOLVABLE_WITH_PAUSE_RESUME`` and returns explicit fail-safe Pause actions.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from enum import StrEnum
from types import MappingProxyType

from collision_monitor.config import MonitorConfig
from collision_monitor.conflicts import ConflictModel, PairwiseCompatibility
from collision_monitor.heuristic import DeterministicComponentHeuristic
from collision_monitor.models import Action, DecisionSource
from collision_monitor.optimiser import validate_component_decisions
from collision_monitor.priority import PriorityBreakdown


class GrantReleaseReason(StrEnum):
    """Stable reason codes for releasing or revoking a grant."""

    AT_GOAL = "AT_GOAL"
    STALE_OR_DISAPPEARED = "STALE_OR_DISAPPEARED"
    CONFLICT_CLEARED = "CONFLICT_CLEARED"
    MAXIMUM_LEASE_FAULT_GUARD = "MAXIMUM_LEASE_FAULT_GUARD"
    UNSAFE_GEOMETRY = "UNSAFE_GEOMETRY"
    COMPONENT_MERGE_CONFLICT = "COMPONENT_MERGE_CONFLICT"


class AlarmCode(StrEnum):
    """Safety and liveness alarms emitted by the grant manager."""

    UNRESOLVABLE_WITH_PAUSE_RESUME = "UNRESOLVABLE_WITH_PAUSE_RESUME"


@dataclass(frozen=True, slots=True)
class GrantRecord:
    """Persistent state for one right-of-way holder."""

    robot_id: str
    acquisition_tick: int
    last_seen_tick: int
    clearance_counter: int = 0

    def __post_init__(self) -> None:
        if not self.robot_id:
            raise ValueError("grant robot ID must not be empty")
        if self.acquisition_tick < 1:
            raise ValueError("grant acquisition tick must be positive")
        if self.last_seen_tick < self.acquisition_tick:
            raise ValueError("grant last-seen tick precedes acquisition")
        if self.clearance_counter < 0:
            raise ValueError("grant clearance counter must not be negative")


@dataclass(frozen=True, slots=True)
class GrantRelease:
    """A traceable grant release event."""

    robot_id: str
    tick_id: int
    reason: GrantReleaseReason


@dataclass(frozen=True, slots=True)
class GrantPreparation:
    """Safe hard assignments and releases produced before optimisation."""

    tick_id: int
    hard_assignments: Mapping[str, Action]
    released_grants: tuple[GrantRelease, ...]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "hard_assignments",
            MappingProxyType(dict(self.hard_assignments)),
        )


@dataclass(frozen=True, slots=True)
class GrantDecisionResult:
    """Validated post-optimisation actions and updated liveness diagnostics."""

    tick_id: int
    decisions: Mapping[str, Action]
    decision_sources: Mapping[str, DecisionSource]
    alarms: tuple[AlarmCode, ...]
    acquired_grants: tuple[str, ...]
    released_grants: tuple[GrantRelease, ...]
    waiting_ages: Mapping[str, int]

    def __post_init__(self) -> None:
        object.__setattr__(self, "decisions", MappingProxyType(dict(self.decisions)))
        object.__setattr__(
            self,
            "decision_sources",
            MappingProxyType(dict(self.decision_sources)),
        )
        object.__setattr__(
            self,
            "waiting_ages",
            MappingProxyType(dict(self.waiting_ages)),
        )
        if set(self.decisions) != set(self.decision_sources):
            raise ValueError("every decision must have exactly one decision source")


@dataclass(frozen=True, slots=True)
class GrantDecisionPreview:
    """A repaired grant decision that has not changed persistent fairness state."""

    decisions: Mapping[str, Action]
    decision_sources: Mapping[str, DecisionSource]
    alarms: tuple[AlarmCode, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "decisions", MappingProxyType(dict(self.decisions)))
        object.__setattr__(
            self,
            "decision_sources",
            MappingProxyType(dict(self.decision_sources)),
        )
        if set(self.decisions) != set(self.decision_sources):
            raise ValueError("every preview decision must have one decision source")


def _constraints_for_component(
    component_ids: frozenset[str],
    conflict_model: ConflictModel,
) -> tuple[PairwiseCompatibility, ...]:
    """Return pairwise constraints contained in one component."""
    return tuple(
        pair
        for pair in conflict_model.pairwise_constraints
        if pair.robot_i in component_ids and pair.robot_j in component_ids
    )


class GrantManager:
    """Own tick-to-tick grant, waiting-age and previous-action state."""

    def __init__(self, config: MonitorConfig) -> None:
        self._config = config
        self._tick_id = 0
        self._waiting_ages: dict[str, int] = {}
        self._active_grants: dict[str, GrantRecord] = {}
        self._previous_actions: dict[str, Action] = {}
        self._tick_open = False
        self._current_hard_assignments: dict[str, Action] = {}
        self._pending_releases: list[GrantRelease] = []
        self._heuristic = DeterministicComponentHeuristic(config)

    @property
    def tick_id(self) -> int:
        """Return the latest monotonically increasing tick ID."""
        return self._tick_id

    @property
    def waiting_ages(self) -> Mapping[str, int]:
        """Return a read-only snapshot of consecutive paused ticks."""
        return MappingProxyType(dict(self._waiting_ages))

    @property
    def active_grants(self) -> Mapping[str, GrantRecord]:
        """Return a read-only snapshot of active right-of-way grants."""
        return MappingProxyType(dict(self._active_grants))

    @property
    def previous_actions(self) -> Mapping[str, Action]:
        """Return the previous published action for each currently seen robot."""
        return MappingProxyType(dict(self._previous_actions))

    def clone(self) -> GrantManager:
        """Return an independent closed-tick copy for transactional decisions."""
        if self._tick_open:
            raise RuntimeError("cannot clone a grant manager with an open tick")
        cloned = GrantManager(self._config)
        cloned._tick_id = self._tick_id
        cloned._waiting_ages = dict(self._waiting_ages)
        cloned._active_grants = dict(self._active_grants)
        cloned._previous_actions = dict(self._previous_actions)
        return cloned

    def revoke_for_safety(self, robot_ids: Sequence[str]) -> None:
        """Revoke selected grants when an external safety validator overrides them."""
        if not self._tick_open:
            raise RuntimeError("safety revocation requires an open prepared tick")
        for robot_id in sorted(set(robot_ids)):
            self._release(robot_id, GrantReleaseReason.UNSAFE_GEOMETRY)

    def _release(self, robot_id: str, reason: GrantReleaseReason) -> None:
        """Release one grant and append a deterministic event."""
        if self._active_grants.pop(robot_id, None) is not None:
            self._pending_releases.append(
                GrantRelease(robot_id=robot_id, tick_id=self._tick_id, reason=reason)
            )

    def _stale_tick_limit(self) -> int:
        """Convert the stale duration into a conservative whole-tick limit."""
        return max(
            1,
            math.ceil(
                self._config.stale_timeout_seconds
                / self._config.tick_interval_seconds
            ),
        )

    def _reconcile_lifecycle(
        self,
        conflict_model: ConflictModel,
        *,
        now_ms: int,
    ) -> None:
        """Update last-seen and clearance counters, releasing expired grants."""
        snapshots = conflict_model.snapshots
        present_ids = frozenset(snapshots)
        stale_limit_ms = int(self._config.stale_timeout_seconds * 1_000)
        stale_tick_limit = self._stale_tick_limit()

        disappeared = set(self._waiting_ages).difference(present_ids)
        for robot_id in disappeared:
            self._waiting_ages.pop(robot_id, None)
            self._previous_actions.pop(robot_id, None)

        for robot_id in sorted(tuple(self._active_grants)):
            record = self._active_grants.get(robot_id)
            if record is None:
                continue
            snapshot = snapshots.get(robot_id)
            if snapshot is None:
                if self._tick_id - record.last_seen_tick >= stale_tick_limit:
                    self._release(robot_id, GrantReleaseReason.STALE_OR_DISAPPEARED)
                continue

            if (
                snapshot.stale
                or now_ms - snapshot.received_at_ms >= stale_limit_ms
            ):
                self._release(robot_id, GrantReleaseReason.STALE_OR_DISAPPEARED)
                continue
            if snapshot.at_goal:
                self._release(robot_id, GrantReleaseReason.AT_GOAL)
                continue

            age = self._tick_id - record.acquisition_tick
            if age >= self._config.grant_maximum_hold_ticks:
                self._release(robot_id, GrantReleaseReason.MAXIMUM_LEASE_FAULT_GUARD)
                continue

            relevant = bool(conflict_model.adjacency.get(robot_id, frozenset()))
            clearance_counter = 0 if relevant else record.clearance_counter + 1
            updated = replace(
                record,
                last_seen_tick=self._tick_id,
                clearance_counter=clearance_counter,
            )
            self._active_grants[robot_id] = updated
            if (
                not relevant
                and clearance_counter >= self._config.grant_clearance_release_ticks
                and age >= self._config.grant_minimum_hold_ticks
            ):
                self._release(robot_id, GrantReleaseReason.CONFLICT_CLEARED)

    def _safe_relevant_grants(
        self,
        conflict_model: ConflictModel,
    ) -> dict[str, Action]:
        """Retain only safe grants, resolving merged-component conflicts oldest first."""
        hard_assignments: dict[str, Action] = {}
        for component in conflict_model.connected_components:
            candidates = sorted(
                (
                    self._active_grants[robot_id]
                    for robot_id in component
                    if robot_id in self._active_grants
                    and conflict_model.adjacency.get(robot_id, frozenset())
                ),
                key=lambda record: (record.acquisition_tick, record.robot_id),
            )
            retained: list[str] = []
            for record in candidates:
                proposed_group = (*retained, record.robot_id)
                if not self._grant_group_has_safe_completion(
                    component,
                    proposed_group,
                    conflict_model,
                ):
                    self._release(
                        record.robot_id,
                        (
                            GrantReleaseReason.COMPONENT_MERGE_CONFLICT
                            if retained
                            else GrantReleaseReason.UNSAFE_GEOMETRY
                        ),
                    )
                    continue
                retained.append(record.robot_id)
                hard_assignments[record.robot_id] = Action.RESUME
        return hard_assignments

    def _grant_group_has_safe_completion(
        self,
        component: Sequence[str],
        resume_group: Sequence[str],
        conflict_model: ConflictModel,
    ) -> bool:
        """Check that hard Resume grants admit a complete validated assignment.

        This permits implication-closed groups while remaining conservative: a
        grant is never forced unless the bounded deterministic heuristic finds
        and independently validates a complete action assignment.
        """
        resume_ids = frozenset(resume_group)
        unavailable_ids = frozenset(
            robot_id
            for robot_id in component
            if (
                conflict_model.snapshots[robot_id].at_goal
                or conflict_model.snapshots[robot_id].stale
            )
        )
        if resume_ids.intersection(unavailable_ids):
            return False

        neutral_priorities = {
            robot_id: PriorityBreakdown(
                robot_id=robot_id,
                base_progress_value=0,
                loaded_bonus=0,
                deadline_urgency=0,
                low_battery_urgency=0,
                waiting_age_bonus=0,
                active_grant_continuation_bonus=0,
                clearance_bonus=0,
                clearance_conflict_count=0,
                remaining_slack_ms=0,
                secondary_score=0,
                throughput_reward=1,
                final_score=1,
            )
            for robot_id in component
        }
        result = self._heuristic.solve(
            component,
            conflict_model,
            neutral_priorities,
            hard_assignments={
                **{robot_id: Action.PAUSE for robot_id in unavailable_ids},
                **{robot_id: Action.RESUME for robot_id in resume_ids},
            },
        )
        return result.feasible and result.progress_made

    def prepare_tick(
        self,
        conflict_model: ConflictModel,
        *,
        now_ms: int,
    ) -> GrantPreparation:
        """Advance state and return safety-checked grant hard assignments."""
        if self._tick_open:
            raise RuntimeError("the previous grant-manager tick has not been finalised")
        if now_ms < 0:
            raise ValueError("now_ms must not be negative")

        self._tick_id += 1
        self._tick_open = True
        self._pending_releases = []
        self._reconcile_lifecycle(conflict_model, now_ms=now_ms)
        self._current_hard_assignments = self._safe_relevant_grants(conflict_model)
        return GrantPreparation(
            tick_id=self._tick_id,
            hard_assignments=self._current_hard_assignments,
            released_grants=tuple(self._pending_releases),
        )

    def _validate_or_repair_components(
        self,
        conflict_model: ConflictModel,
        decisions: dict[str, Action],
        decision_sources: dict[str, DecisionSource],
        priorities: Mapping[str, PriorityBreakdown],
    ) -> tuple[AlarmCode, ...]:
        """Validate progress components and repair any all-Pause component."""
        alarms: list[AlarmCode] = []
        for component in conflict_model.connected_components:
            component_ids = frozenset(component)
            if len(component) < 2 or not any(
                conflict_model.adjacency.get(robot_id, frozenset())
                for robot_id in component
            ):
                continue

            constraints = _constraints_for_component(component_ids, conflict_model)
            component_hard = {
                robot_id: action
                for robot_id, action in self._current_hard_assignments.items()
                if robot_id in component_ids
            }
            component_decisions = {
                robot_id: decisions[robot_id] for robot_id in component
            }
            if all(action is Action.PAUSE for action in component_decisions.values()):
                repair_hard = {
                    **{
                        robot_id: Action.PAUSE
                        for robot_id in component
                        if (
                            conflict_model.snapshots[robot_id].at_goal
                            or conflict_model.snapshots[robot_id].stale
                        )
                    },
                    **component_hard,
                }
                repair = self._heuristic.solve(
                    component,
                    conflict_model,
                    priorities,
                    hard_assignments=repair_hard,
                )
                if repair.feasible and repair.progress_made:
                    for robot_id, action in repair.decisions.items():
                        decisions[robot_id] = action
                        decision_sources[robot_id] = DecisionSource.REPAIR
                    continue

                for robot_id in component:
                    decisions[robot_id] = Action.PAUSE
                    decision_sources[robot_id] = DecisionSource.FAIL_SAFE
                if AlarmCode.UNRESOLVABLE_WITH_PAUSE_RESUME not in alarms:
                    alarms.append(AlarmCode.UNRESOLVABLE_WITH_PAUSE_RESUME)
                continue

            validate_component_decisions(
                component,
                component_decisions,
                constraints,
                component_hard,
            )
        return tuple(alarms)

    def _acquire_grants(
        self,
        conflict_model: ConflictModel,
        decisions: Mapping[str, Action],
    ) -> tuple[str, ...]:
        """Acquire grants for safe Resume actions participating in conflicts."""
        acquired: list[str] = []
        component_by_robot = {
            robot_id: frozenset(component)
            for component in conflict_model.connected_components
            for robot_id in component
        }
        for robot_id in sorted(decisions):
            if decisions[robot_id] is not Action.RESUME:
                continue
            if robot_id in self._active_grants:
                continue
            snapshot = conflict_model.snapshots[robot_id]
            if snapshot.at_goal or snapshot.stale:
                continue
            if not conflict_model.adjacency.get(robot_id, frozenset()):
                continue
            component_ids = component_by_robot[robot_id]
            current_group = tuple(
                active_robot
                for active_robot in sorted(self._active_grants)
                if active_robot in component_ids
            )
            if not self._grant_group_has_safe_completion(
                tuple(sorted(component_ids)),
                (*current_group, robot_id),
                conflict_model,
            ):
                continue
            self._active_grants[robot_id] = GrantRecord(
                robot_id=robot_id,
                acquisition_tick=self._tick_id,
                last_seen_tick=self._tick_id,
            )
            acquired.append(robot_id)
        return tuple(acquired)

    def _update_fairness(
        self,
        conflict_model: ConflictModel,
        decisions: Mapping[str, Action],
    ) -> None:
        """Update capped waiting ages and previous published actions."""
        current_ids = frozenset(conflict_model.snapshots)
        for robot_id in tuple(self._waiting_ages):
            if robot_id not in current_ids:
                self._waiting_ages.pop(robot_id, None)
                self._previous_actions.pop(robot_id, None)

        for robot_id in sorted(current_ids):
            snapshot = conflict_model.snapshots[robot_id]
            action = decisions[robot_id]
            if action is Action.RESUME or snapshot.at_goal:
                self._waiting_ages[robot_id] = 0
            else:
                self._waiting_ages[robot_id] = min(
                    self._waiting_ages.get(robot_id, 0) + 1,
                    self._config.priority_waiting_age_cap_ticks,
                )
            self._previous_actions[robot_id] = action

    def finalise_tick(
        self,
        conflict_model: ConflictModel,
        proposed_decisions: Mapping[str, Action],
        priorities: Mapping[str, PriorityBreakdown],
        *,
        proposed_sources: Mapping[str, DecisionSource] | None = None,
    ) -> GrantDecisionResult:
        """Validate, repair and commit one tick of published action state."""
        try:
            preview = self.preview_tick(
                conflict_model,
                proposed_decisions,
                priorities,
                proposed_sources=proposed_sources,
            )
            return self.commit_tick(conflict_model, preview)
        except Exception:
            self._tick_open = False
            raise

    def preview_tick(
        self,
        conflict_model: ConflictModel,
        proposed_decisions: Mapping[str, Action],
        priorities: Mapping[str, PriorityBreakdown],
        *,
        proposed_sources: Mapping[str, DecisionSource] | None = None,
    ) -> GrantDecisionPreview:
        """Validate and repair actions without committing grants or waiting ages."""
        if not self._tick_open:
            raise RuntimeError("prepare_tick must be called before finalise_tick")
        expected_ids = set(conflict_model.snapshots)
        if set(proposed_decisions) != expected_ids:
            raise ValueError("proposed decisions must contain every observed robot exactly once")
        if set(priorities) != expected_ids:
            raise ValueError("priorities must contain every observed robot exactly once")

        decisions = dict(proposed_decisions)
        decision_sources = (
            dict(proposed_sources)
            if proposed_sources is not None
            else {robot_id: DecisionSource.CP_SAT for robot_id in expected_ids}
        )
        if set(decision_sources) != expected_ids:
            raise ValueError("proposed sources must contain every observed robot exactly once")

        alarms = self._validate_or_repair_components(
            conflict_model,
            decisions,
            decision_sources,
            priorities,
        )
        return GrantDecisionPreview(
            decisions=decisions,
            decision_sources=decision_sources,
            alarms=alarms,
        )

    def commit_tick(
        self,
        conflict_model: ConflictModel,
        preview: GrantDecisionPreview,
    ) -> GrantDecisionResult:
        """Commit a previously reviewed and externally safety-validated decision."""
        if not self._tick_open:
            raise RuntimeError("prepare_tick must be called before commit_tick")
        expected_ids = set(conflict_model.snapshots)
        if set(preview.decisions) != expected_ids:
            raise ValueError("preview decisions must contain every observed robot exactly once")

        try:
            decisions = dict(preview.decisions)
            acquired = self._acquire_grants(conflict_model, decisions)
            self._update_fairness(conflict_model, decisions)
            return GrantDecisionResult(
                tick_id=self._tick_id,
                decisions=decisions,
                decision_sources=preview.decision_sources,
                alarms=preview.alarms,
                acquired_grants=acquired,
                released_grants=tuple(self._pending_releases),
                waiting_ages=self._waiting_ages,
            )
        finally:
            self._tick_open = False
