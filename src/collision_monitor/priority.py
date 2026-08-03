"""Transparent integer priority scoring for Resume decisions.

The objective is lexicographic. Every resumed robot receives a throughput
reward larger than the maximum aggregate secondary score of its component.
Business priorities can therefore choose between equally productive safe
solutions, but can never trade away one unit of safe throughput.

The secondary terms have deliberately direct interpretations:

* base progress values any useful Resume action;
* loaded bonus favours robots carrying work;
* deadline urgency rises as non-negative remaining slack approaches zero;
* low-battery urgency applies only below the configured battery threshold;
* waiting age rewards consecutive paused ticks, subject to a cap;
* grant continuation discourages needless right-of-way changes;
* clearance rewards a geometrically safe move on an incident conflict edge
  while the other robot pauses.

All inputs, including time and state carried across ticks, are explicit. This
module never reads a clock or mutable process-global state.
"""

from __future__ import annotations

from collections.abc import Collection, Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

from collision_monitor.config import MonitorConfig
from collision_monitor.conflicts import ConflictModel, PairwiseCompatibility
from collision_monitor.models import Action, RobotSnapshot


@dataclass(frozen=True, slots=True)
class PriorityBounds:
    """Safe bounds used to encode throughput before secondary priority."""

    component_size: int
    maximum_secondary_per_robot: int
    maximum_component_secondary: int
    throughput_reward: int

    def __post_init__(self) -> None:
        if self.component_size < 1:
            raise ValueError("component_size must be at least one")
        if self.maximum_secondary_per_robot < 0:
            raise ValueError("maximum_secondary_per_robot must not be negative")
        if self.maximum_component_secondary < 0:
            raise ValueError("maximum_component_secondary must not be negative")
        if self.throughput_reward <= self.maximum_component_secondary:
            raise ValueError("throughput_reward must exceed the component secondary-score bound")


@dataclass(frozen=True, slots=True)
class PriorityBreakdown:
    """Integer objective terms for one robot's Resume variable.

    ``final_score`` is the coefficient suitable for a CP-SAT Boolean Resume
    variable. A paused robot contributes zero to this objective.
    """

    robot_id: str
    base_progress_value: int
    loaded_bonus: int
    deadline_urgency: int
    low_battery_urgency: int
    waiting_age_bonus: int
    active_grant_continuation_bonus: int
    clearance_bonus: int
    clearance_conflict_count: int
    remaining_slack_ms: int
    secondary_score: int
    throughput_reward: int
    final_score: int

    def __post_init__(self) -> None:
        terms = (
            self.base_progress_value,
            self.loaded_bonus,
            self.deadline_urgency,
            self.low_battery_urgency,
            self.waiting_age_bonus,
            self.active_grant_continuation_bonus,
            self.clearance_bonus,
            self.clearance_conflict_count,
            self.remaining_slack_ms,
            self.secondary_score,
            self.throughput_reward,
            self.final_score,
        )
        if any(not isinstance(term, int) or term < 0 for term in terms):
            raise ValueError("priority terms must be non-negative integers")

        expected_secondary = (
            self.base_progress_value
            + self.loaded_bonus
            + self.deadline_urgency
            + self.low_battery_urgency
            + self.waiting_age_bonus
            + self.active_grant_continuation_bonus
            + self.clearance_bonus
        )
        if self.secondary_score != expected_secondary:
            raise ValueError("secondary_score does not equal its priority terms")
        if self.final_score != self.throughput_reward + self.secondary_score:
            raise ValueError("final_score does not equal throughput plus secondary priority")

    def as_log_data(self) -> Mapping[str, Any]:
        """Return every term for reproducible per-tick diagnostics."""
        return {
            "robot_id": self.robot_id,
            "base_progress_value": self.base_progress_value,
            "loaded_bonus": self.loaded_bonus,
            "deadline_urgency": self.deadline_urgency,
            "low_battery_urgency": self.low_battery_urgency,
            "waiting_age_bonus": self.waiting_age_bonus,
            "active_grant_continuation_bonus": self.active_grant_continuation_bonus,
            "clearance_bonus": self.clearance_bonus,
            "clearance_conflict_count": self.clearance_conflict_count,
            "remaining_slack_ms": self.remaining_slack_ms,
            "secondary_score": self.secondary_score,
            "throughput_reward": self.throughput_reward,
            "final_score": self.final_score,
        }


def priority_bounds(component_size: int, config: MonitorConfig) -> PriorityBounds:
    """Calculate a conservative bound proving lexicographic throughput priority.

    A robot can have at most ``component_size - 1`` incident conflict edges.
    The remaining terms have explicit configured caps. The throughput reward is
    one greater than the resulting aggregate component bound, so adding one
    Resume outweighs any possible change in all secondary scores combined.
    """
    if component_size < 1:
        raise ValueError("component_size must be at least one")

    maximum_clearance = (component_size - 1) * config.priority_clearance_bonus_per_conflict
    maximum_per_robot = (
        config.priority_base_progress_value
        + config.priority_loaded_bonus
        + config.priority_deadline_urgency_maximum
        + config.priority_low_battery_urgency_bonus
        + (config.priority_waiting_age_cap_ticks * config.priority_waiting_age_bonus_per_tick)
        + config.priority_active_grant_continuation_bonus
        + maximum_clearance
    )
    maximum_component = component_size * maximum_per_robot
    return PriorityBounds(
        component_size=component_size,
        maximum_secondary_per_robot=maximum_per_robot,
        maximum_component_secondary=maximum_component,
        throughput_reward=maximum_component + 1,
    )


def deadline_urgency(
    *,
    deadline_ms: int,
    now_ms: int,
    config: MonitorConfig,
) -> tuple[int, int]:
    """Return bounded urgency and non-negative remaining deadline slack."""
    if now_ms < 0:
        raise ValueError("now_ms must not be negative")
    slack_ms = max(deadline_ms - now_ms, 0)
    bounded_slack = min(slack_ms, config.priority_deadline_horizon_ms)
    urgency = (
        config.priority_deadline_urgency_maximum
        * (config.priority_deadline_horizon_ms - bounded_slack)
        // config.priority_deadline_horizon_ms
    )
    return urgency, slack_ms


def clearance_opportunity_count(
    robot_id: str,
    pairwise_constraints: Sequence[PairwiseCompatibility],
) -> int:
    """Count incident conflict edges on which this robot can safely move alone.

    An all-safe pair is not a conflict and contributes nothing. For a conflict
    edge, the robot receives credit only when its Resume envelope is compatible
    with the other robot's Pause envelope.
    """
    count = 0
    for pair in pairwise_constraints:
        if not pair.forbidden_assignments():
            continue
        if robot_id == pair.robot_i:
            can_clear = pair.is_safe(Action.RESUME, Action.PAUSE)
        elif robot_id == pair.robot_j:
            can_clear = pair.is_safe(Action.PAUSE, Action.RESUME)
        else:
            continue
        if can_clear:
            count += 1
    return count


def score_robot_priority(
    snapshot: RobotSnapshot,
    pairwise_constraints: Sequence[PairwiseCompatibility],
    config: MonitorConfig,
    *,
    now_ms: int,
    waiting_age_ticks: int,
    active_grant: bool,
    component_size: int,
) -> PriorityBreakdown:
    """Return a deterministic integer priority breakdown for one robot."""
    if not isinstance(waiting_age_ticks, int) or waiting_age_ticks < 0:
        raise ValueError("waiting_age_ticks must be a non-negative integer")

    bounds = priority_bounds(component_size, config)
    urgency, slack_ms = deadline_urgency(
        deadline_ms=snapshot.state.deadline,
        now_ms=now_ms,
        config=config,
    )
    loaded_bonus = config.priority_loaded_bonus if snapshot.state.loaded else 0
    low_battery_urgency = (
        config.priority_low_battery_urgency_bonus
        if snapshot.state.battery_level < config.priority_low_battery_threshold
        else 0
    )
    capped_waiting_age = min(waiting_age_ticks, config.priority_waiting_age_cap_ticks)
    waiting_age_bonus = capped_waiting_age * config.priority_waiting_age_bonus_per_tick
    grant_bonus = config.priority_active_grant_continuation_bonus if active_grant else 0
    clearance_count = clearance_opportunity_count(
        snapshot.state.device_id,
        pairwise_constraints,
    )
    if clearance_count > component_size - 1:
        raise ValueError("clearance conflict count exceeds the supplied component-size bound")
    clearance_bonus = clearance_count * config.priority_clearance_bonus_per_conflict

    secondary_score = (
        config.priority_base_progress_value
        + loaded_bonus
        + urgency
        + low_battery_urgency
        + waiting_age_bonus
        + grant_bonus
        + clearance_bonus
    )
    if secondary_score > bounds.maximum_secondary_per_robot:
        raise ValueError("secondary score exceeds its calculated safe upper bound")

    return PriorityBreakdown(
        robot_id=snapshot.state.device_id,
        base_progress_value=config.priority_base_progress_value,
        loaded_bonus=loaded_bonus,
        deadline_urgency=urgency,
        low_battery_urgency=low_battery_urgency,
        waiting_age_bonus=waiting_age_bonus,
        active_grant_continuation_bonus=grant_bonus,
        clearance_bonus=clearance_bonus,
        clearance_conflict_count=clearance_count,
        remaining_slack_ms=slack_ms,
        secondary_score=secondary_score,
        throughput_reward=bounds.throughput_reward,
        final_score=bounds.throughput_reward + secondary_score,
    )


def score_component_priorities(
    component: Sequence[str],
    conflict_model: ConflictModel,
    config: MonitorConfig,
    *,
    now_ms: int,
    waiting_ages: Mapping[str, int],
    active_grants: Collection[str],
) -> Mapping[str, PriorityBreakdown]:
    """Score a whole component from explicit tick-local and persistent state."""
    ordered_component = tuple(sorted(component))
    if not ordered_component:
        raise ValueError("component must contain at least one robot")
    if len(set(ordered_component)) != len(ordered_component):
        raise ValueError("component contains duplicate robot IDs")

    component_ids = frozenset(ordered_component)
    missing_snapshots = component_ids.difference(conflict_model.snapshots)
    if missing_snapshots:
        raise ValueError(f"component contains unknown robots: {sorted(missing_snapshots)!r}")
    missing_wait_ages = component_ids.difference(waiting_ages)
    if missing_wait_ages:
        raise ValueError(f"waiting ages are missing for robots: {sorted(missing_wait_ages)!r}")

    component_constraints = tuple(
        pair
        for pair in conflict_model.pairwise_constraints
        if pair.robot_i in component_ids and pair.robot_j in component_ids
    )
    scores = {
        robot_id: score_robot_priority(
            conflict_model.snapshots[robot_id],
            component_constraints,
            config,
            now_ms=now_ms,
            waiting_age_ticks=waiting_ages[robot_id],
            active_grant=robot_id in active_grants,
            component_size=len(ordered_component),
        )
        for robot_id in ordered_component
    }
    return MappingProxyType(scores)


def resumed_objective_value(
    priorities: Mapping[str, PriorityBreakdown],
    resumed_robot_ids: Collection[str],
) -> int:
    """Sum CP-SAT-compatible coefficients for a proposed Resume set."""
    unknown = set(resumed_robot_ids).difference(priorities)
    if unknown:
        raise ValueError(f"Resume set contains unknown robots: {sorted(unknown)!r}")
    return sum(priorities[robot_id].final_score for robot_id in resumed_robot_ids)
