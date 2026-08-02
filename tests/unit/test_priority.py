"""Unit tests for deterministic, lexicographic robot priorities."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from collision_monitor.config import MonitorConfig
from collision_monitor.conflicts import build_conflict_model
from collision_monitor.models import RobotSnapshot, RobotState
from collision_monitor.priority import (
    PriorityBreakdown,
    priority_bounds,
    resumed_objective_value,
    score_component_priorities,
    score_robot_priority,
)

NOW_MS = 1_720_000_000_000
CONFIG = MonitorConfig(safety_margin_metres=0.0)


def make_snapshot(
    robot_id: str,
    *,
    x: float = 0.0,
    battery_level: float = 75.0,
    loaded: bool = False,
    deadline: int = NOW_MS + 200_000,
    following_x: float | None = None,
) -> RobotSnapshot:
    """Build a snapshot with configurable business-priority inputs."""
    path = (
        []
        if following_x is None
        else [{"x": following_x, "y": 0.0, "theta": 0.0}]
    )
    state = RobotState.model_validate(
        {
            "device_id": robot_id,
            "timestamp": NOW_MS,
            "x": x,
            "y": 0.0,
            "theta": 0.0,
            "battery_level": battery_level,
            "loaded": loaded,
            "deadline": deadline,
            "path": path,
        }
    )
    return RobotSnapshot.from_state(
        state,
        received_at_ms=NOW_MS + 100,
        stale=False,
        tolerance=1e-6,
    )


def score(
    snapshot: RobotSnapshot,
    *,
    config: MonitorConfig = CONFIG,
    now_ms: int = NOW_MS,
    waiting_age_ticks: int = 0,
    active_grant: bool = False,
    component_size: int = 1,
    pairwise_constraints: tuple[Any, ...] = (),
) -> PriorityBreakdown:
    """Score one test robot with explicit deterministic defaults."""
    return score_robot_priority(
        snapshot,
        pairwise_constraints,
        config,
        now_ms=now_ms,
        waiting_age_ticks=waiting_age_ticks,
        active_grant=active_grant,
        component_size=component_size,
    )


def test_earlier_deadline_has_higher_urgency() -> None:
    earlier = make_snapshot("earlier", deadline=NOW_MS + 30_000)
    later = make_snapshot("later", deadline=NOW_MS + 240_000)

    earlier_score = score(earlier)
    later_score = score(later)

    assert earlier_score.remaining_slack_ms == 30_000
    assert earlier_score.deadline_urgency > later_score.deadline_urgency
    assert earlier_score.secondary_score > later_score.secondary_score


def test_loaded_robot_receives_configured_bonus() -> None:
    empty = score(make_snapshot("empty", loaded=False))
    loaded = score(make_snapshot("loaded", loaded=True))

    assert empty.loaded_bonus == 0
    assert loaded.loaded_bonus == CONFIG.priority_loaded_bonus
    assert loaded.secondary_score - empty.secondary_score == CONFIG.priority_loaded_bonus


def test_low_battery_bonus_applies_only_below_threshold() -> None:
    below = score(
        make_snapshot(
            "below",
            battery_level=CONFIG.priority_low_battery_threshold - 0.1,
        )
    )
    threshold = score(
        make_snapshot(
            "threshold",
            battery_level=CONFIG.priority_low_battery_threshold,
        )
    )
    above = score(
        make_snapshot(
            "above",
            battery_level=CONFIG.priority_low_battery_threshold + 0.1,
        )
    )

    assert below.low_battery_urgency == CONFIG.priority_low_battery_urgency_bonus
    assert threshold.low_battery_urgency == 0
    assert above.low_battery_urgency == 0


def test_waiting_age_bonus_is_monotonic_and_capped() -> None:
    snapshot = make_snapshot("waiting")
    ages = (0, 1, 5, CONFIG.priority_waiting_age_cap_ticks, 10_000)
    bonuses = tuple(
        score(snapshot, waiting_age_ticks=age).waiting_age_bonus for age in ages
    )

    assert bonuses == tuple(sorted(bonuses))
    assert bonuses[0] == 0
    assert bonuses[-1] == bonuses[-2]
    assert bonuses[-1] == (
        CONFIG.priority_waiting_age_cap_ticks
        * CONFIG.priority_waiting_age_bonus_per_tick
    )


def test_grant_continuation_dominates_small_priority_changes() -> None:
    snapshot = make_snapshot("granted")
    granted = score(snapshot, active_grant=True)
    modest_wait = score(snapshot, waiting_age_ticks=2, active_grant=False)

    assert granted.active_grant_continuation_bonus == (
        CONFIG.priority_active_grant_continuation_bonus
    )
    assert granted.secondary_score > modest_wait.secondary_score


def test_clearance_bonus_counts_only_geometrically_valid_move() -> None:
    blocked = make_snapshot("robot-a-blocked", x=-2.0, following_x=0.0)
    blocker = make_snapshot("robot-z-blocker", x=0.0, following_x=2.0)
    model = build_conflict_model([blocked, blocker], CONFIG)

    scores = score_component_priorities(
        model.connected_components[0],
        model,
        CONFIG,
        now_ms=NOW_MS,
        waiting_ages={"robot-a-blocked": 0, "robot-z-blocker": 0},
        active_grants=(),
    )

    assert scores["robot-a-blocked"].clearance_conflict_count == 0
    assert scores["robot-a-blocked"].clearance_bonus == 0
    assert scores["robot-z-blocker"].clearance_conflict_count == 1
    assert scores["robot-z-blocker"].clearance_bonus == (
        CONFIG.priority_clearance_bonus_per_conflict
    )


def test_one_extra_resume_dominates_all_secondary_score_differences() -> None:
    component_size = 3
    bounds = priority_bounds(component_size, CONFIG)
    assert bounds.throughput_reward == bounds.maximum_component_secondary + 1

    high_priority = score(
        make_snapshot(
            "high",
            battery_level=0.0,
            loaded=True,
            deadline=NOW_MS,
        ),
        waiting_age_ticks=CONFIG.priority_waiting_age_cap_ticks,
        active_grant=True,
        component_size=component_size,
    )
    low_one = score(
        make_snapshot("low-one", deadline=NOW_MS + 1_000_000),
        component_size=component_size,
    )
    low_two = score(
        make_snapshot("low-two", deadline=NOW_MS + 1_000_000),
        component_size=component_size,
    )
    priorities = {
        priority.robot_id: priority for priority in (high_priority, low_one, low_two)
    }

    one_resume = resumed_objective_value(priorities, {"high"})
    two_resumes = resumed_objective_value(priorities, {"low-one", "low-two"})

    assert two_resumes > one_resume
    assert bounds.throughput_reward > bounds.maximum_component_secondary


def test_every_priority_coefficient_is_configurable() -> None:
    custom = replace(
        CONFIG,
        priority_base_progress_value=1,
        priority_loaded_bonus=2,
        priority_deadline_urgency_maximum=3,
        priority_deadline_horizon_ms=10_000,
        priority_low_battery_threshold=40.0,
        priority_low_battery_urgency_bonus=4,
        priority_waiting_age_bonus_per_tick=5,
        priority_waiting_age_cap_ticks=6,
        priority_active_grant_continuation_bonus=7,
        priority_clearance_bonus_per_conflict=8,
    )
    breakdown = score(
        make_snapshot(
            "custom",
            loaded=True,
            battery_level=39.0,
            deadline=NOW_MS,
        ),
        config=custom,
        waiting_age_ticks=20,
        active_grant=True,
    )

    assert breakdown.base_progress_value == 1
    assert breakdown.loaded_bonus == 2
    assert breakdown.deadline_urgency == 3
    assert breakdown.low_battery_urgency == 4
    assert breakdown.waiting_age_bonus == 30
    assert breakdown.active_grant_continuation_bonus == 7
