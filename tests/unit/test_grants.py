"""Stateful tests for right-of-way grants and liveness safeguards."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import pytest

from collision_monitor.config import MonitorConfig
from collision_monitor.conflicts import (
    ACTION_ASSIGNMENTS,
    ConflictModel,
    PairwiseCompatibility,
    build_adjacency,
    decompose_connected_components,
)
from collision_monitor.grants import (
    AlarmCode,
    GrantManager,
    GrantReleaseReason,
)
from collision_monitor.models import Action, DecisionSource, RobotSnapshot, RobotState
from collision_monitor.optimiser import CpSatComponentOptimiser
from collision_monitor.priority import PriorityBreakdown

NOW_MS = 1_720_000_000_000
CONFIG = MonitorConfig(safety_margin_metres=0.0)
DUMMY_BOUNDS = (0.0, 0.0, 1.0, 1.0)


def make_snapshot(
    robot_id: str,
    *,
    at_goal: bool = False,
    stale: bool = False,
    received_at_ms: int = NOW_MS,
) -> RobotSnapshot:
    """Build a fresh moving or at-goal snapshot."""
    path = [] if at_goal else [{"x": 1.0, "y": 0.0, "theta": 0.0}]
    state = RobotState.model_validate(
        {
            "device_id": robot_id,
            "timestamp": received_at_ms,
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
            "battery_level": 75.0,
            "loaded": False,
            "deadline": NOW_MS + 60_000,
            "path": path,
        }
    )
    return RobotSnapshot.from_state(
        state,
        received_at_ms=received_at_ms,
        stale=stale,
        tolerance=1e-6,
    )


def make_pair(
    robot_i: str,
    robot_j: str,
    *forbidden: tuple[Action, Action],
) -> PairwiseCompatibility:
    """Create a synthetic compatibility record with selected no-goods."""
    assert robot_i < robot_j
    forbidden_set = frozenset(forbidden)
    return PairwiseCompatibility(
        robot_i=robot_i,
        robot_j=robot_j,
        compatibility={
            assignment: assignment not in forbidden_set for assignment in ACTION_ASSIGNMENTS
        },
        envelope_bounds={
            assignment: (DUMMY_BOUNDS, DUMMY_BOUNDS) for assignment in ACTION_ASSIGNMENTS
        },
    )


def make_model(
    snapshots: Sequence[RobotSnapshot],
    pairs: Sequence[PairwiseCompatibility] = (),
) -> ConflictModel:
    """Build a complete conflict model for one manager tick."""
    snapshots_by_id = {
        snapshot.state.device_id: snapshot
        for snapshot in sorted(snapshots, key=lambda item: item.state.device_id)
    }
    edges = tuple(pair.robot_pair for pair in pairs if pair.forbidden_assignments())
    adjacency = build_adjacency(snapshots_by_id.keys(), edges)
    return ConflictModel(
        snapshots=snapshots_by_id,
        pairwise_constraints=tuple(sorted(pairs, key=lambda pair: pair.robot_pair)),
        no_good_constraints=tuple(
            no_good for pair in pairs for no_good in pair.no_good_constraints()
        ),
        edges=edges,
        adjacency=adjacency,
        isolated_robots=tuple(
            robot_id for robot_id in sorted(adjacency) if not adjacency[robot_id]
        ),
        connected_components=decompose_connected_components(adjacency),
    )


def priority(robot_id: str, utility: int = 100) -> PriorityBreakdown:
    """Build a valid optimiser utility for grant-manager tests."""
    return PriorityBreakdown(
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
        throughput_reward=utility,
        final_score=utility,
    )


def priorities(
    robot_ids: Sequence[str],
    utilities: Mapping[str, int] | None = None,
) -> dict[str, PriorityBreakdown]:
    """Build priorities for every observed robot."""
    values = utilities or {}
    return {robot_id: priority(robot_id, values.get(robot_id, 100)) for robot_id in robot_ids}


def finalise(
    manager: GrantManager,
    model: ConflictModel,
    decisions: Mapping[str, Action],
) -> None:
    """Commit one tick with uniform CP-SAT source metadata."""
    robot_ids = tuple(model.snapshots)
    manager.finalise_tick(
        model,
        decisions,
        priorities(robot_ids),
        proposed_sources={robot_id: DecisionSource.CP_SAT for robot_id in robot_ids},
    )


def test_grant_is_retained_as_safe_hard_resume_assignment() -> None:
    manager = GrantManager(CONFIG)
    snapshots = (make_snapshot("robot-a"), make_snapshot("robot-b"))
    pair = make_pair("robot-a", "robot-b", (Action.RESUME, Action.RESUME))
    model = make_model(snapshots, (pair,))

    first = manager.prepare_tick(model, now_ms=NOW_MS)
    assert first.tick_id == 1
    assert first.hard_assignments == {}
    result = manager.finalise_tick(
        model,
        {"robot-a": Action.RESUME, "robot-b": Action.PAUSE},
        priorities(("robot-a", "robot-b")),
    )
    assert result.acquired_grants == ("robot-a",)

    second = manager.prepare_tick(model, now_ms=NOW_MS + 1_000)

    assert second.tick_id == 2
    assert second.hard_assignments == {"robot-a": Action.RESUME}
    assert manager.active_grants["robot-a"].acquisition_tick == 1
    assert manager.active_grants["robot-a"].last_seen_tick == 2


def test_grant_releases_at_goal_and_after_conflict_clearance() -> None:
    config = MonitorConfig(
        safety_margin_metres=0.0,
        grant_minimum_hold_ticks=1,
        grant_clearance_release_ticks=2,
    )
    pair = make_pair("robot-a", "robot-b", (Action.RESUME, Action.RESUME))

    at_goal_manager = GrantManager(config)
    moving_model = make_model(
        (make_snapshot("robot-a"), make_snapshot("robot-b")),
        (pair,),
    )
    at_goal_manager.prepare_tick(moving_model, now_ms=NOW_MS)
    finalise(
        at_goal_manager,
        moving_model,
        {"robot-a": Action.RESUME, "robot-b": Action.PAUSE},
    )
    goal_model = make_model(
        (make_snapshot("robot-a", at_goal=True), make_snapshot("robot-b")),
        (pair,),
    )

    goal_preparation = at_goal_manager.prepare_tick(goal_model, now_ms=NOW_MS + 1_000)

    assert tuple(release.reason for release in goal_preparation.released_grants) == (
        GrantReleaseReason.AT_GOAL,
    )
    assert "robot-a" not in at_goal_manager.active_grants

    clearance_manager = GrantManager(config)
    clearance_manager.prepare_tick(moving_model, now_ms=NOW_MS)
    finalise(
        clearance_manager,
        moving_model,
        {"robot-a": Action.RESUME, "robot-b": Action.PAUSE},
    )
    clear_model = make_model((make_snapshot("robot-a"), make_snapshot("robot-b")))
    clearance_manager.prepare_tick(clear_model, now_ms=NOW_MS + 1_000)
    finalise(
        clearance_manager,
        clear_model,
        {"robot-a": Action.RESUME, "robot-b": Action.RESUME},
    )

    clear_preparation = clearance_manager.prepare_tick(
        clear_model,
        now_ms=NOW_MS + 2_000,
    )

    assert tuple(release.reason for release in clear_preparation.released_grants) == (
        GrantReleaseReason.CONFLICT_CLEARED,
    )


def test_grant_releases_after_disappearance_timeout_and_maximum_lease() -> None:
    pair = make_pair("robot-a", "robot-b", (Action.RESUME, Action.RESUME))
    moving_model = make_model(
        (make_snapshot("robot-a"), make_snapshot("robot-b")),
        (pair,),
    )

    disappearance_config = MonitorConfig(
        safety_margin_metres=0.0,
        stale_timeout_seconds=2.0,
        tick_interval_seconds=1.0,
    )
    disappearance_manager = GrantManager(disappearance_config)
    disappearance_manager.prepare_tick(moving_model, now_ms=NOW_MS)
    finalise(
        disappearance_manager,
        moving_model,
        {"robot-a": Action.RESUME, "robot-b": Action.PAUSE},
    )
    empty_model = make_model(())
    first_missing = disappearance_manager.prepare_tick(
        empty_model,
        now_ms=NOW_MS + 1_000,
    )
    assert first_missing.released_grants == ()
    finalise(disappearance_manager, empty_model, {})

    timed_out = disappearance_manager.prepare_tick(
        empty_model,
        now_ms=NOW_MS + 2_000,
    )
    assert tuple(release.reason for release in timed_out.released_grants) == (
        GrantReleaseReason.STALE_OR_DISAPPEARED,
    )

    lease_config = MonitorConfig(
        safety_margin_metres=0.0,
        grant_minimum_hold_ticks=0,
        grant_maximum_hold_ticks=2,
    )
    lease_manager = GrantManager(lease_config)
    lease_manager.prepare_tick(moving_model, now_ms=NOW_MS)
    finalise(
        lease_manager,
        moving_model,
        {"robot-a": Action.RESUME, "robot-b": Action.PAUSE},
    )
    lease_manager.prepare_tick(moving_model, now_ms=NOW_MS + 1_000)
    finalise(
        lease_manager,
        moving_model,
        {"robot-a": Action.RESUME, "robot-b": Action.PAUSE},
    )

    lease_expired = lease_manager.prepare_tick(moving_model, now_ms=NOW_MS + 2_000)
    assert tuple(release.reason for release in lease_expired.released_grants) == (
        GrantReleaseReason.MAXIMUM_LEASE_FAULT_GUARD,
    )


def test_component_merge_preserves_oldest_compatible_grant() -> None:
    manager = GrantManager(CONFIG)
    robot_ids = ("robot-a", "robot-b", "robot-c", "robot-d")
    snapshots = tuple(make_snapshot(robot_id) for robot_id in robot_ids)
    pair_ab = make_pair("robot-a", "robot-b", (Action.RESUME, Action.RESUME))
    pair_cd = make_pair("robot-c", "robot-d", (Action.RESUME, Action.RESUME))

    first_model = make_model(snapshots, (pair_ab,))
    manager.prepare_tick(first_model, now_ms=NOW_MS)
    finalise(
        manager,
        first_model,
        {
            "robot-a": Action.RESUME,
            "robot-b": Action.PAUSE,
            "robot-c": Action.RESUME,
            "robot-d": Action.RESUME,
        },
    )
    assert tuple(manager.active_grants) == ("robot-a",)

    second_model = make_model(snapshots, (pair_ab, pair_cd))
    manager.prepare_tick(second_model, now_ms=NOW_MS + 1_000)
    finalise(
        manager,
        second_model,
        {
            "robot-a": Action.RESUME,
            "robot-b": Action.PAUSE,
            "robot-c": Action.RESUME,
            "robot-d": Action.PAUSE,
        },
    )
    assert set(manager.active_grants) == {"robot-a", "robot-c"}

    pair_ac = make_pair("robot-a", "robot-c", (Action.RESUME, Action.RESUME))
    merged_model = make_model(snapshots, (pair_ab, pair_cd, pair_ac))
    preparation = manager.prepare_tick(merged_model, now_ms=NOW_MS + 2_000)

    assert preparation.hard_assignments == {"robot-a": Action.RESUME}
    assert tuple((release.robot_id, release.reason) for release in preparation.released_grants) == (
        ("robot-c", GrantReleaseReason.COMPONENT_MERGE_CONFLICT),
    )


def test_waiting_age_updates_reset_and_cap_fairly() -> None:
    config = MonitorConfig(
        safety_margin_metres=0.0,
        priority_waiting_age_cap_ticks=2,
    )
    manager = GrantManager(config)
    first_model = make_model(
        (
            make_snapshot("robot-a"),
            make_snapshot("robot-b"),
            make_snapshot("robot-goal", at_goal=True),
        )
    )
    manager.prepare_tick(first_model, now_ms=NOW_MS)
    result = manager.finalise_tick(
        first_model,
        {
            "robot-a": Action.PAUSE,
            "robot-b": Action.RESUME,
            "robot-goal": Action.PAUSE,
        },
        priorities(("robot-a", "robot-b", "robot-goal")),
    )

    assert result.waiting_ages == {"robot-a": 1, "robot-b": 0, "robot-goal": 0}

    second_model = make_model((make_snapshot("robot-b"), make_snapshot("robot-goal", at_goal=True)))
    manager.prepare_tick(second_model, now_ms=NOW_MS + 1_000)
    second = manager.finalise_tick(
        second_model,
        {"robot-b": Action.PAUSE, "robot-goal": Action.PAUSE},
        priorities(("robot-b", "robot-goal")),
    )
    assert "robot-a" not in second.waiting_ages
    assert "robot-a" not in manager.previous_actions
    assert second.waiting_ages["robot-b"] == 1

    for offset in (2_000, 3_000, 4_000):
        manager.prepare_tick(second_model, now_ms=NOW_MS + offset)
        capped = manager.finalise_tick(
            second_model,
            {"robot-b": Action.PAUSE, "robot-goal": Action.PAUSE},
            priorities(("robot-b", "robot-goal")),
        )
    assert capped.waiting_ages["robot-b"] == 2


@pytest.mark.parametrize("challenger_utility", (201, 1_000))
def test_alternating_raw_priority_does_not_preempt_active_grant(
    challenger_utility: int,
) -> None:
    manager = GrantManager(CONFIG)
    pair = make_pair("robot-a", "robot-b", (Action.RESUME, Action.RESUME))
    optimiser = CpSatComponentOptimiser(CONFIG)
    component = ("robot-a", "robot-b")

    for tick in range(1, 6):
        received_at_ms = NOW_MS + tick * 1_000
        model = make_model(
            (
                make_snapshot("robot-a", received_at_ms=received_at_ms),
                make_snapshot("robot-b", received_at_ms=received_at_ms),
            ),
            (pair,),
        )
        preparation = manager.prepare_tick(model, now_ms=NOW_MS + tick * 1_000)
        raw_utilities = (
            {"robot-a": 200, "robot-b": 100}
            if tick % 2 == 1
            else {"robot-a": 100, "robot-b": challenger_utility}
        )
        scores = priorities(component, raw_utilities)
        solved = optimiser.optimise(
            component,
            model,
            scores,
            hard_assignments=preparation.hard_assignments,
        )
        assert solved.feasible is True
        final = manager.finalise_tick(
            model,
            solved.decisions,
            scores,
            proposed_sources={robot_id: DecisionSource.CP_SAT for robot_id in component},
        )
        assert final.decisions == {
            "robot-a": Action.RESUME,
            "robot-b": Action.PAUSE,
        }


def test_no_all_paused_safeguard_repairs_and_creates_grant() -> None:
    manager = GrantManager(CONFIG)
    snapshots = (make_snapshot("robot-a"), make_snapshot("robot-b"))
    pair = make_pair("robot-a", "robot-b", (Action.PAUSE, Action.PAUSE))
    model = make_model(snapshots, (pair,))
    manager.prepare_tick(model, now_ms=NOW_MS)

    result = manager.finalise_tick(
        model,
        {"robot-a": Action.PAUSE, "robot-b": Action.PAUSE},
        priorities(("robot-a", "robot-b"), {"robot-a": 200, "robot-b": 100}),
    )

    assert result.alarms == ()
    assert any(action is Action.RESUME for action in result.decisions.values())
    assert set(result.decision_sources.values()) == {DecisionSource.REPAIR}
    assert result.acquired_grants


def test_no_all_paused_repair_can_grant_implication_closed_group() -> None:
    manager = GrantManager(CONFIG)
    snapshots = (make_snapshot("robot-a"), make_snapshot("robot-b"))
    pair = make_pair(
        "robot-a",
        "robot-b",
        (Action.PAUSE, Action.PAUSE),
        (Action.RESUME, Action.PAUSE),
        (Action.PAUSE, Action.RESUME),
    )
    model = make_model(snapshots, (pair,))
    manager.prepare_tick(model, now_ms=NOW_MS)

    result = manager.finalise_tick(
        model,
        {"robot-a": Action.PAUSE, "robot-b": Action.PAUSE},
        priorities(("robot-a", "robot-b")),
    )

    assert result.decisions == {
        "robot-a": Action.RESUME,
        "robot-b": Action.RESUME,
    }
    assert result.acquired_grants == ("robot-a", "robot-b")
    assert result.alarms == ()

    preparation = manager.prepare_tick(model, now_ms=NOW_MS + 1_000)
    assert preparation.hard_assignments == {
        "robot-a": Action.RESUME,
        "robot-b": Action.RESUME,
    }


def test_unresolvable_component_returns_honest_alarm_and_fail_safe_pause() -> None:
    manager = GrantManager(CONFIG)
    snapshots = (make_snapshot("robot-a"), make_snapshot("robot-b"))
    pair = make_pair("robot-a", "robot-b", *ACTION_ASSIGNMENTS)
    model = make_model(snapshots, (pair,))
    manager.prepare_tick(model, now_ms=NOW_MS)

    result = manager.finalise_tick(
        model,
        {"robot-a": Action.PAUSE, "robot-b": Action.PAUSE},
        priorities(("robot-a", "robot-b")),
    )

    assert result.decisions == {
        "robot-a": Action.PAUSE,
        "robot-b": Action.PAUSE,
    }
    assert set(result.decision_sources.values()) == {DecisionSource.FAIL_SAFE}
    assert result.alarms == (AlarmCode.UNRESOLVABLE_WITH_PAUSE_RESUME,)
    assert result.acquired_grants == ()


def test_at_goal_robot_is_not_reported_as_progress_or_granted() -> None:
    manager = GrantManager(CONFIG)
    snapshots = (
        make_snapshot("robot-a", at_goal=True),
        make_snapshot("robot-b", at_goal=True),
    )
    pair = make_pair("robot-a", "robot-b", (Action.PAUSE, Action.PAUSE))
    model = make_model(snapshots, (pair,))
    manager.prepare_tick(model, now_ms=NOW_MS)

    result = manager.finalise_tick(
        model,
        {"robot-a": Action.PAUSE, "robot-b": Action.PAUSE},
        priorities(("robot-a", "robot-b")),
    )

    assert set(result.decision_sources.values()) == {DecisionSource.FAIL_SAFE}
    assert result.alarms == (AlarmCode.UNRESOLVABLE_WITH_PAUSE_RESUME,)
    assert result.acquired_grants == ()
