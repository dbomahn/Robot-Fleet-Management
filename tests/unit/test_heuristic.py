"""Adversarial tests for deterministic propagated heuristic decisions."""

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
from collision_monitor.heuristic import (
    BinaryClause,
    DeterministicComponentHeuristic,
    validate_heuristic_decisions,
)
from collision_monitor.models import Action, DecisionSource, RobotSnapshot, RobotState
from collision_monitor.optimiser import UnsafeOptimisationResultError
from collision_monitor.priority import PriorityBreakdown

NOW_MS = 1_720_000_000_000
CONFIG = MonitorConfig(safety_margin_metres=0.0)
DUMMY_BOUNDS = (0.0, 0.0, 1.0, 1.0)


def make_snapshot(robot_id: str) -> RobotSnapshot:
    """Build a stationary snapshot for clause-level heuristic tests."""
    state = RobotState.model_validate(
        {
            "device_id": robot_id,
            "timestamp": NOW_MS,
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
            "battery_level": 75.0,
            "loaded": False,
            "deadline": NOW_MS + 60_000,
            "path": [],
        }
    )
    return RobotSnapshot.from_state(
        state,
        received_at_ms=NOW_MS + 100,
        stale=False,
        tolerance=1e-6,
    )


def make_pair(
    robot_i: str,
    robot_j: str,
    *forbidden: tuple[Action, Action],
) -> PairwiseCompatibility:
    """Create synthetic pair geometry with selected forbidden assignments."""
    assert robot_i < robot_j
    forbidden_set = frozenset(forbidden)
    return PairwiseCompatibility(
        robot_i=robot_i,
        robot_j=robot_j,
        compatibility={
            assignment: assignment not in forbidden_set
            for assignment in ACTION_ASSIGNMENTS
        },
        envelope_bounds={
            assignment: (DUMMY_BOUNDS, DUMMY_BOUNDS)
            for assignment in ACTION_ASSIGNMENTS
        },
    )


def make_model(
    robot_ids: Sequence[str],
    pairs: Sequence[PairwiseCompatibility],
) -> ConflictModel:
    """Build an immutable conflict model from synthetic pair constraints."""
    snapshots = {robot_id: make_snapshot(robot_id) for robot_id in sorted(robot_ids)}
    edges = tuple(pair.robot_pair for pair in pairs if pair.forbidden_assignments())
    adjacency = build_adjacency(snapshots.keys(), edges)
    return ConflictModel(
        snapshots=snapshots,
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


def priority(
    robot_id: str,
    utility: int,
    *,
    clearance_count: int = 0,
) -> PriorityBreakdown:
    """Build a valid score with an explicit ranking and clearance count."""
    return PriorityBreakdown(
        robot_id=robot_id,
        base_progress_value=clearance_count,
        loaded_bonus=0,
        deadline_urgency=0,
        low_battery_urgency=0,
        waiting_age_bonus=0,
        active_grant_continuation_bonus=0,
        clearance_bonus=0,
        clearance_conflict_count=clearance_count,
        remaining_slack_ms=0,
        secondary_score=clearance_count,
        throughput_reward=utility - clearance_count,
        final_score=utility,
    )


def priorities_for(
    robot_ids: Sequence[str],
    utilities: Mapping[str, int] | None = None,
) -> dict[str, PriorityBreakdown]:
    """Build stable priorities, defaulting to equal utility."""
    utility_by_id = utilities or {}
    return {
        robot_id: priority(robot_id, utility_by_id.get(robot_id, 100))
        for robot_id in robot_ids
    }


def test_chain_of_implications_is_propagated() -> None:
    robots = ("robot-a", "robot-b", "robot-c")
    pairs = (
        make_pair("robot-a", "robot-b", (Action.RESUME, Action.PAUSE)),
        make_pair("robot-b", "robot-c", (Action.RESUME, Action.PAUSE)),
    )
    model = make_model(robots, pairs)

    result = DeterministicComponentHeuristic(CONFIG).solve(
        robots,
        model,
        priorities_for(
            robots,
            {"robot-a": 300, "robot-b": 200, "robot-c": 100},
        ),
    )

    assert result.feasible is True
    assert result.decision_source is DecisionSource.HEURISTIC
    assert result.decisions == {robot_id: Action.RESUME for robot_id in robots}


def test_mutual_exclusion_prefers_highest_ranked_robot() -> None:
    robots = ("robot-a", "robot-b")
    pair = make_pair("robot-a", "robot-b", (Action.RESUME, Action.RESUME))
    model = make_model(robots, (pair,))

    result = DeterministicComponentHeuristic(CONFIG).solve(
        robots,
        model,
        priorities_for(robots, {"robot-a": 200, "robot-b": 100}),
    )

    assert result.decisions == {
        "robot-a": Action.RESUME,
        "robot-b": Action.PAUSE,
    }


def test_forbidden_all_paused_pair_forces_progress() -> None:
    robots = ("robot-a", "robot-b")
    pair = make_pair("robot-a", "robot-b", (Action.PAUSE, Action.PAUSE))
    model = make_model(robots, (pair,))

    result = DeterministicComponentHeuristic(CONFIG).solve(
        robots,
        model,
        priorities_for(robots),
        hard_assignments={"robot-a": Action.PAUSE},
    )

    assert result.feasible is True
    assert result.decisions == {
        "robot-a": Action.PAUSE,
        "robot-b": Action.RESUME,
    }


def test_first_ranked_robot_rolls_back_when_only_second_can_move() -> None:
    robots = ("robot-a", "robot-b")
    pair = make_pair(
        "robot-a",
        "robot-b",
        (Action.RESUME, Action.PAUSE),
        (Action.RESUME, Action.RESUME),
    )
    model = make_model(robots, (pair,))

    result = DeterministicComponentHeuristic(CONFIG).solve(
        robots,
        model,
        priorities_for(robots, {"robot-a": 1_000, "robot-b": 100}),
    )

    assert result.decisions == {
        "robot-a": Action.PAUSE,
        "robot-b": Action.RESUME,
    }


def test_output_is_deterministic_across_repeated_runs() -> None:
    robots = ("robot-a", "robot-b", "robot-c")
    pairs = (
        make_pair("robot-a", "robot-b", (Action.RESUME, Action.RESUME)),
        make_pair("robot-b", "robot-c", (Action.RESUME, Action.RESUME)),
    )
    model = make_model(robots, pairs)
    scores = priorities_for(robots)
    heuristic = DeterministicComponentHeuristic(CONFIG)

    decisions = {
        tuple(heuristic.solve(order, model, scores).decisions.items())
        for order in (robots, tuple(reversed(robots)))
        for _ in range(10)
    }

    assert len(decisions) == 1


def test_bounded_repair_prefers_highest_clearance_escape_robot() -> None:
    class RepairOnlyHeuristic(DeterministicComponentHeuristic):
        def _greedy_pass(
            self,
            ranked_robots: Sequence[str],
            seed_assignment: Mapping[str, int],
            clauses: Sequence[BinaryClause],
        ) -> dict[str, int] | None:
            return None

    robots = ("robot-a", "robot-b", "robot-c")
    model = make_model(robots, ())
    scores = {
        "robot-a": priority("robot-a", 300, clearance_count=0),
        "robot-b": priority("robot-b", 200, clearance_count=3),
        "robot-c": priority("robot-c", 100, clearance_count=1),
    }

    result = RepairOnlyHeuristic(CONFIG).solve(robots, model, scores)

    assert result.decision_source is DecisionSource.REPAIR
    assert result.decisions == {
        "robot-a": Action.PAUSE,
        "robot-b": Action.RESUME,
        "robot-c": Action.PAUSE,
    }


def test_final_validator_catches_intentionally_corrupted_assignment() -> None:
    pair = make_pair("robot-a", "robot-b", (Action.RESUME, Action.PAUSE))

    with pytest.raises(UnsafeOptimisationResultError, match="violates no-good"):
        validate_heuristic_decisions(
            ("robot-a", "robot-b"),
            {"robot-a": Action.RESUME, "robot-b": Action.PAUSE},
            (pair,),
        )


def test_unsatisfiable_component_returns_explicit_no_assignment_result() -> None:
    robots = ("robot-a", "robot-b")
    pair = make_pair("robot-a", "robot-b", *ACTION_ASSIGNMENTS)
    model = make_model(robots, (pair,))

    result = DeterministicComponentHeuristic(CONFIG).solve(
        robots,
        model,
        priorities_for(robots),
    )

    assert result.feasible is False
    assert result.progress_made is False
    assert result.decision_source is DecisionSource.FAIL_SAFE
    assert result.fallback_reason == "no_feasible_assignment_found"
    assert result.decisions == {}
