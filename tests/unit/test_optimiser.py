"""Unit tests for bounded CP-SAT component optimisation."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

import pytest
from ortools.sat.python import cp_model

from collision_monitor.config import MonitorConfig
from collision_monitor.conflicts import (
    ACTION_ASSIGNMENTS,
    ConflictModel,
    PairwiseCompatibility,
    build_adjacency,
    decompose_connected_components,
)
from collision_monitor.models import Action, DecisionSource, RobotSnapshot, RobotState
from collision_monitor.optimiser import (
    CpSatComponentOptimiser,
    SolverStatus,
    UnsafeOptimisationResultError,
    validate_component_decisions,
)
from collision_monitor.priority import (
    PriorityBreakdown,
    score_component_priorities,
)

NOW_MS = 1_720_000_000_000
CONFIG = MonitorConfig(safety_margin_metres=0.0, cp_sat_time_limit_seconds=0.050)
DUMMY_BOUNDS = (0.0, 0.0, 1.0, 1.0)


def make_snapshot(
    robot_id: str,
    *,
    loaded: bool = False,
    battery_level: float = 75.0,
    deadline: int = NOW_MS + 1_000_000,
) -> RobotSnapshot:
    """Build a stationary snapshot for optimiser model tests."""
    state = RobotState.model_validate(
        {
            "device_id": robot_id,
            "timestamp": NOW_MS,
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
            "battery_level": battery_level,
            "loaded": loaded,
            "deadline": deadline,
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
    """Build synthetic pairwise geometry results with selected no-goods."""
    assert robot_i < robot_j
    forbidden_set = frozenset(forbidden)
    compatibility = {
        assignment: assignment not in forbidden_set for assignment in ACTION_ASSIGNMENTS
    }
    bounds = {
        assignment: (DUMMY_BOUNDS, DUMMY_BOUNDS) for assignment in ACTION_ASSIGNMENTS
    }
    return PairwiseCompatibility(
        robot_i=robot_i,
        robot_j=robot_j,
        compatibility=compatibility,
        envelope_bounds=bounds,
    )


def make_model(
    snapshots: list[RobotSnapshot],
    pairs: list[PairwiseCompatibility],
) -> ConflictModel:
    """Build a complete immutable conflict model from synthetic pair results."""
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


def simple_priority(robot_id: str, utility: int = 100) -> PriorityBreakdown:
    """Build a valid priority breakdown with a chosen final utility."""
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


def solve_pair(
    forbidden: tuple[Action, Action],
    hard_assignments: dict[str, Action],
) -> Any:
    """Solve a two-robot model used to exercise one no-good pattern."""
    robot_a = make_snapshot("robot-a")
    robot_b = make_snapshot("robot-b")
    pair = make_pair("robot-a", "robot-b", forbidden)
    model = make_model([robot_a, robot_b], [pair])
    priorities = {
        "robot-a": simple_priority("robot-a"),
        "robot-b": simple_priority("robot-b"),
    }
    return CpSatComponentOptimiser(CONFIG).optimise(
        ("robot-a", "robot-b"),
        model,
        priorities,
        hard_assignments=hard_assignments,
    )


@pytest.mark.parametrize(
    "forbidden",
    [
        (Action.RESUME, Action.RESUME),
        (Action.RESUME, Action.PAUSE),
        (Action.PAUSE, Action.RESUME),
        (Action.PAUSE, Action.PAUSE),
    ],
)
def test_all_four_forbidden_patterns_are_encoded(forbidden: tuple[Action, Action]) -> None:
    result = solve_pair(
        forbidden,
        {"robot-a": forbidden[0], "robot-b": forbidden[1]},
    )

    assert result.feasible is False
    assert result.decisions == {}
    assert result.solver_status is SolverStatus.INFEASIBLE
    assert result.fallback_reason == "cp_sat_model_infeasible"


def test_throughput_first_objective_prefers_two_lower_priority_robots() -> None:
    high = make_snapshot(
        "robot-a-high",
        loaded=True,
        battery_level=0.0,
        deadline=NOW_MS,
    )
    low_one = make_snapshot("robot-b-low")
    low_two = make_snapshot("robot-c-low")
    pairs = [
        make_pair(
            "robot-a-high",
            "robot-b-low",
            (Action.RESUME, Action.RESUME),
        ),
        make_pair(
            "robot-a-high",
            "robot-c-low",
            (Action.RESUME, Action.RESUME),
        ),
        make_pair("robot-b-low", "robot-c-low"),
    ]
    model = make_model([high, low_one, low_two], pairs)
    component = ("robot-a-high", "robot-b-low", "robot-c-low")
    priorities = score_component_priorities(
        component,
        model,
        CONFIG,
        now_ms=NOW_MS,
        waiting_ages={robot_id: 0 for robot_id in component},
        active_grants={"robot-a-high"},
    )

    result = CpSatComponentOptimiser(CONFIG).optimise(component, model, priorities)

    assert result.feasible is True
    assert result.decision_source is DecisionSource.CP_SAT
    assert result.decisions == {
        "robot-a-high": Action.PAUSE,
        "robot-b-low": Action.RESUME,
        "robot-c-low": Action.RESUME,
    }


def test_robot_id_rank_breaks_an_equal_utility_tie_deterministically() -> None:
    robot_a = make_snapshot("robot-a")
    robot_b = make_snapshot("robot-b")
    pair = make_pair(
        "robot-a",
        "robot-b",
        (Action.RESUME, Action.RESUME),
    )
    model = make_model([robot_b, robot_a], [pair])
    priorities = {
        "robot-a": simple_priority("robot-a"),
        "robot-b": simple_priority("robot-b"),
    }

    first = CpSatComponentOptimiser(CONFIG).optimise(
        ("robot-b", "robot-a"), model, priorities
    )
    second = CpSatComponentOptimiser(CONFIG).optimise(
        ("robot-a", "robot-b"), model, priorities
    )

    assert first.decisions == second.decisions
    assert first.decisions == {
        "robot-a": Action.RESUME,
        "robot-b": Action.PAUSE,
    }


def test_hard_assignment_from_grant_is_enforced() -> None:
    robot_a = make_snapshot("robot-a")
    robot_b = make_snapshot("robot-b")
    pair = make_pair(
        "robot-a",
        "robot-b",
        (Action.RESUME, Action.RESUME),
    )
    model = make_model([robot_a, robot_b], [pair])
    priorities = {
        "robot-a": simple_priority("robot-a", utility=100),
        "robot-b": simple_priority("robot-b", utility=200),
    }

    result = CpSatComponentOptimiser(CONFIG).optimise(
        ("robot-a", "robot-b"),
        model,
        priorities,
        hard_assignments={"robot-a": Action.RESUME},
    )

    assert result.feasible is True
    assert result.decisions["robot-a"] is Action.RESUME
    assert result.decisions["robot-b"] is Action.PAUSE


def test_solver_configuration_uses_strict_time_and_reproducible_parameters() -> None:
    created_solvers: list[cp_model.CpSolver] = []

    def solver_factory() -> cp_model.CpSolver:
        solver = cp_model.CpSolver()
        created_solvers.append(solver)
        return solver

    config = replace(
        CONFIG,
        cp_sat_time_limit_seconds=0.0125,
        cp_sat_random_seed=37,
        cp_sat_log_search_progress=False,
    )
    robot = make_snapshot("robot-a")
    model = make_model([robot], [])
    priorities = {"robot-a": simple_priority("robot-a")}

    result = CpSatComponentOptimiser(
        config,
        cp_sat_module=cp_model,
        solver_factory=solver_factory,
    ).optimise(("robot-a",), model, priorities)

    assert result.feasible is True
    parameters = created_solvers[0].parameters
    assert parameters.max_time_in_seconds == pytest.approx(0.0125)
    assert parameters.num_search_workers == 1
    assert parameters.random_seed == 37
    assert parameters.log_search_progress is False
    assert parameters.log_to_stdout is False


def test_unavailable_ortools_requests_heuristic_fallback() -> None:
    robot = make_snapshot("robot-a")
    model = make_model([robot], [])
    priorities = {"robot-a": simple_priority("robot-a")}

    result = CpSatComponentOptimiser(CONFIG, cp_sat_module=None).optimise(
        ("robot-a",), model, priorities
    )

    assert result.feasible is False
    assert result.solver_status is SolverStatus.UNAVAILABLE
    assert result.fallback_reason == "cp_sat_unavailable"
    assert result.decisions == {}


def test_independent_validator_rejects_unsafe_assignment() -> None:
    pair = make_pair(
        "robot-a",
        "robot-b",
        (Action.RESUME, Action.PAUSE),
    )

    with pytest.raises(UnsafeOptimisationResultError, match="violates no-good"):
        validate_component_decisions(
            ("robot-a", "robot-b"),
            {"robot-a": Action.RESUME, "robot-b": Action.PAUSE},
            (pair,),
        )
