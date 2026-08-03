"""Deterministic end-to-end simulator scenarios through the in-memory service."""

from __future__ import annotations

import asyncio
from pathlib import Path

import pytest

from collision_monitor.simulator.scenario import (
    SimulationResult,
    load_scenario,
    run_in_memory_scenario,
)

SCENARIO_DIRECTORY = Path(__file__).parents[2] / "scenarios"
RESOLVABLE_SCENARIOS = (
    "no_conflict.yaml",
    "perpendicular_crossing.yaml",
    "three_way.yaml",
    "low_priority_blocker.yaml",
)


def run_named_scenario(filename: str) -> SimulationResult:
    """Load and execute one deterministic repository scenario."""
    return asyncio.run(run_in_memory_scenario(load_scenario(SCENARIO_DIRECTORY / filename)))


@pytest.mark.parametrize("filename", RESOLVABLE_SCENARIOS)
def test_resolvable_scenarios_reach_goals_without_intersecting_actions(
    filename: str,
) -> None:
    scenario = load_scenario(SCENARIO_DIRECTORY / filename)

    result = asyncio.run(run_in_memory_scenario(scenario))

    assert result.completed is True
    assert result.ticks <= scenario.max_ticks
    assert all(robot.goal_reached for robot in result.robots)
    assert all(outcome.safety_valid is True for outcome in result.tick_outcomes)
    assert all(
        outcome.service_result is not None
        and outcome.service_result.fleet_decision.tick_metadata["final_global_safety_violations"]
        == ()
        for outcome in result.tick_outcomes
    )


def test_unresolvable_scenario_reports_alarm_and_never_claims_completion() -> None:
    result = run_named_scenario("unresolvable.yaml")

    assert result.completed is False
    assert all(robot.goal_reached is False for robot in result.robots)
    assert "UNRESOLVABLE_WITH_PAUSE_RESUME" in result.alarms
    assert "FAIL_SAFE_GEOMETRY_REMAINS_UNSAFE" in result.alarms
    assert all(outcome.safety_valid is False for outcome in result.tick_outcomes)


def test_identical_scenario_runs_have_reproducible_summaries_and_actions() -> None:
    first = run_named_scenario("three_way.yaml")
    second = run_named_scenario("three_way.yaml")

    assert first.as_dict() == second.as_dict()
    assert tuple(
        tuple(
            (robot_id, action.action.value) for robot_id, action in sorted(outcome.actions.items())
        )
        for outcome in first.tick_outcomes
    ) == tuple(
        tuple(
            (robot_id, action.action.value) for robot_id, action in sorted(outcome.actions.items())
        )
        for outcome in second.tick_outcomes
    )
