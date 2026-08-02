"""Scenario tests for the pure fleet collision decision engine."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import replace

from collision_monitor.config import MonitorConfig
from collision_monitor.conflicts import (
    ConflictModel,
    build_adjacency,
    build_conflict_model,
    decompose_connected_components,
)
from collision_monitor.engine import CollisionDecisionEngine, EngineAlarmCode
from collision_monitor.models import (
    Action,
    DecisionSource,
    FleetDecision,
    RobotSnapshot,
    RobotState,
)

NOW_MS = 1_720_000_000_000
CONFIG = MonitorConfig(safety_margin_metres=0.0)


def make_snapshot(
    robot_id: str,
    *,
    x: float,
    y: float,
    next_x: float | None,
    next_y: float | None,
    theta: float = 0.0,
    stale: bool = False,
    loaded: bool = False,
    battery_level: float = 50.0,
    deadline_ms: int = NOW_MS + 60_000,
    received_at_ms: int = NOW_MS,
) -> RobotSnapshot:
    """Build one validated scenario snapshot."""
    path = (
        []
        if next_x is None or next_y is None
        else [{"x": next_x, "y": next_y, "theta": theta}]
    )
    state = RobotState.model_validate(
        {
            "device_id": robot_id,
            "timestamp": received_at_ms,
            "x": x,
            "y": y,
            "theta": theta,
            "battery_level": battery_level,
            "loaded": loaded,
            "deadline": deadline_ms,
            "path": path,
        }
    )
    return RobotSnapshot.from_state(
        state,
        received_at_ms=received_at_ms,
        stale=stale,
        tolerance=CONFIG.pose_tolerance,
    )


def actions(fleet_decision: FleetDecision) -> dict[str, Action]:
    """Extract robot actions from a fleet decision."""
    return {
        robot_decision.robot_id: robot_decision.action
        for robot_decision in fleet_decision.decisions
    }


def radial_junction_snapshot(
    robot_id: str,
    angle: float,
    *,
    received_at_ms: int = NOW_MS,
) -> RobotSnapshot:
    """Build one of three crossing sweeps through a common junction."""
    radius = 1.5
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    return make_snapshot(
        robot_id,
        x=x,
        y=y,
        next_x=-x,
        next_y=-y,
        theta=angle,
        received_at_ms=received_at_ms,
    )


def test_no_conflicts_resume_all_active_robots_and_pause_at_goal() -> None:
    engine = CollisionDecisionEngine(CONFIG)
    snapshots = (
        make_snapshot("robot-b", x=10.0, y=0.0, next_x=11.0, next_y=0.0),
        make_snapshot("robot-a", x=-10.0, y=0.0, next_x=-9.0, next_y=0.0),
        make_snapshot("robot-goal", x=0.0, y=10.0, next_x=None, next_y=None),
    )

    result = engine.decide(snapshots, NOW_MS, "tick-1")

    assert actions(result) == {
        "robot-a": Action.RESUME,
        "robot-b": Action.RESUME,
        "robot-goal": Action.PAUSE,
    }
    goal_decision = next(
        decision for decision in result.decisions if decision.robot_id == "robot-goal"
    )
    assert "AT_GOAL_PAUSE" in goal_decision.reason_codes
    assert result.tick_metadata["global_safety_valid"] is True


def test_two_robot_crossing_allows_exactly_one_resume() -> None:
    engine = CollisionDecisionEngine(CONFIG)
    snapshots = (
        make_snapshot("robot-a", x=-1.2, y=0.0, next_x=1.2, next_y=0.0),
        make_snapshot(
            "robot-b",
            x=0.0,
            y=-1.2,
            next_x=0.0,
            next_y=1.2,
            theta=math.pi / 2.0,
        ),
    )

    result = engine.decide(snapshots, NOW_MS, "tick-crossing")

    chosen = actions(result)
    assert tuple(chosen.values()).count(Action.RESUME) == 1
    assert tuple(chosen.values()).count(Action.PAUSE) == 1
    assert result.tick_metadata["conflict_edges"] == (("robot-a", "robot-b"),)
    assert result.tick_metadata["global_safety_valid"] is True


def test_low_priority_blocker_moves_when_it_is_the_only_safe_progress() -> None:
    engine = CollisionDecisionEngine(CONFIG)
    snapshots = (
        make_snapshot(
            "approaching-high-priority",
            x=-2.0,
            y=0.0,
            next_x=0.0,
            next_y=0.0,
            loaded=True,
            battery_level=5.0,
            deadline_ms=NOW_MS,
        ),
        make_snapshot(
            "blocker-low-priority",
            x=0.0,
            y=0.0,
            next_x=2.0,
            next_y=0.0,
            deadline_ms=NOW_MS + 300_000,
        ),
    )

    result = engine.decide(snapshots, NOW_MS, "tick-blocker")

    assert actions(result) == {
        "approaching-high-priority": Action.PAUSE,
        "blocker-low-priority": Action.RESUME,
    }
    assert result.tick_metadata["global_safety_valid"] is True


def test_three_way_junction_clears_in_a_stable_progress_sequence() -> None:
    engine = CollisionDecisionEngine(CONFIG)
    tick_one_snapshots = tuple(
        radial_junction_snapshot(robot_id, angle)
        for robot_id, angle in (
            ("robot-a", 0.0),
            ("robot-b", 2.0 * math.pi / 3.0),
            ("robot-c", 4.0 * math.pi / 3.0),
        )
    )

    tick_one = engine.decide(tick_one_snapshots, NOW_MS, "junction-1")
    assert actions(tick_one) == {
        "robot-a": Action.RESUME,
        "robot-b": Action.PAUSE,
        "robot-c": Action.PAUSE,
    }

    tick_two = engine.decide(
        (
            make_snapshot(
                "robot-a",
                x=10.0,
                y=10.0,
                next_x=None,
                next_y=None,
                received_at_ms=NOW_MS + 1_000,
            ),
            radial_junction_snapshot(
                "robot-b",
                2.0 * math.pi / 3.0,
                received_at_ms=NOW_MS + 1_000,
            ),
            radial_junction_snapshot(
                "robot-c",
                4.0 * math.pi / 3.0,
                received_at_ms=NOW_MS + 1_000,
            ),
        ),
        NOW_MS + 1_000,
        "junction-2",
    )
    assert actions(tick_two) == {
        "robot-a": Action.PAUSE,
        "robot-b": Action.RESUME,
        "robot-c": Action.PAUSE,
    }

    tick_three = engine.decide(
        (
            make_snapshot(
                "robot-a",
                x=10.0,
                y=10.0,
                next_x=None,
                next_y=None,
                received_at_ms=NOW_MS + 2_000,
            ),
            make_snapshot(
                "robot-b",
                x=-10.0,
                y=10.0,
                next_x=None,
                next_y=None,
                received_at_ms=NOW_MS + 2_000,
            ),
            radial_junction_snapshot(
                "robot-c",
                4.0 * math.pi / 3.0,
                received_at_ms=NOW_MS + 2_000,
            ),
        ),
        NOW_MS + 2_000,
        "junction-3",
    )
    assert actions(tick_three) == {
        "robot-a": Action.PAUSE,
        "robot-b": Action.PAUSE,
        "robot-c": Action.RESUME,
    }
    assert all(
        decision.tick_metadata["global_safety_valid"]
        for decision in (tick_one, tick_two, tick_three)
    )
    assert all(
        any(action is Action.RESUME for action in actions(decision).values())
        for decision in (tick_one, tick_two, tick_three)
    )


def test_stale_robot_is_a_forced_stationary_obstacle() -> None:
    engine = CollisionDecisionEngine(CONFIG)
    snapshots = (
        make_snapshot("approaching", x=-2.0, y=0.0, next_x=0.0, next_y=0.0),
        make_snapshot(
            "stale-blocker",
            x=0.0,
            y=0.0,
            next_x=2.0,
            next_y=0.0,
            stale=True,
        ),
    )

    result = engine.decide(snapshots, NOW_MS, "tick-stale")

    assert actions(result) == {
        "approaching": Action.PAUSE,
        "stale-blocker": Action.PAUSE,
    }
    stale_decision = next(
        decision for decision in result.decisions if decision.robot_id == "stale-blocker"
    )
    assert "STALE_STATIC_OBSTACLE" in stale_decision.reason_codes
    assert result.tick_metadata["global_safety_valid"] is True


def test_receive_age_derives_staleness_without_a_global_clock() -> None:
    config = MonitorConfig(safety_margin_metres=0.0, stale_timeout_seconds=2.0)
    engine = CollisionDecisionEngine(config)
    old_received_at_ms = NOW_MS - 2_000
    snapshot = make_snapshot(
        "old-state",
        x=0.0,
        y=0.0,
        next_x=2.0,
        next_y=0.0,
        stale=False,
        received_at_ms=old_received_at_ms,
    )

    result = engine.decide((snapshot,), NOW_MS, "tick-derived-stale")

    assert actions(result) == {"old-state": Action.PAUSE}
    assert "STALE_STATIC_OBSTACLE" in result.decisions[0].reason_codes


def test_impossible_geometry_returns_critical_fail_safe_without_state_commit() -> None:
    engine = CollisionDecisionEngine(CONFIG)
    snapshots = (
        make_snapshot("robot-a", x=0.0, y=0.0, next_x=1.0, next_y=0.0),
        make_snapshot("robot-b", x=0.0, y=0.0, next_x=-1.0, next_y=0.0),
    )

    result = engine.decide(snapshots, NOW_MS, "tick-impossible")

    assert actions(result) == {"robot-a": Action.PAUSE, "robot-b": Action.PAUSE}
    assert result.tick_metadata["global_safety_valid"] is False
    assert result.tick_metadata["state_committed"] is False
    assert EngineAlarmCode.GLOBAL_SAFETY_VALIDATION_FAILED in result.tick_metadata["alarms"]
    assert EngineAlarmCode.FAIL_SAFE_GEOMETRY_REMAINS_UNSAFE in result.tick_metadata["alarms"]
    assert engine.grant_manager.tick_id == 0


def test_global_validator_overrides_a_broken_nominal_graph() -> None:
    def broken_builder(
        snapshots: Sequence[RobotSnapshot],
        config: MonitorConfig,
    ) -> ConflictModel:
        real_model = build_conflict_model(snapshots, config)
        robot_ids = tuple(real_model.snapshots)
        adjacency = build_adjacency(robot_ids, ())
        return replace(
            real_model,
            pairwise_constraints=(),
            no_good_constraints=(),
            edges=(),
            adjacency=adjacency,
            isolated_robots=robot_ids,
            connected_components=decompose_connected_components(adjacency),
        )

    engine = CollisionDecisionEngine(CONFIG, conflict_model_builder=broken_builder)
    snapshots = (
        make_snapshot("robot-a", x=-1.2, y=0.0, next_x=1.2, next_y=0.0),
        make_snapshot(
            "robot-b",
            x=0.0,
            y=-1.2,
            next_x=0.0,
            next_y=1.2,
            theta=math.pi / 2.0,
        ),
    )

    result = engine.decide(snapshots, NOW_MS, "tick-broken-graph")

    assert actions(result) == {"robot-a": Action.PAUSE, "robot-b": Action.PAUSE}
    assert result.tick_metadata["global_safety_valid"] is True
    assert result.tick_metadata["state_committed"] is True
    assert EngineAlarmCode.GLOBAL_SAFETY_VALIDATION_FAILED in result.tick_metadata["alarms"]
    assert all(
        decision.decision_source is DecisionSource.FAIL_SAFE
        for decision in result.decisions
    )


def test_identical_input_and_state_are_reproducible_regardless_of_input_order() -> None:
    snapshots = (
        radial_junction_snapshot("robot-a", 0.0),
        radial_junction_snapshot("robot-b", 2.0 * math.pi / 3.0),
        radial_junction_snapshot("robot-c", 4.0 * math.pi / 3.0),
    )
    first_engine = CollisionDecisionEngine(CONFIG)
    second_engine = CollisionDecisionEngine(CONFIG)

    first = first_engine.decide(snapshots, NOW_MS, "reproducible-tick")
    second = second_engine.decide(
        tuple(reversed(snapshots)),
        NOW_MS,
        "reproducible-tick",
    )

    assert first == second


def test_large_component_uses_deterministic_heuristic() -> None:
    config = MonitorConfig(
        safety_margin_metres=0.0,
        maximum_exact_component_size=1,
    )
    engine = CollisionDecisionEngine(config)
    snapshots = (
        make_snapshot("robot-a", x=-1.2, y=0.0, next_x=1.2, next_y=0.0),
        make_snapshot(
            "robot-b",
            x=0.0,
            y=-1.2,
            next_x=0.0,
            next_y=1.2,
            theta=math.pi / 2.0,
        ),
    )

    result = engine.decide(snapshots, NOW_MS, "tick-heuristic")

    assert set(decision.decision_source for decision in result.decisions) == {
        DecisionSource.HEURISTIC
    }
    assert result.tick_metadata["component_diagnostics"][0]["fallback_reason"] == (
        "component_too_large_for_cp_sat"
    )
