"""Unit tests for pairwise constraints and deterministic conflict graphs."""

from __future__ import annotations

import math
from collections.abc import Sequence

import pytest

from collision_monitor.config import MonitorConfig
from collision_monitor.conflicts import (
    ACTION_ASSIGNMENTS,
    ConflictModel,
    NoGoodConstraint,
    build_conflict_model,
    evaluate_pairwise_compatibility,
)
from collision_monitor.models import Action, RobotSnapshot, RobotState

CONFIG = MonitorConfig(safety_margin_metres=0.0)


def make_snapshot(
    robot_id: str,
    *,
    current: tuple[float, float, float],
    following: tuple[float, float, float] | None = None,
) -> RobotSnapshot:
    """Build a fresh snapshot with zero or one remaining movement step."""
    path = (
        []
        if following is None
        else [{"x": following[0], "y": following[1], "theta": following[2]}]
    )
    state = RobotState.model_validate(
        {
            "device_id": robot_id,
            "timestamp": 1_720_000_000_000,
            "x": current[0],
            "y": current[1],
            "theta": current[2],
            "battery_level": 75.0,
            "loaded": False,
            "deadline": 1_720_000_060_000,
            "path": path,
        }
    )
    return RobotSnapshot.from_state(
        state,
        received_at_ms=state.timestamp + 100,
        stale=False,
        tolerance=1e-6,
    )


def crossing_pair(
    prefix: str,
    *,
    x_offset: float = 0.0,
) -> tuple[RobotSnapshot, RobotSnapshot]:
    """Create two robots whose Resume envelopes cross perpendicularly."""
    horizontal = make_snapshot(
        f"{prefix}-a",
        current=(x_offset - 2.0, 0.0, 0.0),
        following=(x_offset, 0.0, 0.0),
    )
    vertical = make_snapshot(
        f"{prefix}-b",
        current=(x_offset, -2.0, math.pi / 2.0),
        following=(x_offset, 0.0, math.pi / 2.0),
    )
    return horizontal, vertical


def build(snapshots: Sequence[RobotSnapshot]) -> ConflictModel:
    """Build a model with geometry-test margins disabled."""
    return build_conflict_model(snapshots, CONFIG)


def find_no_good(
    no_goods: Sequence[NoGoodConstraint],
    assignment: tuple[Action, Action],
) -> NoGoodConstraint:
    """Return the no-good retaining a requested source assignment."""
    return next(
        no_good for no_good in no_goods if no_good.source_assignment == assignment
    )


def test_no_conflict_robots_are_isolated_singleton_components() -> None:
    robot_b = make_snapshot(
        "robot-b",
        current=(10.0, 0.0, 0.0),
        following=(11.0, 0.0, 0.0),
    )
    robot_a = make_snapshot(
        "robot-a",
        current=(0.0, 0.0, 0.0),
        following=(1.0, 0.0, 0.0),
    )

    model = build([robot_b, robot_a])

    assert model.edges == ()
    assert model.no_good_constraints == ()
    assert model.isolated_robots == ("robot-a", "robot-b")
    assert model.connected_components == (("robot-a",), ("robot-b",))
    assert len(model.pairwise_constraints) == 1
    assert model.pairwise_constraints[0].forbidden_assignments() == ()


def test_perpendicular_crossing_produces_one_component() -> None:
    horizontal, vertical = crossing_pair("junction")

    model = build([vertical, horizontal])

    assert model.edges == (("junction-a", "junction-b"),)
    assert model.isolated_robots == ()
    assert model.connected_components == (("junction-a", "junction-b"),)
    compatibility = model.compatibility_for("junction-b", "junction-a")
    assert compatibility.is_safe(Action.RESUME, Action.RESUME) is False


def test_two_independent_junctions_produce_two_components() -> None:
    first = crossing_pair("alpha", x_offset=0.0)
    second = crossing_pair("bravo", x_offset=20.0)

    model = build([second[1], first[0], second[0], first[1]])

    assert model.edges == (("alpha-a", "alpha-b"), ("bravo-a", "bravo-b"))
    assert model.connected_components == (
        ("alpha-a", "alpha-b"),
        ("bravo-a", "bravo-b"),
    )


def test_component_and_pair_order_is_deterministic() -> None:
    first = crossing_pair("alpha", x_offset=0.0)
    second = crossing_pair("bravo", x_offset=20.0)
    isolated = make_snapshot("charlie", current=(50.0, 50.0, 0.0))
    snapshots = [first[0], first[1], second[0], second[1], isolated]

    forward = build(snapshots)
    reverse = build(list(reversed(snapshots)))

    assert reverse.edges == forward.edges
    assert reverse.connected_components == forward.connected_components
    assert reverse.connected_components == (
        ("alpha-a", "alpha-b"),
        ("bravo-a", "bravo-b"),
        ("charlie",),
    )
    assert tuple(pair.robot_pair for pair in reverse.pairwise_constraints) == tuple(
        pair.robot_pair for pair in forward.pairwise_constraints
    )


def test_asymmetric_resume_pause_can_be_unsafe_when_reverse_is_safe() -> None:
    moving_into_space = make_snapshot(
        "robot-a",
        current=(-2.0, 0.0, 0.0),
        following=(0.0, 0.0, 0.0),
    )
    moving_away = make_snapshot(
        "robot-b",
        current=(0.0, 0.0, 0.0),
        following=(2.0, 0.0, 0.0),
    )

    compatibility = evaluate_pairwise_compatibility(moving_into_space, moving_away, CONFIG)

    assert compatibility.is_safe(Action.RESUME, Action.PAUSE) is False
    assert compatibility.is_safe(Action.PAUSE, Action.RESUME) is True
    assert compatibility.is_safe(Action.PAUSE, Action.PAUSE) is True


def test_low_priority_blocker_must_move_before_blocked_robot_can_move() -> None:
    blocked = make_snapshot(
        "robot-a-blocked",
        current=(-2.0, 0.0, 0.0),
        following=(0.0, 0.0, 0.0),
    )
    blocker = make_snapshot(
        "robot-z-blocker",
        current=(0.0, 0.0, 0.0),
        following=(2.0, 0.0, 0.0),
    )

    model = build([blocker, blocked])
    pair = model.compatibility_for("robot-a-blocked", "robot-z-blocker")

    assert pair.is_safe(Action.RESUME, Action.PAUSE) is False
    assert pair.is_safe(Action.PAUSE, Action.RESUME) is True
    assert pair.is_safe(Action.RESUME, Action.RESUME) is False
    forbidden = find_no_good(
        model.no_good_constraints,
        (Action.RESUME, Action.PAUSE),
    )
    assert tuple(literal.forbidden_value for literal in forbidden.literals) == (1, 0)
    assert forbidden.expression() == (
        "x[robot-a-blocked] != 1 OR x[robot-z-blocker] != 0"
    )


def test_pairwise_compatibility_contains_all_diagnostics_and_is_immutable() -> None:
    horizontal, vertical = crossing_pair("junction")

    compatibility = evaluate_pairwise_compatibility(vertical, horizontal, CONFIG)

    assert compatibility.robot_pair == ("junction-a", "junction-b")
    assert tuple(compatibility.compatibility) == ACTION_ASSIGNMENTS
    assert tuple(compatibility.envelope_bounds) == ACTION_ASSIGNMENTS
    bounds_i, bounds_j = compatibility.bounds_for(Action.PAUSE, Action.RESUME)
    assert len(bounds_i) == 4
    assert len(bounds_j) == 4
    with pytest.raises(TypeError):
        compatibility.compatibility[(Action.PAUSE, Action.PAUSE)] = False  # type: ignore[index]


def test_no_good_preserves_pair_and_action_provenance() -> None:
    horizontal, vertical = crossing_pair("junction")
    model = build([horizontal, vertical])
    no_good = find_no_good(
        model.no_good_constraints,
        (Action.RESUME, Action.RESUME),
    )

    assert no_good.source_pair == ("junction-a", "junction-b")
    assert no_good.source_assignment == (Action.RESUME, Action.RESUME)
    assert tuple(literal.forbidden_value for literal in no_good.literals) == (1, 1)
    assert no_good.is_violated_by({"junction-a": 1, "junction-b": 1}) is True
    assert no_good.is_violated_by({"junction-a": 1, "junction-b": 0}) is False
    assert no_good.as_log_data()["source_actions"] == ("Resume", "Resume")
