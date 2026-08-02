"""Unit tests for robot footprints and conservative action envelopes."""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Any

import pytest
from shapely.geometry import Polygon

from collision_monitor.config import MonitorConfig
from collision_monitor.geometry import (
    GeometryConstructionError,
    RobotDimensions,
    action_envelope,
    geometries_conflict,
    oriented_footprint,
    pause_envelope,
    resume_envelope,
)
from collision_monitor.models import Action, Pose, RobotSnapshot, RobotState

DIMENSIONS = RobotDimensions(width_metres=0.630, length_metres=1.430)


def state_payload(**overrides: Any) -> dict[str, Any]:
    """Return a valid robot state payload for geometry tests."""
    payload: dict[str, Any] = {
        "device_id": "robot-geometry",
        "timestamp": 1_720_000_000_000,
        "x": 0.0,
        "y": 0.0,
        "theta": 0.0,
        "battery_level": 80.0,
        "loaded": False,
        "deadline": 1_720_000_060_000,
        "path": [],
    }
    payload.update(overrides)
    return payload


def snapshot_for(*, path: list[dict[str, float]], **overrides: Any) -> RobotSnapshot:
    """Build a current snapshot with a deterministically selected next pose."""
    state = RobotState.model_validate(state_payload(path=path, **overrides))
    return RobotSnapshot.from_state(
        state,
        received_at_ms=state.timestamp + 100,
        stale=False,
        tolerance=1e-6,
    )


def assert_bounds(
    polygon: Polygon,
    expected: tuple[float, float, float, float],
) -> None:
    """Compare polygon bounds without depending on vertex ordering."""
    assert polygon.bounds == pytest.approx(expected)


def test_axis_aligned_footprint_uses_length_on_x_axis() -> None:
    footprint = oriented_footprint(Pose(0.0, 0.0, 0.0), DIMENSIONS, 0.0)

    assert_bounds(footprint, (-0.715, -0.315, 0.715, 0.315))
    assert footprint.area == pytest.approx(1.430 * 0.630)


def test_ninety_degree_rotation_turns_length_onto_y_axis() -> None:
    footprint = oriented_footprint(
        Pose(2.0, 3.0, math.pi / 2.0),
        DIMENSIONS,
        0.0,
    )

    assert_bounds(footprint, (2.0 - 0.315, 3.0 - 0.715, 2.0 + 0.315, 3.0 + 0.715))


def test_resume_envelope_covers_translated_sweep() -> None:
    snapshot = snapshot_for(path=[{"x": 2.0, "y": 0.0, "theta": 0.0}])
    config = MonitorConfig(safety_margin_metres=0.0)

    envelope = resume_envelope(snapshot, config)

    assert_bounds(envelope, (-0.715, -0.315, 2.715, 0.315))
    assert envelope.area == pytest.approx((2.0 + 1.430) * 0.630)


def test_no_path_resume_envelope_equals_pause_envelope() -> None:
    snapshot = snapshot_for(path=[])
    config = MonitorConfig(safety_margin_metres=0.05)

    assert snapshot.at_goal is True
    assert resume_envelope(snapshot, config).equals(pause_envelope(snapshot, config))


def test_invalid_snapshot_geometry_reports_robot_id() -> None:
    snapshot = snapshot_for(path=[])
    invalid_snapshot = RobotSnapshot(
        state=snapshot.state,
        received_at_ms=snapshot.received_at_ms,
        stale=snapshot.stale,
        next_pose=Pose(math.nan, 0.0, 0.0),
        at_goal=False,
    )

    with pytest.raises(GeometryConstructionError, match="robot 'robot-geometry'"):
        resume_envelope(invalid_snapshot, MonitorConfig())


def test_touching_rectangles_count_as_conflict() -> None:
    left = oriented_footprint(Pose(0.0, 0.0, 0.0), DIMENSIONS, 0.0)
    right = oriented_footprint(Pose(1.430, 0.0, 0.0), DIMENSIONS, 0.0)

    assert geometries_conflict(left, right) is True


def test_gap_larger_than_combined_margins_is_safe() -> None:
    margin = 0.05
    physical_gap = 0.11
    left = oriented_footprint(Pose(0.0, 0.0, 0.0), DIMENSIONS, margin)
    right = oriented_footprint(
        Pose(DIMENSIONS.length_metres + physical_gap, 0.0, 0.0),
        DIMENSIONS,
        margin,
    )

    assert physical_gap > margin
    assert geometries_conflict(left, right) is False


def test_conflict_predicate_is_symmetric() -> None:
    first = oriented_footprint(Pose(0.0, 0.0, 0.0), DIMENSIONS, 0.02)
    second = oriented_footprint(Pose(0.4, 0.2, math.pi / 4.0), DIMENSIONS, 0.02)

    assert geometries_conflict(first, second) is geometries_conflict(second, first)
    assert geometries_conflict(first, second) is True


@pytest.mark.parametrize(
    ("action", "expected_function"),
    [(Action.PAUSE, pause_envelope), (Action.RESUME, resume_envelope)],
)
def test_action_envelope_dispatches_to_selected_action(
    action: Action,
    expected_function: Callable[[RobotSnapshot, MonitorConfig], Polygon],
) -> None:
    snapshot = snapshot_for(path=[{"x": 1.0, "y": 0.0, "theta": 0.0}])
    config = MonitorConfig(safety_margin_metres=0.01)

    assert action_envelope(snapshot, action, config).equals(
        expected_function(snapshot, config)
    )
