"""Unit tests for input parsing and internal model helpers."""

from __future__ import annotations

import math
from typing import Any

import pytest
from pydantic import ValidationError

from collision_monitor.models import (
    Pose,
    RobotSnapshot,
    RobotState,
    derive_next_pose,
    parse_robot_state_batch,
)


def state_payload(**overrides: Any) -> dict[str, Any]:
    """Return a valid input payload with optional field overrides."""
    payload: dict[str, Any] = {
        "device_id": "robot-1",
        "timestamp": 1_720_000_000_000,
        "x": 1.0,
        "y": 2.0,
        "theta": 0.5,
        "battery_level": 73.5,
        "loaded": True,
        "deadline": 1_720_000_060_000,
        "path": [{"x": 2.0, "y": 2.0, "theta": 0.5}],
    }
    payload.update(overrides)
    return payload


def test_robot_state_parses_assignment_json() -> None:
    state = RobotState.model_validate(state_payload())

    assert state.device_id == "robot-1"
    assert state.timestamp == 1_720_000_000_000
    assert state.path[0].x == 2.0
    assert state.current_pose() == Pose(x=1.0, y=2.0, theta=0.5)


def test_path_may_be_empty() -> None:
    state = RobotState.model_validate(state_payload(path=[]))

    assert state.path == []


@pytest.mark.parametrize("battery_level", [-0.01, 100.01])
def test_battery_level_must_be_a_percentage(battery_level: float) -> None:
    with pytest.raises(ValidationError):
        RobotState.model_validate(state_payload(battery_level=battery_level))


@pytest.mark.parametrize("field_name", ["timestamp", "deadline"])
def test_epoch_milliseconds_must_not_be_negative(field_name: str) -> None:
    with pytest.raises(ValidationError):
        RobotState.model_validate(state_payload(**{field_name: -1}))


@pytest.mark.parametrize("field_name", ["x", "y", "theta", "battery_level"])
def test_non_finite_robot_values_are_rejected(field_name: str) -> None:
    with pytest.raises(ValidationError):
        RobotState.model_validate(state_payload(**{field_name: math.inf}))


def test_duplicate_device_ids_are_rejected_in_an_explicit_batch() -> None:
    with pytest.raises(ValueError, match="duplicate device IDs in batch: robot-1"):
        parse_robot_state_batch([state_payload(), state_payload(timestamp=1_720_000_001_000)])


def test_distinct_device_ids_are_accepted_in_an_explicit_batch() -> None:
    states = parse_robot_state_batch(
        [state_payload(), state_payload(device_id="robot-2")]
    )

    assert tuple(state.device_id for state in states) == ("robot-1", "robot-2")


def test_next_pose_skips_a_first_node_matching_current_pose() -> None:
    state = RobotState.model_validate(
        state_payload(
            path=[
                {"x": 1.0, "y": 2.0, "theta": 0.5},
                {"x": 3.0, "y": 4.0, "theta": 1.0},
            ]
        )
    )

    next_pose, at_goal = derive_next_pose(state, tolerance=1e-6)

    assert next_pose == Pose(x=3.0, y=4.0, theta=1.0)
    assert at_goal is False


def test_next_pose_comparison_uses_tolerance_and_wrapped_angles() -> None:
    state = RobotState.model_validate(
        state_payload(
            theta=math.pi,
            path=[
                {"x": 1.0 + 5e-7, "y": 2.0, "theta": -math.pi},
                {"x": 2.0, "y": 3.0, "theta": 0.0},
            ],
        )
    )

    next_pose, _ = derive_next_pose(state, tolerance=1e-6)

    assert next_pose == Pose(x=2.0, y=3.0, theta=0.0)


def test_next_pose_uses_first_node_when_it_differs_from_current_pose() -> None:
    state = RobotState.model_validate(state_payload())

    next_pose, at_goal = derive_next_pose(state, tolerance=1e-6)

    assert next_pose == Pose(x=2.0, y=2.0, theta=0.5)
    assert at_goal is False


def test_empty_path_uses_current_pose_and_marks_robot_at_goal() -> None:
    state = RobotState.model_validate(state_payload(path=[]))

    snapshot = RobotSnapshot.from_state(
        state,
        received_at_ms=1_720_000_000_100,
        stale=False,
        tolerance=1e-6,
    )

    assert snapshot.next_pose == Pose(x=1.0, y=2.0, theta=0.5)
    assert snapshot.at_goal is True
    assert snapshot.received_at_ms == 1_720_000_000_100

