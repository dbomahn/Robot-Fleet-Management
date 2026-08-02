"""Unit tests for monotonic latest-state storage."""

from __future__ import annotations

import pytest

from collision_monitor.models import RobotState
from collision_monitor.state_store import LatestStateStore

NOW_MS = 1_720_000_000_000


def make_state(robot_id: str, timestamp: int) -> RobotState:
    """Build one valid state for store tests."""
    return RobotState.model_validate(
        {
            "device_id": robot_id,
            "timestamp": timestamp,
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
            "battery_level": 50.0,
            "loaded": False,
            "deadline": NOW_MS + 60_000,
            "path": [{"x": 1.0, "y": 0.0, "theta": 0.0}],
        }
    )


def test_store_keeps_latest_source_timestamp_per_robot() -> None:
    store = LatestStateStore()
    newer = make_state("robot-a", NOW_MS + 1_000)
    older = make_state("robot-a", NOW_MS)

    assert store.update(newer, received_monotonic_seconds=10.0).accepted is True
    rejected = store.update(older, received_monotonic_seconds=11.0)

    assert rejected.accepted is False
    assert rejected.reason == "older_source_timestamp"
    assert store.latest("robot-a").state.timestamp == NOW_MS + 1_000
    assert store.latest("robot-a").received_monotonic_seconds == 10.0


def test_snapshot_is_sorted_immutable_and_marks_stale_from_receive_age() -> None:
    store = LatestStateStore()
    store.update(make_state("robot-b", NOW_MS), received_monotonic_seconds=8.5)
    store.update(make_state("robot-a", NOW_MS), received_monotonic_seconds=9.5)

    snapshot = store.snapshot(
        captured_monotonic_seconds=10.5,
        captured_epoch_ms=NOW_MS + 5_000,
        stale_timeout_seconds=2.0,
        pose_tolerance=1e-6,
    )

    assert tuple(item.state.device_id for item in snapshot.snapshots) == (
        "robot-a",
        "robot-b",
    )
    assert snapshot.source_state_ages_ms == {"robot-a": 1_000, "robot-b": 2_000}
    assert snapshot.snapshots[0].stale is False
    assert snapshot.snapshots[1].stale is True
    assert snapshot.snapshots[1].received_at_ms == NOW_MS + 3_000
    with pytest.raises(TypeError):
        snapshot.source_state_ages_ms["robot-a"] = 0  # type: ignore[index]


def test_same_timestamp_uses_the_last_received_valid_state() -> None:
    store = LatestStateStore()
    first = make_state("robot-a", NOW_MS)
    second = first.model_copy(update={"x": 2.0})

    store.update(first, received_monotonic_seconds=1.0)
    result = store.update(second, received_monotonic_seconds=2.0)

    assert result.accepted is True
    assert result.reason == "replaced_latest_state"
    assert store.latest("robot-a").state.x == 2.0
