"""Async application-service tests using in-memory transport adapters."""

from __future__ import annotations

import asyncio
import logging
import uuid
from collections.abc import Mapping
from typing import Any

from collision_monitor.config import MonitorConfig
from collision_monitor.models import Action
from collision_monitor.service import CollisionMonitorService
from collision_monitor.transport.memory import (
    InMemoryActionPublisher,
    InMemoryStateConsumer,
)

NOW_MS = 1_720_000_000_000


class MutableClock:
    """A manually advanced monotonic and epoch clock for deterministic tests."""

    def __init__(self, *, monotonic_seconds: float = 100.0, epoch_ms: int = NOW_MS) -> None:
        self.monotonic_seconds = monotonic_seconds
        self.epoch_ms = epoch_ms

    def monotonic(self) -> float:
        """Return the configured monotonic time."""
        return self.monotonic_seconds

    def epoch(self) -> int:
        """Return the configured epoch time in milliseconds."""
        return self.epoch_ms

    def advance(self, seconds: float) -> None:
        """Advance both clocks by an identical duration."""
        self.monotonic_seconds += seconds
        self.epoch_ms += int(seconds * 1_000)


def state_payload(
    robot_id: str,
    *,
    timestamp: int = NOW_MS,
    x: float = 0.0,
    y: float = 0.0,
    next_x: float = 1.0,
    next_y: float = 0.0,
) -> dict[str, Any]:
    """Return one valid moving robot-state payload."""
    return {
        "device_id": robot_id,
        "timestamp": timestamp,
        "x": x,
        "y": y,
        "theta": 0.0,
        "battery_level": 50.0,
        "loaded": False,
        "deadline": NOW_MS + 60_000,
        "path": [{"x": next_x, "y": next_y, "theta": 0.0}],
    }


def make_service(
    *,
    config: MonitorConfig | None = None,
    clock: MutableClock | None = None,
    publisher: InMemoryActionPublisher | None = None,
    sleeper: Any = asyncio.sleep,
    logger: logging.Logger | None = None,
    run_id: str | None = "test-run",
) -> tuple[
    CollisionMonitorService,
    InMemoryStateConsumer,
    InMemoryActionPublisher,
    MutableClock,
]:
    """Build a service with fully in-memory boundaries and explicit clocks."""
    selected_clock = clock or MutableClock()
    consumer = InMemoryStateConsumer()
    selected_publisher = publisher or InMemoryActionPublisher()
    service = CollisionMonitorService(
        config or MonitorConfig(safety_margin_metres=0.0),
        consumer,
        selected_publisher,
        monotonic_clock=selected_clock.monotonic,
        epoch_clock_ms=selected_clock.epoch,
        sleeper=sleeper,
        logger=logger,
        run_id=run_id,
    )
    return service, consumer, selected_publisher, selected_clock


def test_service_generates_one_run_id_when_not_injected() -> None:
    first, _, _, _ = make_service(run_id=None)
    second, _, _, _ = make_service(run_id=None)

    assert uuid.UUID(first.run_id)
    assert uuid.UUID(second.run_id)
    assert first.run_id != second.run_id


def test_states_arriving_in_arbitrary_order_publish_in_robot_id_order() -> None:
    async def scenario() -> None:
        service, consumer, publisher, _ = make_service()
        await consumer.submit(state_payload("robot-b", x=10.0, next_x=11.0))
        await consumer.submit(state_payload("robot-a", x=-10.0, next_x=-9.0))
        await consumer.close()
        await service.consume_states()

        result = await service.run_tick()

        assert tuple(delivery.settlement for delivery in consumer.deliveries) == (
            "acknowledged",
            "acknowledged",
        )
        assert tuple(message.device_id for message in publisher.published) == (
            "robot-a",
            "robot-b",
        )
        assert all(message.action is Action.RESUME for message in publisher.published)
        assert result.publication_complete is True
        assert result.trace["tick_id"] == "tick-00000001"

    asyncio.run(scenario())


def test_multiple_updates_before_tick_publish_only_latest_state() -> None:
    async def scenario() -> None:
        service, _, publisher, _ = make_service()
        await service.ingest_payload(state_payload("robot-a", timestamp=NOW_MS, x=0.0, next_x=1.0))
        await service.ingest_payload(
            state_payload(
                "robot-a",
                timestamp=NOW_MS + 1_000,
                x=10.0,
                next_x=11.0,
            )
        )

        result = await service.run_tick()

        assert len(publisher.published) == 1
        assert publisher.published[0].source_state_timestamp == NOW_MS + 1_000
        assert result.trace["robots"][0]["source_state_timestamp"] == NOW_MS + 1_000

    asyncio.run(scenario())


def test_stale_state_uses_monotonic_receive_age_and_is_paused() -> None:
    async def scenario() -> None:
        config = MonitorConfig(safety_margin_metres=0.0, stale_timeout_seconds=2.0)
        service, _, publisher, clock = make_service(config=config)
        await service.ingest_payload(state_payload("robot-a"))
        clock.advance(2.0)

        result = await service.run_tick()

        assert publisher.published[0].action is Action.PAUSE
        assert result.trace["robots"][0]["source_state_age_ms"] == 2_000
        assert result.trace["robots"][0]["stale"] is True
        assert "STALE_STATIC_OBSTACLE" in publisher.published[0].reason_codes
        assert "PROLONGED_STATE_LOSS_STATIC_OBSTACLE" in result.trace["alarms"]
        assert "PROLONGED_STATE_LOSS_STATIC_OBSTACLE" in (publisher.published[0].reason_codes)
        assert service.state_store.robot_ids == ("robot-a",)

    asyncio.run(scenario())


def test_each_tick_publishes_exactly_one_action_per_known_robot() -> None:
    async def scenario() -> None:
        service, _, publisher, clock = make_service()
        for robot_id, x in (("robot-a", -10.0), ("robot-b", 10.0)):
            await service.ingest_payload(state_payload(robot_id, x=x, next_x=x + 1.0))

        first = await service.run_tick()
        clock.advance(1.0)
        second = await service.run_tick()

        assert first.fleet_decision.tick_id == "tick-00000001"
        assert second.fleet_decision.tick_id == "tick-00000002"
        assert tuple((message.tick_id, message.device_id) for message in publisher.published) == (
            ("tick-00000001", "robot-a"),
            ("tick-00000001", "robot-b"),
            ("tick-00000002", "robot-a"),
            ("tick-00000002", "robot-b"),
        )

    asyncio.run(scenario())


def test_trace_contains_deterministic_required_fields_without_wkt() -> None:
    async def produce_trace() -> Mapping[str, Any]:
        service, _, _, _ = make_service()
        await service.ingest_payload(
            state_payload("robot-a", x=-1.2, y=0.0, next_x=1.2, next_y=0.0)
        )
        vertical = state_payload(
            "robot-b",
            x=0.0,
            y=-1.2,
            next_x=0.0,
            next_y=1.2,
        )
        vertical["theta"] = 1.5707963267948966
        vertical["path"][0]["theta"] = 1.5707963267948966
        await service.ingest_payload(vertical)
        return (await service.run_tick()).trace

    first = asyncio.run(produce_trace())
    second = asyncio.run(produce_trace())

    def without_runtime_measurements(trace: Mapping[str, Any]) -> Mapping[str, Any]:
        normalised = dict(trace)
        normalised["solver"] = tuple(
            {key: value for key, value in component.items() if key != "wall_time_seconds"}
            for component in trace["solver"]
        )
        return normalised

    required = {
        "run_id",
        "tick_id",
        "tick_epoch_ms",
        "tick_monotonic_seconds",
        "robots",
        "component_membership",
        "forbidden_action_combinations",
        "priority_breakdown",
        "active_grants",
        "solver",
        "solver_timing",
        "final_actions",
        "safety_validation",
        "alarms",
        "publication",
        "detailed_geometry",
    }
    assert set(first) == required
    assert first["run_id"] == "test-run"
    assert without_runtime_measurements(first) == without_runtime_measurements(second)
    assert first["detailed_geometry"] == ()
    assert "forbidden_bounds" in first["forbidden_action_combinations"][0]
    assert "wall_time_limit_seconds" in first["solver"][0]
    assert first["solver"][0]["solver_status"] in {"OPTIMAL", "FEASIBLE"}


def test_cp_sat_trace_contains_actual_wall_time_and_configured_limit() -> None:
    async def scenario() -> None:
        config = MonitorConfig(
            safety_margin_metres=0.0,
            cp_sat_time_limit_seconds=0.025,
        )
        service, _, _, _ = make_service(config=config)
        await service.ingest_payload(
            state_payload("robot-a", x=-1.2, y=0.0, next_x=1.2, next_y=0.0)
        )
        vertical = state_payload(
            "robot-b",
            x=0.0,
            y=-1.2,
            next_x=0.0,
            next_y=1.2,
        )
        vertical["theta"] = 1.5707963267948966
        vertical["path"][0]["theta"] = 1.5707963267948966
        await service.ingest_payload(vertical)

        result = await service.run_tick()

        solver = result.trace["solver"][0]
        assert solver["method"] == "cp_sat"
        assert solver["solver_status"] in {"OPTIMAL", "FEASIBLE"}
        assert isinstance(solver["wall_time_seconds"], float)
        assert solver["wall_time_seconds"] >= 0.0
        assert solver["wall_time_limit_seconds"] == 0.025

    asyncio.run(scenario())


def test_detailed_geometry_wkt_is_explicitly_opt_in() -> None:
    async def scenario() -> None:
        config = MonitorConfig(
            safety_margin_metres=0.0,
            trace_detailed_geometry=True,
        )
        service, _, _, _ = make_service(config=config)
        await service.ingest_payload(state_payload("robot-a"))

        result = await service.run_tick()

        geometry = result.trace["detailed_geometry"]
        assert geometry[0]["robot_id"] == "robot-a"
        assert geometry[0]["pause_wkt"].startswith("POLYGON")
        assert geometry[0]["resume_wkt"].startswith("POLYGON")

    asyncio.run(scenario())


def test_safety_diagnostics_emit_a_stable_critical_log(caplog: Any) -> None:
    async def scenario() -> None:
        logger = logging.getLogger("collision_monitor.test.critical")
        service, _, _, _ = make_service(logger=logger)
        await service.ingest_payload(state_payload("robot-a", x=0.0, next_x=1.0))
        await service.ingest_payload(state_payload("robot-b", x=0.0, next_x=-1.0))

        with caplog.at_level(logging.INFO, logger=logger.name):
            await service.run_tick()

        critical = next(
            record for record in caplog.records if record.message == "decision_tick_critical_alarm"
        )
        data = critical.collision_monitor_data
        assert critical.levelno == logging.CRITICAL
        assert data["tick_id"] == "tick-00000001"
        assert data["alarm_codes"] == (
            "CURRENT_FOOTPRINT_OVERLAP",
            "GLOBAL_SAFETY_VALIDATION_FAILED",
            "FAIL_SAFE_GEOMETRY_REMAINS_UNSAFE",
        )

    asyncio.run(scenario())


def test_publication_retries_then_succeeds_without_duplicate_success() -> None:
    async def scenario() -> None:
        delays: list[float] = []

        async def sleeper(delay: float) -> None:
            delays.append(delay)

        config = MonitorConfig(
            safety_margin_metres=0.0,
            publication_maximum_attempts=3,
            publication_retry_delay_seconds=0.25,
        )
        publisher = InMemoryActionPublisher(failures_before_success={"robot-a": 2})
        service, _, _, _ = make_service(
            config=config,
            publisher=publisher,
            sleeper=sleeper,
        )
        await service.ingest_payload(state_payload("robot-a"))

        result = await service.run_tick()

        assert result.publication_complete is True
        assert publisher.published == (result.action_messages[0],)
        assert publisher.attempt_counts[("test-run", "robot-a", "tick-00000001")] == 3
        assert publisher.published[0].idempotency_key == (
            "test-run",
            "robot-a",
            "tick-00000001",
        )
        assert delays == [0.25, 0.25]

    asyncio.run(scenario())


def test_equal_tick_ids_from_different_service_runs_do_not_collide() -> None:
    async def scenario() -> None:
        first_service, _, _, _ = make_service(run_id="run-one")
        second_service, _, _, _ = make_service(run_id="run-two")
        await first_service.ingest_payload(state_payload("robot-a"))
        await second_service.ingest_payload(state_payload("robot-a"))

        first = await first_service.run_tick()
        second = await second_service.run_tick()

        first_message = first.action_messages[0]
        second_message = second.action_messages[0]
        assert first_message.tick_id == second_message.tick_id == "tick-00000001"
        assert first_message.idempotency_key == (
            "run-one",
            "robot-a",
            "tick-00000001",
        )
        assert second_message.idempotency_key == (
            "run-two",
            "robot-a",
            "tick-00000001",
        )
        assert first_message.idempotency_key != second_message.idempotency_key
        assert first.trace["run_id"] == "run-one"
        assert second.trace["run_id"] == "run-two"

    asyncio.run(scenario())


def test_exhausted_publication_is_reported_as_incomplete_tick() -> None:
    async def scenario() -> None:
        async def no_wait(_: float) -> None:
            return None

        config = MonitorConfig(
            safety_margin_metres=0.0,
            publication_maximum_attempts=2,
            publication_retry_delay_seconds=0.0,
        )
        publisher = InMemoryActionPublisher(failures_before_success={"robot-a": 5})
        service, _, _, _ = make_service(
            config=config,
            publisher=publisher,
            sleeper=no_wait,
        )
        await service.ingest_payload(state_payload("robot-a"))
        await service.ingest_payload(state_payload("robot-b", x=10.0, next_x=11.0))

        result = await service.run_tick()

        assert result.publication_complete is False
        assert tuple(message.device_id for message in publisher.published) == ("robot-b",)
        assert result.publication_failures[0].robot_id == "robot-a"
        assert result.publication_failures[0].attempts == 2
        assert result.trace["publication"]["complete"] is False
        assert result.trace["publication"]["failures"][0]["robot_id"] == "robot-a"
        assert result.trace["publication"]["successful_robot_ids"] == ("robot-b",)

    asyncio.run(scenario())


def test_invalid_input_is_rejected_and_logged(caplog: Any) -> None:
    async def scenario() -> None:
        logger = logging.getLogger("collision_monitor.test.validation")
        service, _, _, _ = make_service(logger=logger)
        with caplog.at_level(logging.WARNING, logger=logger.name):
            result = await service.ingest_payload({"device_id": "broken"})

        assert result.accepted is False
        assert service.rejected_state_count == 1
        assert any(record.message == "state_rejected_validation" for record in caplog.records)

    asyncio.run(scenario())


def test_consumed_invalid_state_is_rejected_without_requeue() -> None:
    async def scenario() -> None:
        service, consumer, _, _ = make_service()
        await consumer.submit({"device_id": "broken"})
        await consumer.close()

        await service.consume_states()

        assert consumer.deliveries[0].settlement == "rejected"
        assert service.rejected_state_count == 1

    asyncio.run(scenario())


def test_malformed_delivery_does_not_stop_following_valid_ingestion() -> None:
    async def scenario() -> None:
        service, consumer, publisher, _ = make_service()
        await consumer.submit({"device_id": "broken"})
        await consumer.submit(state_payload("robot-valid", x=10.0, next_x=11.0))
        await consumer.close()

        await service.consume_states()
        result = await service.run_tick()

        assert tuple(delivery.settlement for delivery in consumer.deliveries) == (
            "rejected",
            "acknowledged",
        )
        assert service.rejected_state_count == 1
        assert service.state_store.robot_ids == ("robot-valid",)
        assert tuple(message.device_id for message in publisher.published) == ("robot-valid",)
        assert result.publication_complete is True

    asyncio.run(scenario())


def test_out_of_order_state_is_rejected_but_equal_timestamp_correction_is_used() -> None:
    async def scenario() -> None:
        service, _, publisher, _ = make_service()
        newest = await service.ingest_payload(
            state_payload(
                "robot-a",
                timestamp=NOW_MS + 1_000,
                x=10.0,
                next_x=11.0,
            )
        )
        older = await service.ingest_payload(
            state_payload("robot-a", timestamp=NOW_MS, x=-10.0, next_x=-9.0)
        )
        correction = await service.ingest_payload(
            state_payload(
                "robot-a",
                timestamp=NOW_MS + 1_000,
                x=20.0,
                next_x=21.0,
            )
        )

        result = await service.run_tick()

        assert newest.accepted is True
        assert older.accepted is False
        assert older.reason == "older_source_timestamp"
        assert correction.accepted is True
        assert result.trace["robots"][0]["source_state_timestamp"] == NOW_MS + 1_000
        assert result.fleet_decision.tick_metadata["observed_states"][0]["current_pose"][0] == 20.0
        assert publisher.published[0].source_state_timestamp == NOW_MS + 1_000

    asyncio.run(scenario())


def test_periodic_loop_uses_configured_one_hertz_interval() -> None:
    async def scenario() -> None:
        stop_event = asyncio.Event()
        delays: list[float] = []

        async def stop_after_delay(delay: float) -> None:
            delays.append(delay)
            stop_event.set()

        service, _, _, _ = make_service(sleeper=stop_after_delay)

        await service.decision_loop(stop_event)

        assert delays == [1.0]

    asyncio.run(scenario())
