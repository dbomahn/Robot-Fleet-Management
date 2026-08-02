"""Async application orchestration around the pure collision decision engine."""

from __future__ import annotations

import asyncio
import logging
import time
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

from pydantic import ValidationError

from collision_monitor.config import MonitorConfig
from collision_monitor.engine import CollisionDecisionEngine
from collision_monitor.logging_utils import log_json_event
from collision_monitor.models import (
    FleetDecision,
    RobotDecision,
    RobotSnapshot,
    RobotState,
)
from collision_monitor.state_store import (
    FleetStateSnapshot,
    LatestStateStore,
)
from collision_monitor.transport.base import (
    ActionMessage,
    ActionPublisher,
    StateConsumer,
)

MonotonicClock = Callable[[], float]
EpochClock = Callable[[], int]
AsyncSleeper = Callable[[float], Awaitable[None]]


def _epoch_clock_ms() -> int:
    """Return the current epoch time in integer milliseconds."""
    return time.time_ns() // 1_000_000


@dataclass(frozen=True, slots=True)
class IngestionResult:
    """Outcome of validating and storing one consumed state payload."""

    accepted: bool
    robot_id: str | None
    reason: str


@dataclass(frozen=True, slots=True)
class PublicationFailure:
    """One robot action that exhausted its bounded publication attempts."""

    robot_id: str
    tick_id: str
    attempts: int
    error: str


@dataclass(frozen=True, slots=True)
class ServiceTickResult:
    """One decision tick, its output messages and publication outcome."""

    fleet_decision: FleetDecision
    action_messages: tuple[ActionMessage, ...]
    publication_failures: tuple[PublicationFailure, ...]
    trace: Mapping[str, Any]

    def __post_init__(self) -> None:
        object.__setattr__(self, "trace", MappingProxyType(dict(self.trace)))

    @property
    def publication_complete(self) -> bool:
        """Return whether every robot action was published successfully."""
        return not self.publication_failures


class CollisionMonitorService:
    """Continuously aggregate states and publish one complete action tick."""

    def __init__(
        self,
        config: MonitorConfig,
        state_consumer: StateConsumer,
        action_publisher: ActionPublisher,
        *,
        engine: CollisionDecisionEngine | None = None,
        state_store: LatestStateStore | None = None,
        monotonic_clock: MonotonicClock = time.monotonic,
        epoch_clock_ms: EpochClock = _epoch_clock_ms,
        sleeper: AsyncSleeper = asyncio.sleep,
        logger: logging.Logger | None = None,
    ) -> None:
        self._config = config
        self._consumer = state_consumer
        self._publisher = action_publisher
        self._engine = engine or CollisionDecisionEngine(config)
        self._store = state_store or LatestStateStore()
        self._monotonic_clock = monotonic_clock
        self._epoch_clock_ms = epoch_clock_ms
        self._sleeper = sleeper
        self._logger = logger or logging.getLogger("collision_monitor.service")
        self._tick_sequence = 0
        self._rejected_state_count = 0

    @property
    def state_store(self) -> LatestStateStore:
        """Return the current latest-state store for inspection."""
        return self._store

    @property
    def rejected_state_count(self) -> int:
        """Return the number of invalid or superseded input states."""
        return self._rejected_state_count

    def _next_tick_id(self) -> str:
        """Return a deterministic process-local tick identifier."""
        self._tick_sequence += 1
        return f"tick-{self._tick_sequence:08d}"

    async def ingest_payload(self, payload: Mapping[str, Any]) -> IngestionResult:
        """Validate and store one decoded input payload with rejection logging."""
        received_monotonic_seconds = self._monotonic_clock()
        try:
            state = RobotState.model_validate(payload)
        except ValidationError as exc:
            self._rejected_state_count += 1
            robot_id = payload.get("device_id")
            safe_robot_id = robot_id if isinstance(robot_id, str) else None
            log_json_event(
                self._logger,
                logging.WARNING,
                "state_rejected_validation",
                {
                    "robot_id": safe_robot_id,
                    "error": str(exc),
                    "rejected_state_count": self._rejected_state_count,
                },
            )
            return IngestionResult(
                accepted=False,
                robot_id=safe_robot_id,
                reason="input_validation_failed",
            )

        update = self._store.update(
            state,
            received_monotonic_seconds=received_monotonic_seconds,
        )
        if not update.accepted:
            self._rejected_state_count += 1
            log_json_event(
                self._logger,
                logging.WARNING,
                "state_rejected_superseded",
                {
                    "robot_id": update.robot_id,
                    "source_state_timestamp": state.timestamp,
                    "reason": update.reason,
                    "rejected_state_count": self._rejected_state_count,
                },
            )
        return IngestionResult(
            accepted=update.accepted,
            robot_id=update.robot_id,
            reason=update.reason,
        )

    async def consume_states(self) -> None:
        """Consume and validate states continuously until the adapter closes."""
        async for delivery in self._consumer.receive_states():
            result = await self.ingest_payload(delivery.payload)
            if result.accepted:
                await delivery.acknowledge()
            else:
                await delivery.reject(requeue=False)

    def _take_state_snapshot(self) -> FleetStateSnapshot:
        """Capture one immutable view using the two injected clocks once each."""
        captured_monotonic_seconds = self._monotonic_clock()
        captured_epoch_ms = self._epoch_clock_ms()
        return self._store.snapshot(
            captured_monotonic_seconds=captured_monotonic_seconds,
            captured_epoch_ms=captured_epoch_ms,
            stale_timeout_seconds=self._config.stale_timeout_seconds,
            pose_tolerance=self._config.pose_tolerance,
        )

    @staticmethod
    def _action_message(
        decision: RobotDecision,
        snapshot_by_robot: Mapping[str, RobotSnapshot],
        *,
        decision_timestamp: int,
        grant_active: bool,
    ) -> ActionMessage:
        """Build one validated output payload from a final engine decision."""
        snapshot = snapshot_by_robot[decision.robot_id]
        raw_context = decision.diagnostic_metadata.get("reason_context", ())
        reason_context = tuple(str(item) for item in raw_context)
        return ActionMessage(
            device_id=decision.robot_id,
            action=decision.action,
            tick_id=decision.tick_id,
            decision_timestamp=decision_timestamp,
            source_state_timestamp=snapshot.state.timestamp,
            reason_codes=decision.reason_codes,
            reason_context=reason_context,
            decision_source=decision.decision_source,
            grant_active=grant_active,
        )

    async def _publish_with_retry(
        self,
        message: ActionMessage,
    ) -> PublicationFailure | None:
        """Publish one action with bounded retries and explicit final failure."""
        for attempt in range(1, self._config.publication_maximum_attempts + 1):
            try:
                await self._publisher.publish_action(message)
            except Exception as exc:
                log_json_event(
                    self._logger,
                    logging.ERROR,
                    "action_publication_attempt_failed",
                    {
                        "tick_id": message.tick_id,
                        "robot_id": message.device_id,
                        "attempt": attempt,
                        "maximum_attempts": self._config.publication_maximum_attempts,
                        "error": str(exc),
                    },
                )
                if attempt < self._config.publication_maximum_attempts:
                    await self._sleeper(self._config.publication_retry_delay_seconds)
                    continue
                return PublicationFailure(
                    robot_id=message.device_id,
                    tick_id=message.tick_id,
                    attempts=attempt,
                    error=str(exc),
                )
            else:
                return None
        raise RuntimeError("publication retry loop exhausted without an outcome")

    def _build_trace(
        self,
        tick_id: str,
        state_snapshot: FleetStateSnapshot,
        fleet_decision: FleetDecision,
        messages: tuple[ActionMessage, ...],
        failures: tuple[PublicationFailure, ...],
    ) -> Mapping[str, Any]:
        """Build the stable per-tick JSON-compatible diagnostic object."""
        metadata = fleet_decision.tick_metadata
        component_membership = tuple(
            (robot_id, tuple(component))
            for component in metadata["connected_components"]
            for robot_id in component
        )
        solver_trace = tuple(
            {
                **dict(component),
                "wall_time_limit_seconds": self._config.cp_sat_time_limit_seconds,
            }
            for component in metadata["component_diagnostics"]
        )
        active_grants = tuple(
            {
                "robot_id": robot_id,
                "acquisition_tick": record.acquisition_tick,
                "last_seen_tick": record.last_seen_tick,
                "clearance_counter": record.clearance_counter,
            }
            for robot_id, record in sorted(self._engine.grant_manager.active_grants.items())
        )
        return {
            "tick_id": tick_id,
            "tick_epoch_ms": state_snapshot.captured_epoch_ms,
            "tick_monotonic_seconds": state_snapshot.captured_monotonic_seconds,
            "robots": tuple(
                {
                    "robot_id": snapshot.state.device_id,
                    "source_state_timestamp": snapshot.state.timestamp,
                    "source_state_age_ms": state_snapshot.source_state_ages_ms[
                        snapshot.state.device_id
                    ],
                    "stale": snapshot.stale,
                }
                for snapshot in state_snapshot.snapshots
            ),
            "component_membership": component_membership,
            "forbidden_action_combinations": metadata["constraints"],
            "priority_breakdown": metadata["priority_scores"],
            "active_grants": active_grants,
            "solver": solver_trace,
            "solver_timing": {
                "wall_time_limit_seconds": self._config.cp_sat_time_limit_seconds,
                "tick_interval_seconds": self._config.tick_interval_seconds,
            },
            "final_actions": tuple(
                {
                    "robot_id": message.device_id,
                    "action": message.action.value,
                    "decision_source": message.decision_source.value,
                    "reason_codes": message.reason_codes,
                }
                for message in messages
            ),
            "safety_validation": {
                "valid": metadata["global_safety_valid"],
                "violations": metadata["final_global_safety_violations"],
                "state_committed": metadata["state_committed"],
            },
            "alarms": metadata["alarms"],
            "publication": {
                "complete": not failures,
                "successful_robot_ids": tuple(
                    message.device_id
                    for message in messages
                    if all(failure.robot_id != message.device_id for failure in failures)
                ),
                "failures": tuple(
                    {
                        "robot_id": failure.robot_id,
                        "attempts": failure.attempts,
                        "error": failure.error,
                    }
                    for failure in failures
                ),
            },
            "detailed_geometry": metadata.get("detailed_geometry", ()),
        }

    async def run_tick(self) -> ServiceTickResult:
        """Decide and publish exactly one action for every currently known robot."""
        tick_id = self._next_tick_id()
        state_snapshot = self._take_state_snapshot()
        fleet_decision = self._engine.decide(
            state_snapshot.snapshots,
            state_snapshot.captured_epoch_ms,
            tick_id,
        )
        snapshot_by_robot = {
            snapshot.state.device_id: snapshot for snapshot in state_snapshot.snapshots
        }
        messages = tuple(
            self._action_message(
                decision,
                snapshot_by_robot,
                decision_timestamp=state_snapshot.captured_epoch_ms,
                grant_active=(
                    decision.robot_id in self._engine.grant_manager.active_grants
                ),
            )
            for decision in fleet_decision.decisions
        )

        failures: list[PublicationFailure] = []
        for message in messages:
            failure = await self._publish_with_retry(message)
            if failure is not None:
                failures.append(failure)
        failure_tuple = tuple(failures)
        trace = self._build_trace(
            tick_id,
            state_snapshot,
            fleet_decision,
            messages,
            failure_tuple,
        )
        log_json_event(self._logger, logging.INFO, "decision_tick", trace)
        if failure_tuple:
            log_json_event(
                self._logger,
                logging.CRITICAL,
                "decision_tick_publication_incomplete",
                {
                    "tick_id": tick_id,
                    "failed_robot_ids": tuple(
                        failure.robot_id for failure in failure_tuple
                    ),
                },
            )
        return ServiceTickResult(
            fleet_decision=fleet_decision,
            action_messages=messages,
            publication_failures=failure_tuple,
            trace=trace,
        )

    async def decision_loop(self, stop_event: asyncio.Event) -> None:
        """Run decisions periodically at the configured interval until stopped."""
        while not stop_event.is_set():
            tick_started = self._monotonic_clock()
            await self.run_tick()
            elapsed = max(self._monotonic_clock() - tick_started, 0.0)
            remaining = max(self._config.tick_interval_seconds - elapsed, 0.0)
            if remaining > 0 and not stop_event.is_set():
                await self._sleeper(remaining)

    async def run(self, stop_event: asyncio.Event) -> None:
        """Run continuous consumption and periodic decisions concurrently."""
        consumer_task = asyncio.create_task(self.consume_states())
        decision_task = asyncio.create_task(self.decision_loop(stop_event))
        stop_task = asyncio.create_task(stop_event.wait())
        try:
            watched: set[asyncio.Task[Any]] = {
                consumer_task,
                decision_task,
                stop_task,
            }
            while True:
                done, _ = await asyncio.wait(
                    watched,
                    return_when=asyncio.FIRST_COMPLETED,
                )
                if stop_task in done:
                    break
                if decision_task in done:
                    exception = decision_task.exception()
                    if exception is not None:
                        raise exception
                    raise RuntimeError("periodic decision loop ended before stop was requested")
                if consumer_task in done:
                    exception = consumer_task.exception()
                    if exception is not None:
                        raise exception
                    watched.remove(consumer_task)
        finally:
            for task in (consumer_task, decision_task, stop_task):
                task.cancel()
            await asyncio.gather(
                consumer_task,
                decision_task,
                stop_task,
                return_exceptions=True,
            )
