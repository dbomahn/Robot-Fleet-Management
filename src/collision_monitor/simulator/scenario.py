"""YAML scenario loading and in-memory/RabbitMQ simulation orchestration."""

from __future__ import annotations

import asyncio
import json
import time
from collections.abc import Awaitable, Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Protocol, Self

import aio_pika
import yaml
from aio_pika.abc import (
    AbstractChannel,
    AbstractExchange,
    AbstractQueue,
    AbstractRobustConnection,
)
from pydantic import BaseModel, ConfigDict, Field, model_validator

from collision_monitor.config import MonitorConfig
from collision_monitor.models import Pose
from collision_monitor.service import CollisionMonitorService, ServiceTickResult
from collision_monitor.simulator.robot import RobotSimulationSummary, SimulatedRobot
from collision_monitor.transport.base import ActionMessage
from collision_monitor.transport.memory import (
    InMemoryActionPublisher,
    InMemoryStateConsumer,
)

AsyncSleeper = Callable[[float], Awaitable[None]]


class ScenarioPose(BaseModel):
    """Human-readable scenario pose in metres and radians."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    x: float
    y: float
    theta: float

    def to_pose(self) -> Pose:
        """Convert the scenario boundary value to an internal pose."""
        return Pose(x=self.x, y=self.y, theta=self.theta)


class ScenarioRobot(BaseModel):
    """One robot's attributes, initial pose and future path nodes."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    device_id: str = Field(min_length=1)
    initial_pose: ScenarioPose
    path: tuple[ScenarioPose, ...]
    loaded: bool
    battery_level: float = Field(ge=0, le=100)
    deadline_offset_ms: int = Field(ge=0)


class ScenarioDefinition(BaseModel):
    """Validated simulator scenario and its nominal timing metadata."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    name: str = Field(min_length=1)
    description: str = Field(min_length=1)
    nominal_speed_mps: float = Field(gt=0)
    tick_interval_seconds: float = Field(default=1.0, gt=0)
    max_ticks: int = Field(default=30, ge=1)
    resolvable: bool = True
    robots: tuple[ScenarioRobot, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_robot_ids(self) -> Self:
        """Reject repeated device IDs within a scenario."""
        robot_ids = tuple(robot.device_id for robot in self.robots)
        if len(set(robot_ids)) != len(robot_ids):
            raise ValueError("scenario robot device IDs must be unique")
        return self

    def create_robots(self, *, start_epoch_ms: int) -> tuple[SimulatedRobot, ...]:
        """Create mutable deterministic robots in stable device-ID order."""
        return tuple(
            SimulatedRobot(
                robot_id=definition.device_id,
                pose=definition.initial_pose.to_pose(),
                future_path=tuple(node.to_pose() for node in definition.path),
                loaded=definition.loaded,
                battery_level=definition.battery_level,
                deadline_ms=start_epoch_ms + definition.deadline_offset_ms,
            )
            for definition in sorted(self.robots, key=lambda robot: robot.device_id)
        )


def load_scenario(path: str | Path) -> ScenarioDefinition:
    """Load a UTF-8 YAML or JSON scenario through one validated boundary."""
    scenario_path = Path(path)
    try:
        raw_data = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"could not load scenario {scenario_path}: {exc}") from exc
    if not isinstance(raw_data, dict):
        raise ValueError("scenario document root must be a mapping")
    return ScenarioDefinition.model_validate(raw_data)


@dataclass(frozen=True, slots=True)
class SimulatorTickOutcome:
    """Actions and diagnostics returned by one simulator transport tick."""

    actions: Mapping[str, ActionMessage]
    alarms: tuple[str, ...]
    safety_valid: bool | None
    service_result: ServiceTickResult | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "actions", MappingProxyType(dict(self.actions)))


class SimulatorTickTransport(Protocol):
    """Exchange one logical 1 Hz state/action tick with a monitor."""

    async def exchange_tick(
        self,
        state_payloads: Sequence[Mapping[str, Any]],
        robot_ids: Sequence[str],
    ) -> SimulatorTickOutcome:
        """Publish current states and return the latest robot actions."""
        ...

    async def close(self) -> None:
        """Release transport resources."""
        ...


class InMemorySimulatorTransport:
    """Run the real application service against its in-memory adapters."""

    def __init__(
        self,
        config: MonitorConfig,
        *,
        start_epoch_ms: int = 1_720_000_000_000,
    ) -> None:
        self._start_epoch_ms = start_epoch_ms
        self._epoch_ms = start_epoch_ms
        self._monotonic_seconds = 0.0
        self._consumer = InMemoryStateConsumer()
        self._publisher = InMemoryActionPublisher()
        self._service = CollisionMonitorService(
            config,
            self._consumer,
            self._publisher,
            monotonic_clock=lambda: self._monotonic_seconds,
            epoch_clock_ms=lambda: self._epoch_ms,
        )
        self._consumer_task = asyncio.create_task(self._service.consume_states())

    async def exchange_tick(
        self,
        state_payloads: Sequence[Mapping[str, Any]],
        robot_ids: Sequence[str],
    ) -> SimulatorTickOutcome:
        """Settle all state deliveries, run one decision and return its actions."""
        if state_payloads:
            timestamps = tuple(int(payload["timestamp"]) for payload in state_payloads)
            self._epoch_ms = max(timestamps)
            self._monotonic_seconds = max(
                (self._epoch_ms - self._start_epoch_ms) / 1_000.0,
                0.0,
            )
        deliveries = [await self._consumer.submit(payload) for payload in state_payloads]
        for delivery in deliveries:
            settlement = await delivery.wait_settled()
            if settlement != "acknowledged":
                raise RuntimeError("simulator state was rejected by the monitor")
        result = await self._service.run_tick()
        messages = {message.device_id: message for message in result.action_messages}
        missing = set(robot_ids).difference(messages)
        if missing:
            raise RuntimeError(f"monitor omitted simulator actions for robots: {sorted(missing)!r}")
        return SimulatorTickOutcome(
            actions=messages,
            alarms=tuple(str(alarm) for alarm in result.trace["alarms"]),
            safety_valid=bool(result.trace["safety_validation"]["valid"]),
            service_result=result,
        )

    async def close(self) -> None:
        """Close state iteration and await the application consumer task."""
        await self._consumer.close()
        await self._consumer_task


class RabbitMQSimulatorTransport:
    """Publish simulator states and consume actions from a running monitor."""

    def __init__(
        self,
        config: MonitorConfig,
        *,
        action_timeout_seconds: float = 5.0,
    ) -> None:
        if action_timeout_seconds <= 0:
            raise ValueError("action timeout must be greater than zero")
        self._config = config
        self._action_timeout_seconds = action_timeout_seconds
        self._connection: AbstractRobustConnection | None = None
        self._channel: AbstractChannel | None = None
        self._exchange: AbstractExchange | None = None
        self._action_queues: dict[str, AbstractQueue] = {}
        self._last_state_timestamp: dict[str, int] = {}

    async def _open(self, robot_ids: Sequence[str]) -> None:
        """Open robust topology and declare all simulator-facing queues."""
        if self._connection is None or self._connection.is_closed:
            self._connection = await aio_pika.connect_robust(
                self._config.rabbitmq_url,
                timeout=self._config.rabbitmq_connection_timeout_seconds,
            )
            self._channel = await self._connection.channel(publisher_confirms=True)
            self._exchange = await self._channel.declare_exchange(
                self._config.rabbitmq_exchange_name,
                type=aio_pika.ExchangeType.DIRECT,
                durable=True,
                auto_delete=False,
            )
            state_queue = await self._channel.declare_queue(
                self._config.rabbitmq_state_queue,
                durable=True,
                auto_delete=False,
            )
            await state_queue.bind(
                self._exchange,
                routing_key=self._config.rabbitmq_state_routing_key,
            )
        if self._channel is None or self._exchange is None:
            raise RuntimeError("RabbitMQ simulator topology did not initialise")
        for robot_id in sorted(robot_ids):
            if robot_id in self._action_queues:
                continue
            queue_name = f"{self._config.rabbitmq_action_queue_prefix}{robot_id}"
            queue = await self._channel.declare_queue(
                queue_name,
                durable=True,
                auto_delete=False,
            )
            await queue.bind(self._exchange, routing_key=queue_name)
            self._action_queues[robot_id] = queue

    async def exchange_tick(
        self,
        state_payloads: Sequence[Mapping[str, Any]],
        robot_ids: Sequence[str],
    ) -> SimulatorTickOutcome:
        """Publish persistent states, then await one confirmed action per robot."""
        await self._open(robot_ids)
        if self._exchange is None:
            raise RuntimeError("RabbitMQ simulator exchange is unavailable")
        for payload in state_payloads:
            robot_id = str(payload["device_id"])
            self._last_state_timestamp[robot_id] = int(payload["timestamp"])
            body = json.dumps(
                dict(payload),
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
            await self._exchange.publish(
                aio_pika.Message(
                    body=body,
                    content_type="application/json",
                    content_encoding="utf-8",
                    delivery_mode=aio_pika.DeliveryMode.PERSISTENT,
                    type="collision_monitor.state",
                    app_id="collision-monitor-simulator",
                ),
                routing_key=self._config.rabbitmq_state_routing_key,
                mandatory=True,
            )

        actions: dict[str, ActionMessage] = {}
        alarms: list[str] = []
        for robot_id in sorted(robot_ids):
            action_message = await self._receive_fresh_action(robot_id)
            if action_message is None:
                continue
            actions[robot_id] = action_message
            alarms.extend(
                code
                for code in action_message.reason_codes
                if (
                    "FAIL_SAFE" in code
                    or "UNRESOLVABLE" in code
                    or "SAFETY_VALIDATION_FAILED" in code
                    or "GEOMETRY_REMAINS_UNSAFE" in code
                )
            )
        return SimulatorTickOutcome(
            actions=actions,
            alarms=tuple(dict.fromkeys(alarms)),
            safety_valid=None,
        )

    async def _receive_fresh_action(self, robot_id: str) -> ActionMessage | None:
        """Wait for one action matching the robot's latest published state."""
        queue = self._action_queues[robot_id]
        minimum_timestamp = self._last_state_timestamp.get(robot_id, 0)
        try:
            async with asyncio.timeout(self._action_timeout_seconds):
                async with queue.iterator(no_ack=False) as iterator:
                    async for incoming in iterator:
                        try:
                            action_message = ActionMessage.model_validate_json(incoming.body)
                        except ValueError:
                            await incoming.reject(requeue=False)
                            continue
                        await incoming.ack()
                        if action_message.device_id != robot_id:
                            continue
                        if action_message.source_state_timestamp < minimum_timestamp:
                            continue
                        return action_message
        except TimeoutError:
            return None
        return None

    async def close(self) -> None:
        """Close the RabbitMQ simulator channel and robust connection."""
        if self._channel is not None and not self._channel.is_closed:
            await self._channel.close()
        if self._connection is not None and not self._connection.is_closed:
            await self._connection.close()
        self._channel = None
        self._connection = None
        self._exchange = None
        self._action_queues.clear()


@dataclass(frozen=True, slots=True)
class SimulationResult:
    """Deterministic scenario result and end-of-run summary data."""

    scenario_name: str
    nominal_speed_mps: float
    ticks: int
    completed: bool
    robots: tuple[RobotSimulationSummary, ...]
    alarms: tuple[str, ...]
    tick_outcomes: tuple[SimulatorTickOutcome, ...]

    @property
    def total_pauses(self) -> int:
        """Return all Pause commands observed during the run."""
        return sum(robot.pauses for robot in self.robots)

    @property
    def total_resumes(self) -> int:
        """Return all Resume commands observed during the run."""
        return sum(robot.resumes for robot in self.robots)

    def as_dict(self) -> Mapping[str, Any]:
        """Return a JSON-compatible end-of-run summary."""
        return {
            "scenario": self.scenario_name,
            "nominal_speed_mps": self.nominal_speed_mps,
            "movement_semantics": "one_path_node_per_tick",
            "ticks": self.ticks,
            "pauses": self.total_pauses,
            "resumes": self.total_resumes,
            "completed": self.completed,
            "robots": tuple(
                {
                    "device_id": robot.robot_id,
                    "pauses": robot.pauses,
                    "resumes": robot.resumes,
                    "nodes_advanced": robot.nodes_advanced,
                    "goal_reached": robot.goal_reached,
                    "goal_tick": robot.goal_tick,
                }
                for robot in self.robots
            ),
            "alarms": self.alarms,
        }


async def run_scenario(
    scenario: ScenarioDefinition,
    transport: SimulatorTickTransport,
    *,
    start_epoch_ms: int = 1_720_000_000_000,
    maximum_ticks: int | None = None,
    realtime: bool = False,
    sleeper: AsyncSleeper = asyncio.sleep,
    monotonic_clock: Callable[[], float] = time.monotonic,
) -> SimulationResult:
    """Run a scenario with deterministic logical time and optional real pacing."""
    robots = scenario.create_robots(start_epoch_ms=start_epoch_ms)
    robot_by_id = {robot.robot_id: robot for robot in robots}
    robot_ids = tuple(sorted(robot_by_id))
    tick_limit = scenario.max_ticks if maximum_ticks is None else maximum_ticks
    if tick_limit < 1:
        raise ValueError("maximum simulation ticks must be positive")
    tick_duration_ms = int(scenario.tick_interval_seconds * 1_000)
    outcomes: list[SimulatorTickOutcome] = []
    alarms: list[str] = []
    ticks_run = 0

    try:
        for tick in range(1, tick_limit + 1):
            tick_started = monotonic_clock()
            ticks_run = tick
            timestamp_ms = start_epoch_ms + (tick - 1) * tick_duration_ms
            for robot in robots:
                robot.advance_for_tick(tick)

            publishing_robots = tuple(robot for robot in robots if robot.should_publish_state)
            payloads = tuple(
                robot.state_payload(timestamp_ms=timestamp_ms) for robot in publishing_robots
            )
            outcome = await transport.exchange_tick(payloads, robot_ids)
            outcomes.append(outcome)
            for robot in publishing_robots:
                robot.mark_state_published()
            for robot_id, action_message in outcome.actions.items():
                robot = robot_by_id[robot_id]
                if not robot.at_goal:
                    robot.receive_action(action_message.action)
            alarms.extend(outcome.alarms)

            if all(robot.at_goal for robot in robots):
                break
            if realtime:
                elapsed = max(monotonic_clock() - tick_started, 0.0)
                await sleeper(max(scenario.tick_interval_seconds - elapsed, 0.0))
    finally:
        await transport.close()

    summaries = tuple(robot.summary() for robot in robots)
    return SimulationResult(
        scenario_name=scenario.name,
        nominal_speed_mps=scenario.nominal_speed_mps,
        ticks=ticks_run,
        completed=all(robot.goal_reached for robot in summaries),
        robots=summaries,
        alarms=tuple(dict.fromkeys(alarms)),
        tick_outcomes=tuple(outcomes),
    )


async def run_in_memory_scenario(
    scenario: ScenarioDefinition,
    *,
    config: MonitorConfig | None = None,
    start_epoch_ms: int = 1_720_000_000_000,
    maximum_ticks: int | None = None,
) -> SimulationResult:
    """Run the scenario deterministically through the real service and engine."""
    selected_config = config or MonitorConfig(
        tick_interval_seconds=scenario.tick_interval_seconds,
    )
    transport = InMemorySimulatorTransport(
        selected_config,
        start_epoch_ms=start_epoch_ms,
    )
    return await run_scenario(
        scenario,
        transport,
        start_epoch_ms=start_epoch_ms,
        maximum_ticks=maximum_ticks,
        realtime=False,
    )
