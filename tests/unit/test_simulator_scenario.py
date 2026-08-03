"""Unit tests for simulator transport action waiting and filtering."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from types import TracebackType
from typing import Self

from collision_monitor.config import MonitorConfig
from collision_monitor.models import Action, DecisionSource
from collision_monitor.simulator.scenario import RabbitMQSimulatorTransport
from collision_monitor.transport.base import ActionMessage


class FakeIncomingMessage:
    """Record settlement of one simulated RabbitMQ action delivery."""

    def __init__(self, message: ActionMessage) -> None:
        self.body = message.model_dump_json().encode("utf-8")
        self.acknowledged = False

    async def ack(self) -> None:
        """Record acknowledgement of a valid action."""
        self.acknowledged = True

    async def reject(self, *, requeue: bool = False) -> None:
        """Reject malformed input; this fake only accepts valid input."""
        raise AssertionError(f"valid action was rejected with requeue={requeue}")


class FakeQueueIterator:
    """Provide existing deliveries through the queue-iterator interface."""

    def __init__(self, messages: tuple[FakeIncomingMessage, ...]) -> None:
        self._messages = iter(messages)

    async def __aenter__(self) -> Self:
        """Enter the fake consumer context."""
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Leave the fake consumer context."""

    def __aiter__(self) -> AsyncIterator[FakeIncomingMessage]:
        """Return this asynchronous iterator."""
        return self

    async def __anext__(self) -> FakeIncomingMessage:
        """Yield the next delivery, then wait as a live consumer would."""
        try:
            return next(self._messages)
        except StopIteration:
            await asyncio.Future()
            raise StopAsyncIteration from None


class FakeQueue:
    """Expose RabbitMQ's iterator factory for deterministic deliveries."""

    def __init__(self, messages: tuple[FakeIncomingMessage, ...]) -> None:
        self._messages = messages

    def iterator(self, *, no_ack: bool) -> FakeQueueIterator:
        """Return a manual-acknowledgement action iterator."""
        assert no_ack is False
        return FakeQueueIterator(self._messages)


def action_message(*, timestamp: int) -> ActionMessage:
    """Create one complete simulator-facing action message."""
    return ActionMessage(
        run_id="test-run",
        device_id="robot-a",
        action=Action.RESUME,
        tick_id="tick-00000001",
        decision_timestamp=timestamp + 100,
        source_state_timestamp=timestamp,
        reason_codes=("CP_SAT_COMPONENT_DECISION",),
        reason_context=("The bounded exact optimiser selected Resume.",),
        decision_source=DecisionSource.CP_SAT,
        grant_active=False,
    )


def test_rabbitmq_simulator_waits_past_stale_actions_for_latest_state() -> None:
    async def scenario() -> None:
        transport = RabbitMQSimulatorTransport(
            MonitorConfig(),
            action_timeout_seconds=0.1,
        )
        stale = FakeIncomingMessage(action_message(timestamp=999))
        fresh = FakeIncomingMessage(action_message(timestamp=1_000))
        transport._action_queues["robot-a"] = FakeQueue(  # type: ignore[assignment]  # noqa: SLF001
            (stale, fresh)
        )
        transport._last_state_timestamp["robot-a"] = 1_000  # noqa: SLF001

        result = await transport._receive_fresh_action("robot-a")  # noqa: SLF001

        assert result == action_message(timestamp=1_000)
        assert stale.acknowledged is True
        assert fresh.acknowledged is True

    asyncio.run(scenario())


def test_rabbitmq_simulator_action_wait_is_bounded() -> None:
    async def scenario() -> None:
        transport = RabbitMQSimulatorTransport(
            MonitorConfig(),
            action_timeout_seconds=0.001,
        )
        transport._action_queues["robot-a"] = FakeQueue(())  # type: ignore[assignment]  # noqa: SLF001

        assert await transport._receive_fresh_action("robot-a") is None  # noqa: SLF001

    asyncio.run(scenario())
