"""Unit tests for broker-free in-memory transport adapters."""

from __future__ import annotations

import asyncio

import pytest

from collision_monitor.models import Action, DecisionSource
from collision_monitor.transport.base import ActionMessage
from collision_monitor.transport.memory import (
    InMemoryActionPublisher,
    InMemoryStateConsumer,
)


def action_message(robot_id: str = "robot-a") -> ActionMessage:
    """Build a complete output message for adapter tests."""
    return ActionMessage(
        device_id=robot_id,
        action=Action.RESUME,
        tick_id="tick-00000001",
        decision_timestamp=1_720_000_000_100,
        source_state_timestamp=1_720_000_000_000,
        reason_codes=("ISOLATED_ACTIVE_RESUME",),
        reason_context=("No conflict applies.",),
        decision_source=DecisionSource.POLICY,
        grant_active=False,
    )


def test_state_consumer_preserves_arbitrary_submission_order() -> None:
    async def scenario() -> None:
        consumer = InMemoryStateConsumer()
        await consumer.submit({"device_id": "robot-b"})
        await consumer.submit({"device_id": "robot-a"})
        await consumer.close()

        received = [delivery async for delivery in consumer.receive_states()]

        assert tuple(delivery.payload["device_id"] for delivery in received) == (
            "robot-b",
            "robot-a",
        )

    asyncio.run(scenario())


def test_action_publisher_records_success_and_injected_attempts() -> None:
    async def scenario() -> None:
        publisher = InMemoryActionPublisher(failures_before_success={"robot-a": 1})
        message = action_message()

        with pytest.raises(RuntimeError, match="injected publication failure"):
            await publisher.publish_action(message)
        await publisher.publish_action(message)

        assert publisher.published == (message,)
        assert publisher.attempt_counts[(message.tick_id, message.device_id)] == 2

    asyncio.run(scenario())
