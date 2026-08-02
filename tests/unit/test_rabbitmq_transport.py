"""Mocked unit tests for the aio-pika RabbitMQ transport."""

from __future__ import annotations

import asyncio
import json
from collections.abc import Mapping
from typing import Any

from pamqp.commands import Basic

from collision_monitor.config import MonitorConfig
from collision_monitor.models import Action, DecisionSource
from collision_monitor.transport.base import ActionMessage
from collision_monitor.transport.rabbitmq import RabbitMQTransport


class FakeIncomingMessage:
    """Minimal incoming-message double with settlement recording."""

    def __init__(self, body: bytes) -> None:
        self.body = body
        self.acknowledged = False
        self.rejected_requeue: bool | None = None

    async def ack(self) -> None:
        """Record acknowledgement."""
        self.acknowledged = True

    async def reject(self, *, requeue: bool = False) -> None:
        """Record rejection and the requeue flag."""
        self.rejected_requeue = requeue


class FakeExchange:
    """Record published messages and return a broker acknowledgement."""

    def __init__(self) -> None:
        self.published: list[tuple[Any, str, bool]] = []

    async def publish(
        self,
        message: Any,
        routing_key: str,
        *,
        mandatory: bool,
    ) -> Basic.Ack:
        """Record one publish call."""
        self.published.append((message, routing_key, mandatory))
        return Basic.Ack()


class FakeQueue:
    """Record queue bindings to the configured direct exchange."""

    def __init__(self) -> None:
        self.bindings: list[tuple[FakeExchange, str]] = []

    async def bind(self, exchange: FakeExchange, *, routing_key: str) -> None:
        """Record one idempotent binding."""
        self.bindings.append((exchange, routing_key))


class FakeChannel:
    """Minimal publisher-channel double."""

    def __init__(self) -> None:
        self.is_closed = False
        self.default_exchange = FakeExchange()
        self.exchange = FakeExchange()
        self.exchange_declarations: list[tuple[str, Any, bool, bool]] = []
        self.queue_declarations: list[tuple[str, bool, bool]] = []
        self.queues: list[FakeQueue] = []
        self.prefetch_counts: list[int] = []

    async def set_qos(self, *, prefetch_count: int) -> None:
        """Record consumer prefetch configuration."""
        self.prefetch_counts.append(prefetch_count)

    async def declare_exchange(
        self,
        name: str,
        *,
        type: Any,
        durable: bool,
        auto_delete: bool,
    ) -> FakeExchange:
        """Record a durable direct-exchange declaration."""
        self.exchange_declarations.append((name, type, durable, auto_delete))
        return self.exchange

    async def declare_queue(
        self,
        name: str,
        *,
        durable: bool,
        auto_delete: bool,
    ) -> FakeQueue:
        """Record an idempotent queue declaration."""
        self.queue_declarations.append((name, durable, auto_delete))
        queue = FakeQueue()
        self.queues.append(queue)
        return queue

    async def close(self) -> None:
        """Close the fake channel."""
        self.is_closed = True


class FakeConnection:
    """Minimal robust-connection double returning a publisher channel."""

    def __init__(self) -> None:
        self.is_closed = False
        self.channel_instance = FakeChannel()
        self.channel_options: list[Mapping[str, Any]] = []

    async def channel(self, **kwargs: Any) -> FakeChannel:
        """Record requested publisher-confirm options."""
        self.channel_options.append(dict(kwargs))
        return self.channel_instance

    async def close(self) -> None:
        """Close the fake connection."""
        self.is_closed = True


def action_message() -> ActionMessage:
    """Build a complete action-schema example."""
    return ActionMessage(
        device_id="robot-a",
        action=Action.RESUME,
        tick_id="tick-00000001",
        decision_timestamp=1_720_000_000_100,
        source_state_timestamp=1_720_000_000_000,
        reason_codes=("CP_SAT_COMPONENT_DECISION",),
        reason_context=("The bounded exact optimiser selected Resume.",),
        decision_source=DecisionSource.CP_SAT,
        grant_active=True,
    )


def test_valid_json_delivery_is_manually_acknowledged_after_yield() -> None:
    async def scenario() -> None:
        transport = RabbitMQTransport(MonitorConfig())
        incoming = FakeIncomingMessage(b'{"device_id":"robot-a"}')

        delivery = await transport._decode_delivery(incoming)  # noqa: SLF001

        assert delivery is not None
        assert delivery.payload == {"device_id": "robot-a"}
        assert incoming.acknowledged is False
        await delivery.acknowledge()
        assert incoming.acknowledged is True

    asyncio.run(scenario())


def test_malformed_utf8_and_non_object_json_are_rejected_without_requeue() -> None:
    async def scenario() -> None:
        transport = RabbitMQTransport(MonitorConfig())
        malformed = FakeIncomingMessage(b"\xff")
        non_object = FakeIncomingMessage(b"[]")

        assert await transport._decode_delivery(malformed) is None  # noqa: SLF001
        assert await transport._decode_delivery(non_object) is None  # noqa: SLF001
        assert malformed.rejected_requeue is False
        assert non_object.rejected_requeue is False

    asyncio.run(scenario())


def test_publish_declares_durable_robot_queue_and_confirms_utf8_json() -> None:
    async def scenario() -> None:
        connection = FakeConnection()

        async def connect(*args: Any, **kwargs: Any) -> Any:
            return connection

        transport = RabbitMQTransport(
            MonitorConfig(rabbitmq_action_queue_prefix="actions."),
            connect_factory=connect,
        )
        message = action_message()

        await transport.publish_action(message)

        channel = connection.channel_instance
        assert connection.channel_options == [
            {"publisher_confirms": True, "on_return_raises": True}
        ]
        assert channel.exchange_declarations == [
            ("collision_monitor", "direct", True, False)
        ]
        assert channel.queue_declarations == [("actions.robot-a", True, False)]
        assert channel.queues[0].bindings == [(channel.exchange, "actions.robot-a")]
        published, routing_key, mandatory = channel.exchange.published[0]
        assert routing_key == "actions.robot-a"
        assert mandatory is True
        assert published.content_type == "application/json"
        assert published.content_encoding == "utf-8"
        assert published.message_id == "tick-00000001:robot-a"
        assert json.loads(published.body.decode("utf-8")) == message.model_dump(mode="json")

    asyncio.run(scenario())


def test_consumer_declares_and_binds_one_durable_shared_queue() -> None:
    async def scenario() -> None:
        connection = FakeConnection()

        async def connect(*args: Any, **kwargs: Any) -> Any:
            return connection

        config = MonitorConfig(
            rabbitmq_exchange_name="fleet",
            rabbitmq_state_queue="fleet.states",
            rabbitmq_state_routing_key="fleet.state",
            rabbitmq_prefetch_count=1,
        )
        transport = RabbitMQTransport(config, connect_factory=connect)

        channel = await transport._consumer_channel_or_open()  # noqa: SLF001

        assert connection.channel_options == [{"publisher_confirms": False}]
        assert channel.prefetch_counts == [1]  # type: ignore[attr-defined]
        assert channel.exchange_declarations == [  # type: ignore[attr-defined]
            ("fleet", "direct", True, False)
        ]
        assert channel.queue_declarations == [  # type: ignore[attr-defined]
            ("fleet.states", True, False)
        ]
        fake_channel = connection.channel_instance
        assert fake_channel.queues[0].bindings == [(fake_channel.exchange, "fleet.state")]

    asyncio.run(scenario())


def test_initial_connection_retries_with_bounded_exponential_delay() -> None:
    async def scenario() -> None:
        attempts = 0
        delays: list[float] = []
        connection = FakeConnection()

        async def connect(*args: Any, **kwargs: Any) -> Any:
            nonlocal attempts
            attempts += 1
            if attempts < 4:
                raise OSError("broker unavailable")
            return connection

        async def sleeper(delay: float) -> None:
            delays.append(delay)

        config = MonitorConfig(
            rabbitmq_reconnect_initial_delay_seconds=0.5,
            rabbitmq_reconnect_max_delay_seconds=1.0,
        )
        transport = RabbitMQTransport(
            config,
            connect_factory=connect,
            sleeper=sleeper,
        )

        connected = await transport._connection_or_connect()  # noqa: SLF001

        assert connected is connection
        assert attempts == 4
        assert delays == [0.5, 1.0, 1.0]

    asyncio.run(scenario())


def test_transport_close_is_clean_and_idempotent() -> None:
    async def scenario() -> None:
        connection = FakeConnection()

        async def connect(*args: Any, **kwargs: Any) -> Any:
            return connection

        transport = RabbitMQTransport(MonitorConfig(), connect_factory=connect)
        await transport.publish_action(action_message())

        await transport.close()
        await transport.close()

        assert connection.channel_instance.is_closed is True
        assert connection.is_closed is True

    asyncio.run(scenario())
