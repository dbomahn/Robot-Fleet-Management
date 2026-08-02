"""aio-pika RabbitMQ adapter with explicit settlement and publisher confirms."""

from __future__ import annotations

import asyncio
import json
import logging
from collections.abc import AsyncIterator, Awaitable, Callable, Mapping
from types import MappingProxyType
from typing import Any

import aio_pika
from aio_pika.abc import (
    AbstractChannel,
    AbstractIncomingMessage,
    AbstractRobustConnection,
)
from pamqp.commands import Basic

from collision_monitor.config import MonitorConfig
from collision_monitor.logging_utils import log_json_event
from collision_monitor.transport.base import ActionMessage

ConnectFactory = Callable[..., Awaitable[AbstractRobustConnection]]
AsyncSleeper = Callable[[float], Awaitable[None]]


class RabbitStateDelivery:
    """A decoded RabbitMQ delivery settled explicitly by the service."""

    def __init__(
        self,
        payload: Mapping[str, Any],
        incoming_message: AbstractIncomingMessage,
    ) -> None:
        self._payload = MappingProxyType(dict(payload))
        self._incoming_message = incoming_message
        self._settled = False

    @property
    def payload(self) -> Mapping[str, Any]:
        """Return the immutable decoded JSON object."""
        return self._payload

    async def acknowledge(self) -> None:
        """Acknowledge exactly once after state-store acceptance."""
        if self._settled:
            raise RuntimeError("RabbitMQ state delivery has already been settled")
        await self._incoming_message.ack()
        self._settled = True

    async def reject(self, *, requeue: bool = False) -> None:
        """Reject exactly once, with requeue disabled for invalid input."""
        if self._settled:
            raise RuntimeError("RabbitMQ state delivery has already been settled")
        await self._incoming_message.reject(requeue=requeue)
        self._settled = True


class RabbitMQTransport:
    """Consume a shared state queue and publish to durable per-robot queues."""

    def __init__(
        self,
        config: MonitorConfig,
        *,
        connect_factory: ConnectFactory = aio_pika.connect_robust,
        sleeper: AsyncSleeper = asyncio.sleep,
        logger: logging.Logger | None = None,
    ) -> None:
        self._config = config
        self._connect_factory = connect_factory
        self._sleeper = sleeper
        self._logger = logger or logging.getLogger("collision_monitor.transport.rabbitmq")
        self._connection: AbstractRobustConnection | None = None
        self._consumer_channel: AbstractChannel | None = None
        self._publisher_channel: AbstractChannel | None = None
        self._connection_lock = asyncio.Lock()
        self._consumer_channel_lock = asyncio.Lock()
        self._publisher_channel_lock = asyncio.Lock()
        self._closed = False

    @property
    def state_queue_name(self) -> str:
        """Return the configured shared durable input queue name."""
        return self._config.rabbitmq_state_queue

    def action_queue_name(self, robot_id: str) -> str:
        """Return the durable output queue name for one robot."""
        if not robot_id:
            raise ValueError("robot ID must not be empty")
        return f"{self._config.rabbitmq_action_queue_prefix}{robot_id}"

    async def _connect_with_backoff(self) -> AbstractRobustConnection:
        """Open a robust connection using an exponentially increasing bounded delay."""
        delay = self._config.rabbitmq_reconnect_initial_delay_seconds
        while not self._closed:
            try:
                connection = await self._connect_factory(
                    self._config.rabbitmq_url,
                    timeout=self._config.rabbitmq_connection_timeout_seconds,
                    reconnect_interval=(
                        self._config.rabbitmq_reconnect_initial_delay_seconds
                    ),
                    client_properties={"connection_name": "collision-monitor"},
                )
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log_json_event(
                    self._logger,
                    logging.ERROR,
                    "rabbitmq_connection_failed",
                    {
                        "error": str(exc),
                        "retry_delay_seconds": delay,
                    },
                )
                await self._sleeper(delay)
                delay = min(
                    delay * 2.0,
                    self._config.rabbitmq_reconnect_max_delay_seconds,
                )
                continue
            log_json_event(
                self._logger,
                logging.INFO,
                "rabbitmq_connected",
                {"state_queue": self._config.rabbitmq_state_queue},
            )
            return connection
        raise RuntimeError("RabbitMQ transport closed while connecting")

    async def _connection_or_connect(self) -> AbstractRobustConnection:
        """Return the current open robust connection or establish one."""
        if self._closed:
            raise RuntimeError("RabbitMQ transport is closed")
        connection = self._connection
        if connection is not None and not connection.is_closed:
            return connection
        async with self._connection_lock:
            connection = self._connection
            if connection is None or connection.is_closed:
                connected = await self._connect_with_backoff()
                self._connection = connected
                self._consumer_channel = None
                self._publisher_channel = None
                return connected
            return connection

    async def _consumer_channel_or_open(self) -> AbstractChannel:
        """Open a restored consumer channel and declare the shared queue."""
        channel = self._consumer_channel
        if channel is not None and not channel.is_closed:
            return channel
        async with self._consumer_channel_lock:
            channel = self._consumer_channel
            if channel is None or channel.is_closed:
                connection = await self._connection_or_connect()
                channel = await connection.channel(publisher_confirms=False)
                await channel.set_qos(prefetch_count=self._config.rabbitmq_prefetch_count)
                exchange = await channel.declare_exchange(
                    self._config.rabbitmq_exchange_name,
                    type=aio_pika.ExchangeType.DIRECT,
                    durable=True,
                    auto_delete=False,
                )
                queue = await channel.declare_queue(
                    self._config.rabbitmq_state_queue,
                    durable=True,
                    auto_delete=False,
                )
                await queue.bind(
                    exchange,
                    routing_key=self._config.rabbitmq_state_routing_key,
                )
                self._consumer_channel = channel
            return channel

    async def _publisher_channel_or_open(self) -> AbstractChannel:
        """Open a publisher-confirm channel on the robust connection."""
        channel = self._publisher_channel
        if channel is not None and not channel.is_closed:
            return channel
        async with self._publisher_channel_lock:
            channel = self._publisher_channel
            if channel is None or channel.is_closed:
                connection = await self._connection_or_connect()
                channel = await connection.channel(
                    publisher_confirms=True,
                    on_return_raises=True,
                )
                self._publisher_channel = channel
            return channel

    async def _decode_delivery(
        self,
        incoming_message: AbstractIncomingMessage,
    ) -> RabbitStateDelivery | None:
        """Decode UTF-8 JSON or reject malformed messages without requeue."""
        try:
            decoded = incoming_message.body.decode("utf-8", errors="strict")
            payload = json.loads(decoded)
            if not isinstance(payload, dict):
                raise ValueError("state JSON root must be an object")
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
            await incoming_message.reject(requeue=False)
            log_json_event(
                self._logger,
                logging.WARNING,
                "rabbitmq_state_message_rejected_malformed",
                {
                    "error": str(exc),
                    "body_size_bytes": len(incoming_message.body),
                },
            )
            return None
        return RabbitStateDelivery(payload, incoming_message)

    async def receive_states(self) -> AsyncIterator[RabbitStateDelivery]:
        """Yield one unsettled valid JSON delivery at a time without task fan-out."""
        reconnect_delay = self._config.rabbitmq_reconnect_initial_delay_seconds
        while not self._closed:
            try:
                channel = await self._consumer_channel_or_open()
                exchange = await channel.declare_exchange(
                    self._config.rabbitmq_exchange_name,
                    type=aio_pika.ExchangeType.DIRECT,
                    durable=True,
                    auto_delete=False,
                )
                queue = await channel.declare_queue(
                    self._config.rabbitmq_state_queue,
                    durable=True,
                    auto_delete=False,
                )
                await queue.bind(
                    exchange,
                    routing_key=self._config.rabbitmq_state_routing_key,
                )
                async with queue.iterator(no_ack=False) as iterator:
                    async for incoming_message in iterator:
                        delivery = await self._decode_delivery(incoming_message)
                        if delivery is not None:
                            yield delivery
                        if self._closed:
                            return
                reconnect_delay = self._config.rabbitmq_reconnect_initial_delay_seconds
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                if self._closed:
                    return
                self._consumer_channel = None
                if self._connection is not None and self._connection.is_closed:
                    self._connection = None
                    self._publisher_channel = None
                log_json_event(
                    self._logger,
                    logging.ERROR,
                    "rabbitmq_consumer_interrupted",
                    {
                        "error": str(exc),
                        "retry_delay_seconds": reconnect_delay,
                    },
                )
                await self._sleeper(reconnect_delay)
                reconnect_delay = min(
                    reconnect_delay * 2.0,
                    self._config.rabbitmq_reconnect_max_delay_seconds,
                )

    async def publish_action(self, message: ActionMessage) -> None:
        """Publish persistent explicit UTF-8 JSON with broker confirmation."""
        channel = await self._publisher_channel_or_open()
        queue_name = self.action_queue_name(message.device_id)
        exchange = await channel.declare_exchange(
            self._config.rabbitmq_exchange_name,
            type=aio_pika.ExchangeType.DIRECT,
            durable=True,
            auto_delete=False,
        )
        queue = await channel.declare_queue(
            queue_name,
            durable=True,
            auto_delete=False,
        )
        await queue.bind(exchange, routing_key=queue_name)
        body = message.model_dump_json().encode("utf-8")
        outgoing = aio_pika.Message(
            body=body,
            content_type="application/json",
            content_encoding="utf-8",
            delivery_mode=aio_pika.DeliveryMode.PERSISTENT,
            message_id=f"{message.tick_id}:{message.device_id}",
            type="collision_monitor.action",
            app_id="collision-monitor",
        )
        confirmation = await exchange.publish(
            outgoing,
            routing_key=queue_name,
            mandatory=True,
        )
        if confirmation is None or isinstance(confirmation, (Basic.Nack, Basic.Reject)):
            raise RuntimeError(
                f"RabbitMQ did not confirm action for robot {message.device_id!r}"
            )

    async def close(self) -> None:
        """Close channels and the robust connection idempotently."""
        if self._closed:
            return
        self._closed = True
        for channel in (self._consumer_channel, self._publisher_channel):
            if channel is not None and not channel.is_closed:
                await channel.close()
        if self._connection is not None and not self._connection.is_closed:
            await self._connection.close()
        self._consumer_channel = None
        self._publisher_channel = None
        self._connection = None
