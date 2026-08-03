"""Optional live RabbitMQ integration test; disabled unless explicitly requested."""

from __future__ import annotations

import asyncio
import json
import os
import uuid

import aio_pika
import pytest

from collision_monitor.config import MonitorConfig
from collision_monitor.models import Action, DecisionSource
from collision_monitor.transport.base import ActionMessage
from collision_monitor.transport.rabbitmq import RabbitMQTransport

pytestmark = [
    pytest.mark.rabbitmq,
    pytest.mark.skipif(
        os.environ.get("RUN_RABBITMQ_INTEGRATION") != "1",
        reason="set RUN_RABBITMQ_INTEGRATION=1 with a reachable RabbitMQ broker",
    ),
]


def test_persistent_action_round_trip_against_live_rabbitmq() -> None:
    async def scenario() -> None:
        suffix = uuid.uuid4().hex
        queue_prefix = f"collision_monitor.integration.actions.{suffix}."
        config = MonitorConfig(
            rabbitmq_url=os.environ.get(
                "COLLISION_MONITOR_RABBITMQ_URL",
                "amqp://guest:guest@localhost/",
            ),
            rabbitmq_action_queue_prefix=queue_prefix,
        )
        transport = RabbitMQTransport(config)
        message = ActionMessage(
            run_id=f"integration-run-{suffix}",
            device_id="robot-a",
            action=Action.RESUME,
            tick_id="integration-tick",
            decision_timestamp=1_720_000_000_100,
            source_state_timestamp=1_720_000_000_000,
            reason_codes=("INTEGRATION_TEST",),
            reason_context=("Live broker round trip.",),
            decision_source=DecisionSource.POLICY,
            grant_active=False,
        )
        connection = await aio_pika.connect_robust(config.rabbitmq_url)
        channel = await connection.channel()
        queue_name = transport.action_queue_name(message.device_id)
        try:
            await transport.publish_action(message)
            queue = await channel.declare_queue(queue_name, durable=True)
            incoming = await queue.get(timeout=5.0, fail=False)

            assert incoming is not None
            assert json.loads(incoming.body.decode("utf-8")) == message.model_dump(mode="json")
            await incoming.ack()
        finally:
            await transport.close()
            await channel.queue_delete(queue_name)
            await connection.close()

    asyncio.run(scenario())
