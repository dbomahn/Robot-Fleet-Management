"""Deterministic in-memory transport adapters for tests and local simulation."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Mapping
from typing import Any

from collision_monitor.transport.base import ActionMessage

_CLOSE = object()


class InMemoryStateDelivery:
    """An inspectable state delivery with single-settlement enforcement."""

    def __init__(self, payload: Mapping[str, Any]) -> None:
        self._payload = dict(payload)
        self._settlement: str | None = None
        self._settled_event = asyncio.Event()

    @property
    def payload(self) -> Mapping[str, Any]:
        """Return a copied decoded payload."""
        return dict(self._payload)

    @property
    def settlement(self) -> str | None:
        """Return ``acknowledged``, ``rejected`` or None while unsettled."""
        return self._settlement

    async def acknowledge(self) -> None:
        """Record successful application acceptance."""
        if self._settlement is not None:
            raise RuntimeError("state delivery has already been settled")
        self._settlement = "acknowledged"
        self._settled_event.set()

    async def reject(self, *, requeue: bool = False) -> None:
        """Record rejection and reject in-memory requeue requests."""
        if self._settlement is not None:
            raise RuntimeError("state delivery has already been settled")
        if requeue:
            raise ValueError("in-memory state delivery does not support requeue")
        self._settlement = "rejected"
        self._settled_event.set()

    async def wait_settled(self) -> str:
        """Wait until the service acknowledges or rejects this delivery."""
        await self._settled_event.wait()
        if self._settlement is None:
            raise RuntimeError("settlement event completed without a settlement")
        return self._settlement


class InMemoryStateConsumer:
    """Queue decoded state payloads through the StateConsumer protocol."""

    def __init__(self) -> None:
        self._queue: asyncio.Queue[InMemoryStateDelivery | object] = asyncio.Queue()
        self._closed = False
        self._deliveries: list[InMemoryStateDelivery] = []

    @property
    def deliveries(self) -> tuple[InMemoryStateDelivery, ...]:
        """Return submitted deliveries for settlement assertions."""
        return tuple(self._deliveries)

    async def submit(self, payload: Mapping[str, Any]) -> InMemoryStateDelivery:
        """Queue a copied payload, preserving submission order."""
        if self._closed:
            raise RuntimeError("cannot submit to a closed in-memory state consumer")
        delivery = InMemoryStateDelivery(payload)
        self._deliveries.append(delivery)
        await self._queue.put(delivery)
        return delivery

    async def close(self) -> None:
        """End iteration after every already queued payload is consumed."""
        if not self._closed:
            self._closed = True
            await self._queue.put(_CLOSE)

    async def receive_states(self) -> AsyncIterator[InMemoryStateDelivery]:
        """Yield queued deliveries until the close marker is reached."""
        while True:
            item = await self._queue.get()
            if item is _CLOSE:
                return
            if not isinstance(item, InMemoryStateDelivery):
                raise RuntimeError("in-memory state queue contained an invalid item")
            yield item


class InMemoryActionPublisher:
    """Record successful actions and support deterministic injected failures."""

    def __init__(self, *, failures_before_success: Mapping[str, int] | None = None) -> None:
        configured_failures = dict(failures_before_success or {})
        if any(count < 0 for count in configured_failures.values()):
            raise ValueError("configured publication failure counts must not be negative")
        self._remaining_failures = configured_failures
        self._attempt_counts: dict[tuple[str, str, str], int] = {}
        self._published: list[ActionMessage] = []

    @property
    def published(self) -> tuple[ActionMessage, ...]:
        """Return successful messages in publication order."""
        return tuple(self._published)

    @property
    def attempt_counts(self) -> Mapping[tuple[str, str, str], int]:
        """Return attempts keyed by ``(run_id, device_id, tick_id)``."""
        return dict(self._attempt_counts)

    async def publish_action(self, message: ActionMessage) -> None:
        """Fail a configured number of attempts, then record the action."""
        key = message.idempotency_key
        self._attempt_counts[key] = self._attempt_counts.get(key, 0) + 1
        remaining = self._remaining_failures.get(message.device_id, 0)
        if remaining > 0:
            self._remaining_failures[message.device_id] = remaining - 1
            raise RuntimeError(f"injected publication failure for robot {message.device_id!r}")
        self._published.append(message)
