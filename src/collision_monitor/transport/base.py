"""Broker-independent state-consumer and action-publisher contracts."""

from __future__ import annotations

from collections.abc import AsyncIterator, Mapping
from typing import Any, Protocol

from pydantic import BaseModel, ConfigDict, Field

from collision_monitor.models import Action, DecisionSource


class ActionMessage(BaseModel):
    """Validated action payload published to one robot-specific destination."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    device_id: str = Field(min_length=1)
    action: Action
    tick_id: str = Field(min_length=1)
    decision_timestamp: int = Field(ge=0)
    source_state_timestamp: int = Field(ge=0)
    reason_codes: tuple[str, ...] = Field(min_length=1)
    reason_context: tuple[str, ...] = Field(min_length=1)
    decision_source: DecisionSource
    grant_active: bool


class StateDelivery(Protocol):
    """One decoded state payload with explicit broker settlement controls."""

    @property
    def payload(self) -> Mapping[str, Any]:
        """Return the decoded JSON object awaiting application validation."""
        ...

    async def acknowledge(self) -> None:
        """Acknowledge a state accepted into the latest-state store."""
        ...

    async def reject(self, *, requeue: bool = False) -> None:
        """Reject an invalid or superseded state, normally without requeue."""
        ...


class StateConsumer(Protocol):
    """Asynchronously yield decoded but not yet validated state payloads."""

    def receive_states(self) -> AsyncIterator[StateDelivery]:
        """Yield unsettled decoded deliveries until the consumer closes."""
        ...


class ActionPublisher(Protocol):
    """Publish one validated action to its robot-specific destination."""

    async def publish_action(self, message: ActionMessage) -> None:
        """Publish one idempotent tick action or raise on failure."""
        ...
