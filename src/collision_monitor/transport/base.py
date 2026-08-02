"""Transport contracts independent of any message broker."""

from __future__ import annotations

from collections.abc import AsyncIterator, Sequence
from typing import Protocol

from collision_monitor.models import RobotDecision, RobotState


class StateTransport(Protocol):
    """Boundary contract for observing states and publishing actions."""

    def receive_states(self) -> AsyncIterator[RobotState]:
        """Yield validated robot states."""
        ...

    async def publish_decisions(self, decisions: Sequence[RobotDecision]) -> None:
        """Publish one action for each decided robot."""
        ...

