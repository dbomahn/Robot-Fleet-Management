"""Structured standard-library logging helpers."""

from __future__ import annotations

import json
import logging
from collections.abc import Mapping
from datetime import UTC, datetime
from typing import Any

_STRUCTURED_DATA_ATTRIBUTE = "collision_monitor_data"


class JsonFormatter(logging.Formatter):
    """Render each log record as one deterministic JSON object."""

    def format(self, record: logging.LogRecord) -> str:
        """Serialise a record without embedding free-form prefixes."""
        payload: dict[str, Any] = {
            "timestamp": datetime.fromtimestamp(record.created, tz=UTC).isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }
        structured_data = getattr(record, _STRUCTURED_DATA_ATTRIBUTE, None)
        if structured_data is not None:
            if not isinstance(structured_data, Mapping):
                raise TypeError("structured log data must be a mapping")
            payload["data"] = dict(structured_data)
        if record.exc_info:
            payload["exception"] = self.formatException(record.exc_info)
        return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def log_json_event(
    logger: logging.Logger,
    level: int,
    event: str,
    data: Mapping[str, Any],
) -> None:
    """Emit one event whose structured data is handled by ``JsonFormatter``."""
    logger.log(
        level,
        event,
        extra={_STRUCTURED_DATA_ATTRIBUTE: {"event": event, **dict(data)}},
    )
