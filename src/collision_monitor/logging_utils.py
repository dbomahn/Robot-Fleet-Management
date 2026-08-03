"""Structured standard-library logging helpers."""

from __future__ import annotations

import json
import logging
import re
from collections.abc import Mapping
from datetime import UTC, datetime
from typing import Any

_STRUCTURED_DATA_ATTRIBUTE = "collision_monitor_data"
_AMQP_CREDENTIAL_PATTERN = re.compile(r"(?P<scheme>\bamqps?://)[^@\s/]+@")


def redact_credentials(value: str) -> str:
    """Remove AMQP user information from text before it reaches a log sink."""
    return _AMQP_CREDENTIAL_PATTERN.sub(r"\g<scheme>***:***@", value)


def _sanitise_log_value(value: Any) -> Any:
    """Recursively redact credentials while retaining JSON-compatible structure."""
    if isinstance(value, str):
        return redact_credentials(value)
    if isinstance(value, Mapping):
        return {str(key): _sanitise_log_value(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return tuple(_sanitise_log_value(item) for item in value)
    if isinstance(value, list):
        return [_sanitise_log_value(item) for item in value]
    return value


class JsonFormatter(logging.Formatter):
    """Render each log record as one deterministic JSON object."""

    def format(self, record: logging.LogRecord) -> str:
        """Serialise a record without embedding free-form prefixes."""
        payload: dict[str, Any] = {
            "timestamp": datetime.fromtimestamp(record.created, tz=UTC).isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": redact_credentials(record.getMessage()),
        }
        structured_data = getattr(record, _STRUCTURED_DATA_ATTRIBUTE, None)
        if structured_data is not None:
            if not isinstance(structured_data, Mapping):
                raise TypeError("structured log data must be a mapping")
            payload["data"] = _sanitise_log_value(structured_data)
        if record.exc_info:
            payload["exception"] = redact_credentials(self.formatException(record.exc_info))
        return json.dumps(payload, sort_keys=True, separators=(",", ":"))


class ConsoleFormatter(logging.Formatter):
    """Render a concise development-oriented line without losing event identity."""

    _CONTEXT_KEYS = (
        "tick_id",
        "robot_id",
        "signal",
        "state_queue",
        "attempt",
        "retry_delay_seconds",
        "failed_robot_ids",
        "error",
    )

    def format(self, record: logging.LogRecord) -> str:
        """Render the stable event and its most useful compact context."""
        structured_data = getattr(record, _STRUCTURED_DATA_ATTRIBUTE, {})
        if structured_data and not isinstance(structured_data, Mapping):
            raise TypeError("structured log data must be a mapping")
        data = _sanitise_log_value(structured_data)
        event = str(data.get("event", redact_credentials(record.getMessage())))
        context = " ".join(
            f"{key}={json.dumps(data[key], sort_keys=True, separators=(',', ':'))}"
            for key in self._CONTEXT_KEYS
            if key in data and data[key] is not None
        )
        rendered = f"{record.levelname:<8} {event}"
        return f"{rendered} {context}" if context else rendered


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
        extra={_STRUCTURED_DATA_ATTRIBUTE: _sanitise_log_value({"event": event, **dict(data)})},
    )
