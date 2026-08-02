"""Unit tests for one-object-per-line structured JSON logging."""

from __future__ import annotations

import io
import json
import logging

from collision_monitor.logging_utils import JsonFormatter, log_json_event


def test_structured_event_formats_as_one_json_object() -> None:
    stream = io.StringIO()
    handler = logging.StreamHandler(stream)
    handler.setFormatter(JsonFormatter())
    logger = logging.getLogger("collision_monitor.test.json")
    logger.handlers = [handler]
    logger.propagate = False
    logger.setLevel(logging.INFO)

    log_json_event(
        logger,
        logging.INFO,
        "decision_tick",
        {"tick_id": "tick-00000001", "robot_ids": ("robot-a", "robot-b")},
    )

    lines = stream.getvalue().splitlines()
    assert len(lines) == 1
    payload = json.loads(lines[0])
    assert payload["message"] == "decision_tick"
    assert payload["data"] == {
        "event": "decision_tick",
        "robot_ids": ["robot-a", "robot-b"],
        "tick_id": "tick-00000001",
    }
