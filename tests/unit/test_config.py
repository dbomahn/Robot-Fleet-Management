"""Unit tests for complete environment-backed monitor configuration."""

from __future__ import annotations

from dataclasses import fields
from pathlib import Path

import pytest

from collision_monitor.config import MonitorConfig


def test_every_configuration_field_can_be_loaded_from_its_environment_name() -> None:
    defaults = MonitorConfig()
    environment: dict[str, str] = {}
    for field_definition in fields(defaults):
        value = getattr(defaults, field_definition.name)
        environment_name = f"COLLISION_MONITOR_{field_definition.name.upper()}"
        environment[environment_name] = (
            str(value).lower() if isinstance(value, bool) else str(value)
        )

    loaded = MonitorConfig.from_environment(environment)

    assert loaded == defaults


def test_environment_overrides_strings_numbers_and_booleans() -> None:
    config = MonitorConfig.from_environment(
        {
            "COLLISION_MONITOR_TICK_INTERVAL_SECONDS": "0.5",
            "COLLISION_MONITOR_MAXIMUM_EXACT_COMPONENT_SIZE": "12",
            "COLLISION_MONITOR_TRACE_DETAILED_GEOMETRY": "yes",
            "COLLISION_MONITOR_RABBITMQ_STATE_QUEUE": "fleet.states",
            "COLLISION_MONITOR_RABBITMQ_ACTION_QUEUE_PREFIX": "fleet.actions.",
        }
    )

    assert config.tick_interval_seconds == 0.5
    assert config.maximum_exact_component_size == 12
    assert config.trace_detailed_geometry is True
    assert config.rabbitmq_state_queue == "fleet.states"
    assert config.rabbitmq_action_queue_prefix == "fleet.actions."


def test_env_example_documents_every_configuration_default() -> None:
    example_path = Path(__file__).parents[2] / ".env.example"
    documented_names = {
        line.split("=", maxsplit=1)[0]
        for line in example_path.read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
    }
    expected_names = {
        f"COLLISION_MONITOR_{field_definition.name.upper()}"
        for field_definition in fields(MonitorConfig())
    }

    assert documented_names == expected_names


def test_invalid_environment_boolean_is_rejected() -> None:
    with pytest.raises(ValueError, match="must be a Boolean value"):
        MonitorConfig.from_environment(
            {"COLLISION_MONITOR_TRACE_DETAILED_GEOMETRY": "sometimes"}
        )
