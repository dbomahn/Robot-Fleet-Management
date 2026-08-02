from __future__ import annotations

import math
import os
from collections.abc import Mapping
from dataclasses import dataclass
from dataclasses import fields as dataclass_fields
from typing import Any, get_type_hints
from urllib.parse import urlparse

_ENVIRONMENT_PREFIX = "COLLISION_MONITOR_"


def _parse_boolean(value: str, *, name: str) -> bool:
    """Parse a conventional environment Boolean with a precise error."""
    normalised = value.strip().lower()
    if normalised in {"1", "true", "yes", "on"}:
        return True
    if normalised in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be a Boolean value")


def _parse_environment_value(value: str, value_type: type[Any], *, name: str) -> Any:
    """Parse one primitive dataclass field from an environment string."""
    if value_type is bool:
        return _parse_boolean(value, name=name)
    try:
        if value_type is int:
            return int(value)
        if value_type is float:
            return float(value)
        if value_type is str:
            return value
    except ValueError as exc:
        raise ValueError(f"{name} has an invalid {value_type.__name__} value") from exc
    raise TypeError(f"unsupported configuration field type {value_type!r}")


@dataclass(frozen=True, slots=True)
class MonitorConfig:
    """All safety, timing, optimisation, grant and priority settings."""

    robot_width_metres: float = 0.630
    robot_length_metres: float = 1.430
    safety_margin_metres: float = 0.050
    pose_tolerance: float = 1e-6
    tick_interval_seconds: float = 1.0
    stale_timeout_seconds: float = 3.0
    publication_maximum_attempts: int = 3
    publication_retry_delay_seconds: float = 0.050
    trace_detailed_geometry: bool = False
    log_level: str = "INFO"
    rabbitmq_url: str = "amqp://guest:guest@localhost/"
    rabbitmq_exchange_name: str = "collision_monitor"
    rabbitmq_state_queue: str = "collision_monitor.robot_states"
    rabbitmq_state_routing_key: str = "collision_monitor.state"
    rabbitmq_action_queue_prefix: str = "collision_monitor.robot_actions."
    rabbitmq_prefetch_count: int = 1
    rabbitmq_connection_timeout_seconds: float = 10.0
    rabbitmq_reconnect_initial_delay_seconds: float = 1.0
    rabbitmq_reconnect_max_delay_seconds: float = 30.0
    cp_sat_time_limit_seconds: float = 0.050
    cp_sat_random_seed: int = 0
    cp_sat_log_search_progress: bool = False
    maximum_exact_component_size: int = 20
    heuristic_maximum_repair_candidates: int = 20
    grant_minimum_hold_ticks: int = 2
    grant_maximum_hold_ticks: int = 10
    grant_clearance_release_ticks: int = 2
    grant_waiting_age_weight: float = 1.0
    priority_base_progress_value: int = 10
    priority_loaded_bonus: int = 20
    priority_deadline_urgency_maximum: int = 100
    priority_deadline_horizon_ms: int = 300_000
    priority_low_battery_threshold: float = 20.0
    priority_low_battery_urgency_bonus: int = 30
    priority_waiting_age_bonus_per_tick: int = 5
    priority_waiting_age_cap_ticks: int = 20
    priority_active_grant_continuation_bonus: int = 50
    priority_clearance_bonus_per_conflict: int = 25

    @classmethod
    def from_environment(
        cls,
        environ: Mapping[str, str] | None = None,
    ) -> MonitorConfig:
        """Load every setting from ``COLLISION_MONITOR_<FIELD_NAME>`` variables."""
        source = os.environ if environ is None else environ
        type_hints = get_type_hints(cls)
        values: dict[str, Any] = {}
        for field_definition in dataclass_fields(cls):
            environment_name = (
                f"{_ENVIRONMENT_PREFIX}{field_definition.name.upper()}"
            )
            raw_value = source.get(environment_name)
            if raw_value is None:
                continue
            values[field_definition.name] = _parse_environment_value(
                raw_value,
                type_hints[field_definition.name],
                name=environment_name,
            )
        return cls(**values)

    def __post_init__(self) -> None:
        """Reject configuration values that cannot produce meaningful decisions."""
        positive_values = {
            "robot_width_metres": self.robot_width_metres,
            "robot_length_metres": self.robot_length_metres,
            "pose_tolerance": self.pose_tolerance,
            "tick_interval_seconds": self.tick_interval_seconds,
            "stale_timeout_seconds": self.stale_timeout_seconds,
            "cp_sat_time_limit_seconds": self.cp_sat_time_limit_seconds,
            "rabbitmq_connection_timeout_seconds": (
                self.rabbitmq_connection_timeout_seconds
            ),
            "rabbitmq_reconnect_initial_delay_seconds": (
                self.rabbitmq_reconnect_initial_delay_seconds
            ),
            "rabbitmq_reconnect_max_delay_seconds": (
                self.rabbitmq_reconnect_max_delay_seconds
            ),
        }
        for name, value in positive_values.items():
            if value <= 0:
                raise ValueError(f"{name} must be greater than zero")

        if self.safety_margin_metres < 0:
            raise ValueError("safety_margin_metres must not be negative")
        if (
            not isinstance(self.publication_maximum_attempts, int)
            or self.publication_maximum_attempts < 1
        ):
            raise ValueError("publication_maximum_attempts must be a positive integer")
        if (
            not math.isfinite(self.publication_retry_delay_seconds)
            or self.publication_retry_delay_seconds < 0
        ):
            raise ValueError(
                "publication_retry_delay_seconds must be finite and non-negative"
            )
        if not isinstance(self.trace_detailed_geometry, bool):
            raise ValueError("trace_detailed_geometry must be Boolean")
        if self.log_level.upper() not in {
            "CRITICAL",
            "ERROR",
            "WARNING",
            "INFO",
            "DEBUG",
        }:
            raise ValueError("log_level must be a standard logging level")
        parsed_rabbitmq_url = urlparse(self.rabbitmq_url)
        if parsed_rabbitmq_url.scheme not in {"amqp", "amqps"}:
            raise ValueError("rabbitmq_url must use the amqp or amqps scheme")
        if not parsed_rabbitmq_url.hostname:
            raise ValueError("rabbitmq_url must include a hostname")
        if not self.rabbitmq_state_queue.strip():
            raise ValueError("rabbitmq_state_queue must not be empty")
        if not self.rabbitmq_exchange_name.strip():
            raise ValueError("rabbitmq_exchange_name must not be empty")
        if not self.rabbitmq_state_routing_key.strip():
            raise ValueError("rabbitmq_state_routing_key must not be empty")
        if not self.rabbitmq_action_queue_prefix.strip():
            raise ValueError("rabbitmq_action_queue_prefix must not be empty")
        if (
            not isinstance(self.rabbitmq_prefetch_count, int)
            or self.rabbitmq_prefetch_count < 1
        ):
            raise ValueError("rabbitmq_prefetch_count must be a positive integer")
        if (
            self.rabbitmq_reconnect_max_delay_seconds
            < self.rabbitmq_reconnect_initial_delay_seconds
        ):
            raise ValueError(
                "rabbitmq_reconnect_max_delay_seconds must be at least the initial delay"
            )
        if self.maximum_exact_component_size < 1:
            raise ValueError("maximum_exact_component_size must be at least one")
        if (
            not isinstance(self.heuristic_maximum_repair_candidates, int)
            or self.heuristic_maximum_repair_candidates < 1
        ):
            raise ValueError(
                "heuristic_maximum_repair_candidates must be a positive integer"
            )
        if not isinstance(self.cp_sat_random_seed, int) or self.cp_sat_random_seed < 0:
            raise ValueError("cp_sat_random_seed must be a non-negative integer")
        if not isinstance(self.cp_sat_log_search_progress, bool):
            raise ValueError("cp_sat_log_search_progress must be Boolean")
        if self.grant_minimum_hold_ticks < 0:
            raise ValueError("grant_minimum_hold_ticks must not be negative")
        if self.grant_maximum_hold_ticks < self.grant_minimum_hold_ticks:
            raise ValueError(
                "grant_maximum_hold_ticks must be at least grant_minimum_hold_ticks"
            )
        if self.grant_clearance_release_ticks < 1:
            raise ValueError("grant_clearance_release_ticks must be at least one")
        if self.grant_waiting_age_weight < 0:
            raise ValueError("grant_waiting_age_weight must not be negative")

        non_negative_integer_values = {
            "priority_base_progress_value": self.priority_base_progress_value,
            "priority_loaded_bonus": self.priority_loaded_bonus,
            "priority_deadline_urgency_maximum": self.priority_deadline_urgency_maximum,
            "priority_low_battery_urgency_bonus": self.priority_low_battery_urgency_bonus,
            "priority_waiting_age_bonus_per_tick": self.priority_waiting_age_bonus_per_tick,
            "priority_waiting_age_cap_ticks": self.priority_waiting_age_cap_ticks,
            "priority_active_grant_continuation_bonus": (
                self.priority_active_grant_continuation_bonus
            ),
            "priority_clearance_bonus_per_conflict": (
                self.priority_clearance_bonus_per_conflict
            ),
        }
        for name, value in non_negative_integer_values.items():
            if not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")

        if (
            not isinstance(self.priority_deadline_horizon_ms, int)
            or self.priority_deadline_horizon_ms <= 0
        ):
            raise ValueError("priority_deadline_horizon_ms must be a positive integer")
        if not 0 < self.priority_low_battery_threshold <= 100:
            raise ValueError("priority_low_battery_threshold must be in (0, 100]")
