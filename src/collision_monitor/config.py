"""Configuration for collision monitoring and bounded decision-making."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class MonitorConfig:
    """Numerical, timing and grant settings for the monitor."""

    robot_width_metres: float = 0.630
    robot_length_metres: float = 1.430
    safety_margin_metres: float = 0.050
    pose_tolerance: float = 1e-6
    tick_interval_seconds: float = 1.0
    stale_timeout_seconds: float = 3.0
    cp_sat_time_limit_seconds: float = 0.050
    maximum_exact_component_size: int = 12
    grant_minimum_hold_ticks: int = 2
    grant_maximum_hold_ticks: int = 10
    grant_waiting_age_weight: float = 1.0

    def __post_init__(self) -> None:
        """Reject configuration values that cannot produce meaningful decisions."""
        positive_values = {
            "robot_width_metres": self.robot_width_metres,
            "robot_length_metres": self.robot_length_metres,
            "pose_tolerance": self.pose_tolerance,
            "tick_interval_seconds": self.tick_interval_seconds,
            "stale_timeout_seconds": self.stale_timeout_seconds,
            "cp_sat_time_limit_seconds": self.cp_sat_time_limit_seconds,
        }
        for name, value in positive_values.items():
            if value <= 0:
                raise ValueError(f"{name} must be greater than zero")

        if self.safety_margin_metres < 0:
            raise ValueError("safety_margin_metres must not be negative")
        if self.maximum_exact_component_size < 1:
            raise ValueError("maximum_exact_component_size must be at least one")
        if self.grant_minimum_hold_ticks < 0:
            raise ValueError("grant_minimum_hold_ticks must not be negative")
        if self.grant_maximum_hold_ticks < self.grant_minimum_hold_ticks:
            raise ValueError(
                "grant_maximum_hold_ticks must be at least grant_minimum_hold_ticks"
            )
        if self.grant_waiting_age_weight < 0:
            raise ValueError("grant_waiting_age_weight must not be negative")

