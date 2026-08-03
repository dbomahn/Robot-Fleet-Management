"""Unit tests for mock-fleet simulator command-line selection."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from collision_monitor.simulator.cli import build_parser, main

SCENARIO_DIRECTORY = Path(__file__).parents[2] / "scenarios"


def test_rabbitmq_transport_option_is_available() -> None:
    arguments = build_parser().parse_args(
        [
            "--scenario",
            str(SCENARIO_DIRECTORY / "three_way.yaml"),
            "--transport",
            "rabbitmq",
        ]
    )

    assert arguments.transport == "rabbitmq"


def test_memory_cli_prints_complete_end_of_run_summary(
    capsys: pytest.CaptureFixture[str],
) -> None:
    exit_code = main(
        [
            "--scenario",
            str(SCENARIO_DIRECTORY / "no_conflict.yaml"),
            "--transport",
            "memory",
        ]
    )

    summary = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert summary["completed"] is True
    assert summary["ticks"] == 3
    assert summary["movement_semantics"] == "one_path_node_per_tick"
    assert summary["pauses"] >= 0
    assert summary["resumes"] > 0
    assert summary["alarms"] == []
