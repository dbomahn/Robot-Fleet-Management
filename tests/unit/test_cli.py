"""Unit tests for collision-monitor command dispatch."""

from __future__ import annotations

from collision_monitor.__main__ import build_parser, main


def test_run_subcommand_is_available() -> None:
    arguments = build_parser().parse_args(["run"])

    assert arguments.command == "run"


def test_validate_config_reports_success(capsys: object) -> None:
    assert main(["validate-config"]) == 0

    captured = capsys.readouterr()  # type: ignore[attr-defined]
    assert captured.out == "Configuration is valid.\n"


def test_validate_config_reports_environment_error(monkeypatch: object, capsys: object) -> None:
    monkeypatch.setenv(  # type: ignore[attr-defined]
        "COLLISION_MONITOR_RABBITMQ_RECONNECT_INITIAL_DELAY_SECONDS",
        "invalid",
    )

    assert main(["validate-config"]) == 2

    captured = capsys.readouterr()  # type: ignore[attr-defined]
    assert "Invalid configuration" in captured.err
