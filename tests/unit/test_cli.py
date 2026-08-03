"""Unit tests for collision-monitor command dispatch."""

from __future__ import annotations

import socket

from collision_monitor.__main__ import build_parser, main


def test_run_subcommand_is_available() -> None:
    arguments = build_parser().parse_args(["run"])

    assert arguments.command == "run"


def test_healthcheck_subcommand_is_available() -> None:
    arguments = build_parser().parse_args(["healthcheck"])

    assert arguments.command == "healthcheck"


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


def test_healthcheck_reports_reachable_dependency(
    monkeypatch: object,
    capsys: object,
) -> None:
    monkeypatch.setenv(  # type: ignore[attr-defined]
        "COLLISION_MONITOR_RABBITMQ_URL",
        "amqp://guest:guest@localhost/",
    )

    class Connection:
        def __enter__(self) -> Connection:
            return self

        def __exit__(self, *args: object) -> None:
            return None

    def connect(address: tuple[str, int], timeout: float) -> Connection:
        assert address == ("localhost", 5672)
        assert timeout == 2.0
        return Connection()

    monkeypatch.setattr(socket, "create_connection", connect)  # type: ignore[attr-defined]

    assert main(["healthcheck"]) == 0
    assert capsys.readouterr().out == (  # type: ignore[attr-defined]
        "Collision Monitor dependencies are reachable.\n"
    )


def test_healthcheck_reports_unreachable_dependency_without_credentials(
    monkeypatch: object,
    capsys: object,
) -> None:
    monkeypatch.setenv(  # type: ignore[attr-defined]
        "COLLISION_MONITOR_RABBITMQ_URL",
        "amqp://private-user:private-password@rabbitmq/",
    )

    def fail_connect(address: tuple[str, int], timeout: float) -> None:
        del address, timeout
        raise OSError("cannot connect to amqp://private-user:private-password@rabbitmq/")

    monkeypatch.setattr(  # type: ignore[attr-defined]
        socket,
        "create_connection",
        fail_connect,
    )

    assert main(["healthcheck"]) == 1
    error = capsys.readouterr().err  # type: ignore[attr-defined]
    assert "private-user" not in error
    assert "private-password" not in error
    assert "amqp://***:***@rabbitmq/" in error
