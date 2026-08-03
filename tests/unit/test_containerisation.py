"""Unit checks for the reproducible container and Compose contract."""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import Any

import yaml

REPOSITORY_ROOT = Path(__file__).parents[2]


def _compose_configuration() -> dict[str, Any]:
    return yaml.safe_load((REPOSITORY_ROOT / "docker-compose.yml").read_text(encoding="utf-8"))


def test_dockerfile_has_separate_non_root_runtime_and_test_stages() -> None:
    dockerfile = (REPOSITORY_ROOT / "Dockerfile").read_text(encoding="utf-8")

    assert dockerfile.startswith("# syntax=docker/dockerfile:1.7@sha256:")
    assert "FROM python:3.12.13-slim@sha256:" in dockerfile
    assert " AS builder" in dockerfile
    assert "FROM builder AS test-builder" in dockerfile
    assert " AS runtime" in dockerfile
    assert "FROM runtime AS test" in dockerfile
    assert "USER collision-monitor:collision-monitor" in dockerfile
    assert "STOPSIGNAL SIGTERM" in dockerfile
    assert "COPY . " not in dockerfile


def test_monitor_waits_for_healthy_rabbitmq_and_uses_bounded_logs() -> None:
    services = _compose_configuration()["services"]
    monitor = services["monitor"]
    rabbitmq = services["rabbitmq"]

    assert rabbitmq["image"].startswith("rabbitmq:3.13.7-management@sha256:")
    assert rabbitmq["healthcheck"]["test"] == [
        "CMD",
        "rabbitmq-diagnostics",
        "-q",
        "ping",
    ]
    assert monitor["depends_on"]["rabbitmq"]["condition"] == "service_healthy"
    assert monitor["read_only"] is True
    assert monitor["stop_signal"] == "SIGTERM"
    assert monitor["cap_drop"] == ["ALL"]
    assert monitor["logging"]["options"] == {
        "max-size": "10m",
        "max-file": "3",
    }


def test_demo_and_hermetic_test_services_are_opt_in() -> None:
    services = _compose_configuration()["services"]

    assert services["simulator"]["profiles"] == ["demo"]
    assert services["simulator"]["volumes"] == ["./scenarios:/app/scenarios:ro"]
    assert services["test"]["profiles"] == ["test"]
    assert services["test"]["build"]["target"] == "test"
    assert services["test"]["environment"]["PYTEST_ADDOPTS"] == ("-p no:cacheprovider")
    assert services["test"]["environment"]["RUN_RABBITMQ_INTEGRATION"] == "1"


def test_docker_context_excludes_local_credentials_and_caches() -> None:
    ignored = set((REPOSITORY_ROOT / ".dockerignore").read_text(encoding="utf-8").splitlines())

    assert ".env" in ignored
    assert ".git" in ignored
    assert ".venv" in ignored


def test_declared_python_dependencies_are_exactly_pinned() -> None:
    configuration = tomllib.loads((REPOSITORY_ROOT / "pyproject.toml").read_text(encoding="utf-8"))

    assert all("==" in requirement for requirement in configuration["build-system"]["requires"])
    assert all("==" in requirement for requirement in configuration["project"]["dependencies"])
    assert all(
        "==" in requirement
        for requirement in configuration["project"]["optional-dependencies"]["dev"]
    )
