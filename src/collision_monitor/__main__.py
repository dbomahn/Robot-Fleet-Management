"""Command-line entry point for configuration validation and service execution."""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import socket
import sys
from collections.abc import Sequence
from urllib.parse import urlparse

from collision_monitor.config import MonitorConfig
from collision_monitor.logging_utils import (
    ConsoleFormatter,
    JsonFormatter,
    log_json_event,
    redact_credentials,
)
from collision_monitor.service import CollisionMonitorService
from collision_monitor.transport.rabbitmq import RabbitMQTransport


def build_parser() -> argparse.ArgumentParser:
    """Build the collision-monitor command-line parser."""
    parser = argparse.ArgumentParser(
        prog="collision-monitor",
        description="Safety-first robot fleet collision monitor.",
    )
    parser.add_argument(
        "--version",
        action="version",
        version="%(prog)s 0.1.0",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser(
        "run",
        help="Run the RabbitMQ-backed collision monitor service.",
    )
    subparsers.add_parser(
        "validate-config",
        help="Validate environment configuration without connecting to RabbitMQ.",
    )
    subparsers.add_parser(
        "healthcheck",
        help="Check that the configured RabbitMQ endpoint is reachable.",
    )
    return parser


def _configure_logging(config: MonitorConfig) -> logging.Logger:
    """Configure production JSON or concise local console logging."""
    handler = logging.StreamHandler()
    formatter: logging.Formatter = (
        ConsoleFormatter() if config.log_format.lower() == "console" else JsonFormatter()
    )
    handler.setFormatter(formatter)
    root_logger = logging.getLogger()
    root_logger.handlers = [handler]
    root_logger.setLevel(config.log_level.upper())
    return logging.getLogger("collision_monitor")


def _check_rabbitmq_health(config: MonitorConfig) -> None:
    """Open a bounded TCP connection without exposing configured credentials."""
    parsed_url = urlparse(config.rabbitmq_url)
    if parsed_url.hostname is None:
        raise ValueError("RabbitMQ URL does not contain a hostname")
    default_port = 5671 if parsed_url.scheme == "amqps" else 5672
    endpoint = (parsed_url.hostname, parsed_url.port or default_port)
    with socket.create_connection(
        endpoint,
        timeout=config.healthcheck_timeout_seconds,
    ):
        return


async def _run_service(config: MonitorConfig) -> None:
    """Run until SIGINT/SIGTERM and close RabbitMQ cleanly."""
    logger = _configure_logging(config)
    transport = RabbitMQTransport(config, logger=logger.getChild("transport.rabbitmq"))
    service = CollisionMonitorService(
        config,
        transport,
        transport,
        logger=logger.getChild("service"),
    )
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()

    def request_shutdown(signal_name: str) -> None:
        log_json_event(
            logger,
            logging.INFO,
            "shutdown_requested",
            {"signal": signal_name},
        )
        stop_event.set()

    installed_signals: list[signal.Signals] = []
    for shutdown_signal in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(
                shutdown_signal,
                request_shutdown,
                shutdown_signal.name,
            )
        except NotImplementedError:
            continue
        installed_signals.append(shutdown_signal)

    log_json_event(
        logger,
        logging.INFO,
        "service_starting",
        {
            "state_queue": config.rabbitmq_state_queue,
            "action_queue_prefix": config.rabbitmq_action_queue_prefix,
            "tick_interval_seconds": config.tick_interval_seconds,
        },
    )
    try:
        await service.run(stop_event)
    finally:
        for installed_signal in installed_signals:
            loop.remove_signal_handler(installed_signal)
        await transport.close()
        log_json_event(logger, logging.INFO, "service_stopped", {})


def main(argv: Sequence[str] | None = None) -> int:
    """Validate configuration or run the transport-backed application."""
    arguments = build_parser().parse_args(argv)
    try:
        config = MonitorConfig.from_environment()
    except (TypeError, ValueError) as exc:
        print(f"Invalid configuration: {exc}", file=sys.stderr)
        return 2

    if arguments.command == "validate-config":
        print("Configuration is valid.")
        return 0
    if arguments.command == "healthcheck":
        try:
            _check_rabbitmq_health(config)
        except OSError as exc:
            safe_error = redact_credentials(str(exc))
            print(f"Health check failed: {safe_error}", file=sys.stderr)
            return 1
        print("Collision Monitor dependencies are reachable.")
        return 0
    if arguments.command == "run":
        try:
            asyncio.run(_run_service(config))
        except KeyboardInterrupt:
            return 130
        except Exception as exc:
            log_json_event(
                logging.getLogger("collision_monitor"),
                logging.CRITICAL,
                "service_failed",
                {"error": str(exc)},
            )
            return 1
        return 0
    raise RuntimeError(f"unsupported command {arguments.command!r}")


if __name__ == "__main__":
    raise SystemExit(main())
