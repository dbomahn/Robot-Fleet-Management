"""Command-line runner for deterministic mock-fleet scenarios."""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from collections.abc import Sequence

from collision_monitor.config import MonitorConfig
from collision_monitor.simulator.scenario import (
    RabbitMQSimulatorTransport,
    load_scenario,
    run_in_memory_scenario,
    run_scenario,
)


def build_parser() -> argparse.ArgumentParser:
    """Build simulator arguments for scenario and transport selection."""
    parser = argparse.ArgumentParser(
        prog="collision-monitor-simulator",
        description="Run a one-node-per-tick mock robot fleet.",
    )
    parser.add_argument("--scenario", required=True, help="YAML or JSON scenario path.")
    parser.add_argument(
        "--transport",
        choices=("memory", "rabbitmq"),
        default="memory",
        help="Run an embedded in-memory monitor or use a running RabbitMQ monitor.",
    )
    parser.add_argument(
        "--max-ticks",
        type=int,
        default=None,
        help="Override the scenario's bounded maximum tick count.",
    )
    return parser


async def _run(arguments: argparse.Namespace) -> int:
    """Load and execute one selected simulator configuration."""
    scenario = load_scenario(arguments.scenario)
    config = MonitorConfig.from_environment()
    if arguments.transport == "memory":
        result = await run_in_memory_scenario(
            scenario,
            config=config,
            maximum_ticks=arguments.max_ticks,
        )
    else:
        transport = RabbitMQSimulatorTransport(config)
        result = await run_scenario(
            scenario,
            transport,
            maximum_ticks=arguments.max_ticks,
            realtime=True,
        )
    print(json.dumps(result.as_dict(), sort_keys=True, separators=(",", ":")))
    return 0 if result.completed == scenario.resolvable else 1


def main(argv: Sequence[str] | None = None) -> int:
    """Run the simulator and report configuration/scenario errors cleanly."""
    arguments = build_parser().parse_args(argv)
    try:
        return asyncio.run(_run(arguments))
    except (OSError, TypeError, ValueError) as exc:
        print(f"Simulator error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
