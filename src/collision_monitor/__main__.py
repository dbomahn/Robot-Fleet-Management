"""Command-line entry point for the collision monitor."""

from __future__ import annotations

import argparse
from collections.abc import Sequence


def build_parser() -> argparse.ArgumentParser:
    """Build the placeholder command-line parser."""
    parser = argparse.ArgumentParser(
        prog="collision-monitor",
        description="Safety-first robot fleet collision monitor.",
    )
    parser.add_argument(
        "--version",
        action="version",
        version="%(prog)s 0.1.0",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Parse command-line arguments without starting transport services yet."""
    build_parser().parse_args(argv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

