.PHONY: test lint run

test:
	python -m pytest

lint:
	python -m ruff check src tests
	python -m mypy

run:
	python -m collision_monitor --help

