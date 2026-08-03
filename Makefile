.PHONY: test lint run compose-up compose-demo compose-test compose-down

test:
	python -m pytest

lint:
	python -m ruff check src tests
	python -m mypy

run:
	python -m collision_monitor run

compose-up:
	docker compose up --build

compose-demo:
	docker compose --profile demo up --build --attach simulator --abort-on-container-exit --exit-code-from simulator

compose-test:
	docker compose run --rm --build test

compose-down:
	docker compose down
