# Collision Monitor

A headless service that makes one deterministic Pause or Resume decision per
robot at each decision tick.

The core decision engine is independent of RabbitMQ. Transport, state
aggregation, geometry, optimisation and simulation are separate modules.


## Requirements

- Python 3.11 or 3.12
- Shapely 2.x for geometry


## Development

Create and activate a virtual environment, then install the package:

```console
python -m pip install -e '.[dev]'
```


Validate environment configuration or run the service:

```console
python -m collision_monitor validate-config
python -m collision_monitor run
```

The service handles SIGINT and SIGTERM, stops both async loops and closes its
RabbitMQ channels and robust connection cleanly.

## Action message schema

Every robot-specific action queue receives UTF-8 JSON with this schema:

| Field | Type | Meaning |
|---|---:|---|
| `device_id` | string | Target robot identifier. |
| `action` | `Pause` or `Resume` | Command selected for this tick. |
| `tick_id` | string | Deterministic service tick identifier. |
| `decision_timestamp` | integer | Decision time in milliseconds since epoch. |
| `source_state_timestamp` | integer | Robot-provided source timestamp in milliseconds since epoch. |
| `reason_codes` | string array | Stable machine-readable decision reasons. |
| `reason_context` | string array | Concise human-readable explanations. |
| `decision_source` | string | `policy`, `cp_sat`, `heuristic`, `repair` or `fail_safe`. |
| `grant_active` | Boolean | Whether the robot holds a right-of-way grant after this decision. |

Example:

```json
{
  "device_id": "robot-17",
  "action": "Resume",
  "tick_id": "tick-00000042",
  "decision_timestamp": 1720000000100,
  "source_state_timestamp": 1720000000000,
  "reason_codes": ["CP_SAT_COMPONENT_DECISION"],
  "reason_context": ["The bounded exact optimiser selected this component action."],
  "decision_source": "cp_sat",
  "grant_active": true
}
```

## Configuration

Configuration is supplied through `COLLISION_MONITOR_*` environment variables.

Copy the example configuration if local overrides are required:

```console
cp .env.example .env
```

Validate the effective configuration:

```console
python -m collision_monitor validate-config
```

All supported settings and defaults are documented in [`.env.example`](.env.example).


## Running the service

Start RabbitMQ, then run:

```console
python -m collision_monitor run
```

The service handles `SIGINT` and `SIGTERM` and closes its asynchronous tasks, RabbitMQ channels and connection cleanly.


## RabbitMQ topology

Robots publish state messages to one shared durable state queue. The monitor retains the latest valid state for each robot and publishes actions to durable robot-specific queues.

State messages are acknowledged only after validation and acceptance into the latest-state store. Malformed and superseded messages are rejected without requeue.


## Tests

Run the complete test suite:

```console
python -m pytest
```

Run lint and formatting checks:

```console
ruff check .
ruff format --check .
```

The live RabbitMQ integration test is skipped by default. With a test broker available, run:

```console
RUN_RABBITMQ_INTEGRATION=1 python -m pytest -m rabbitmq
```
