# Collision Monitor

A headless, safety-first service that makes one deterministic Pause or Resume
decision per robot at each decision tick.

The core decision engine does not depend on RabbitMQ. Transport, state
aggregation, geometry, optimisation and simulation remain separate modules, so
the decision pipeline can be tested entirely in memory.

The implemented design is summarised in
[`docs/architecture.md`](docs/architecture.md).

## Safety, liveness and delivery guarantees

Safety is enforced before publication: the engine independently validates all
selected action envelopes across the fleet before the service constructs and
publishes action messages. Boundary contact is treated as unsafe.

Liveness is conditional on at least one safe progress assignment existing
under the available Pause/Resume interface. If no robot, or compatible
implication-closed group, can safely Resume in a conflict component, that
component is failed safely with Pause decisions and an explicit
`UNRESOLVABLE_WITH_PAUSE_RESUME` alarm. The service does not claim that every
geometry can be cleared without route replanning or another control action.

The service creates exactly one logical decision for each known robot in each
tick. RabbitMQ action delivery is at least once: an ambiguous publisher-confirm
failure can cause a retry and therefore a duplicate message. The service
creates one `run_id` at process start and includes it in every action and tick
trace. Robot action consumers must process actions idempotently using
`(run_id, device_id, tick_id)`, so retries within a run are duplicates while an
equal process-local tick ID after restart remains a distinct action.

## Requirements

- Docker Engine with Docker Compose for the one-command workflow; or
- Python 3.11 or 3.12 for host development.

## One-command container workflow

Start RabbitMQ and the monitor, building the production runtime image if
needed:

```console
docker compose up --build
```

RabbitMQ is available on port `5672` and its local management console is at
<http://localhost:15672>. The monitor waits for RabbitMQ's health check before
starting and then exposes its own dependency health check through Compose.
Press Ctrl+C for a graceful SIGTERM shutdown.

Run the three-way junction demo and stop the stack automatically when the
simulator finishes:

```console
docker compose --profile demo up --build --attach simulator --abort-on-container-exit --exit-code-from simulator
```

Run the complete test suite, including the live RabbitMQ round-trip test, in a
fresh hermetic test image:

```console
docker compose run --rm --build test
```

Remove stopped containers and the Compose network when finished:

```console
docker compose down
```

Add `--volumes` to the final command only when the local RabbitMQ data should
also be discarded.

The Compose defaults `collision_monitor` / `local-development-only` are
deliberately non-secret local credentials. Override `RABBITMQ_USER` and
`RABBITMQ_PASSWORD` in the gitignored `.env` file for any shared environment;
never commit real credentials. Compose assembles the AMQP URL inside the
container and the application redacts AMQP credentials from log messages.

## Container design

The Dockerfile builds dependency wheels in one stage, installs only runtime
dependencies in the final application stage, and provides a separate test
stage containing pytest and repository tests. Application containers run as
the unprivileged `collision-monitor` user with a read-only root filesystem,
dropped Linux capabilities, a small temporary filesystem and
`no-new-privileges`. SIGTERM is handled by the service, which cancels its async
loops and closes RabbitMQ channels and connections cleanly.

Python dependencies are pinned to the versions in `pyproject.toml`. The Python
3.12.13 and RabbitMQ 3.13.7 container images are also pinned by repository
digest so a rebuild does not silently select a different base image.

All Compose services use bounded `json-file` logging: three files of at most
10 MB each. Production application logs remain one compact JSON object per
line with stable event names. Relevant events include `tick_id` and
`robot_id`; per-tick traces include the full deterministic diagnostics without
large geometry WKT by default.

## Host development

Create and activate a virtual environment, then install the package:

```console
python -m pip install -e '.[dev]'
```

Copy local configuration only when overrides are needed:

```console
cp .env.example .env
```

Validate configuration, check the configured RabbitMQ dependency, or run the
service:

```console
python -m collision_monitor validate-config
python -m collision_monitor healthcheck
python -m collision_monitor run
```

For concise local log lines, set
`COLLISION_MONITOR_LOG_FORMAT=console`. The default and the Compose setting are
`json`.

Useful Make targets mirror the direct commands:

```console
make test
make lint
make compose-up
make compose-demo
make compose-test
make compose-down
```

Every supported `COLLISION_MONITOR_*` setting and its default is documented in
[`.env.example`](.env.example). Robot dimensions, safety margins, numerical
tolerances, tick timing, stale-state timing, optimiser limits, grants,
publication retries, health checks and logging are configurable.

## RabbitMQ topology

Robots publish UTF-8 JSON state messages to one shared durable state queue.
The monitor retains the latest valid state per robot and publishes one action
to each durable robot-specific action queue. State messages are acknowledged
only after validation and acceptance into the latest-state store. Malformed or
superseded messages are rejected without requeue.

State ordering is evaluated independently for each robot. A source timestamp
older than the stored timestamp is rejected and does not refresh receive time.
An equal timestamp is accepted in receive order so a corrected retransmission
replaces the prior value; a newer timestamp always replaces it. Staleness is
still based on local receive time rather than the robot's source clock.

A robot whose receive age reaches the stale timeout is retained as a stationary
forced-Pause obstacle. The tick emits
`PROLONGED_STATE_LOSS_STATIC_OBSTACLE`, because prolonged state loss can block
liveness even when other robots have valid states. The monitor does not evict a
stale robot automatically: removal requires trusted external evidence that the
physical robot has left the monitored area, which this service does not
currently receive.

### Action message schema

The complete consumer contract is also available in
[`docs/action_message_schema.md`](docs/action_message_schema.md).

| Field | Type | Meaning |
|---|---:|---|
| `run_id` | string | Restart-unique service-run identifier used for idempotency. |
| `device_id` | string | Target robot identifier. |
| `action` | `Pause` or `Resume` | Command selected for this tick. |
| `tick_id` | string | Deterministic service tick identifier. |
| `decision_timestamp` | integer | Decision time in milliseconds since epoch. |
| `source_state_timestamp` | integer | Robot source timestamp in milliseconds since epoch. |
| `reason_codes` | string array | Stable machine-readable decision reasons. |
| `reason_context` | string array | Concise human-readable explanations. |
| `decision_source` | string | `policy`, `cp_sat`, `heuristic`, `repair` or `fail_safe`. |
| `grant_active` | Boolean | Whether the robot holds a right-of-way grant. |

Example:

```json
{"action":"Resume","decision_source":"cp_sat","decision_timestamp":1720000000100,"device_id":"robot-17","grant_active":true,"reason_codes":["CP_SAT_COMPONENT_DECISION"],"reason_context":["The bounded exact optimiser selected this component action."],"run_id":"8e67ab21-d9a8-44d8-a7de-3c73d1e125a3","source_state_timestamp":1720000000000,"tick_id":"tick-00000042"}
```

## Tests and quality checks

Run tests and static checks on the host:

```console
python -m pytest
python -m ruff check src tests
python -m ruff format --check src tests
python -m mypy
```

The live broker test is skipped by default during host testing. With a broker
available at the configured URL, opt in explicitly:

```console
RUN_RABBITMQ_INTEGRATION=1 python -m pytest -m rabbitmq
```

The hermetic Compose test command above enables this test automatically after
RabbitMQ becomes healthy.

## Mock fleet simulator

Five scenarios are provided in [`scenarios/`](scenarios): no conflict,
perpendicular crossing, a Yellow/Green/Blue three-way junction, a low-priority
blocker inside the junction and a deliberately unresolvable arrangement.
Scenario `nominal_speed_mps` is metadata; movement remains exactly one path
node per tick.

Run an embedded in-memory monitor deterministically:

```console
python -m collision_monitor.simulator.cli --scenario scenarios/three_way.yaml --transport memory
```

For RabbitMQ, start the monitor and select `--transport rabbitmq`, or use the
documented Compose demo profile. Robots default to Pause until an action is
received. The final simulator line is a compact JSON summary containing ticks,
Pause/Resume counts, completion and alarms.
