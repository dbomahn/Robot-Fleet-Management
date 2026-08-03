# Final audit

Review date: 2026-08-03. Repository reviewed at
`/home/ak121396/Desktop/collision_monitor`.

This is a fresh review of the current repository. Generated caches and package
metadata were excluded. The final correction is limited to restart-safe action
identity, stale-state observability, conservative margin geometry and build
pinning; the decision model is unchanged.

## Criteria

| Order | Criterion | Result | Evidence and assessment |
|---:|---|:---:|---|
| 1 | Safety correctness | PASS | `src/collision_monitor/geometry.py::_conservative_buffer_distance`, `oriented_footprint`, `pause_envelope` and `resume_envelope` construct conservative occupancy, including finite-chord margin compensation and intermediate heading changes. `src/collision_monitor/conflicts.py::build_conflict_model` evaluates all four action pairs. `src/collision_monitor/optimiser.py::validate_component_decisions`, `src/collision_monitor/heuristic.py::validate_heuristic_decisions` and `src/collision_monitor/engine.py::validate_global_safety` independently reject selected no-goods and final intersecting envelopes. `CollisionDecisionEngine.decide` forces affected graph components to Pause and records a critical alarm; already-overlapping stationary footprints remain explicitly unsafe. |
| 2 | Liveness safeguards and honest failure modes | PASS | `src/collision_monitor/grants.py::GrantManager._safe_relevant_grants`, `_validate_or_repair_components`, `_reconcile_lifecycle` and `_update_fairness` provide retained grants, deterministic repair, bounded waiting age and explicit failure. Safety always overrides liveness. `src/collision_monitor/heuristic.py::DeterministicComponentHeuristic._greedy_pass` solves the binary no-good clauses by trial assignment plus implication propagation; `_repair_pass` specifically searches progress after an all-Pause result. `AlarmCode.UNRESOLVABLE_WITH_PAUSE_RESUME` is emitted when no propagated Resume seed yields a valid assignment, and the README explicitly limits liveness to the available Pause/Resume controls. |
| 3 | Deterministic behaviour | PASS | `src/collision_monitor/conflicts.py::decompose_connected_components` uses sorted BFS; model and component inputs are sorted by robot ID. `src/collision_monitor/optimiser.py::CpSatComponentOptimiser.optimise` uses one worker and a fixed seed, and `build_objective_encoding` adds a lower-order robot-ID rank term. `src/collision_monitor/heuristic.py::DeterministicComponentHeuristic.solve` uses stable ordering. Actual solver elapsed time is correctly treated as non-deterministic runtime metadata in `tests/unit/test_engine.py::test_identical_input_and_state_are_reproducible_regardless_of_input_order`. |
| 4 | Separation of core logic from RabbitMQ | PASS | `src/collision_monitor/engine.py`, `geometry.py`, `conflicts.py`, `priority.py`, `optimiser.py`, `heuristic.py` and `grants.py` contain no RabbitMQ, sleeping, I/O or wall-clock calls. Broker-independent protocols are in `src/collision_monitor/transport/base.py`; `src/collision_monitor/service.py::CollisionMonitorService` composes them; aio-pika is confined to `transport/rabbitmq.py`, CLI wiring and the simulator adapter. |
| 5 | Test quality | PASS | The suite has 213 passing unit, seeded property-style and in-memory integration tests. `tests/unit/test_safety_properties.py` covers geometry invariants, every Boolean encoding, permutations, final-envelope safety, component equivalence, hard-constraint validation, grants, fairness, stale robots, repair safety and existing overlap. `tests/unit/test_geometry.py::test_polygonal_buffer_conservatively_contains_requested_round_margin` checks margin circumscription, while the service tests cover retry identity across runs and explicit stale-state alarms. Total measured coverage is 86%; `engine.py` is 95%. The live broker test is explicitly opt-in, and robust consumer reconnection remains comparatively under-tested. |
| 6 | One-command build and run | PASS | `Dockerfile` provides separate wheel-builder, non-root runtime and test stages using a digest-pinned Python 3.12.13 image. `docker-compose.yml` uses digest-pinned RabbitMQ 3.13.7 and provides health gating, bounded logs, read-only application containers, a demo profile and a test profile. Direct Python dependencies and development tools are exactly pinned in `pyproject.toml`. `README.md` and `Makefile` document the build, run, demo, test and teardown commands. Current build and smoke results are recorded below. |
| 7 | Observability and reproducibility | PASS | `src/collision_monitor/service.py::CollisionMonitorService._build_trace` records `run_id`, observed ages, components, forbidden combinations, priorities, grants, solver status, actual and configured solver timing, final actions, validation, alarms and publication failures. `run_tick` emits `decision_tick_critical_alarm` at CRITICAL and `prolonged_state_loss_alarm` at WARNING. `src/collision_monitor/logging_utils.py::JsonFormatter` emits one redacted JSON object per line, and WKT output is opt-in through `CollisionDecisionEngine._detailed_geometry_trace`. |
| 8 | Documentation consistency | PASS | `README.md` and `docs/action_message_schema.md` document safety-before-publication, conditional liveness, explicit unresolvable and prolonged-staleness alarms, timestamp ordering, the complete action schema and at-least-once delivery. `src/collision_monitor/service.py::CollisionMonitorService.__init__` creates or accepts one `run_id`; `transport.base.ActionMessage.idempotency_key` and RabbitMQ `message_id` use `(run_id, device_id, tick_id)`, matching the documented restart-safe consumer key. Stale robots are explicitly retained until trusted external evidence permits removal. |
| 9 | Unnecessary complexity | PASS | The implementation uses explicit dataclasses, Pydantic only at boundaries, standard mappings and pure geometry/constraint functions. `GrantManager` is the largest policy class, but its preview/commit split prevents persistent fairness state from changing before global validation. No MAPF, replanning, LNS, Benders decomposition or distributed infrastructure is present. |

## Five highest-risk remaining defects or limitations

1. **Medium:** `state_store.LatestStateStore` has no trusted robot-retirement
   input. `CollisionMonitorService` deliberately retains disappeared robots;
   after the stale timeout they remain forced-Pause obstacles and can
   permanently block otherwise valid progress. The explicit alarm makes this
   visible, but external operational resolution is still required.
2. **Medium operational:** `CollisionMonitorService._publish_with_retry` and
   `RabbitMQTransport.publish_action` provide at-least-once delivery. An
   ambiguous publisher-confirm failure may duplicate an action. The restart-safe
   key is documented, but robot consumer behaviour is outside this repository.
3. **Low:** grants, waiting ages and tick sequence are process-local. A restart
   creates a new `run_id` and avoids identity collision, but fairness history is
   intentionally reset.
4. **Low:** `collision_monitor.__main__::_check_rabbitmq_health` checks RabbitMQ
   TCP reachability, not decision-loop progress. A failed main loop exits the
   process and relies on the container restart policy.
5. **Low scalability:** `conflicts.AllPairsCandidateGenerator.generate` remains
   `O(n²)`. This is explicit and appropriate for the take-home scope, but it
   bounds fleet size at a one-second tick interval.

## Known limitations

- Liveness is conditional on the Pause/Resume interface; the fallback is a
  feasibility policy, not an exact utility optimiser.
- Collision reasoning covers the current pose to one next path node; it does
  not perform route replanning or long-horizon reservation.
- Candidate generation is all-pairs, so each tick performs `O(n²)` pair work.
- The rotation-safe circumscribed-square sweep is deliberately conservative and
  may reduce throughput for large heading changes.
- Grants, waiting ages and tick numbering are process-local and are lost on
  restart; the new `run_id` keeps action identity distinct across that reset.
- Already-overlapping current footprints cannot be made geometrically safe by
  Pause; the monitor publishes fail-safe Pause actions and a critical alarm.
- RabbitMQ action delivery is at least once, not exactly once.
- Tick IDs are process-local; `run_id` makes the composite identity
  restart-safe.
- The health command checks RabbitMQ TCP reachability rather than internal
  decision-loop progress.

## Prioritised fixes fitting within four hours

1. **One to two hours:** define a trusted external robot-retirement signal and
   test that only this signal, never elapsed time alone, removes a stale
   obstacle.
2. **Under one hour:** add a robot-consumer contract fixture demonstrating
   `(run_id, device_id, tick_id)` duplicate suppression while retaining
   at-least-once broker semantics.
3. **One to two hours:** expose a minimal internal decision-loop heartbeat to
   the existing health command without adding a web framework.
4. **One to two hours:** record and alert on decision-loop overruns relative to
   `tick_interval_seconds`.
5. **Two to four hours:** benchmark all-pairs construction at the expected
   maximum fleet size before deciding whether a spatial candidate index is
   justified.

## Commands actually run

### Fresh baseline

- `.venv/bin/python -m pytest --cov=collision_monitor --cov-report=term-missing`
  — 209 passed, 1 skipped in 2.39 seconds; total coverage 85%, engine coverage
  95%. The skip is the explicitly opt-in live RabbitMQ test.
- `.venv/bin/python -m ruff check src tests` — passed.
- `.venv/bin/python -m ruff format --check src tests` — passed; 44 files
  already formatted.
- `.venv/bin/python -m mypy` — passed; 22 source files checked.
- `.venv/bin/python -m pip check` — no broken requirements.
- `docker compose config --quiet` — passed.
- `.venv/bin/python -m collision_monitor --help` — exited 0 and listed `run`,
  `validate-config` and `healthcheck`.
- Docker Engine 29.7.1 and Docker Compose 5.3.1 are available.
- A fixed-seed, 20,000-instance four-variable diagnostic did not find a
  heuristic feasibility false-negative.

### Final correction verification

- `.venv/bin/python -m pytest` — 213 passed, 1 skipped in 1.28 seconds.
  The skipped test is the explicitly opt-in live RabbitMQ integration test.
- `.venv/bin/python -m pytest --cov=collision_monitor
  --cov-report=term-missing` — 213 passed, 1 skipped in 2.46 seconds; total
  coverage 86%, engine coverage 95%.
- `.venv/bin/python -m ruff check src tests` — passed.
- `.venv/bin/python -m ruff format --check src tests` — passed; 44 files
  already formatted.
- `.venv/bin/python -m mypy` — passed; 22 source files checked.
- `.venv/bin/python -m pip check` — no broken requirements.
- `docker compose config` — exited 0 and produced the resolved configuration.
- `docker compose -p collision-monitor-final-correction build --no-cache
  monitor` — exited 0; the `collision-monitor:local` runtime image built
  successfully without layer-cache reuse.
- `.venv/bin/python -m collision_monitor.simulator.cli --scenario
  scenarios/three_way.yaml --transport memory` — exited 0; all three robots
  reached goal in 4 ticks, with 3 Pause commands, 3 Resume commands and no
  alarms.
- `docker compose -p collision-monitor-final-correction --profile demo up --build
  --attach simulator --abort-on-container-exit --exit-code-from simulator` —
  exited 0. RabbitMQ and the monitor became healthy; the RabbitMQ-backed
  three-way scenario completed in 4 ticks with all three robots at goal and no
  alarms.
- `docker compose -p collision-monitor-final-correction --profile demo down
  --volumes --remove-orphans` — exited 0 and removed only the isolated audit
  containers, network and temporary RabbitMQ volume.
- `docker compose -p collision-monitor-readme-verification run --rm --build
  test` — exited 0; all 214 tests passed in the hermetic test container,
  including the live RabbitMQ round-trip test documented by the README.
- `docker compose -p collision-monitor-readme-verification down --volumes
  --remove-orphans` — exited 0 and removed the isolated command-verification
  container, network and temporary RabbitMQ volume.
- `COMPOSE_PROJECT_NAME=collision-monitor-readme-up-verification docker compose
  up --build` — RabbitMQ and the monitor both became healthy; SIGINT then
  stopped both services gracefully, as expected for this foreground command.
- `docker compose -p collision-monitor-readme-up-verification down --volumes
  --remove-orphans` — exited 0 and removed the isolated startup-verification
  containers, network and temporary RabbitMQ volume.
