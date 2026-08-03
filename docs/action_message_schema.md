# Action message schema

The Collision Monitor publishes one logical action for every known robot on
each decision tick. RabbitMQ delivery is at least once, so a retry after an
ambiguous publisher-confirm failure may deliver the same logical action more
than once.

Consumers must handle actions idempotently using this composite key:

```text
(run_id, device_id, tick_id)
```

`run_id` is created once when the application service starts. Retries retain
the same `run_id`, `device_id` and `tick_id`. A restarted service creates a new
`run_id`, so a process-local tick ID reused after restart does not collide with
an action from the earlier run.

Each robot-specific durable action queue receives a UTF-8 JSON object with the
following fields:

| Field | JSON type | Meaning |
|---|---|---|
| `run_id` | string | Restart-unique service-run identifier. |
| `device_id` | string | Target robot identifier. |
| `action` | string | `Pause` or `Resume`. |
| `tick_id` | string | Deterministic process-local tick identifier. |
| `decision_timestamp` | integer | Decision time in milliseconds since epoch. |
| `source_state_timestamp` | integer | Robot source timestamp in milliseconds since epoch. |
| `reason_codes` | array of strings | Stable machine-readable decision reasons and tick alarms. |
| `reason_context` | array of strings | Concise human-readable explanations. |
| `decision_source` | string | `policy`, `cp_sat`, `heuristic`, `repair` or `fail_safe`. |
| `grant_active` | Boolean | Whether the robot holds a right-of-way grant after the decision. |

Example:

```json
{"action":"Resume","decision_source":"cp_sat","decision_timestamp":1720000000100,"device_id":"robot-17","grant_active":true,"reason_codes":["CP_SAT_COMPONENT_DECISION"],"reason_context":["The bounded exact optimiser selected this component action."],"run_id":"8e67ab21-d9a8-44d8-a7de-3c73d1e125a3","source_state_timestamp":1720000000000,"tick_id":"tick-00000042"}
```
