# Justification of approach — Collision Monitor

## 1. Summary

The implemented monitor makes one binary decision for each known robot on each
tick: `Pause` or `Resume`. It does not change routes. It constructs
conservative one-step action envelopes, evaluates all four action combinations
for every unordered robot pair, decomposes the resulting conflict graph and
solves each connected component. Small components use time-bounded CP-SAT;
other components use a deterministic bounded heuristic. Grants, waiting age
and an all-Pause repair reduce oscillation and avoid some zero-progress
outcomes. A fleet-wide validator checks the selected envelopes before the
service publishes actions. [T1, T3–T9]

The decision engine is independent of RabbitMQ. The asynchronous service owns
state ingestion, clocks, ticks, traces and publication; the engine receives an
immutable snapshot plus explicit `now_ms` and `tick_id` values. [T10]
The [implemented architecture](architecture.md) contains the corresponding
Mermaid component and decision-flow diagrams.

## 2. Scope, assumptions and guarantees

### 2.1 Scope and state assumptions

The model uses the latest accepted state for each robot. A source timestamp
older than the stored timestamp is rejected; an equal timestamp is accepted in
receive order as a corrected retransmission. Staleness is calculated from
local receive time rather than the robot timestamp. [T11]

The geometric reasoning assumes that the reported current pose and next path
pose describe the movement to be controlled, that the robot follows the
published action, and that its centre translates along the straight segment
between those poses during a Resume step. The configured margin is a policy
input, not a measured uncertainty certificate.

### 2.2 Safety statement

Safety is conditional on the state, dimensions, margin and one-step motion
model being conservative representations of the physical system. Within that
model, boundary contact is unsafe and every selected action set is subjected
to a fleet-wide pairwise envelope check before publication. The validator is
independent of CP-SAT, the heuristic and conflict-graph decomposition, but it
deliberately reuses the same geometry model; it cannot detect an error in that
shared model. [T2, T9]

If selected envelopes intersect, the affected graph components are replaced
with Pause decisions and checked again. If even the stationary envelopes
intersect, the engine records a critical alarm, does not commit grant or
waiting-age state, and returns explicit fail-safe Pause decisions. The service
publishes those Pause commands with the alarm; it does not describe the
already-intersecting geometry as safe. [T9]

### 2.3 Liveness statement

Liveness is conditional on the available Pause/Resume controls, fresh enough
state, and the implemented solver or bounded heuristic finding a validated
positive-progress assignment. An active grant prevents priority changes alone
from reversing an established safe order, and waiting age affects later
ordering after the grant is released. [T7, T8]

The heuristic and its repair pass are bounded, not exhaustive. Failure to find
progress therefore does **not** prove that no progress assignment exists. When
the implemented search does not find a validated progress assignment, the
component receives fail-safe Pause decisions and the
`UNRESOLVABLE_WITH_PAUSE_RESUME` alarm. [T6, T8]

### 2.4 Stale robots

The latest-state store does not evict robots because time has elapsed. A stale
robot remains a known stationary obstacle indefinitely and is forced to Pause.
Each stale tick includes the `PROLONGED_STATE_LOSS_STATIC_OBSTACLE` alarm.
This is a deliberate safety policy and may block progress for other robots.
[T9, T11]

## 3. Geometric safety model

### 3.1 Pose and action notation

For robot (i) at tick (t), the pose is

\[
q_{i,t}=(p_{x,i,t},p_{y,i,t},\theta_{i,t}).
\]

The next pose derived from the remaining path is denoted
\(\widehat q_{i,t}\). The separate binary action variable is

\[
z_{i,t}\in\{0,1\},\qquad
z_{i,t}=0\ \text{for Pause},\qquad
z_{i,t}=1\ \text{for Resume}.
\]

The simulator advances exactly one path node on the next step after Resume and
does not move after Pause. An empty path keeps the current pose and marks the
robot at goal. [T1, T14]

### 3.2 Implemented footprint and motion envelopes

Let (L=1.430\,\mathrm{m}), (W=0.630\,\mathrm{m}), and

\[
R=[-L/2,L/2]\times[-W/2,W/2].
\]

With the length initially aligned to the positive (x)-axis, the physical
footprint at (q=(p_x,p_y,\theta)) is

\[
F(q)=
\begin{bmatrix}
\cos\theta & -\sin\theta\\
\sin\theta & \cos\theta
\end{bmatrix}R+
\begin{bmatrix}p_x\\p_y\end{bmatrix}.
\]

Shapely approximates a round buffer with straight chords. With
\(N=8\) segments per quadrant and requested margin \(\delta\), the code uses

\[
\delta'=
\frac{\delta}{\cos\!\left(\frac{\pi}{4N}\right)}
\left(1+10^{-12}\right).
\]

The chord apothem is then greater than \(\delta\), so the polygonal buffer
circumscribes rather than under-approximates the requested round offset. The
Pause envelope is the compensated buffered current footprint. [T2]

For equal headings, the Resume envelope is

\[
E^1_{i,t}=\operatorname{conv}\!\left(
F_{\delta'}(q_{i,t})\cup
F_{\delta'}(\widehat q_{i,t})
\right),
\]

which covers straight translation between the endpoint footprints. For a
heading change, endpoint rectangles alone are insufficient. The implementation
uses

\[
\rho=\frac{1}{2}\sqrt{L^2+W^2}+\delta'
\]

and constructs an axis-aligned square of half-size \(\rho\) at each endpoint.
The convex hull of those squares covers every intermediate centre on the
straight segment and every rectangle heading at that centre. This is more
conservative than the equal-heading hull and may reduce throughput. [T2]

This implemented coverage is not a formal continuous-time certificate for a
physical robot. Such certification would also require verified bounds for
pose interpolation, localisation error, actuation, braking and communication
latency. Section 12.3 separates that work from the geometry already present.

### 3.3 Pairwise compatibility

For every unordered pair \(\{i,j\}\), the implementation evaluates

\[
(z_i,z_j)\in\{(0,0),(0,1),(1,0),(1,1)\}.
\]

An intersecting envelope pair is forbidden. Each forbidden assignment
\((a,b)\) is stored as the two-literal clause

\[
(z_i\ne a)\lor(z_j\ne b).
\]

The conflict graph contains an edge whenever at least one of the four
assignments is forbidden. All-safe pairs create no edge. [T3]

## 4. Conflict graph and component decisions

`AllPairsCandidateGenerator.generate` currently emits every unordered pair, so
candidate generation is \(O(n^2)\). Connected components are produced by
sorted breadth-first search. Isolated active robots Resume immediately,
at-goal robots Pause, and stale robots are forced to Pause. [T3, T9]

Components are solved independently because no pairwise no-good joins two
different components. The final validator nevertheless checks every selected
pair across the fleet, including nominally separate components. [T3, T9]

## 5. Priority and lexicographic objective

Let raw deadline and battery values be (D_i) and (B_i); they are not reused
as urgency symbols. Define:

- \(V_0\): configured base progress value;
- \(V_i^L\): configured loaded bonus or zero;
- \(U_i^D\): deadline urgency derived from non-negative slack;
- \(U_i^B\): configured low-battery bonus or zero;
- \(V_i^W\): capped waiting-age bonus;
- \(V_i^G\): active-grant continuation bonus or zero;
- \(V_i^C\): clearance count times the per-conflict bonus.

For horizon (H_D), maximum deadline urgency (U^D_{\max}), and explicit
`now_ms`, the implemented integer deadline term is

\[
s_i=\max(D_i-\text{now\_ms},0),
\]

\[
U_i^D=
\left\lfloor
\frac{U^D_{\max}\left(H_D-\min(s_i,H_D)\right)}{H_D}
\right\rfloor.
\]

The secondary score is

\[
S_i=V_0+V_i^L+U_i^D+U_i^B+V_i^W+V_i^G+V_i^C.
\]

For a component of size (k), `priority_bounds` calculates

\[
S_{\max}(k)=V_0+V^L_{\max}+U^D_{\max}+U^B_{\max}
+A_{\max}V^W_{\text{tick}}+V^G_{\max}+(k-1)V^C_{\text{edge}},
\]

\[
S^{C}_{\max}=kS_{\max}(k),qquad M_C=S^{C}_{\max}+1.
\]

Each Resume coefficient first receives (M_C+S_i). One extra Resume therefore
improves this utility by at least

\[
M_C-S^{C}_{\max}=1,
\]

even against the maximum possible aggregate secondary-score difference. [T4]

The optimiser then adds a deterministic robot-ID rank. If
\(r_i=k-\operatorname{index}(i)-1\),
\(R_{\max}=\sum_i r_i=k(k-1)/2\), and
\(\lambda=R_{\max}+1\), the final CP-SAT coefficient is

\[
c_i=\lambda(M_C+S_i)+r_i.
\]

Because \(\lambda>R_{\max}\), a one-point utility difference dominates every
possible aggregate rank difference. The tertiary tie-break cannot weaken
throughput dominance. [T4, T5]

## 6. Exact and heuristic decision logic

### 6.1 CP-SAT

For components no larger than the configured exact-size threshold,
`CpSatComponentOptimiser.optimise` creates one Boolean Resume variable per
robot, adds every forbidden pair assignment as a Boolean clause, applies safe
grant hard assignments and maximises the coefficients above. It uses one
worker, a fixed seed, disabled verbose logging and the configured wall-time
limit. `OPTIMAL` and `FEASIBLE` incumbents are usable; lack of an incumbent
requests fallback; `MODEL_INVALID` raises an internal error. Every returned
assignment is independently checked against the no-goods. [T5]

### 6.2 Deterministic fallback

The heuristic sorts robots by the same objective coefficients. For each
unassigned robot it tries Resume, propagates two-literal clauses to a fixed
point and tries Pause if Resume contradicts the current assignment. It performs
no recursive backtracking. If the greedy result has no progress or fails, the
repair pass tries at most the configured number of escape candidates, ordered
by clearance and utility. Each candidate may force a group through propagation;
remaining variables are tentatively paused and the completed assignment is
validated. [T6]

The result is either a validated assignment with progress or an explicit
fail-safe no-progress result. It is not an infeasibility proof. [T6]

## 7. Complexity

Let (n) be the fleet size, (k) a component size, (m) the number of graph
edges, (p) the number of two-literal clauses in that component, and
\(r\leq\min(k,R)\) the number of repair candidates allowed by configured cap
\(R\).

- all-pairs compatibility construction: \(O(n^2)\) geometric checks;
- adjacency construction and breadth-first decomposition: \(O(n+m)\), plus
  deterministic sorting;
- final fleet validation: \(O(n^2)\) envelope comparisons;
- stored graph, clauses and working assignments: \(O(n+m)\) fleet memory and
  \(O(k+p)\) heuristic working memory.

In `_propagate`, at most (k) assignments can be added and each pass scans all
\(p\) two-literal clauses, giving \(O(kp)\). `_try_assignment` also copies up
to (k) assignments. The greedy pass makes at most two trials for each of
\(k\) robots, so a conservative bound is

\[
T_{\text{greedy}}=O\!\left(k^2(p+1)\right).
\]

For each of (r) repair candidates, the code performs one Resume trial and up
to (k) Pause trials, each with propagation and assignment copying. Therefore

\[
T_{\text{repair}}=O\!\left(rk^2(p+1)\right),
\]

which becomes \(O(k^3(p+1))\) if the configured candidate cap permits all
robots. Sorting adds \(O(k\log k)\). This follows the actual repeated clause
scans rather than assuming a single additive clause pass. [T3, T6, T9]

CP-SAT retains exponential worst-case complexity; the implementation bounds
elapsed solver time rather than claiming a polynomial bound. [T5]

## 8. Grants, order retention and progress safeguards

### 8.1 Concrete premature-reversal example

The regression component contains `robot-a` and `robot-b` with the
Resume/Resume assignment forbidden. On tick 1, raw utilities are 200 for A and
100 for B, so A Resumes and receives a grant. On tick 2, the raw order is
reversed: A has 100 while B has either 201 or 1,000. Reversing the action to
\((z_A,z_B)=(0,1)\) would pre-empt A before its conflict cleared.

`GrantManager._safe_relevant_grants` retains A's geometrically feasible grant
as a hard Resume assignment. The optimiser consequently keeps
\((z_A,z_B)=(1,0)\) for five ticks despite alternating raw priority. This exact
sequence is asserted by
`test_alternating_raw_priority_does_not_preempt_active_grant`. [T7]

### 8.2 Release, merge and fairness

`GrantManager._reconcile_lifecycle` releases a grant at goal, after configured
conflict clearance, on staleness or sufficiently long disappearance, or at the
maximum lease fault guard. If components merge, `_safe_relevant_grants`
considers grants by acquisition tick and robot ID and keeps only a compatible
oldest-first set. A non-terminal paused robot gains capped waiting age; Resume,
goal or disappearance resets or removes it. [T7]

### 8.3 All-Pause handling

`GrantManager._validate_or_repair_components` invokes the bounded heuristic
when an entire conflict component is paused. A validated escape robot or
implication-closed group is accepted and may receive a grant. Otherwise every
robot remains paused and `UNRESOLVABLE_WITH_PAUSE_RESUME` is emitted. This is
honest fail-safe behaviour, not proof that the component has no mathematical
progress assignment. [T8]

## 9. Why the lower-priority blocker may move

Suppose a lower-priority blocker (B) is already in a junction and a
higher-priority robot (H) approaches. If geometry permits B to Resume while H
Pauses, but forbids H Resume while B Pauses, then priority cannot make H's
unsafe action feasible. When \((z_B,z_H)=(1,0)\) is the only validated
positive-progress choice, the throughput-first objective moves B and pauses H.
[T3, T9]

## 10. Engineering decisions and trade-offs

### 10.1 Conservative oriented-rectangle geometry

The implementation uses exact oriented endpoint rectangles. Equal-heading
translation uses their compensated buffered convex hull. Heading changes use
the convex hull of endpoint squares based on the rectangle's circumscribed
radius plus compensated margin, thereby covering intermediate headings under
the one-step centre-motion model. Chord-apothem compensation prevents the
finite Shapely round buffer from shrinking inside the requested margin. [T2]

The trade-off is additional false conflict, particularly for heading changes.
This conservatism is implemented and tested; formal physical continuous-time
certification is not.

### 10.2 Component-wise solving

Pairwise no-goods express mutual exclusion, implication and already-unsafe
all-Pause states in one binary form. Sorted connected components reduce each
solver call while preserving the complete pairwise model. The fleet-wide
validator provides a separate check against graph or solver integration errors,
but shares the geometry implementation. [T3, T5, T9]

### 10.3 Transport-independent engine and observability

`CollisionDecisionEngine.decide` performs no RabbitMQ access, logging, sleep or
clock read. `CollisionMonitorService` supplies snapshots and explicit time,
constructs action messages, publishes with bounded retries and records the tick
trace. The trace contains observed ages, constraints, components, priorities,
grants, solver timing, final actions, validation, alarms and publication
failures. [T10, T12]

## 11. Transport identity, delivery and restart

The service creates exactly one logical decision for every known robot on each
tick. A bounded publication retry reuses the same `ActionMessage`, but an
ambiguous publisher-confirm failure can produce a duplicate broker delivery.
RabbitMQ delivery is therefore at least once, not exactly once. [T12]

One `run_id` is created when the service instance starts and is included in
every action and tick trace. The action idempotency key and RabbitMQ
`message_id` are

\[
(\text{run\_id},\text{device\_id},\text{tick\_id}).
\]

Retries within a run retain the same key; equal process-local tick IDs from two
runs do not collide. Consumers must deduplicate using the full key. [T12, T13]

The latest-state store, grants and waiting ages are process memory. After a
restart, an unknown robot has no persisted pose and cannot safely be described
as a stationary obstacle. The implemented service waits for received states;
it does not publish an action for an unknown robot. Treating a robot as a
stationary obstacle after restart would require a persisted pose or another
trusted source, neither of which is implemented. [T10, T11]

## 12. Explicitly unimplemented refinements

### 12.1 Automatic conflict-zone extraction

The current system derives pairwise conflicts directly from action envelopes;
it has no semantic conflict-zone model. [T3] A possible offline extraction
process would be:

```text
buffer each route segment by the certified robot occupancy radius
intersect buffered routes from different route definitions
cluster connected overlaps into candidate conflict zones
derive deterministic zone identifiers, entries and exits
validate every zone against recorded trajectories and physical map data
publish only reviewed zone metadata to the decision system
```

This is not present in `src/collision_monitor`.

### 12.2 Calibrated uncertainty margins

The configured margin is currently a supplied scalar. It is not calculated
from sensor or latency evidence. [T2] A calibration process would be:

```text
collect bounded position, heading, state-age and controller-latency errors
derive translational displacement during observation and actuation delay
derive the radial effect of heading error for the robot rectangle
combine the bounds using an agreed safety case, without assuming independence
round the result upwards by the numerical geometry guard
validate the margin against held-out measurements and worst-case tests
configure the reviewed bound and record its provenance
```

This process and its data are not implemented.

### 12.3 Formal continuous-time certification

The implemented Resume envelope already covers intermediate headings under the
straight-centre, one-step model, and its compensated polygonal buffer does not
under-approximate the requested round margin. [T2] A stronger physical
certificate would additionally bind the actual controller trajectory and
uncertainty over continuous time:

```text
obtain a verified pose tube (p_x(tau), p_y(tau), theta(tau)) for tau in [0, 1]
include calibrated localisation, timing, braking and tracking-error bounds
prove that the occupied-set union is contained in a computable envelope
prove pairwise separation of those envelopes, including boundary contact
validate the implementation against independent analytic cases
```

No such certification is claimed by the present monitor.

## 13. Fleet-size consequences of the implemented model

### 13.1 Ten robots

Ten known robots create \(\binom{10}{2}=45\) unordered pairs and 180 action
combinations before the final validator. The implemented all-pairs approach is
the code path exercised by the supplied scenarios. [T3, T15]

### 13.2 One hundred robots

One hundred known robots create \(\binom{100}{2}=4{,}950\) pairs and 19,800
action combinations. The same code remains quadratic; this document makes no
latency claim for that fleet size. [T3]

### 13.3 One thousand robots

One thousand known robots create

\[
\binom{1000}{2}=499{,}500
\]

unordered pairs and 1,998,000 action-combination checks before the independent
all-pairs validation. The present implementation remains a single all-pairs
decision domain; it does not contain spatial ownership or distributed
coordination. [T3, T9]

A tick still creates one **logical** decision per known robot, so 1,000 known
robots imply 1,000 logical action messages. RabbitMQ delivery is at least once,
so retries may cause more than 1,000 physical deliveries. Every duplicate for
that tick retains the key
\((\text{run\_id},\text{device\_id},\text{tick\_id})\). [T12, T13]

No restart claim is made for robots whose poses have not been received in the
new run. Without a persisted pose, an unknown robot cannot be inserted into the
geometry as a stationary obstacle. [T11]

## 14. Testing and verification

The final host suite recorded in the [final audit](final_audit.md) passed 213
tests with one explicitly opt-in RabbitMQ test skipped. Coverage was 86%
overall and 95% for
`engine.py`. Ruff checking, Ruff format checking, mypy, `pip check` and
`docker compose config` passed. The no-cache monitor image built successfully.
The in-memory and RabbitMQ-backed three-way scenarios each completed in four
ticks with all three robots at goal and no alarms. A later hermetic Compose run
passed all 214 tests, including the live RabbitMQ round trip. [T15]

The fixed-seed property-style tests cover footprint symmetry, every no-good
truth table, input permutations, selected-envelope safety, component
equivalence, solver and heuristic feasibility, grant retention, waiting-age
ordering, stale actions, repair safety and current-overlap alarms. Focused
tests cover intermediate-heading envelopes, buffer circumscription, restart
identity, malformed input and trace contents. [T2–T13]

## 15. Implementation traceability

The following table is the reference for implemented-behaviour claims above.
Every file, symbol and test name exists in the repository.

| ID | Implemented behaviour | Source and symbol | Regression test |
|---|---|---|---|
| T1 | Input models and next-pose selection | `models.py`: `RobotState`, `RobotSnapshot.from_state` | `test_models.py::test_next_pose_skips_a_first_node_matching_current_pose`; `test_empty_path_uses_current_pose_and_marks_robot_at_goal` |
| T2 | Oriented footprint, intermediate-heading sweep, margin compensation and boundary conflict | `geometry.py`: `oriented_footprint`, `_conservative_buffer_distance`, `resume_envelope`, `geometries_conflict` | `test_geometry.py::test_resume_envelope_contains_intermediate_footprint_during_heading_change`; `test_polygonal_buffer_conservatively_contains_requested_round_margin`; `test_touching_rectangles_count_as_conflict` |
| T3 | Every unordered pair, four assignments, no-goods and deterministic components | `conflicts.py`: `AllPairsCandidateGenerator.generate`, `evaluate_pairwise_compatibility`, `build_conflict_model`, `decompose_connected_components` | `test_conflicts.py::test_pairwise_compatibility_contains_all_diagnostics_and_is_immutable`; `test_component_and_pair_order_is_deterministic`; `test_safety_properties.py::test_component_solves_equal_the_whole_small_pairwise_model` |
| T4 | Bounded secondary score and throughput dominance | `priority.py`: `priority_bounds`, `deadline_urgency`, `score_robot_priority` | `test_priority.py::test_one_extra_resume_dominates_all_secondary_score_differences`; `test_earlier_deadline_has_higher_urgency` |
| T5 | CP-SAT clauses, deterministic tie-break, hard assignments and result validation | `optimiser.py`: `build_objective_encoding`, `validate_component_decisions`, `CpSatComponentOptimiser.optimise` | `test_optimiser.py::test_all_four_forbidden_patterns_are_encoded`; `test_robot_id_rank_breaks_an_equal_utility_tie_deterministically`; `test_hard_assignment_from_grant_is_enforced` |
| T6 | Bounded greedy propagation, repair and independent validation | `heuristic.py`: `_propagate`, `DeterministicComponentHeuristic._greedy_pass`, `_repair_pass`, `validate_heuristic_decisions` | `test_heuristic.py::test_chain_of_implications_is_propagated`; `test_bounded_repair_prefers_highest_clearance_escape_robot`; `test_unsatisfiable_component_returns_explicit_no_assignment_result` |
| T7 | Grant retention, merge handling, release and waiting fairness | `grants.py`: `GrantManager._safe_relevant_grants`, `_reconcile_lifecycle`, `_update_fairness` | `test_grants.py::test_alternating_raw_priority_does_not_preempt_active_grant`; `test_component_merge_preserves_oldest_compatible_grant`; `test_waiting_age_updates_reset_and_cap_fairly` |
| T8 | All-Pause repair and explicit unresolved alarm | `grants.py`: `GrantManager._validate_or_repair_components`, `AlarmCode.UNRESOLVABLE_WITH_PAUSE_RESUME` | `test_grants.py::test_no_all_paused_repair_can_grant_implication_closed_group`; `test_unresolvable_component_returns_honest_alarm_and_fail_safe_pause` |
| T9 | Policies, stale alarm and global safety override | `engine.py`: `CollisionDecisionEngine.decide`, `validate_global_safety`, `EngineAlarmCode.PROLONGED_STATE_LOSS_STATIC_OBSTACLE` | `test_engine.py::test_stale_robot_is_a_forced_stationary_obstacle`; `test_global_validator_overrides_a_broken_nominal_graph`; `test_service.py::test_stale_state_uses_monotonic_receive_age_and_is_paused`; `test_safety_properties.py::test_current_footprint_overlap_emits_explicit_critical_alarm` |
| T10 | Pure engine and asynchronous service boundary | `engine.py`: `CollisionDecisionEngine.decide`; `service.py`: `CollisionMonitorService.run_tick` | `test_engine.py::test_identical_input_and_state_are_reproducible_regardless_of_input_order`; `test_service.py::test_periodic_loop_uses_configured_one_hertz_interval` |
| T11 | Latest-state ordering, receive-time staleness and indefinite retention | `state_store.py`: `LatestStateStore.update`, `LatestStateStore.snapshot`; `service.py`: `CollisionMonitorService.ingest_payload` | `test_state_store.py::test_store_keeps_latest_source_timestamp_per_robot`; `test_snapshot_is_sorted_immutable_and_marks_stale_from_receive_age`; `test_service.py::test_stale_state_uses_monotonic_receive_age_and_is_paused` |
| T12 | One logical action, bounded retry, `run_id` and tick trace | `service.py`: `CollisionMonitorService.__init__`, `_action_message`, `_publish_with_retry`, `_build_trace` | `test_service.py::test_each_tick_publishes_exactly_one_action_per_known_robot`; `test_publication_retries_then_succeeds_without_duplicate_success`; `test_trace_contains_deterministic_required_fields_without_wkt` |
| T13 | Composite idempotency key and RabbitMQ message identity | `transport/base.py`: `ActionMessage.idempotency_key`; `transport/rabbitmq.py`: `RabbitMQTransport.publish_action` | `test_service.py::test_equal_tick_ids_from_different_service_runs_do_not_collide`; `test_rabbitmq_transport.py::test_publish_declares_durable_robot_queue_and_confirms_utf8_json` |
| T14 | One-node-per-tick simulator semantics | `simulator/robot.py`: `SimulatedRobot.advance_for_tick`, `state_payload` | `test_simulator_robot.py::test_resume_advances_exactly_one_node_on_the_next_step`; `test_pause_keeps_current_pose_and_published_path_starts_there` |
| T15 | Scenario and complete verification results | `simulator/scenario.py`: `run_scenario`, `run_in_memory_scenario`; `docs/final_audit.md`: `Commands actually run` | `test_simulator_scenarios.py::test_resolvable_scenarios_reach_goals_without_intersecting_actions`; `test_rabbitmq_integration.py::test_persistent_action_round_trip_against_live_rabbitmq` |
