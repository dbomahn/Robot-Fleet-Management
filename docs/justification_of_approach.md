# Justification of Approach — Collision Monitor

## 1. Executive summary

I treat the Collision Monitor as a **discrete-time resource-scheduling problem on fixed routes**. At each approximately one-second tick, the routes are already given; the monitor decides only whether each robot may execute its next movement (`Resume`) or must remain at its current pose (`Pause`).

The implemented decision process is:

1. retain the latest valid state for every known robot;
2. construct conservative occupied regions for both possible actions;
3. evaluate all four `Pause`/`Resume` combinations for every unordered robot pair;
4. represent unsafe combinations as hard binary constraints;
5. build one conflict graph and solve its connected components independently;
6. use time-limited CP-SAT for small components and a deterministic bounded heuristic otherwise;
7. retain a geometrically valid right-of-way decision until the selected robot clears the conflict;
8. repair avoidable all-Pause decisions where a validated progress action is found;
9. validate the complete fleet decision independently of the optimiser and graph decomposition;
10. publish one logical action per known robot and emit a structured, reproducible tick trace.

This is a **component-decomposed matheuristic**: the feasibility model is exact, small components are optimised with a mathematical-programming solver, and a bounded heuristic protects the service-time budget when exact optimisation is unsuitable.

The three-day deadline strongly influenced the scope. I prioritised correct geometry, explicit feasibility constraints, deterministic behaviour, adversarial tests, observability and a complete runnable service. I did not implement route replanning, full multi-agent path finding, a long rolling horizon, Large Neighbourhood Search or Benders decomposition. Those methods would require additional information and validation effort without improving the most important deliverable: a safe, understandable and reproducible `Pause`/`Resume` monitor.

---

## 2. Scope, assumptions and guarantees

### 2.1 Scope

The monitor does not assign tasks, generate routes or change path geometry. Its input is the latest reported state and remaining path of each robot. Its output is one binary decision per known robot:

- `Resume`: authorise the next path movement;
- `Pause`: remain at the current pose.

The implementation follows the assignment's discrete movement semantics: after `Resume`, the simulator advances exactly one path node on the next step; after `Pause`, it does not move.

A paused robot remains a physical obstacle. This point is central to both the safety constraints and the deadlock analysis.

### 2.2 State handling

States are stored by `device_id`. Staleness is measured from local receive time rather than from the robot's source timestamp. Older source timestamps are rejected; an equal timestamp may be accepted as a corrected retransmission according to the documented policy.

A stale robot is forced to `Pause` and remains represented by its last known occupied region. The store does not automatically remove it merely because time has elapsed. Prolonged state loss emits the alarm:

```text
PROLONGED_STATE_LOSS_STATIC_OBSTACLE
```

This may block progress indefinitely, but automatically removing an unobserved robot would create an unjustified collision risk.

After a service restart, a robot for which no new state has been received is unknown rather than safely stationary: the current implementation does not persist its last pose. A production recovery procedure should therefore wait for fresh states from the expected fleet or restore state from a trusted persistent source before authorising movement.

### 2.3 Safety statement

Safety is conditional on the reported poses, robot dimensions, configured margin and one-step motion model being conservative representations of the physical system.

Within that model:

- boundary contact is unsafe;
- every unsafe pairwise action combination is excluded from the feasible region;
- every final fleet decision is checked again by a pairwise geometric validator before publication.

The final validator is separate from CP-SAT, the heuristic and the conflict-graph decomposition. It can therefore catch an optimisation-encoding, component-construction or solver-integration defect. It deliberately reuses the same geometry implementation, so it cannot detect a common error in the geometric model itself.

If the selected action envelopes intersect, the affected component is replaced with fail-safe Pause decisions and validated again. If even the stationary envelopes intersect, the input state is already unsafe. The engine emits a critical alarm and does not represent that situation as collision-free.

### 2.4 Conditional liveness

The monitor promotes progress through:

- a primary objective that maximises the number of robots resumed safely;
- persistent right-of-way grants that prevent premature order reversal;
- waiting-age terms that reduce repeated postponement;
- a separate repair pass for avoidable all-Pause outcomes.

Liveness is nevertheless conditional. The available controls are only `Pause` and `Resume`, routes are fixed, states may become stale, and the heuristic repair is bounded rather than exhaustive. Failure to find progress does not prove that no mathematical progress assignment exists.

When the implemented search does not produce a validated positive-progress assignment, the component remains paused and the monitor emits:

```text
UNRESOLVABLE_WITH_PAUSE_RESUME
```

This is an explicit and safe failure mode rather than an unsupported claim of universal liveness.

---

## 3. One-tick mathematical model

### 3.1 State transition and decision variable

Let \(\mathcal R_t\) be the set of robots known at tick \(t\). Robot \(i\)'s reported pose is

$$
q_{i,t} =
\left(
p^x_{i,t},
p^y_{i,t},
\theta_{i,t}
\right),
$$

and \(\widehat q_{i,t}\) is the next pose derived from its remaining path.

Define the binary action variable

$$
z_{i,t} =
\begin{cases}
1, & \text{Resume},\\
0, & \text{Pause}.
\end{cases}
$$

Under the assignment's one-node-per-step semantics,

$$
q_{i,t+1} =
\begin{cases}
\widehat q_{i,t}, & z_{i,t}=1,\\
q_{i,t}, & z_{i,t}=0.
\end{cases}
$$

This is not a route-planning model. The path and next node are inputs; \(z_{i,t}\) only controls whether the next movement is authorised during the present tick.

### 3.2 Oriented footprint

The robot is represented as a rectangle of length

$$
L=1.430\ \mathrm m
$$

and width

$$
W=0.630\ \mathrm m,
$$

centred at its reported pose. Before rotation, the length is aligned with the positive \(x\)-axis. For pose \(q=(p^x,p^y,\theta)\), the unbuffered footprint is

$$
F(q)=
\begin{bmatrix}
\cos\theta & -\sin\theta\\
\sin\theta & \cos\theta
\end{bmatrix}
\left(
[-L/2,L/2]\times[-W/2,W/2]
\right)
+
\begin{bmatrix}
p^x\\
p^y
\end{bmatrix}.
$$

A configurable margin \(\delta\geq0\) expands this footprint. Because Shapely represents a round buffer using polygonal chords, the implementation compensates the requested distance. With \(N\) segments per quadrant, it applies

$$
\delta' =
\frac{\delta}{\cos\!\left(\frac{\pi}{4N}\right)}
(1+\varepsilon),
$$

for a very small numerical guard \(\varepsilon>0\). This makes the polygonal buffer circumscribe, rather than under-approximate, the requested round margin.

### 3.3 Action-dependent occupied regions

For `Pause`, the occupied region is the compensated buffered footprint at the current pose:

$$E^0_{i,t}=F_{\delta'}(q_{i,t}).$$

For a `Resume` step with unchanged heading, the occupied region is

$$
E^1_{i,t}
=
\mathrm{conv}
\left(
F_{\delta'}(q_{i,t})
\cup
F_{\delta'}(\widehat q_{i,t})
\right).
$$

This convex hull covers straight translation between the endpoint footprints.

For a heading change, the endpoint rectangles alone are not sufficient. The implementation uses the rectangle's circumscribed radius,

$$
\rho =
\frac{1}{2}\sqrt{L^2+W^2}+\delta',
$$

constructs a square of half-size \(\rho\) at each endpoint centre, and takes the convex hull of those squares. This contains every rectangle orientation whose centre lies on the straight segment between the two poses.

The heading-change envelope is intentionally conservative. It may create false conflicts and reduce throughput, but it avoids under-representing intermediate occupancy under the implemented one-step centre-motion model.

This geometry is not presented as a certified physical continuous-time safety case. Certification would additionally require measured bounds for localisation, control tracking, braking, communication delay and the actual interpolation followed by the vehicle controller.

### 3.4 Pairwise compatibility and no-good constraints

For every unordered pair \(\{i,j\}\), the monitor evaluates all four action combinations:

$$
(z_i,z_j)\in
\{(0,0),(0,1),(1,0),(1,1)\}.
$$

A combination is unsafe if the corresponding occupied regions intersect. Each unsafe combination becomes a hard binary no-good:

| Unsafe assignment | Equivalent constraint |
|---|---:|
| \((z_i,z_j)=(0,0)\) | \(z_i+z_j\geq1\) |
| \((z_i,z_j)=(0,1)\) | \(z_j\leq z_i\) |
| \((z_i,z_j)=(1,0)\) | \(z_i\leq z_j\) |
| \((z_i,z_j)=(1,1)\) | \(z_i+z_j\leq1\) |

The familiar mutual-exclusion constraint \(z_i+z_j\leq1\) is therefore only one possible relation. The model also captures implications. For example,

$$
z_i\leq z_j
$$

means that robot \(i\) may move only if robot \(j\) also moves out of its blocking position.

If \((0,0)\) is unsafe, the robots' current occupied regions already intersect. The new action decision cannot undo that observed violation instantaneously; the monitor raises a critical alarm and returns fail-safe Pause decisions.

### 3.5 Feasible region

For a connected conflict component \(C\), let \(\mathcal N_C\) be its set of forbidden assignments and let \(\mathcal H_C\) contain valid hard assignments, such as stale robots forced to Pause or an active, geometrically valid grant forced to Resume.

The feasible region is

$$
\mathcal F_C =
\left\{
z\in\{0,1\}^{|C|}:
z\text{ satisfies every no-good in }\mathcal N_C
\text{ and every assignment in }\mathcal H_C
\right\}.
$$

Deadlines, battery level, load state and waiting time do not relax these constraints. They are used only to choose among solutions already inside \(\mathcal F_C\).

---

## 4. Conflict graph and decomposition

The monitor builds an undirected graph \(G=(V,E)\):

- one vertex represents one robot;
- an edge \((i,j)\) is present if at least one of the four pairwise action assignments is unsafe.

A pair for which all four assignments are safe creates no edge.

Connected components are found deterministically using sorted traversal. Components can be solved independently because no pairwise constraint joins robots in different components. This is an exact decomposition of the current pairwise model, not a heuristic clustering step.

The practical benefits are:

- the optimisation dimension depends on local contention rather than total fleet size;
- each component receives its own solver status, time limit and trace;
- large, geographically separated groups do not create one unnecessarily large model;
- future parallel execution is possible without changing the mathematical formulation.

Isolated, fresh robots that are not at goal Resume immediately. Robots at goal Pause. Stale robots remain stationary obstacles and are forced to Pause.

The final validator still checks all selected fleet-wide action pairs, including robots assigned to different nominal components. This duplicates some work deliberately to provide a last line of defence against graph-construction or solver-integration defects.

---

## 5. Feasible solutions, throughput and priority

This section separates three levels of decision-making:

1. **Safety constraints define the feasible region.**
2. **The primary objective maximises safe one-step progress.**
3. **Priority selects among maximum-progress feasible solutions.**

A priority score cannot make an unsafe action feasible.

### 5.1 Primary objective

For component \(C\), the first objective is

$$
\max_{z\in\mathcal F_C}
\sum_{i\in C} z_i.
$$

This maximises the number of robots authorised to execute their next movement during the current tick.

It prevents an all-Pause solution being preferred when a feasible positive-progress assignment has been identified. It is a one-period throughput objective, not a claim of global route optimality.

### 5.2 Secondary priority score

Several feasible assignments may Resume the same number of robots. A bounded integer score then represents business urgency and temporal fairness:

$$
S_i =
V_0
+V_i^{\mathrm{load}}
+U_i^{\mathrm{deadline}}
+U_i^{\mathrm{battery}}
+V_i^{\mathrm{wait}}
+V_i^{\mathrm{grant}}
+V_i^{\mathrm{clear}}.
$$

The terms have the following meanings:

| Term | OR interpretation | Practical role |
|---|---|---|
| \(V_0\) | base processing value | assigns positive value to progress |
| \(V_i^{\mathrm{load}}\) | job-class weight | favours loaded robots |
| \(U_i^{\mathrm{deadline}}\) | due-date urgency | increases as non-negative deadline slack decreases |
| \(U_i^{\mathrm{battery}}\) | state-dependent urgency | applies below the configured battery threshold |
| \(V_i^{\mathrm{wait}}\) | ageing term | increases with consecutive denied ticks, up to a cap |
| \(V_i^{\mathrm{grant}}\) | sequence-continuation value | supports an active right-of-way decision |
| \(V_i^{\mathrm{clear}}\) | blocking-clearance contribution | rewards movement that removes several conflicts |

The deadline, battery and waiting features are converted to bounded integers. This keeps the objective transparent, deterministic and suitable for CP-SAT.

### 5.3 Lexicographic implementation

The intended ordering is:

1. maximise the number of resumed robots;
2. subject to that, maximise aggregate secondary score;
3. subject to both, use a deterministic robot-identifier rank.

The implementation assigns a sufficiently large base reward \(M_C\) to every Resume decision:

$$
\max_{z\in\mathcal F_C}
\sum_{i\in C}(M_C+S_i)z_i.
$$

Let \(S_{\max}(C)\) be a valid upper bound on the total secondary score in component \(C\). Choosing

$$
M_C>S_{\max}(C)
$$

guarantees that one additional Resume dominates every possible secondary-score advantage of a solution with one fewer resumed robot.

A smaller, final rank term breaks otherwise exact ties without changing throughput or priority dominance.

### 5.4 Practical consequence

A loaded robot with an urgent deadline may still be forced to Pause if its Resume action is infeasible.

Conversely, a lower-priority robot may be selected when moving it is the only feasible way to release occupied capacity. Priority ranks safe alternatives; it does not decide which physical constraints may be ignored.

---

## 6. Exact and heuristic solution methods

### 6.1 Time-limited CP-SAT for small components

For a component below the configured size threshold, the monitor solves

$$
\begin{aligned}
\max \quad
& \sum_{i\in C}(M_C+S_i)z_i,\\
\text{subject to}\quad
& z\in\mathcal F_C,\\
& z_i\in\{0,1\}
\qquad \forall i\in C.
\end{aligned}
$$

The OR-Tools CP-SAT solver uses:

- a strict per-component wall-time limit, nominally 50 ms;
- one worker;
- a fixed random seed;
- deterministic ordering;
- disabled verbose solver output.

The status policy is:

| Status | Action |
|---|---|
| `OPTIMAL` | use the proven optimal assignment |
| `FEASIBLE` | use the validated incumbent without claiming optimality |
| `UNKNOWN` without an incumbent | use the heuristic fallback |
| `INFEASIBLE` | record the diagnostic and use the fallback |
| `MODEL_INVALID` | treat as an internal error and do not publish unchecked actions |

The formulation is exact, but a time-limited call does not necessarily prove optimality.

### 6.2 Deterministic bounded fallback

The fallback solves the same hard constraints but does not claim an optimal solution.

It:

1. orders unassigned robots by the same utility used in the exact model;
2. tentatively assigns `Resume` to the highest-ranked robot;
3. propagates all consequences of the two-literal no-goods;
4. rejects that tentative assignment if it creates a contradiction;
5. tries `Pause` instead;
6. continues until all variables are assigned.

There is no unrestricted recursive backtracking. This keeps execution bounded and reproducible.

If the first pass finds no progress or fails, a repair pass tests a configured number of candidate escape robots or implication-closed groups. It prefers decisions with high conflict-clearance value and validates every completed assignment.

### 6.3 What is exact and what is heuristic

- **Feasibility is mandatory.** Every published assignment must satisfy all no-goods and the fleet-wide geometric validator.
- **Optimality is conditional.** CP-SAT may prove it for a small component; the fallback may return a feasible but suboptimal result.
- **Search completeness is not claimed.** A bounded fallback may fail to find a positive-progress solution that a deeper search could discover.

The heuristic protects the real-time service budget; it does not relax safety.

### 6.4 Why use a hybrid method?

A fixed priority rule is fast but cannot correctly represent asymmetric implications such as \(z_i\leq z_j\). A single fleet-wide exact model is mathematically neat but its solution time depends on the size and density of the largest coupled conflict.

The hybrid method uses exact optimisation where the local problem is small and predictable, and deterministic construction where it is not. This is a practical OR compromise between objective quality and computational reliability.

---

## 7. Time complexity and the one-second budget

Let:

- \(n\) be the number of known robots;
- \(m\) be the number of conflict-graph edges;
- \(k\) be the size of one connected component;
- \(p\) be the number of two-literal no-goods in that component;
- \(r\) be the number of repair candidates actually examined.

### 7.1 Pairwise model construction

The present implementation examines every unordered pair:

$$
\binom n2=\frac{n(n-1)}2.
$$

It evaluates four action combinations for each pair. Treating the low-complexity polygon operations as bounded geometric work gives

$$
T_{\mathrm{pair}}(n)=O(n^2).
$$

### 7.2 Graph construction

After pairwise compatibility is available:

- adjacency construction is \(O(n+m)\);
- connected-component traversal is \(O(n+m)\), excluding deterministic sorting.

### 7.3 Exact optimisation

Binary optimisation has exponential worst-case complexity in component size \(k\). No polynomial guarantee is claimed.

The practical degradation is controlled by the wall-time limit:

1. small or easy component: optimality may be proved;
2. harder component: a feasible incumbent may be returned without proof;
3. no incumbent within the limit: fallback is used.

The most relevant scale measure is therefore the largest connected component, not only the total fleet size.

### 7.4 Heuristic complexity

The implementation repeatedly scans the component's clauses during propagation. A conservative bound for the greedy pass is

$$
O\!\left(k^2(p+1)\right).
$$

The bounded repair pass examines at most \(r\) candidates, giving

$$
O\!\left(rk^2(p+1)\right).
$$

If the configured cap allows every robot to be examined, \(r\leq k\). Working memory is \(O(k+p)\).

These are conservative implementation-level bounds, not claims about an idealised linear-time propagator.

### 7.5 Independent validation

The final fleet-wide validator performs another all-pairs check:

$$
O(n^2).
$$

The duplication is reasonable for a small take-home fleet because it protects the primary safety invariant. It is not the intended architecture for a 1,000-robot decision domain.

### 7.6 One-hertz service budget

The approximately one-second update interval must also accommodate:

- state aggregation and validation;
- envelope construction;
- graph construction;
- grant and fairness updates;
- final validation;
- structured tracing;
- action publication.

The current design limits each exact component solve. A production improvement would also impose one global tick deadline and allocate the remaining time explicitly across components.

---

## 8. Deadlock, oscillation and progress

### 8.1 A premature reversal that creates a new blocking state

Consider two robots \(A\) and \(B\) using the same narrow resource in opposite directions.

At tick \(t\), a feasible order is

$$
(z_A,z_B)=(1,0).
$$

Robot \(A\) starts clearing the resource while \(B\) waits.

Suppose a stateless policy reverses the order at tick \(t+1\) because \(B\)'s raw priority has become slightly higher:

$$
(1,0)\rightarrow(0,1).
$$

Robot \(A\) is now paused while still occupying capacity needed by \(B\). In the new geometry, the model may contain

$$
z_A\leq z_B,
\qquad
z_B\leq z_A,
\qquad
z_A+z_B\leq1.
$$

Together, these leave only

$$
(z_A,z_B)=(0,0).
$$

A feasible sequence existed, but pre-empting the selected robot before it cleared the constrained area created a zero-progress blocking state.

The same mechanism extends to a cycle of three or more robots, including the assignment's junction example: each paused robot may occupy capacity needed by the next robot in the sequence.

### 8.2 Rule that prevents premature reversal

The grant manager treats the selected right of way as a temporary non-pre-emptive scheduling commitment.

A grant is retained while:

- the robot remains in a relevant conflict;
- forcing it to Resume is still geometrically safe;
- it has not reached its goal;
- its state remains fresh;
- the configured clearance or fault-guard release condition has not been met.

Priority changes alone do not reverse the order.

If previously independent components merge and active grants conflict, the oldest geometrically feasible grant is preserved and newer incompatible grants are revoked with explicit reason codes.

The alternating-priority regression test verifies that a valid grant remains active even when raw priorities swap repeatedly.

### 8.3 Avoiding an unnecessary all-Pause decision

The primary objective will not choose all-Pause when a positive-progress assignment with a better objective has been found. A separate repair step protects against solver timeout, bounded-heuristic limitations and integration defects.

```text
if every non-terminal robot in a component is paused:
    rank candidate Resume decisions by:
        clearance contribution,
        waiting age,
        business score,
        deterministic identifier

    for each candidate within the repair limit:
        assign candidate to Resume
        propagate every forced implication

        if the forced group is feasible:
            pause the remaining unassigned robots
            validate the complete assignment

            if validation passes:
                accept the repair
                create or retain a continuation grant
                stop

    if no validated repair is found:
        keep the component paused
        emit UNRESOLVABLE_WITH_PAUSE_RESUME
```

The repair removes avoidable zero-progress outcomes that it finds. Because it is bounded, it is not a proof that no other progress assignment exists.

### 8.4 Oscillation and starvation

A stateless optimiser can alternate between equivalent assignments as deadlines, battery values or waiting terms change. The active grant prevents this repeated pre-emption until the selected robot clears.

After a grant is released, waiting age increases the secondary score of robots that have repeatedly been commanded to Pause. This is the classical scheduling technique of ageing. It reduces starvation where several alternatives remain feasible over time, but cannot create feasibility when physical constraints leave only zero progress.

---

## 9. Why pausing the lowest-priority robot can be wrong

Assume a low-priority robot \(B\) is already occupying a junction and a high-priority robot \(H\) is approaching it.

Suppose the geometry gives

$$
z_H\leq z_B,
$$

so \(H\) may move only if \(B\) also moves out of the blocking position, and

$$
z_B+z_H\leq1,
$$

so they may not both move during the same tick.

These constraints imply

$$
z_H=0.
$$

The feasible alternatives are

$$
(z_B,z_H)=(0,0)
\quad\text{or}\quad
(1,0).
$$

The throughput-first objective selects

$$
(z_B,z_H)=(1,0).
$$

The lower-priority robot moves because completing its current movement is the only feasible way to release the constrained capacity. The higher-priority robot waits because its movement is infeasible at that tick.

A rule that simply pauses the lowest-priority robot would select \((0,0)\), retain the blockage and unnecessarily stop the entire component.

In classical OR terms, a low-priority job already holding a scarce resource may need to complete before a higher-priority job can begin.

---

## 10. Engineering decisions and trade-offs

| Decision | Reason | Advantage | Limitation |
|---|---|---|---|
| One-tick binary model | matches the available `Pause`/`Resume` interface | small, explicit and easy to validate | does not optimise a long future horizon |
| Conservative action envelopes | safety takes precedence over utilisation | simple and robust geometric test | false conflicts may reduce throughput |
| Four pairwise assignments | paused occupancy creates more than mutual exclusion | captures exclusion, implication and already-unsafe states | current candidate generation is quadratic |
| Connected-component decomposition | constraints are local in the conflict graph | smaller independent subproblems | a dense component may still be difficult |
| Time-limited CP-SAT | exact formulation with controlled runtime | may prove optimality for small components | may return only an incumbent or none |
| Deterministic heuristic fallback | service must continue without unbounded search | bounded and reproducible | may miss a feasible or better solution |
| Persistent grants | prevent premature reversal | stable sequencing and less oscillation | a poor grant can temporarily reduce throughput |
| Waiting-age fairness | reduce repeated postponement | transparent anti-starvation mechanism | cannot overcome infeasibility |
| Independent final validation | protect against optimiser or graph defects | strong last safety barrier | repeats all-pairs geometric work |
| Pure engine separated from RabbitMQ | isolate decision logic from I/O | deterministic unit testing and replay | requires explicit transport and service layers |
| Structured per-tick trace | make decisions observable and reproducible | supports audit and diagnosis | increases log volume |

### 10.1 Alternatives not selected

#### Fixed-priority rule

A fixed priority order is easy to implement but does not represent asymmetric constraints, stale obstacles, blocking clearance or temporal order retention. It would be simpler but less general and easier to break with adversarial geometry.

#### Full multi-agent path finding or route replanning

Those methods choose routes. The assignment supplies routes and gives the monitor only `Pause`/`Resume` control. Implementing route planning would require a factory map, motion primitives and an interface for replacing paths, none of which are provided.

#### Rolling-horizon MIP or CP

A multi-period model could anticipate future blocking and optimise tardiness. It was not prioritised because it multiplies the model size by the horizon length and requires stronger assumptions about future movement, latency and state accuracy.

#### Large Neighbourhood Search

LNS is most useful for improving a large incumbent schedule. The implemented subproblems are small binary compatibility models. LNS would add tuning and stopping-policy complexity without addressing the first-order safety risks.

#### Benders decomposition

Benders decomposition would become meaningful after the factory is modelled as stable capacity-constrained zones, with a master problem for allocation or order and geometric subproblems for detailed feasibility. The present one-step pairwise model does not yet have a useful master/subproblem structure. Implementing valid cuts within three days would have displaced higher-value safety and testing work.

---

## 11. Improvements not implemented

The improvements below are ordered by expected engineering value.

### 11.1 Spatial broad phase

**Reason for deferral:** exhaustive pairing is simple, transparent and adequate for the small assignment scenarios.

**Proposed approach:**

```text
construct Pause and Resume envelopes
insert one union bounding box per robot into a spatial index

for each robot in deterministic order:
    query potentially overlapping bounding boxes
    add each unordered candidate pair to a sorted set

for each candidate pair:
    evaluate the four exact action combinations
```

The exact narrow-phase geometry and no-good model would remain unchanged.

### 11.2 Global tick-budget allocation

**Reason for deferral:** the current per-component CP-SAT limit already bounds the main unpredictable computation; a full end-to-end budget controller was lower priority than correctness.

**Proposed approach:**

```text
tick_deadline = monotonic_now + configured_tick_budget
reserve time for:
    final validation,
    trace construction,
    action publication

for component in deterministic order:
    remaining = tick_deadline - monotonic_now

    if remaining <= reserve:
        use deterministic heuristic
    else:
        solver_limit = min(default_component_limit, remaining - reserve)
        run CP-SAT with solver_limit

validate the complete fleet decision
publish the complete tick
```

### 11.3 Calibrated uncertainty and formal motion certification

**Reason for deferral:** the case study does not provide localisation error, braking distance, tracking error or communication-latency bounds. Inventing them would not constitute a valid safety case.

**Proposed approach:**

```text
collect certified bounds for:
    position error,
    heading error,
    state age,
    actuation latency,
    controller tracking error,
    braking displacement

derive a pose tube over one decision interval
compute a conservative occupied-set envelope for that tube
apply a documented numerical guard
validate against independent analytic and recorded cases
configure the reviewed bound with recorded provenance
```

### 11.4 Rolling-horizon resource scheduling

**Reason for deferral:** a one-step model plus continuation grants addresses the principal sequencing problem with much lower modelling and validation cost.

**Proposed approach:**

```text
for each tick:
    predict the next H route nodes for each robot
    create binary movement variables z[i, tau]
    add path-index transition constraints
    add pairwise space-time conflict constraints
    add grant-continuation and resource-capacity constraints
    optimise:
        safe progress,
        deadline tardiness,
        waiting,
        action changes
    solve within one global deadline
    publish only the first-period action
```

### 11.5 Semantic conflict zones and decomposition

**Reason for deferral:** no reviewed factory map or aisle topology was supplied.

**Proposed approach:**

```text
offline:
    buffer route segments by certified occupancy
    intersect routes
    cluster connected overlaps into candidate shared resources
    define deterministic zone entries, exits and capacities
    validate zones against the physical map

online:
    schedule access to the reviewed zones
    use detailed geometry as a final feasibility check
```

Such zones could later support a rolling-horizon MIP, CP model or Benders-style decomposition.

---

## 12. Production deployment considerations

The important scale is the number of robots within one coupled decision domain, not merely the organisation's total fleet.

### 12.1 Ten robots

Ten robots create

$$
\binom{10}{2}=45
$$

unordered pairs and at most 180 pairwise action-combination checks before final validation.

The present architecture is suitable after measured latency testing:

- one active monitor instance;
- one RabbitMQ service with durable queues;
- exhaustive pairwise construction;
- CP-SAT for small components and deterministic fallback;
- complete fleet-wide validation;
- JSON tick traces and replayable scenarios;
- optional passive standby with a single-writer lease.

Operational checks should include p50, p95 and p99 tick duration, stale-state alarms, publication failures, restart recovery and deterministic replay.

### 12.2 One hundred robots

One hundred robots create

$$
\binom{100}{2}=4{,}950
$$

pairs and up to 19,800 action-combination checks before the final validator.

A single process may remain sufficient, but that must be demonstrated by load testing. I would introduce:

- a spatial broad phase;
- a global tick budget;
- component-size and constraint-density metrics;
- active/passive failover with one logical writer;
- bounded queues and backpressure;
- explicit handling of duplicate, delayed and out-of-order messages;
- fault-injection tests for broker reconnect and partial publication.

Independent components could be solved concurrently only if results are collected deterministically and the complete fleet decision is still validated before publication.

### 12.3 One thousand robots

One thousand robots create

$$
\binom{1000}{2}=499{,}500
$$

pairs and almost two million action-combination checks before the independent validation repeats all-pairs work.

The current single-process \(O(n^2)\) implementation is not appropriate.

A migration path would be:

1. partition the factory into reviewed spatial cells or capacity-constrained zones;
2. assign each zone to one deterministic decision owner;
3. partition the state stream by current and near-future zone;
4. solve local connected components within each owner;
5. represent boundary movements as coupling constraints;
6. require a destination reservation before authorising a cross-zone movement;
7. maintain one logical publisher per robot;
8. retain ordered events for replay and recovery;
9. enforce bounded queues, backpressure and explicit overload behaviour.

A cross-zone protocol could be:

```text
source owner detects a proposed boundary movement
source requests a time-limited reservation from destination owner

destination checks its local feasible region

if capacity is reserved:
    source treats the reservation as a hard constraint
    source may authorise Resume
else:
    source authorises Pause

after confirmed entry:
    transfer decision ownership

on timeout or disagreement:
    authorise Pause and retry
```

Action delivery remains at least once. Each logical decision carries the restart-safe identity

$$
(\text{run\_id},\text{device\_id},\text{tick\_id}),
$$

so consumers can handle duplicates idempotently.

The distributed protocol must first be validated in one process. Distribution should not be used to conceal an unverified local feasibility model.

---

## 13. Engineering architecture and observability

The decision engine has no RabbitMQ dependency, sleeping or implicit wall-clock reads. It receives snapshots, `now_ms` and `tick_id` explicitly.

The main implementation responsibilities are:

| Module | Responsibility |
|---|---|
| `geometry.py` | oriented footprints and action envelopes |
| `conflicts.py` | four-way pair compatibility and graph decomposition |
| `priority.py` | bounded secondary scores and throughput dominance |
| `optimiser.py` | time-limited CP-SAT model |
| `heuristic.py` | deterministic propagation and repair |
| `grants.py` | continuation grants, fairness and all-Pause handling |
| `engine.py` | complete decision pipeline and global validation |
| `state_store.py` | latest-state ordering and staleness |
| `service.py` | ticks, publication retries and structured trace |
| `transport/rabbitmq.py` | RabbitMQ-specific I/O |
| `simulator/` | deterministic assignment scenarios |

The per-tick trace records the information needed to reproduce and explain a decision:

- source-state ages;
- connected-component membership;
- forbidden action combinations;
- priority breakdowns;
- active grants;
- solver status, wall time and time limit;
- final actions and reason codes;
- safety-validation result;
- alarms and publication failures.

The service creates one logical decision for each known robot per tick. RabbitMQ publication is at least once because an ambiguous confirm failure may cause a retry and duplicate delivery. One `run_id` is generated for each process execution, and the action identity is

$$
\(\text{run\_id},\text{device\_id},\text{tick\_id}\).
$$

Retries within the same run retain the same identity; equal process-local tick numbers from separate runs do not collide.

---

## 14. Testing strategy and verification

The test suite emphasises safety invariants rather than relying only on example scenarios.

### 14.1 Focused and adversarial tests

The tests cover:

- footprint dimensions, rotations and boundary contact;
- conservative translation and intermediate-heading envelopes;
- conservative safety-margin buffering;
- all four no-good truth tables;
- deterministic component decomposition;
- throughput dominance and deterministic tie-breaking;
- CP-SAT and heuristic feasibility;
- implication propagation and bounded repair;
- grant acquisition, retention, release and merge behaviour;
- alternating raw priorities under an active grant;
- waiting-age ordering after grant release;
- stale-state and out-of-order-state policies;
- all-Pause repair;
- already-overlapping current footprints;
- malformed transport messages;
- restart-safe action identity;
- trace completeness and actual solver timing.

### 14.2 Scenario and service tests

The deterministic scenarios include:

- no conflict;
- two-robot perpendicular crossing;
- the assignment's three-way junction;
- the low-priority blocker;
- a deliberately unresolvable arrangement;
- stale-state handling.

For resolvable scenarios, tests assert geometric safety and bounded goal completion. Asynchronous service tests cover arbitrary state arrival order, multiple updates before a tick, exactly one logical action per known robot, bounded publication retries and deterministic trace fields.

The final reported verification was:

- 213 tests passed;
- one explicitly opt-in live RabbitMQ integration test skipped in the standard suite;
- 86% overall coverage and 95% coverage for `engine.py`;
- Ruff lint and format checks passed;
- mypy passed;
- `pip check` passed;
- Docker Compose configuration validation passed;
- a no-cache production image build passed;
- in-memory and RabbitMQ-backed three-way scenarios each reached all goals in four ticks with no alarms.

Coverage is evidence that code was exercised, not a direct correctness percentage. The more important evidence is the focused coverage of geometry, no-good construction, exact and heuristic feasibility, grants and the independent final validator.

---

## 15. Conclusion

The implemented monitor is deliberately narrower than a complete fleet-management system. It solves the decision that the available interface actually permits: which robots may execute one fixed-route movement safely during the current tick.

The approach can be stated in classical OR terms:

1. binary action variables describe Pause and Resume;
2. pairwise geometry defines the feasible region;
3. connected components decompose independent subproblems;
4. a lexicographic objective maximises safe throughput before business priority;
5. time-limited CP-SAT provides exact optimisation where practical;
6. a deterministic bounded heuristic provides predictable fallback;
7. continuation grants preserve a safe non-pre-emptive order;
8. independent validation protects the published fleet decision.

The principal trade-off is intentional. The model does not optimise a long planning horizon, but its hard constraints are explicit, its decisions are reproducible, its failure modes are honest and its scale limitations are clear. Within the three-day implementation budget, that is a more defensible engineering result than a broader but weakly validated optimisation architecture.
