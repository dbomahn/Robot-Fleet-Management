"""Pure, transport-independent orchestration for fleet collision decisions.

The engine has no transport, sleeping, logging or clock dependencies. The
caller supplies both ``now_ms`` and the trace ``tick_id``. At-goal robots are
commanded Pause: this explicitly communicates that no movement is required and
prevents a harmless Resume from being counted as liveness progress.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
from itertools import combinations
from typing import Any

from collision_monitor.config import MonitorConfig
from collision_monitor.conflicts import ConflictModel, build_conflict_model
from collision_monitor.geometry import action_envelope, geometries_conflict
from collision_monitor.grants import (
    GrantDecisionPreview,
    GrantDecisionResult,
    GrantManager,
)
from collision_monitor.heuristic import DeterministicComponentHeuristic
from collision_monitor.models import (
    Action,
    DecisionSource,
    FleetDecision,
    RobotDecision,
    RobotSnapshot,
)
from collision_monitor.optimiser import CpSatComponentOptimiser, OptimisationResult
from collision_monitor.priority import PriorityBreakdown, score_component_priorities

ConflictModelBuilder = Callable[[Sequence[RobotSnapshot], MonitorConfig], ConflictModel]


def _plain_bounds(geometry_bounds: Sequence[float]) -> tuple[float, float, float, float]:
    """Convert four Shapely bounds to a statically precise plain tuple."""
    min_x, min_y, max_x, max_y = geometry_bounds
    return (float(min_x), float(min_y), float(max_x), float(max_y))


class EngineAlarmCode:
    """Stable engine-level safety and liveness alarm codes."""

    PROLONGED_STATE_LOSS_STATIC_OBSTACLE = "PROLONGED_STATE_LOSS_STATIC_OBSTACLE"
    CURRENT_FOOTPRINT_OVERLAP = "CURRENT_FOOTPRINT_OVERLAP"
    GLOBAL_SAFETY_VALIDATION_FAILED = "GLOBAL_SAFETY_VALIDATION_FAILED"
    FAIL_SAFE_GEOMETRY_REMAINS_UNSAFE = "FAIL_SAFE_GEOMETRY_REMAINS_UNSAFE"


@dataclass(frozen=True, slots=True)
class GlobalSafetyViolation:
    """One final action-envelope pair rejected by independent geometry checks."""

    robot_i: str
    action_i: Action
    robot_j: str
    action_j: Action
    bounds_i: tuple[float, float, float, float]
    bounds_j: tuple[float, float, float, float]

    def as_log_data(self) -> Mapping[str, Any]:
        """Return deterministic diagnostic data for a per-tick trace."""
        return {
            "robot_i": self.robot_i,
            "action_i": self.action_i.value,
            "robot_j": self.robot_j,
            "action_j": self.action_j.value,
            "bounds_i": self.bounds_i,
            "bounds_j": self.bounds_j,
        }


def _normalise_snapshots(
    snapshots: Sequence[RobotSnapshot],
) -> tuple[RobotSnapshot, ...]:
    """Validate snapshot types and IDs, then return stable robot-ID order."""
    if any(not isinstance(snapshot, RobotSnapshot) for snapshot in snapshots):
        raise TypeError("snapshots must contain RobotSnapshot values")
    ordered = tuple(sorted(snapshots, key=lambda item: item.state.device_id))
    for snapshot in ordered:
        if (
            not isinstance(snapshot.received_at_ms, int)
            or isinstance(snapshot.received_at_ms, bool)
            or snapshot.received_at_ms < 0
        ):
            raise ValueError("snapshot received_at_ms must be a non-negative integer")
        if not isinstance(snapshot.stale, bool):
            raise TypeError("snapshot stale flags must be Boolean")
    for previous, current in zip(ordered, ordered[1:], strict=False):
        if previous.state.device_id == current.state.device_id:
            raise ValueError(f"duplicate device IDs in snapshots: {current.state.device_id}")
    return ordered


def _derive_staleness(
    snapshots: Sequence[RobotSnapshot],
    *,
    now_ms: int,
    config: MonitorConfig,
) -> tuple[RobotSnapshot, ...]:
    """Apply the configured receive-age threshold without reading a clock."""
    stale_limit_ms = int(config.stale_timeout_seconds * 1_000)
    return tuple(
        replace(
            snapshot,
            stale=(snapshot.stale or now_ms - snapshot.received_at_ms >= stale_limit_ms),
        )
        for snapshot in snapshots
    )


def validate_global_safety(
    snapshots: Sequence[RobotSnapshot],
    decisions: Mapping[str, Action],
    config: MonitorConfig,
) -> tuple[GlobalSafetyViolation, ...]:
    """Rebuild and compare every final envelope independently of the graph.

    Boundary contact remains unsafe because ``geometries_conflict`` deliberately
    uses Shapely ``intersects``. No conflict-model edges or solver constraints
    are reused here, making this a separate last line of defence.
    """
    ordered = _normalise_snapshots(snapshots)
    robot_ids = {snapshot.state.device_id for snapshot in ordered}
    if set(decisions) != robot_ids:
        raise ValueError("global validation requires one action for every snapshot")
    if any(not isinstance(action, Action) for action in decisions.values()):
        raise TypeError("global validation decisions must contain Action values")

    envelopes = {
        snapshot.state.device_id: action_envelope(
            snapshot,
            decisions[snapshot.state.device_id],
            config,
        )
        for snapshot in ordered
    }
    violations: list[GlobalSafetyViolation] = []
    for snapshot_i, snapshot_j in combinations(ordered, 2):
        robot_i = snapshot_i.state.device_id
        robot_j = snapshot_j.state.device_id
        envelope_i = envelopes[robot_i]
        envelope_j = envelopes[robot_j]
        if geometries_conflict(envelope_i, envelope_j):
            violations.append(
                GlobalSafetyViolation(
                    robot_i=robot_i,
                    action_i=decisions[robot_i],
                    robot_j=robot_j,
                    action_j=decisions[robot_j],
                    bounds_i=_plain_bounds(envelope_i.bounds),
                    bounds_j=_plain_bounds(envelope_j.bounds),
                )
            )
    return tuple(violations)


def _component_for_robot(
    conflict_model: ConflictModel,
) -> Mapping[str, tuple[str, ...]]:
    """Map every robot ID to its deterministic connected component."""
    return {
        robot_id: component
        for component in conflict_model.connected_components
        for robot_id in component
    }


class CollisionDecisionEngine:
    """Stateful grant-aware decision policy with pure tick inputs and outputs."""

    def __init__(
        self,
        config: MonitorConfig,
        *,
        optimiser: CpSatComponentOptimiser | None = None,
        heuristic: DeterministicComponentHeuristic | None = None,
        grant_manager: GrantManager | None = None,
        conflict_model_builder: ConflictModelBuilder = build_conflict_model,
    ) -> None:
        self._config = config
        self._optimiser = optimiser or CpSatComponentOptimiser(config)
        self._heuristic = heuristic or DeterministicComponentHeuristic(config)
        self._grant_manager = grant_manager or GrantManager(config)
        self._conflict_model_builder = conflict_model_builder

    @property
    def grant_manager(self) -> GrantManager:
        """Return the current grant manager for read-only state inspection."""
        return self._grant_manager

    def _score_priorities(
        self,
        conflict_model: ConflictModel,
        grant_manager: GrantManager,
        *,
        now_ms: int,
    ) -> dict[str, PriorityBreakdown]:
        """Score all components using explicit time and provisional grant state."""
        waiting_ages = {
            robot_id: grant_manager.waiting_ages.get(robot_id, 0)
            for robot_id in conflict_model.snapshots
        }
        scores: dict[str, PriorityBreakdown] = {}
        for component in conflict_model.connected_components:
            scores.update(
                score_component_priorities(
                    component,
                    conflict_model,
                    self._config,
                    now_ms=now_ms,
                    waiting_ages=waiting_ages,
                    active_grants=grant_manager.active_grants,
                )
            )
        return scores

    def _component_hard_assignments(
        self,
        component: Sequence[str],
        conflict_model: ConflictModel,
        grant_hard_assignments: Mapping[str, Action],
    ) -> dict[str, Action]:
        """Combine safety Pause rules with compatible retained Resume grants."""
        hard_assignments = {
            robot_id: grant_hard_assignments[robot_id]
            for robot_id in component
            if robot_id in grant_hard_assignments
        }
        for robot_id in component:
            snapshot = conflict_model.snapshots[robot_id]
            if snapshot.stale or snapshot.at_goal:
                hard_assignments[robot_id] = Action.PAUSE
        return hard_assignments

    def _solve_components(
        self,
        conflict_model: ConflictModel,
        priorities: Mapping[str, PriorityBreakdown],
        grant_hard_assignments: Mapping[str, Action],
    ) -> tuple[
        dict[str, Action],
        dict[str, DecisionSource],
        tuple[Mapping[str, Any], ...],
    ]:
        """Select deterministic actions for isolated robots and graph components."""
        decisions: dict[str, Action] = {}
        sources: dict[str, DecisionSource] = {}
        component_diagnostics: list[Mapping[str, Any]] = []

        for robot_id in conflict_model.isolated_robots:
            snapshot = conflict_model.snapshots[robot_id]
            decisions[robot_id] = (
                Action.PAUSE if snapshot.stale or snapshot.at_goal else Action.RESUME
            )
            sources[robot_id] = DecisionSource.POLICY

        for component in conflict_model.connected_components:
            if len(component) == 1 and component[0] in conflict_model.isolated_robots:
                continue
            hard_assignments = self._component_hard_assignments(
                component,
                conflict_model,
                grant_hard_assignments,
            )
            exact_result: OptimisationResult | None = None
            fallback_reason: str | None = None
            if len(component) <= self._config.maximum_exact_component_size:
                exact_result = self._optimiser.optimise(
                    component,
                    conflict_model,
                    priorities,
                    hard_assignments=hard_assignments,
                )

            if exact_result is not None and exact_result.feasible:
                decisions.update(exact_result.decisions)
                sources.update(
                    {robot_id: DecisionSource.CP_SAT for robot_id in exact_result.decisions}
                )
                component_diagnostics.append(
                    {
                        "component": tuple(component),
                        "method": DecisionSource.CP_SAT.value,
                        "solver_status": exact_result.solver_status.value,
                        "objective_value": exact_result.objective_value,
                        "best_bound": exact_result.best_bound,
                        "wall_time_seconds": exact_result.wall_time_seconds,
                        "optimality_proven": exact_result.optimality_proven,
                    }
                )
                continue

            if exact_result is not None:
                fallback_reason = exact_result.fallback_reason
            elif len(component) > self._config.maximum_exact_component_size:
                fallback_reason = "component_too_large_for_cp_sat"

            heuristic_result = self._heuristic.solve(
                component,
                conflict_model,
                priorities,
                hard_assignments=hard_assignments,
            )
            if heuristic_result.feasible:
                decisions.update(heuristic_result.decisions)
                sources.update(
                    {
                        robot_id: heuristic_result.decision_source
                        for robot_id in heuristic_result.decisions
                    }
                )
            else:
                decisions.update({robot_id: Action.PAUSE for robot_id in component})
                sources.update({robot_id: DecisionSource.FAIL_SAFE for robot_id in component})
            component_diagnostics.append(
                {
                    "component": tuple(component),
                    "method": heuristic_result.decision_source.value,
                    "solver_status": (
                        exact_result.solver_status.value if exact_result is not None else None
                    ),
                    "wall_time_seconds": (
                        exact_result.wall_time_seconds if exact_result is not None else None
                    ),
                    "fallback_reason": fallback_reason,
                    "heuristic_reason": heuristic_result.fallback_reason,
                    "feasible": heuristic_result.feasible,
                    "progress_made": heuristic_result.progress_made,
                }
            )

        return decisions, sources, tuple(component_diagnostics)

    def _affected_robots(
        self,
        violations: Sequence[GlobalSafetyViolation],
        conflict_model: ConflictModel,
    ) -> tuple[str, ...]:
        """Expand violating pairs to their full nominal graph components."""
        component_by_robot = _component_for_robot(conflict_model)
        affected: set[str] = set()
        for violation in violations:
            affected.update(component_by_robot[violation.robot_i])
            affected.update(component_by_robot[violation.robot_j])
        return tuple(sorted(affected))

    def _constraint_trace(
        self,
        conflict_model: ConflictModel,
    ) -> tuple[Mapping[str, Any], ...]:
        """Serialise pairwise unsafe assignments in stable order."""
        return tuple(
            {
                "robot_pair": pair.robot_pair,
                "forbidden_assignments": tuple(
                    (action_i.value, action_j.value)
                    for action_i, action_j in pair.forbidden_assignments()
                ),
                "forbidden_bounds": tuple(
                    {
                        "actions": (action_i.value, action_j.value),
                        "robot_i_bounds": pair.bounds_for(action_i, action_j)[0],
                        "robot_j_bounds": pair.bounds_for(action_i, action_j)[1],
                    }
                    for action_i, action_j in pair.forbidden_assignments()
                ),
            }
            for pair in conflict_model.pairwise_constraints
            if pair.forbidden_assignments()
        )

    def _detailed_geometry_trace(
        self,
        conflict_model: ConflictModel,
    ) -> tuple[Mapping[str, Any], ...]:
        """Return opt-in WKT action envelopes for explicit geometry debugging."""
        if not self._config.trace_detailed_geometry:
            return ()
        return tuple(
            {
                "robot_id": robot_id,
                "pause_wkt": action_envelope(
                    conflict_model.snapshots[robot_id],
                    Action.PAUSE,
                    self._config,
                ).wkt,
                "resume_wkt": action_envelope(
                    conflict_model.snapshots[robot_id],
                    Action.RESUME,
                    self._config,
                ).wkt,
            }
            for robot_id in sorted(conflict_model.snapshots)
        )

    def _reason_data(
        self,
        robot_id: str,
        snapshot: RobotSnapshot,
        source: DecisionSource,
        conflict_model: ConflictModel,
        priorities: Mapping[str, PriorityBreakdown],
        grant_result: GrantDecisionResult | None,
        grant_hard_assignments: Mapping[str, Action],
        globally_overridden: frozenset[str],
    ) -> tuple[tuple[str, ...], Mapping[str, Any]]:
        """Build concise reason codes and deterministic human context."""
        codes: list[str] = []
        context: list[str] = []
        if snapshot.stale:
            codes.append("STALE_STATIC_OBSTACLE")
            context.append("The state is stale, so the robot is forced to Pause as an obstacle.")
        elif snapshot.at_goal:
            codes.append("AT_GOAL_PAUSE")
            context.append("The remaining path is empty, so no movement is required.")
        elif robot_id in conflict_model.isolated_robots:
            codes.append("ISOLATED_ACTIVE_RESUME")
            context.append("No unsafe pairwise action combination involves this robot.")

        source_codes = {
            DecisionSource.CP_SAT: "CP_SAT_COMPONENT_DECISION",
            DecisionSource.HEURISTIC: "HEURISTIC_COMPONENT_DECISION",
            DecisionSource.REPAIR: "NO_ALL_PAUSED_REPAIR",
            DecisionSource.FAIL_SAFE: "FAIL_SAFE_PAUSE",
            DecisionSource.POLICY: "DIRECT_POLICY_DECISION",
        }
        codes.append(source_codes[source])
        source_context = {
            DecisionSource.CP_SAT: "The bounded exact optimiser selected this component action.",
            DecisionSource.HEURISTIC: (
                "The deterministic constraint-propagating fallback selected this action."
            ),
            DecisionSource.REPAIR: (
                "The no-all-paused safeguard selected a safe progress assignment."
            ),
            DecisionSource.FAIL_SAFE: (
                "No validated progress assignment was retained for this component."
            ),
            DecisionSource.POLICY: "A direct safety policy selected this action.",
        }
        context.append(source_context[source])

        if grant_result is not None and robot_id in grant_result.acquired_grants:
            codes.append("RIGHT_OF_WAY_GRANT_ACQUIRED")
            context.append("This progressing conflict participant received a grant.")
        if grant_hard_assignments.get(robot_id) is Action.RESUME:
            codes.append("ACTIVE_GRANT_RESUME")
            context.append("A safety-checked right-of-way grant retained Resume.")
        if robot_id in globally_overridden:
            codes.append("GLOBAL_SAFETY_OVERRIDE")
            context.append("Fleet-wide validation replaced the component with Pause.")

        metadata: dict[str, Any] = {
            "reason_context": tuple(context),
            "component": _component_for_robot(conflict_model)[robot_id],
            "priority": priorities[robot_id].as_log_data(),
        }
        return tuple(dict.fromkeys(codes)), metadata

    def decide(
        self,
        snapshots: Sequence[RobotSnapshot],
        now_ms: int,
        tick_id: str,
    ) -> FleetDecision:
        """Return one fully traced, globally checked action per observed robot."""
        if not isinstance(now_ms, int) or isinstance(now_ms, bool) or now_ms < 0:
            raise ValueError("now_ms must be a non-negative integer")
        if not isinstance(tick_id, str) or not tick_id.strip():
            raise ValueError("tick_id must be a non-empty string")

        ordered = _derive_staleness(
            _normalise_snapshots(snapshots),
            now_ms=now_ms,
            config=self._config,
        )
        conflict_model = self._conflict_model_builder(ordered, self._config)
        current_footprint_violations = validate_global_safety(
            ordered,
            {snapshot.state.device_id: Action.PAUSE for snapshot in ordered},
            self._config,
        )

        # Grant changes remain provisional until the global validator accepts the fleet.
        provisional_grants = self._grant_manager.clone()
        grant_preparation = provisional_grants.prepare_tick(
            conflict_model,
            now_ms=now_ms,
        )
        priorities = self._score_priorities(
            conflict_model,
            provisional_grants,
            now_ms=now_ms,
        )
        proposed, proposed_sources, component_diagnostics = self._solve_components(
            conflict_model,
            priorities,
            grant_preparation.hard_assignments,
        )
        preview = provisional_grants.preview_tick(
            conflict_model,
            proposed,
            priorities,
            proposed_sources=proposed_sources,
        )

        decisions = dict(preview.decisions)
        sources = dict(preview.decision_sources)
        initial_violations = validate_global_safety(ordered, decisions, self._config)
        final_violations = initial_violations
        affected_robots: tuple[str, ...] = ()
        critical_diagnostics: list[Mapping[str, Any]] = []
        if current_footprint_violations:
            critical_diagnostics.append(
                {
                    "severity": "critical",
                    "code": EngineAlarmCode.CURRENT_FOOTPRINT_OVERLAP,
                    "context": (
                        "Current stationary robot footprints intersect; safety is "
                        "already violated before any movement decision."
                    ),
                }
            )

        if initial_violations:
            affected_robots = self._affected_robots(initial_violations, conflict_model)
            for robot_id in affected_robots:
                decisions[robot_id] = Action.PAUSE
                sources[robot_id] = DecisionSource.FAIL_SAFE
            critical_diagnostics.append(
                {
                    "severity": "critical",
                    "code": EngineAlarmCode.GLOBAL_SAFETY_VALIDATION_FAILED,
                    "context": (
                        "Independent fleet-wide validation rejected the chosen action "
                        "envelopes; affected components were forced to Pause."
                    ),
                }
            )
            final_violations = validate_global_safety(ordered, decisions, self._config)

        grant_result: GrantDecisionResult | None = None
        state_committed = not final_violations
        if state_committed:
            if affected_robots:
                provisional_grants.revoke_for_safety(affected_robots)
            committed_preview = GrantDecisionPreview(
                decisions=decisions,
                decision_sources=sources,
                alarms=preview.alarms,
            )
            grant_result = provisional_grants.commit_tick(
                conflict_model,
                committed_preview,
            )
            self._grant_manager = provisional_grants
        else:
            critical_diagnostics.append(
                {
                    "severity": "critical",
                    "code": EngineAlarmCode.FAIL_SAFE_GEOMETRY_REMAINS_UNSAFE,
                    "context": (
                        "Even stationary fail-safe envelopes intersect; no grant or "
                        "waiting-age state was committed for this tick."
                    ),
                }
            )

        globally_overridden = frozenset(affected_robots)
        robot_decisions: list[RobotDecision] = []
        for snapshot in ordered:
            robot_id = snapshot.state.device_id
            reason_codes, diagnostic_metadata = self._reason_data(
                robot_id,
                snapshot,
                sources[robot_id],
                conflict_model,
                priorities,
                grant_result,
                grant_preparation.hard_assignments,
                globally_overridden,
            )
            robot_decisions.append(
                RobotDecision(
                    robot_id=robot_id,
                    action=decisions[robot_id],
                    reason_codes=reason_codes,
                    tick_id=tick_id,
                    decision_source=sources[robot_id],
                    diagnostic_metadata=diagnostic_metadata,
                )
            )

        stale_robot_ids = tuple(snapshot.state.device_id for snapshot in ordered if snapshot.stale)
        alarm_codes = tuple(
            dict.fromkeys(
                (
                    *(alarm.value for alarm in preview.alarms),
                    *(
                        (EngineAlarmCode.PROLONGED_STATE_LOSS_STATIC_OBSTACLE,)
                        if stale_robot_ids
                        else ()
                    ),
                    *(str(diagnostic["code"]) for diagnostic in critical_diagnostics),
                )
            )
        )
        tick_metadata: dict[str, Any] = {
            "now_ms": now_ms,
            "grant_tick_id": provisional_grants.tick_id,
            "observed_states": tuple(
                {
                    "robot_id": snapshot.state.device_id,
                    "state_timestamp_ms": snapshot.state.timestamp,
                    "received_at_ms": snapshot.received_at_ms,
                    "stale": snapshot.stale,
                    "at_goal": snapshot.at_goal,
                    "current_pose": (
                        snapshot.state.x,
                        snapshot.state.y,
                        snapshot.state.theta,
                    ),
                    "next_pose": (
                        snapshot.next_pose.x,
                        snapshot.next_pose.y,
                        snapshot.next_pose.theta,
                    ),
                }
                for snapshot in ordered
            ),
            "constraints": self._constraint_trace(conflict_model),
            "conflict_edges": conflict_model.edges,
            "connected_components": conflict_model.connected_components,
            "isolated_robots": conflict_model.isolated_robots,
            "component_diagnostics": component_diagnostics,
            "final_decision_sources": tuple(
                (robot_id, sources[robot_id].value) for robot_id in sorted(sources)
            ),
            "priority_scores": tuple(
                priorities[robot_id].as_log_data() for robot_id in sorted(priorities)
            ),
            "grant_hard_assignments": tuple(
                (robot_id, action.value)
                for robot_id, action in sorted(grant_preparation.hard_assignments.items())
            ),
            "grant_alarms": tuple(alarm.value for alarm in preview.alarms),
            "alarms": alarm_codes,
            "critical_diagnostics": tuple(critical_diagnostics),
            "prolonged_stale_robot_ids": stale_robot_ids,
            "current_footprint_overlaps": tuple(
                violation.as_log_data() for violation in current_footprint_violations
            ),
            "initial_global_safety_violations": tuple(
                violation.as_log_data() for violation in initial_violations
            ),
            "final_global_safety_violations": tuple(
                violation.as_log_data() for violation in final_violations
            ),
            "global_safety_valid": not final_violations,
            "state_committed": state_committed,
            "acquired_grants": (grant_result.acquired_grants if grant_result is not None else ()),
            "released_grants": (
                tuple(
                    (release.robot_id, release.reason.value)
                    for release in grant_result.released_grants
                )
                if grant_result is not None
                else ()
            ),
            "waiting_ages": (
                tuple(sorted(grant_result.waiting_ages.items()))
                if grant_result is not None
                else tuple(sorted(self._grant_manager.waiting_ages.items()))
            ),
            "detailed_geometry": self._detailed_geometry_trace(conflict_model),
        }
        return FleetDecision(
            decisions=tuple(robot_decisions),
            tick_id=tick_id,
            tick_metadata=tick_metadata,
        )
