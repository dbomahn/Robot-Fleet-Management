"""Strictly time-bounded CP-SAT optimisation for one conflict component."""

from __future__ import annotations

import importlib
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from types import MappingProxyType
from typing import Any

from collision_monitor.config import MonitorConfig
from collision_monitor.conflicts import (
    ConflictModel,
    NoGoodConstraint,
    PairwiseCompatibility,
    action_to_binary,
)
from collision_monitor.models import Action, DecisionSource
from collision_monitor.priority import PriorityBreakdown

_USE_DEFAULT_BACKEND = object()
_CP_SAT_INTEGER_LIMIT = (1 << 63) - 1


class OptimiserInternalError(RuntimeError):
    """Raised when the optimiser cannot safely trust its model or solution."""


class CpSatModelInvalidError(OptimiserInternalError):
    """Raised when OR-Tools reports that the generated model is invalid."""


class UnsafeOptimisationResultError(OptimiserInternalError):
    """Raised when independent validation rejects solver decisions."""


class SolverStatus(StrEnum):
    """Stable solver status names exposed to the decision engine."""

    OPTIMAL = "OPTIMAL"
    FEASIBLE = "FEASIBLE"
    UNKNOWN = "UNKNOWN"
    INFEASIBLE = "INFEASIBLE"
    MODEL_INVALID = "MODEL_INVALID"
    UNAVAILABLE = "UNAVAILABLE"
    COMPONENT_TOO_LARGE = "COMPONENT_TOO_LARGE"


@dataclass(frozen=True, slots=True)
class OptimisationResult:
    """A usable component assignment or an explicit request for fallback."""

    feasible: bool
    decisions: Mapping[str, Action]
    decision_source: DecisionSource | None
    solver_status: SolverStatus
    objective_value: int | None
    best_bound: int | None
    wall_time_seconds: float
    optimality_proven: bool
    fallback_reason: str | None

    def __post_init__(self) -> None:
        object.__setattr__(self, "decisions", MappingProxyType(dict(self.decisions)))
        if self.wall_time_seconds < 0 or not math.isfinite(self.wall_time_seconds):
            raise ValueError("wall_time_seconds must be finite and non-negative")
        if self.feasible and not self.decisions:
            raise ValueError("a feasible optimisation result must contain decisions")
        if self.feasible and self.decision_source is not DecisionSource.CP_SAT:
            raise ValueError("a feasible CP-SAT result must identify CP-SAT as its source")
        if self.feasible and self.fallback_reason is not None:
            raise ValueError("a feasible optimisation result cannot request fallback")
        if not self.feasible and self.decisions:
            raise ValueError("a fallback optimisation result cannot contain decisions")
        if not self.feasible and self.decision_source is not None:
            raise ValueError("a result without decisions cannot identify a decision source")
        if self.optimality_proven != (self.solver_status is SolverStatus.OPTIMAL):
            raise ValueError("optimality_proven is inconsistent with solver_status")

    @property
    def wall_time(self) -> float:
        """Return solver wall time in seconds."""
        return self.wall_time_seconds

    @property
    def reason_for_fallback(self) -> str | None:
        """Return the stable fallback reason code, if any."""
        return self.fallback_reason


@dataclass(frozen=True, slots=True)
class ObjectiveEncoding:
    """Integer coefficients with robot-ID rank as the least-significant term."""

    coefficients: Mapping[str, int]
    utility_scale: int
    rank_bonuses: Mapping[str, int]

    def __post_init__(self) -> None:
        object.__setattr__(self, "coefficients", MappingProxyType(dict(self.coefficients)))
        object.__setattr__(self, "rank_bonuses", MappingProxyType(dict(self.rank_bonuses)))
        if self.utility_scale < 1:
            raise ValueError("utility_scale must be positive")


def build_objective_encoding(
    component: Sequence[str],
    priorities: Mapping[str, PriorityBreakdown],
) -> ObjectiveEncoding:
    """Build deterministic tertiary tie-break coefficients.

    Rank bonuses use unit increments and favour smaller robot IDs. Utilities are
    scaled by one more than the maximum aggregate rank bonus, so even a
    one-point utility difference dominates every possible rank difference.
    Throughput dominance already encoded by ``PriorityBreakdown.final_score``
    is therefore preserved exactly.
    """
    ordered = tuple(sorted(component))
    missing = set(ordered).difference(priorities)
    if missing:
        raise ValueError(f"priorities are missing for robots: {sorted(missing)!r}")

    rank_bonuses = {robot_id: len(ordered) - index - 1 for index, robot_id in enumerate(ordered)}
    maximum_rank_total = sum(rank_bonuses.values())
    utility_scale = maximum_rank_total + 1
    coefficients = {
        robot_id: priorities[robot_id].final_score * utility_scale + rank_bonuses[robot_id]
        for robot_id in ordered
    }
    if any(coefficient > _CP_SAT_INTEGER_LIMIT for coefficient in coefficients.values()):
        raise ValueError("objective coefficient exceeds the CP-SAT integer range")
    return ObjectiveEncoding(
        coefficients=coefficients,
        utility_scale=utility_scale,
        rank_bonuses=rank_bonuses,
    )


def _component_constraints(
    component_ids: frozenset[str],
    conflict_model: ConflictModel,
) -> tuple[PairwiseCompatibility, ...]:
    """Return pairwise constraints whose two robots belong to the component."""
    return tuple(
        pair
        for pair in conflict_model.pairwise_constraints
        if pair.robot_i in component_ids and pair.robot_j in component_ids
    )


def _component_no_goods(
    pairwise_constraints: Sequence[PairwiseCompatibility],
) -> tuple[NoGoodConstraint, ...]:
    """Regenerate no-goods from pairwise compatibility for independent provenance."""
    return tuple(no_good for pair in pairwise_constraints for no_good in pair.no_good_constraints())


def validate_component_decisions(
    component: Sequence[str],
    decisions: Mapping[str, Action],
    pairwise_constraints: Sequence[PairwiseCompatibility],
    hard_assignments: Mapping[str, Action] | None = None,
) -> None:
    """Independently validate a complete assignment against every no-good."""
    ordered = tuple(sorted(component))
    if set(decisions) != set(ordered):
        raise UnsafeOptimisationResultError(
            "solver decisions do not contain exactly the component robots"
        )

    binary_assignment = {robot_id: action_to_binary(decisions[robot_id]) for robot_id in ordered}
    for no_good in _component_no_goods(pairwise_constraints):
        if no_good.is_violated_by(binary_assignment):
            raise UnsafeOptimisationResultError(
                f"solver assignment violates no-good {no_good.expression()}"
            )

    for robot_id, required_action in (hard_assignments or {}).items():
        if decisions.get(robot_id) is not required_action:
            raise UnsafeOptimisationResultError(
                f"solver assignment violates hard action for robot {robot_id!r}"
            )


def _load_default_backend() -> Any | None:
    """Load OR-Tools lazily so absence can request deterministic fallback."""
    try:
        return importlib.import_module("ortools.sat.python.cp_model")
    except ImportError:
        return None


def _integer_solver_metric(value: float, *, name: str) -> int:
    """Convert an integral CP-SAT floating-point diagnostic back to an integer."""
    if not math.isfinite(value):
        raise OptimiserInternalError(f"solver returned non-finite {name}")
    rounded = round(value)
    if not math.isclose(value, rounded, rel_tol=0.0, abs_tol=1e-6):
        raise OptimiserInternalError(f"solver returned non-integral {name}: {value!r}")
    return int(rounded)


class CpSatComponentOptimiser:
    """Build and solve one deterministic, bounded CP-SAT component model."""

    def __init__(
        self,
        config: MonitorConfig,
        *,
        cp_sat_module: Any = _USE_DEFAULT_BACKEND,
        solver_factory: Callable[[], Any] | None = None,
    ) -> None:
        self._config = config
        self._cp_sat = (
            _load_default_backend() if cp_sat_module is _USE_DEFAULT_BACKEND else cp_sat_module
        )
        self._solver_factory = solver_factory

    def _fallback(
        self,
        *,
        status: SolverStatus,
        reason: str,
        wall_time_seconds: float = 0.0,
        best_bound: int | None = None,
    ) -> OptimisationResult:
        """Create a result explicitly delegating the component to the heuristic."""
        return OptimisationResult(
            feasible=False,
            decisions={},
            decision_source=None,
            solver_status=status,
            objective_value=None,
            best_bound=best_bound,
            wall_time_seconds=wall_time_seconds,
            optimality_proven=False,
            fallback_reason=reason,
        )

    def optimise(
        self,
        component: Sequence[str],
        conflict_model: ConflictModel,
        priorities: Mapping[str, PriorityBreakdown],
        *,
        hard_assignments: Mapping[str, Action] | None = None,
    ) -> OptimisationResult:
        """Optimise a component or return a precise heuristic-fallback reason."""
        ordered = tuple(sorted(component))
        if not ordered:
            raise ValueError("component must contain at least one robot")
        if len(set(ordered)) != len(ordered):
            raise ValueError("component contains duplicate robot IDs")

        component_ids = frozenset(ordered)
        unknown = component_ids.difference(conflict_model.snapshots)
        if unknown:
            raise ValueError(f"component contains unknown robots: {sorted(unknown)!r}")
        missing_priorities = component_ids.difference(priorities)
        if missing_priorities:
            raise ValueError(f"priorities are missing for robots: {sorted(missing_priorities)!r}")
        for robot_id in ordered:
            if priorities[robot_id].robot_id != robot_id:
                raise ValueError(f"priority key does not match robot {robot_id!r}")

        hard_actions = dict(hard_assignments or {})
        unknown_hard_assignments = set(hard_actions).difference(component_ids)
        if unknown_hard_assignments:
            raise ValueError(
                "hard assignments contain robots outside the component: "
                f"{sorted(unknown_hard_assignments)!r}"
            )
        if any(not isinstance(action, Action) for action in hard_actions.values()):
            raise ValueError("hard assignments must contain Action values")

        if len(ordered) > self._config.maximum_exact_component_size:
            return self._fallback(
                status=SolverStatus.COMPONENT_TOO_LARGE,
                reason="component_too_large_for_cp_sat",
            )
        if self._cp_sat is None:
            return self._fallback(
                status=SolverStatus.UNAVAILABLE,
                reason="cp_sat_unavailable",
            )

        pairwise_constraints = _component_constraints(component_ids, conflict_model)
        no_goods = _component_no_goods(pairwise_constraints)
        objective = build_objective_encoding(ordered, priorities)

        model = self._cp_sat.CpModel()
        variables = {robot_id: model.new_bool_var(f"resume_{robot_id}") for robot_id in ordered}

        for no_good in no_goods:
            clause = []
            for literal in no_good.literals:
                variable = variables[literal.robot_id]
                # x != 0 is x; x != 1 is not x.
                clause.append(variable if literal.forbidden_value == 0 else variable.Not())
            model.add_bool_or(clause)

        for robot_id, action in sorted(hard_actions.items()):
            model.add(variables[robot_id] == action_to_binary(action))

        model.maximize(
            sum(objective.coefficients[robot_id] * variables[robot_id] for robot_id in ordered)
        )

        solver = (
            self._solver_factory() if self._solver_factory is not None else self._cp_sat.CpSolver()
        )
        solver.parameters.max_time_in_seconds = self._config.cp_sat_time_limit_seconds
        solver.parameters.num_search_workers = 1
        solver.parameters.random_seed = self._config.cp_sat_random_seed
        solver.parameters.log_search_progress = self._config.cp_sat_log_search_progress
        solver.parameters.log_to_stdout = self._config.cp_sat_log_search_progress

        status = solver.solve(model)
        wall_time = float(solver.wall_time)

        if status == self._cp_sat.MODEL_INVALID:
            status_detail = solver.status_name(status)
            raise CpSatModelInvalidError(f"CP-SAT rejected the model: {status_detail}")
        if status == self._cp_sat.UNKNOWN:
            return self._fallback(
                status=SolverStatus.UNKNOWN,
                reason="cp_sat_unknown_without_incumbent",
                wall_time_seconds=wall_time,
            )
        if status == self._cp_sat.INFEASIBLE:
            return self._fallback(
                status=SolverStatus.INFEASIBLE,
                reason="cp_sat_model_infeasible",
                wall_time_seconds=wall_time,
            )
        if status not in (self._cp_sat.OPTIMAL, self._cp_sat.FEASIBLE):
            raise OptimiserInternalError(
                f"CP-SAT returned unsupported status {solver.status_name(status)!r}"
            )

        decisions = {
            robot_id: (Action.RESUME if solver.value(variables[robot_id]) == 1 else Action.PAUSE)
            for robot_id in ordered
        }
        validate_component_decisions(
            ordered,
            decisions,
            pairwise_constraints,
            hard_actions,
        )

        objective_value = _integer_solver_metric(
            float(solver.objective_value),
            name="objective value",
        )
        best_bound = _integer_solver_metric(
            float(solver.best_objective_bound),
            name="best objective bound",
        )
        optimal = status == self._cp_sat.OPTIMAL
        return OptimisationResult(
            feasible=True,
            decisions=decisions,
            decision_source=DecisionSource.CP_SAT,
            solver_status=(SolverStatus.OPTIMAL if optimal else SolverStatus.FEASIBLE),
            objective_value=objective_value,
            best_bound=best_bound,
            wall_time_seconds=wall_time,
            optimality_proven=optimal,
            fallback_reason=None,
        )
