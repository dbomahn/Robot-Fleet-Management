"""Deterministic bounded fallback for one binary conflict component.

The heuristic consumes the same two-literal no-good clauses and objective
ranking as the CP-SAT optimiser. It performs one greedy pass with unit-style
propagation and at most one bounded repair pass; it is not a general search
procedure and deliberately does not implement LNS or exponential backtracking.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType

from collision_monitor.config import MonitorConfig
from collision_monitor.conflicts import (
    ConflictModel,
    NoGoodConstraint,
    NoGoodLiteral,
    PairwiseCompatibility,
    action_to_binary,
)
from collision_monitor.models import Action, DecisionSource
from collision_monitor.optimiser import (
    UnsafeOptimisationResultError,
    build_objective_encoding,
    validate_component_decisions,
)
from collision_monitor.priority import PriorityBreakdown


@dataclass(frozen=True, slots=True)
class BinaryClause:
    """A two-literal no-good clause used by propagation."""

    literals: tuple[NoGoodLiteral, NoGoodLiteral]
    source: NoGoodConstraint

    @classmethod
    def from_no_good(cls, no_good: NoGoodConstraint) -> BinaryClause:
        """Convert a pairwise no-good while rejecting unsupported arity."""
        if len(no_good.literals) != 2:
            raise ValueError("heuristic requires exactly two literals per no-good")
        return cls(
            literals=(no_good.literals[0], no_good.literals[1]),
            source=no_good,
        )


@dataclass(frozen=True, slots=True)
class HeuristicResult:
    """A validated fallback assignment or an explicit infeasibility outcome."""

    feasible: bool
    decisions: Mapping[str, Action]
    decision_source: DecisionSource
    progress_made: bool
    fallback_reason: str | None

    def __post_init__(self) -> None:
        object.__setattr__(self, "decisions", MappingProxyType(dict(self.decisions)))
        if self.feasible and not self.decisions:
            raise ValueError("a feasible heuristic result must contain decisions")
        if not self.feasible and self.decisions:
            raise ValueError("an infeasible heuristic result cannot contain decisions")
        if self.progress_made and not self.feasible:
            raise ValueError("an infeasible result cannot report progress")
        if self.progress_made and self.decision_source is DecisionSource.FAIL_SAFE:
            raise ValueError("fail-safe decisions cannot report progress")
        if not self.progress_made and self.decision_source is not DecisionSource.FAIL_SAFE:
            raise ValueError("a no-progress result must identify the fail-safe source")
        if self.progress_made and self.fallback_reason is not None:
            raise ValueError("a progressing result cannot contain a fallback reason")
        if not self.progress_made and self.fallback_reason is None:
            raise ValueError("a no-progress result must contain a fallback reason")


def _component_constraints(
    component_ids: frozenset[str],
    conflict_model: ConflictModel,
) -> tuple[PairwiseCompatibility, ...]:
    """Select pairwise constraints wholly contained in this component."""
    return tuple(
        pair
        for pair in conflict_model.pairwise_constraints
        if pair.robot_i in component_ids and pair.robot_j in component_ids
    )


def _build_clauses(
    pairwise_constraints: Sequence[PairwiseCompatibility],
) -> tuple[BinaryClause, ...]:
    """Build stable two-literal clauses directly from pairwise constraints."""
    return tuple(
        BinaryClause.from_no_good(no_good)
        for pair in pairwise_constraints
        for no_good in pair.no_good_constraints()
    )


def _assign_binary(assignment: dict[str, int], robot_id: str, value: int) -> bool:
    """Assign a binary value, returning false on a conflicting existing value."""
    existing = assignment.get(robot_id)
    if existing is not None:
        return existing == value
    assignment[robot_id] = value
    return True


def _propagate(assignment: dict[str, int], clauses: Sequence[BinaryClause]) -> bool:
    """Apply unit-style propagation until fixed point or contradiction.

    A literal means ``x != forbidden_value``. If every assigned literal in a
    clause is false and only one variable remains, that variable is forced to
    the opposite of its forbidden value. At most one new variable is assigned
    per propagation event, so termination is bounded by the component size.
    """
    changed = True
    while changed:
        changed = False
        for clause in clauses:
            satisfied = False
            unassigned: list[NoGoodLiteral] = []
            for literal in clause.literals:
                value = assignment.get(literal.robot_id)
                if value is None:
                    unassigned.append(literal)
                elif value != literal.forbidden_value:
                    satisfied = True
                    break

            if satisfied:
                continue
            if not unassigned:
                return False
            if len(unassigned) == 1:
                literal = unassigned[0]
                forced_value = 1 - literal.forbidden_value
                if not _assign_binary(assignment, literal.robot_id, forced_value):
                    return False
                changed = True
    return True


def _try_assignment(
    assignment: Mapping[str, int],
    robot_id: str,
    value: int,
    clauses: Sequence[BinaryClause],
) -> dict[str, int] | None:
    """Try one branch in a copied assignment and propagate its consequences."""
    trial = dict(assignment)
    if not _assign_binary(trial, robot_id, value):
        return None
    if not _propagate(trial, clauses):
        return None
    return trial


def _actions_from_binary(
    component: Sequence[str],
    assignment: Mapping[str, int],
) -> dict[str, Action]:
    """Convert a complete stable binary assignment to actions."""
    return {
        robot_id: Action.RESUME if assignment[robot_id] == 1 else Action.PAUSE
        for robot_id in sorted(component)
    }


def validate_heuristic_decisions(
    component: Sequence[str],
    decisions: Mapping[str, Action],
    pairwise_constraints: Sequence[PairwiseCompatibility],
    hard_assignments: Mapping[str, Action] | None = None,
) -> None:
    """Run the independent pairwise validator used after every complete result."""
    validate_component_decisions(
        component,
        decisions,
        pairwise_constraints,
        hard_assignments,
    )


class DeterministicComponentHeuristic:
    """Greedy propagation with a single bounded escape-repair pass."""

    def __init__(self, config: MonitorConfig) -> None:
        self._config = config

    def _greedy_pass(
        self,
        ranked_robots: Sequence[str],
        seed_assignment: Mapping[str, int],
        clauses: Sequence[BinaryClause],
    ) -> dict[str, int] | None:
        """Try Resume then Pause once for every highest-ranked unassigned robot."""
        assignment = dict(seed_assignment)
        if not _propagate(assignment, clauses):
            return None

        for robot_id in ranked_robots:
            if robot_id in assignment:
                continue

            resume_trial = _try_assignment(assignment, robot_id, 1, clauses)
            if resume_trial is not None:
                assignment = resume_trial
                continue

            pause_trial = _try_assignment(assignment, robot_id, 0, clauses)
            if pause_trial is None:
                return None
            assignment = pause_trial

        return assignment

    def _repair_pass(
        self,
        component: Sequence[str],
        ranked_robots: Sequence[str],
        seed_assignment: Mapping[str, int],
        clauses: Sequence[BinaryClause],
        priorities: Mapping[str, PriorityBreakdown],
        pairwise_constraints: Sequence[PairwiseCompatibility],
        hard_assignments: Mapping[str, Action],
    ) -> dict[str, Action] | None:
        """Try one escape robot and any group forced by unit propagation."""
        base = dict(seed_assignment)
        if not _propagate(base, clauses):
            return None

        rank_index = {robot_id: index for index, robot_id in enumerate(ranked_robots)}
        candidates = sorted(
            component,
            key=lambda robot_id: (
                -priorities[robot_id].clearance_conflict_count,
                rank_index[robot_id],
                robot_id,
            ),
        )[: self._config.heuristic_maximum_repair_candidates]

        for escape_robot in candidates:
            if base.get(escape_robot) == 0:
                continue
            trial = _try_assignment(base, escape_robot, 1, clauses)
            if trial is None:
                continue

            valid = True
            for robot_id in sorted(component):
                if robot_id in trial:
                    continue
                pause_trial = _try_assignment(trial, robot_id, 0, clauses)
                if pause_trial is None:
                    valid = False
                    break
                trial = pause_trial
            if not valid or len(trial) != len(component):
                continue

            decisions = _actions_from_binary(component, trial)
            if not any(action is Action.RESUME for action in decisions.values()):
                continue
            validate_heuristic_decisions(
                component,
                decisions,
                pairwise_constraints,
                hard_assignments,
            )
            return decisions
        return None

    def solve(
        self,
        component: Sequence[str],
        conflict_model: ConflictModel,
        priorities: Mapping[str, PriorityBreakdown],
        *,
        hard_assignments: Mapping[str, Action] | None = None,
    ) -> HeuristicResult:
        """Produce a bounded safe assignment or an explicit fail-safe result."""
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
        unknown_hard = set(hard_actions).difference(component_ids)
        if unknown_hard:
            raise ValueError(
                f"hard assignments contain robots outside the component: {sorted(unknown_hard)!r}"
            )
        if any(not isinstance(action, Action) for action in hard_actions.values()):
            raise ValueError("hard assignments must contain Action values")

        pairwise_constraints = _component_constraints(component_ids, conflict_model)
        clauses = _build_clauses(pairwise_constraints)
        objective = build_objective_encoding(ordered, priorities)
        ranked_robots = tuple(
            sorted(
                ordered,
                key=lambda robot_id: (-objective.coefficients[robot_id], robot_id),
            )
        )
        seed_assignment = {
            robot_id: action_to_binary(action) for robot_id, action in hard_actions.items()
        }

        greedy_assignment = self._greedy_pass(
            ranked_robots,
            seed_assignment,
            clauses,
        )
        if greedy_assignment is not None and len(greedy_assignment) == len(ordered):
            greedy_decisions = _actions_from_binary(ordered, greedy_assignment)
            validate_heuristic_decisions(
                ordered,
                greedy_decisions,
                pairwise_constraints,
                hard_actions,
            )
            if any(action is Action.RESUME for action in greedy_decisions.values()):
                return HeuristicResult(
                    feasible=True,
                    decisions=greedy_decisions,
                    decision_source=DecisionSource.HEURISTIC,
                    progress_made=True,
                    fallback_reason=None,
                )

        repaired = self._repair_pass(
            ordered,
            ranked_robots,
            seed_assignment,
            clauses,
            priorities,
            pairwise_constraints,
            hard_actions,
        )
        if repaired is not None:
            return HeuristicResult(
                feasible=True,
                decisions=repaired,
                decision_source=DecisionSource.REPAIR,
                progress_made=True,
                fallback_reason=None,
            )

        all_paused = {robot_id: Action.PAUSE for robot_id in ordered}
        if all(action is Action.PAUSE for action in hard_actions.values()):
            try:
                validate_heuristic_decisions(
                    ordered,
                    all_paused,
                    pairwise_constraints,
                    hard_actions,
                )
            except UnsafeOptimisationResultError:
                pass
            else:
                return HeuristicResult(
                    feasible=True,
                    decisions=all_paused,
                    decision_source=DecisionSource.FAIL_SAFE,
                    progress_made=False,
                    fallback_reason="no_feasible_progress",
                )

        return HeuristicResult(
            feasible=False,
            decisions={},
            decision_source=DecisionSource.FAIL_SAFE,
            progress_made=False,
            fallback_reason="no_feasible_assignment_found",
        )
