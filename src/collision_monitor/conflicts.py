"""Pairwise action compatibility, no-good constraints and conflict graphs.

This module evaluates geometry only. It deliberately does not choose robot
actions; optimisation and deterministic fallback policies consume the model
produced here.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Collection, Iterable, Mapping, Sequence
from dataclasses import dataclass
from itertools import combinations
from types import MappingProxyType
from typing import Any, Protocol, TypeAlias, TypeVar

from shapely.geometry.base import BaseGeometry

from collision_monitor.config import MonitorConfig
from collision_monitor.geometry import action_envelope, geometries_conflict
from collision_monitor.models import Action, RobotSnapshot

ActionAssignment: TypeAlias = tuple[Action, Action]
RobotPair: TypeAlias = tuple[str, str]
Bounds: TypeAlias = tuple[float, float, float, float]
PairBounds: TypeAlias = tuple[Bounds, Bounds]

ACTION_ORDER: tuple[Action, Action] = (Action.PAUSE, Action.RESUME)
ACTION_ASSIGNMENTS: tuple[ActionAssignment, ...] = tuple(
    (action_i, action_j) for action_i in ACTION_ORDER for action_j in ACTION_ORDER
)

_Key = TypeVar("_Key")
_Value = TypeVar("_Value")


def _immutable_mapping(values: Mapping[_Key, _Value]) -> Mapping[_Key, _Value]:
    """Copy a mapping into a read-only standard-library proxy."""
    return MappingProxyType(dict(values))


def action_to_binary(action: Action) -> int:
    """Encode Pause as zero and Resume as one for Boolean decision variables."""
    if action is Action.PAUSE:
        return 0
    if action is Action.RESUME:
        return 1
    raise ValueError(f"unsupported action {action!r}")


@dataclass(frozen=True, slots=True)
class NoGoodLiteral:
    """A literal requiring one binary variable to differ from a value."""

    robot_id: str
    forbidden_value: int

    def __post_init__(self) -> None:
        if not self.robot_id:
            raise ValueError("no-good robot ID must not be empty")
        if self.forbidden_value not in (0, 1):
            raise ValueError("no-good values must be zero or one")


@dataclass(frozen=True, slots=True)
class NoGoodConstraint:
    """A disjunction of inequality literals with source geometry provenance."""

    literals: tuple[NoGoodLiteral, ...]
    source_pair: RobotPair
    source_assignment: ActionAssignment

    def __post_init__(self) -> None:
        if not self.literals:
            raise ValueError("a no-good constraint must contain at least one literal")
        if self.source_pair[0] >= self.source_pair[1]:
            raise ValueError("a no-good source pair must use deterministic robot ID order")

    def is_violated_by(self, assignment: Mapping[str, int]) -> bool:
        """Return whether every literal is false under a complete assignment."""
        return all(
            assignment[literal.robot_id] == literal.forbidden_value for literal in self.literals
        )

    def expression(self) -> str:
        """Return the no-good clause in a concise diagnostic form."""
        return " OR ".join(
            f"x[{literal.robot_id}] != {literal.forbidden_value}" for literal in self.literals
        )

    def as_log_data(self) -> Mapping[str, Any]:
        """Serialise the clause and its original action pair for trace logging."""
        return {
            "clause": self.expression(),
            "source_pair": self.source_pair,
            "source_actions": tuple(action.value for action in self.source_assignment),
        }


@dataclass(frozen=True, slots=True)
class PairwiseCompatibility:
    """Safety results and diagnostics for every action pair of two robots."""

    robot_i: str
    robot_j: str
    compatibility: Mapping[ActionAssignment, bool]
    envelope_bounds: Mapping[ActionAssignment, PairBounds]

    def __post_init__(self) -> None:
        if self.robot_i >= self.robot_j:
            raise ValueError("pairwise robot IDs must be distinct and deterministically ordered")

        expected = frozenset(ACTION_ASSIGNMENTS)
        if frozenset(self.compatibility) != expected:
            raise ValueError("compatibility must contain all four action assignments")
        if frozenset(self.envelope_bounds) != expected:
            raise ValueError("envelope bounds must contain all four action assignments")

        object.__setattr__(self, "compatibility", _immutable_mapping(self.compatibility))
        object.__setattr__(self, "envelope_bounds", _immutable_mapping(self.envelope_bounds))

    @property
    def robot_pair(self) -> RobotPair:
        """Return the deterministically ordered pair of robot IDs."""
        return (self.robot_i, self.robot_j)

    def is_safe(self, action_i: Action, action_j: Action) -> bool:
        """Return the recorded safety result for one action assignment."""
        return self.compatibility[(action_i, action_j)]

    def bounds_for(self, action_i: Action, action_j: Action) -> PairBounds:
        """Return the two envelopes' bounds for one action assignment."""
        return self.envelope_bounds[(action_i, action_j)]

    def forbidden_assignments(self) -> tuple[ActionAssignment, ...]:
        """Return unsafe action pairs in stable Pause-before-Resume order."""
        return tuple(
            assignment for assignment in ACTION_ASSIGNMENTS if not self.compatibility[assignment]
        )

    def no_good_constraints(self) -> tuple[NoGoodConstraint, ...]:
        """Translate every unsafe action pair into a binary no-good clause."""
        return tuple(
            NoGoodConstraint(
                literals=(
                    NoGoodLiteral(self.robot_i, action_to_binary(action_i)),
                    NoGoodLiteral(self.robot_j, action_to_binary(action_j)),
                ),
                source_pair=self.robot_pair,
                source_assignment=(action_i, action_j),
            )
            for action_i, action_j in self.forbidden_assignments()
        )


class CandidatePairGenerator(Protocol):
    """Replaceable interface for selecting robot pairs requiring evaluation."""

    def generate(
        self,
        snapshots: Sequence[RobotSnapshot],
    ) -> Iterable[tuple[RobotSnapshot, RobotSnapshot]]:
        """Yield candidate pairs from snapshots in deterministic ID order."""
        ...


@dataclass(frozen=True, slots=True)
class AllPairsCandidateGenerator:
    """Generate every unordered pair; a spatial index can replace this later."""

    def generate(
        self,
        snapshots: Sequence[RobotSnapshot],
    ) -> Iterable[tuple[RobotSnapshot, RobotSnapshot]]:
        """Yield all unordered pairs sorted by robot ID."""
        ordered = sorted(snapshots, key=lambda snapshot: snapshot.state.device_id)
        return combinations(ordered, 2)


@dataclass(frozen=True, slots=True)
class ConflictModel:
    """Immutable pairwise constraints and graph decomposition for one tick."""

    snapshots: Mapping[str, RobotSnapshot]
    pairwise_constraints: tuple[PairwiseCompatibility, ...]
    no_good_constraints: tuple[NoGoodConstraint, ...]
    edges: tuple[RobotPair, ...]
    adjacency: Mapping[str, frozenset[str]]
    isolated_robots: tuple[str, ...]
    connected_components: tuple[tuple[str, ...], ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "snapshots", _immutable_mapping(self.snapshots))
        object.__setattr__(
            self,
            "adjacency",
            _immutable_mapping(
                {robot_id: frozenset(neighbours) for robot_id, neighbours in self.adjacency.items()}
            ),
        )

    def compatibility_for(self, robot_a: str, robot_b: str) -> PairwiseCompatibility:
        """Return compatibility for a pair regardless of caller argument order."""
        pair = tuple(sorted((robot_a, robot_b)))
        if len(pair) != 2 or pair[0] == pair[1]:
            raise KeyError(f"invalid robot pair {pair!r}")
        for constraint in self.pairwise_constraints:
            if constraint.robot_pair == pair:
                return constraint
        raise KeyError(f"robot pair {pair!r} was not evaluated")


def _geometry_bounds(geometry: BaseGeometry) -> Bounds:
    """Convert Shapely bounds into a fixed, plain-Python diagnostic tuple."""
    min_x, min_y, max_x, max_y = geometry.bounds
    return (float(min_x), float(min_y), float(max_x), float(max_y))


def _build_envelope_cache(
    snapshots: Sequence[RobotSnapshot],
    config: MonitorConfig,
) -> dict[str, dict[Action, BaseGeometry]]:
    """Construct each robot/action envelope exactly once for a model build."""
    return {
        snapshot.state.device_id: {
            action: action_envelope(snapshot, action, config) for action in ACTION_ORDER
        }
        for snapshot in snapshots
    }


def _evaluate_cached_pair(
    snapshot_i: RobotSnapshot,
    snapshot_j: RobotSnapshot,
    envelopes: Mapping[str, Mapping[Action, BaseGeometry]],
) -> PairwiseCompatibility:
    """Evaluate four action assignments using pre-built envelopes."""
    if snapshot_i.state.device_id > snapshot_j.state.device_id:
        snapshot_i, snapshot_j = snapshot_j, snapshot_i

    robot_i = snapshot_i.state.device_id
    robot_j = snapshot_j.state.device_id
    if robot_i == robot_j:
        raise ValueError(f"duplicate robot ID {robot_i!r} in candidate pair")

    compatibility: dict[ActionAssignment, bool] = {}
    bounds: dict[ActionAssignment, PairBounds] = {}
    for action_i, action_j in ACTION_ASSIGNMENTS:
        envelope_i = envelopes[robot_i][action_i]
        envelope_j = envelopes[robot_j][action_j]
        assignment = (action_i, action_j)
        compatibility[assignment] = not geometries_conflict(envelope_i, envelope_j)
        bounds[assignment] = (
            _geometry_bounds(envelope_i),
            _geometry_bounds(envelope_j),
        )

    return PairwiseCompatibility(
        robot_i=robot_i,
        robot_j=robot_j,
        compatibility=compatibility,
        envelope_bounds=bounds,
    )


def evaluate_pairwise_compatibility(
    snapshot_a: RobotSnapshot,
    snapshot_b: RobotSnapshot,
    config: MonitorConfig,
) -> PairwiseCompatibility:
    """Evaluate all four action combinations for one unordered robot pair."""
    if snapshot_a.state.device_id == snapshot_b.state.device_id:
        raise ValueError(f"duplicate robot ID {snapshot_a.state.device_id!r} in pair")
    snapshots = tuple(
        sorted((snapshot_a, snapshot_b), key=lambda snapshot: snapshot.state.device_id)
    )
    envelopes = _build_envelope_cache(snapshots, config)
    return _evaluate_cached_pair(snapshots[0], snapshots[1], envelopes)


def build_adjacency(
    robot_ids: Collection[str],
    edges: Iterable[RobotPair],
) -> Mapping[str, frozenset[str]]:
    """Build an immutable undirected adjacency mapping from standard collections."""
    mutable: dict[str, set[str]] = {robot_id: set() for robot_id in sorted(robot_ids)}
    for robot_i, robot_j in sorted(edges):
        if robot_i not in mutable or robot_j not in mutable:
            raise ValueError(f"edge {(robot_i, robot_j)!r} refers to an unknown robot")
        if robot_i >= robot_j:
            raise ValueError(f"edge {(robot_i, robot_j)!r} is not deterministically ordered")
        mutable[robot_i].add(robot_j)
        mutable[robot_j].add(robot_i)
    return _immutable_mapping(
        {robot_id: frozenset(neighbours) for robot_id, neighbours in mutable.items()}
    )


def decompose_connected_components(
    adjacency: Mapping[str, Collection[str]],
) -> tuple[tuple[str, ...], ...]:
    """Return stable connected components using deterministic breadth-first search."""
    visited: set[str] = set()
    components: list[tuple[str, ...]] = []

    for root in sorted(adjacency):
        if root in visited:
            continue
        queue = deque([root])
        visited.add(root)
        component: list[str] = []

        while queue:
            robot_id = queue.popleft()
            component.append(robot_id)
            for neighbour in sorted(adjacency[robot_id]):
                if neighbour not in adjacency:
                    raise ValueError(f"adjacency refers to unknown robot {neighbour!r}")
                if neighbour not in visited:
                    visited.add(neighbour)
                    queue.append(neighbour)

        components.append(tuple(sorted(component)))

    return tuple(components)


def _normalise_snapshots(
    snapshots: Sequence[RobotSnapshot],
) -> tuple[RobotSnapshot, ...]:
    """Sort snapshots and reject duplicate robot IDs before geometry evaluation."""
    ordered = tuple(sorted(snapshots, key=lambda snapshot: snapshot.state.device_id))
    for previous, current in zip(ordered, ordered[1:], strict=False):
        if previous.state.device_id == current.state.device_id:
            raise ValueError(f"duplicate robot ID {current.state.device_id!r} in snapshots")
    return ordered


def build_conflict_model(
    snapshots: Sequence[RobotSnapshot],
    config: MonitorConfig,
    *,
    pair_generator: CandidatePairGenerator | None = None,
) -> ConflictModel:
    """Evaluate candidate pairs and build deterministic constraints and components."""
    ordered = _normalise_snapshots(snapshots)
    snapshots_by_id = {snapshot.state.device_id: snapshot for snapshot in ordered}
    envelopes = _build_envelope_cache(ordered, config)
    generator = pair_generator or AllPairsCandidateGenerator()

    constraints_by_pair: dict[RobotPair, PairwiseCompatibility] = {}
    for candidate_a, candidate_b in generator.generate(ordered):
        candidate_ids = (candidate_a.state.device_id, candidate_b.state.device_id)
        if candidate_ids[0] == candidate_ids[1]:
            raise ValueError(f"candidate pair repeats robot ID {candidate_ids[0]!r}")
        if any(robot_id not in snapshots_by_id for robot_id in candidate_ids):
            raise ValueError(f"candidate pair {candidate_ids!r} contains an unknown robot")

        robot_i, robot_j = sorted(candidate_ids)
        pair = (robot_i, robot_j)
        if pair in constraints_by_pair:
            continue
        constraints_by_pair[pair] = _evaluate_cached_pair(
            snapshots_by_id[robot_i],
            snapshots_by_id[robot_j],
            envelopes,
        )

    pairwise_constraints = tuple(constraints_by_pair[pair] for pair in sorted(constraints_by_pair))
    edges = tuple(
        constraint.robot_pair
        for constraint in pairwise_constraints
        if constraint.forbidden_assignments()
    )
    adjacency = build_adjacency(snapshots_by_id.keys(), edges)
    isolated = tuple(robot_id for robot_id in sorted(adjacency) if not adjacency[robot_id])
    components = decompose_connected_components(adjacency)
    no_goods = tuple(
        no_good
        for constraint in pairwise_constraints
        for no_good in constraint.no_good_constraints()
    )

    return ConflictModel(
        snapshots=snapshots_by_id,
        pairwise_constraints=pairwise_constraints,
        no_good_constraints=no_goods,
        edges=edges,
        adjacency=adjacency,
        isolated_robots=isolated,
        connected_components=components,
    )
