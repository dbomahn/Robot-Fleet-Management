"""Fast fixed-seed properties for cross-module safety invariants."""

from __future__ import annotations

import math
import random
from itertools import combinations, permutations

import pytest
from shapely import affinity

from collision_monitor.config import MonitorConfig
from collision_monitor.conflicts import (
    ACTION_ASSIGNMENTS,
    ConflictModel,
    PairwiseCompatibility,
    action_to_binary,
    build_adjacency,
    decompose_connected_components,
)
from collision_monitor.engine import (
    CollisionDecisionEngine,
    EngineAlarmCode,
    validate_global_safety,
)
from collision_monitor.geometry import (
    RobotDimensions,
    geometries_conflict,
    oriented_footprint,
    pause_envelope,
)
from collision_monitor.grants import GrantManager, GrantReleaseReason
from collision_monitor.heuristic import (
    DeterministicComponentHeuristic,
    validate_heuristic_decisions,
)
from collision_monitor.models import (
    Action,
    DecisionSource,
    Pose,
    RobotSnapshot,
    RobotState,
)
from collision_monitor.optimiser import (
    CpSatComponentOptimiser,
    validate_component_decisions,
)
from collision_monitor.priority import PriorityBreakdown, score_component_priorities

NOW_MS = 1_720_000_000_000
RANDOM_SEED = 20_260_803
DUMMY_BOUNDS = (0.0, 0.0, 1.0, 1.0)
DIMENSIONS = RobotDimensions(width_metres=0.630, length_metres=1.430)


def make_snapshot(
    robot_id: str,
    *,
    x: float,
    y: float,
    next_x: float | None = None,
    next_y: float | None = None,
    theta: float = 0.0,
    next_theta: float | None = None,
    stale: bool = False,
) -> RobotSnapshot:
    """Build one moving or at-goal snapshot with explicit geometry."""
    path = (
        []
        if next_x is None or next_y is None
        else [
            {
                "x": next_x,
                "y": next_y,
                "theta": theta if next_theta is None else next_theta,
            }
        ]
    )
    state = RobotState.model_validate(
        {
            "device_id": robot_id,
            "timestamp": NOW_MS,
            "x": x,
            "y": y,
            "theta": theta,
            "battery_level": 50.0,
            "loaded": False,
            "deadline": NOW_MS + 300_000,
            "path": path,
        }
    )
    return RobotSnapshot.from_state(
        state,
        received_at_ms=NOW_MS,
        stale=stale,
        tolerance=1e-6,
    )


def make_pair(
    robot_i: str,
    robot_j: str,
    forbidden: tuple[tuple[Action, Action], ...],
) -> PairwiseCompatibility:
    """Build a synthetic pair record with complete truth-table data."""
    forbidden_set = frozenset(forbidden)
    return PairwiseCompatibility(
        robot_i=robot_i,
        robot_j=robot_j,
        compatibility={
            assignment: assignment not in forbidden_set for assignment in ACTION_ASSIGNMENTS
        },
        envelope_bounds={
            assignment: (DUMMY_BOUNDS, DUMMY_BOUNDS) for assignment in ACTION_ASSIGNMENTS
        },
    )


def make_model(
    robot_ids: tuple[str, ...],
    pairs: tuple[PairwiseCompatibility, ...],
) -> ConflictModel:
    """Build a complete synthetic conflict model in stable order."""
    snapshots = {
        robot_id: make_snapshot(
            robot_id,
            x=index * 10.0,
            y=0.0,
            next_x=index * 10.0 + 1.0,
            next_y=0.0,
        )
        for index, robot_id in enumerate(sorted(robot_ids))
    }
    edges = tuple(pair.robot_pair for pair in pairs if pair.forbidden_assignments())
    adjacency = build_adjacency(snapshots, edges)
    return ConflictModel(
        snapshots=snapshots,
        pairwise_constraints=tuple(sorted(pairs, key=lambda pair: pair.robot_pair)),
        no_good_constraints=tuple(
            no_good for pair in pairs for no_good in pair.no_good_constraints()
        ),
        edges=tuple(sorted(edges)),
        adjacency=adjacency,
        isolated_robots=tuple(
            robot_id for robot_id in sorted(adjacency) if not adjacency[robot_id]
        ),
        connected_components=decompose_connected_components(adjacency),
    )


def make_priorities(
    robot_ids: tuple[str, ...],
) -> dict[str, PriorityBreakdown]:
    """Give every Resume set a distinct deterministic additive utility."""
    ordered = tuple(sorted(robot_ids))
    return {
        robot_id: PriorityBreakdown(
            robot_id=robot_id,
            base_progress_value=0,
            loaded_bonus=0,
            deadline_urgency=0,
            low_battery_urgency=0,
            waiting_age_bonus=0,
            active_grant_continuation_bonus=0,
            clearance_bonus=0,
            clearance_conflict_count=0,
            remaining_slack_ms=0,
            secondary_score=0,
            throughput_reward=1 << (len(ordered) - index),
            final_score=1 << (len(ordered) - index),
        )
        for index, robot_id in enumerate(ordered)
    }


def generated_satisfiable_model(seed: int) -> ConflictModel:
    """Generate a small pairwise model around one known safe assignment."""
    generator = random.Random(RANDOM_SEED + seed)
    robot_ids = tuple(f"robot-{index}" for index in range(generator.randint(2, 6)))
    witness = {robot_id: generator.choice((Action.PAUSE, Action.RESUME)) for robot_id in robot_ids}
    pairs: list[PairwiseCompatibility] = []
    for robot_i, robot_j in combinations(robot_ids, 2):
        forbidden: tuple[tuple[Action, Action], ...] = ()
        if generator.random() < 0.65:
            candidates = [
                assignment
                for assignment in ACTION_ASSIGNMENTS
                if assignment != (witness[robot_i], witness[robot_j])
            ]
            generator.shuffle(candidates)
            forbidden = tuple(candidates[: generator.randint(1, 2)])
        pairs.append(make_pair(robot_i, robot_j, forbidden))
    return make_model(robot_ids, tuple(pairs))


@pytest.mark.parametrize(
    "theta",
    (
        -2.75,
        -math.pi / 2.0,
        -0.125,
        0.0,
        math.pi / 3.0,
        math.pi,
        5.25,
    ),
)
def test_footprint_rotation_preserves_centre_area_and_half_turn_symmetry(
    theta: float,
) -> None:
    pose = Pose(x=1.25, y=-0.75, theta=theta)
    footprint = oriented_footprint(pose, DIMENSIONS, 0.03)
    reference = oriented_footprint(Pose(pose.x, pose.y, 0.0), DIMENSIONS, 0.03)
    rotated_reference = affinity.rotate(
        reference,
        theta,
        origin=(pose.x, pose.y),
        use_radians=True,
    )
    half_turn = oriented_footprint(
        Pose(pose.x, pose.y, theta + math.pi),
        DIMENSIONS,
        0.03,
    )

    assert footprint.centroid.x == pytest.approx(pose.x, abs=1e-12)
    assert footprint.centroid.y == pytest.approx(pose.y, abs=1e-12)
    assert footprint.area == pytest.approx(reference.area, rel=1e-12)
    assert footprint.symmetric_difference(rotated_reference).area < 1e-12
    assert footprint.symmetric_difference(half_turn).area < 1e-12


@pytest.mark.parametrize("forbidden", ACTION_ASSIGNMENTS)
def test_every_no_good_encoding_has_the_exact_forbidden_truth_table(
    forbidden: tuple[Action, Action],
) -> None:
    pair = make_pair("robot-a", "robot-b", (forbidden,))
    (no_good,) = pair.no_good_constraints()

    for candidate in ACTION_ASSIGNMENTS:
        binary = {
            "robot-a": action_to_binary(candidate[0]),
            "robot-b": action_to_binary(candidate[1]),
        }
        assert no_good.is_violated_by(binary) is (candidate == forbidden)
    assert no_good.source_assignment == forbidden
    assert no_good.source_pair == ("robot-a", "robot-b")


def test_decisions_are_identical_for_seeded_input_permutations() -> None:
    config = MonitorConfig(safety_margin_metres=0.0)
    snapshots = (
        make_snapshot("robot-a", x=-2.0, y=0.0, next_x=2.0, next_y=0.0),
        make_snapshot(
            "robot-b",
            x=0.0,
            y=-2.0,
            next_x=0.0,
            next_y=2.0,
            theta=math.pi / 2.0,
        ),
        make_snapshot("robot-c", x=8.0, y=0.0, next_x=12.0, next_y=0.0),
        make_snapshot(
            "robot-d",
            x=10.0,
            y=-2.0,
            next_x=10.0,
            next_y=2.0,
            theta=math.pi / 2.0,
        ),
    )
    generator = random.Random(RANDOM_SEED)
    orders = list(permutations(snapshots))
    generator.shuffle(orders)

    outcomes = {
        tuple(
            (decision.robot_id, decision.action, decision.decision_source)
            for decision in CollisionDecisionEngine(config)
            .decide(order, NOW_MS, "tick-permutation")
            .decisions
        )
        for order in orders[:16]
    }

    assert len(outcomes) == 1


@pytest.mark.parametrize("seed", range(8))
def test_every_final_selected_envelope_is_pairwise_safe(seed: int) -> None:
    config = MonitorConfig(safety_margin_metres=0.02)
    generator = random.Random(RANDOM_SEED + seed)
    snapshots: list[RobotSnapshot] = []
    current_envelopes = []
    for index in range(4):
        for _attempt in range(200):
            x = generator.uniform(-5.0, 5.0)
            y = generator.uniform(-5.0, 5.0)
            theta = generator.choice((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0))
            candidate = make_snapshot(
                f"robot-{index}",
                x=x,
                y=y,
                next_x=x + generator.uniform(-2.5, 2.5),
                next_y=y + generator.uniform(-2.5, 2.5),
                theta=theta,
                next_theta=theta + generator.choice((-math.pi / 2.0, 0.0, math.pi / 2.0)),
            )
            current = pause_envelope(candidate, config)
            if all(not geometries_conflict(current, existing) for existing in current_envelopes):
                snapshots.append(candidate)
                current_envelopes.append(current)
                break
        else:
            raise AssertionError("fixed-seed generator could not place a safe footprint")

    result = CollisionDecisionEngine(config).decide(
        tuple(snapshots),
        NOW_MS,
        f"tick-random-{seed}",
    )
    decisions = {decision.robot_id: decision.action for decision in result.decisions}

    assert result.tick_metadata["global_safety_valid"] is True
    assert validate_global_safety(snapshots, decisions, config) == ()


@pytest.mark.parametrize("seed", range(10))
def test_component_solves_equal_the_whole_small_pairwise_model(seed: int) -> None:
    config = MonitorConfig(
        safety_margin_metres=0.0,
        cp_sat_time_limit_seconds=0.05,
    )
    model = generated_satisfiable_model(seed)
    robot_ids = tuple(model.snapshots)
    priorities = make_priorities(robot_ids)
    optimiser = CpSatComponentOptimiser(config)

    whole = optimiser.optimise(robot_ids, model, priorities)
    decomposed: dict[str, Action] = {}
    for component in model.connected_components:
        component_result = optimiser.optimise(component, model, priorities)
        assert component_result.feasible is True
        decomposed.update(component_result.decisions)

    assert whole.feasible is True
    assert decomposed == whole.decisions


@pytest.mark.parametrize("seed", range(10))
def test_solver_and_heuristic_outputs_satisfy_every_hard_constraint(seed: int) -> None:
    generator = random.Random(RANDOM_SEED + seed)
    robot_ids = tuple(f"robot-{index}" for index in range(5))
    pairs = tuple(
        make_pair(
            robot_i,
            robot_j,
            (((Action.RESUME, Action.RESUME),) if generator.random() < 0.55 else ()),
        )
        for robot_i, robot_j in combinations(robot_ids, 2)
    )
    model = make_model(robot_ids, pairs)
    priorities = make_priorities(robot_ids)
    hard_robot = generator.choice(robot_ids)
    hard_assignments = {hard_robot: Action.RESUME}

    solver_result = CpSatComponentOptimiser(MonitorConfig()).optimise(
        robot_ids,
        model,
        priorities,
        hard_assignments=hard_assignments,
    )
    heuristic_result = DeterministicComponentHeuristic(MonitorConfig()).solve(
        robot_ids,
        model,
        priorities,
        hard_assignments=hard_assignments,
    )

    assert solver_result.feasible is True
    assert heuristic_result.feasible is True
    validate_component_decisions(
        robot_ids,
        solver_result.decisions,
        model.pairwise_constraints,
        hard_assignments,
    )
    validate_heuristic_decisions(
        robot_ids,
        heuristic_result.decisions,
        model.pairwise_constraints,
        hard_assignments,
    )


def test_waiting_age_changes_order_after_grant_fault_guard_release() -> None:
    config = MonitorConfig(
        safety_margin_metres=0.0,
        stale_timeout_seconds=100.0,
        grant_minimum_hold_ticks=0,
        grant_maximum_hold_ticks=3,
        priority_base_progress_value=0,
        priority_loaded_bonus=0,
        priority_deadline_urgency_maximum=0,
        priority_low_battery_urgency_bonus=0,
        priority_waiting_age_bonus_per_tick=20,
        priority_active_grant_continuation_bonus=50,
        priority_clearance_bonus_per_conflict=0,
    )
    robot_ids = ("robot-a", "robot-b")
    model = make_model(
        robot_ids,
        (make_pair("robot-a", "robot-b", ((Action.RESUME, Action.RESUME),)),),
    )
    manager = GrantManager(config)
    optimiser = CpSatComponentOptimiser(config)

    for tick in range(1, 4):
        preparation = manager.prepare_tick(model, now_ms=NOW_MS + tick * 1_000)
        scores = score_component_priorities(
            robot_ids,
            model,
            config,
            now_ms=NOW_MS + tick * 1_000,
            waiting_ages={
                robot_id: manager.waiting_ages.get(robot_id, 0) for robot_id in robot_ids
            },
            active_grants=manager.active_grants,
        )
        solved = optimiser.optimise(
            robot_ids,
            model,
            scores,
            hard_assignments=preparation.hard_assignments,
        )
        assert solved.decisions == {
            "robot-a": Action.RESUME,
            "robot-b": Action.PAUSE,
        }
        manager.finalise_tick(model, solved.decisions, scores)

    released = manager.prepare_tick(model, now_ms=NOW_MS + 4_000)
    scores_after_release = score_component_priorities(
        robot_ids,
        model,
        config,
        now_ms=NOW_MS + 4_000,
        waiting_ages=dict(manager.waiting_ages),
        active_grants=manager.active_grants,
    )
    reordered = optimiser.optimise(robot_ids, model, scores_after_release)

    assert tuple(release.reason for release in released.released_grants) == (
        GrantReleaseReason.MAXIMUM_LEASE_FAULT_GUARD,
    )
    assert scores_after_release["robot-b"].waiting_age_bonus > 0
    assert reordered.decisions == {
        "robot-a": Action.PAUSE,
        "robot-b": Action.RESUME,
    }


@pytest.mark.parametrize(
    ("stale_robot", "order"),
    (
        ("robot-a", ("robot-a", "robot-b", "robot-c")),
        ("robot-b", ("robot-c", "robot-b", "robot-a")),
        ("robot-c", ("robot-b", "robot-a", "robot-c")),
    ),
)
def test_stale_robots_never_receive_resume(
    stale_robot: str,
    order: tuple[str, ...],
) -> None:
    snapshots = {
        robot_id: make_snapshot(
            robot_id,
            x=index * 5.0,
            y=0.0,
            next_x=index * 5.0 + 1.0,
            next_y=0.0,
            stale=robot_id == stale_robot,
        )
        for index, robot_id in enumerate(("robot-a", "robot-b", "robot-c"))
    }
    result = CollisionDecisionEngine(MonitorConfig()).decide(
        tuple(snapshots[robot_id] for robot_id in order),
        NOW_MS,
        "tick-stale",
    )

    action_by_robot = {decision.robot_id: decision.action for decision in result.decisions}
    assert action_by_robot[stale_robot] is Action.PAUSE


@pytest.mark.parametrize("seed", range(6))
def test_all_paused_repair_is_constraint_safe(seed: int) -> None:
    generator = random.Random(RANDOM_SEED + seed)
    robot_ids = tuple(f"robot-{index}" for index in range(5))
    edge_pairs = {(robot_ids[index], robot_ids[index + 1]) for index in range(len(robot_ids) - 1)}
    edge_pairs.update(pair for pair in combinations(robot_ids, 2) if generator.random() < 0.25)
    pairs = tuple(
        make_pair(
            robot_i,
            robot_j,
            ((Action.RESUME, Action.RESUME),) if (robot_i, robot_j) in edge_pairs else (),
        )
        for robot_i, robot_j in combinations(robot_ids, 2)
    )
    model = make_model(robot_ids, pairs)
    priorities = make_priorities(robot_ids)
    config = MonitorConfig()
    manager = GrantManager(config)
    manager.prepare_tick(model, now_ms=NOW_MS)

    preview = manager.preview_tick(
        model,
        {robot_id: Action.PAUSE for robot_id in robot_ids},
        priorities,
    )

    assert preview.alarms == ()
    assert any(action is Action.RESUME for action in preview.decisions.values())
    assert any(source is DecisionSource.REPAIR for source in preview.decision_sources.values())
    validate_component_decisions(
        robot_ids,
        preview.decisions,
        model.pairwise_constraints,
    )
    assert (
        validate_global_safety(
            tuple(model.snapshots.values()),
            preview.decisions,
            config,
        )
        == ()
    )


def test_current_footprint_overlap_emits_explicit_critical_alarm() -> None:
    snapshots = (
        make_snapshot("robot-a", x=0.0, y=0.0, next_x=2.0, next_y=0.0),
        make_snapshot("robot-b", x=0.0, y=0.0, next_x=-2.0, next_y=0.0),
    )

    result = CollisionDecisionEngine(MonitorConfig(safety_margin_metres=0.0)).decide(
        snapshots, NOW_MS, "tick-current-overlap"
    )

    assert EngineAlarmCode.CURRENT_FOOTPRINT_OVERLAP in result.tick_metadata["alarms"]
    assert result.tick_metadata["current_footprint_overlaps"]
    assert any(
        diagnostic["severity"] == "critical"
        and diagnostic["code"] == EngineAlarmCode.CURRENT_FOOTPRINT_OVERLAP
        for diagnostic in result.tick_metadata["critical_diagnostics"]
    )
    assert result.tick_metadata["global_safety_valid"] is False
    assert result.tick_metadata["state_committed"] is False
    assert all(decision.action is Action.PAUSE for decision in result.decisions)
