"""Unit tests for one-node-per-tick simulated robot behaviour."""

from __future__ import annotations

from collision_monitor.models import Action, Pose
from collision_monitor.simulator.robot import SimulatedRobot


def make_robot() -> SimulatedRobot:
    """Build a robot with two future nodes."""
    return SimulatedRobot(
        robot_id="robot-a",
        pose=Pose(0.0, 0.0, 0.0),
        future_path=(Pose(1.0, 0.0, 0.0), Pose(2.0, 0.0, 0.0)),
        loaded=False,
        battery_level=80.0,
        deadline_ms=1_720_000_060_000,
    )


def test_no_received_action_defaults_to_safe_pause() -> None:
    robot = make_robot()

    robot.advance_for_tick(1)

    assert robot.pose == Pose(0.0, 0.0, 0.0)
    assert robot.nodes_advanced == 0
    assert robot.action_received is False


def test_resume_advances_exactly_one_node_on_the_next_step() -> None:
    robot = make_robot()
    robot.receive_action(Action.RESUME)

    robot.advance_for_tick(2)

    assert robot.pose == Pose(1.0, 0.0, 0.0)
    assert robot.remaining_path == (
        Pose(1.0, 0.0, 0.0),
        Pose(2.0, 0.0, 0.0),
    )
    assert robot.nodes_advanced == 1


def test_pause_keeps_current_pose_and_published_path_starts_there() -> None:
    robot = make_robot()
    robot.receive_action(Action.PAUSE)
    robot.advance_for_tick(2)

    payload = robot.state_payload(timestamp_ms=1_720_000_001_000)

    assert (payload["x"], payload["y"], payload["theta"]) == (0.0, 0.0, 0.0)
    assert payload["path"][0] == {"x": 0.0, "y": 0.0, "theta": 0.0}


def test_goal_has_one_terminal_empty_path_publication_then_stops() -> None:
    robot = SimulatedRobot(
        robot_id="robot-a",
        pose=Pose(0.0, 0.0, 0.0),
        future_path=(Pose(1.0, 0.0, 0.0),),
        loaded=False,
        battery_level=80.0,
        deadline_ms=1_720_000_060_000,
    )
    robot.receive_action(Action.RESUME)
    robot.advance_for_tick(2)

    assert robot.at_goal is True
    assert robot.state_payload(timestamp_ms=1_720_000_001_000)["path"] == []
    assert robot.should_publish_state is True
    robot.mark_state_published()
    assert robot.should_publish_state is False
