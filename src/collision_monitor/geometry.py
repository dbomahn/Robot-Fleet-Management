"""Conservative geometric envelopes for robot actions.

Boundary contact is unsafe for the collision monitor. Shapely's ``intersects``
predicate is therefore intentional: unlike overlap-only predicates, it returns
``True`` when two polygon boundaries merely touch.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TypeAlias

from shapely import affinity, make_valid
from shapely.geometry import Polygon, box
from shapely.geometry.base import BaseGeometry

from collision_monitor.config import MonitorConfig
from collision_monitor.models import Action, Pose, RobotSnapshot


class GeometryConstructionError(RuntimeError):
    """Raised when a safe, non-empty polygon cannot be constructed."""

    def __init__(self, robot_id: str, detail: str) -> None:
        super().__init__(f"geometry construction failed for robot {robot_id!r}: {detail}")
        self.robot_id = robot_id


@dataclass(frozen=True, slots=True)
class RobotDimensions:
    """Physical robot dimensions in metres."""

    width_metres: float
    length_metres: float


DimensionsLike: TypeAlias = RobotDimensions | tuple[float, float]
_BUFFER_QUADRANT_SEGMENTS = 8
_BUFFER_ROUNDING_GUARD = 1e-12


def _conservative_buffer_distance(safety_margin: float) -> float:
    """Inflate a round-buffer radius so its chords contain the requested offset.

    GEOS represents each quarter-circle with a finite number of chords. A
    polygon using the requested radius directly lies slightly inside the exact
    circular offset between vertices. Dividing by the chord apothem factor
    makes the generated polygon circumscribe, rather than inscribe, the exact
    requested safety margin.
    """
    half_segment_angle = math.pi / (4 * _BUFFER_QUADRANT_SEGMENTS)
    compensated = safety_margin / math.cos(half_segment_angle)
    return compensated * (1.0 + _BUFFER_ROUNDING_GUARD)


def _normalise_dimensions(
    dimensions: DimensionsLike,
    *,
    robot_id: str,
) -> RobotDimensions:
    """Return validated dimensions from the public supported representations."""
    if isinstance(dimensions, RobotDimensions):
        result = dimensions
    else:
        try:
            width_metres, length_metres = dimensions
        except (TypeError, ValueError) as exc:
            raise GeometryConstructionError(
                robot_id,
                "dimensions must contain width and length",
            ) from exc
        result = RobotDimensions(
            width_metres=width_metres,
            length_metres=length_metres,
        )

    if not math.isfinite(result.width_metres) or result.width_metres <= 0:
        raise GeometryConstructionError(robot_id, "width must be finite and greater than zero")
    if not math.isfinite(result.length_metres) or result.length_metres <= 0:
        raise GeometryConstructionError(robot_id, "length must be finite and greater than zero")
    return result


def _validated_polygon(geometry: BaseGeometry, *, robot_id: str) -> Polygon:
    """Validate a polygon, using ``make_valid`` only for an invalid result.

    ``make_valid`` is the minimal documented Shapely repair used here: it is
    called only after construction unexpectedly produces invalid geometry. A
    repair that changes the result into a non-polygon is rejected rather than
    being silently coerced into a potentially unsafe shape.
    """
    candidate = geometry
    if not candidate.is_valid:
        candidate = make_valid(candidate)

    if candidate.is_empty:
        raise GeometryConstructionError(robot_id, "constructed geometry is empty")
    if not isinstance(candidate, Polygon):
        raise GeometryConstructionError(
            robot_id,
            f"constructed geometry has unsupported type {candidate.geom_type!r}",
        )
    if not candidate.is_valid:
        raise GeometryConstructionError(robot_id, "constructed geometry remains invalid")
    return candidate


def _oriented_footprint(
    pose: Pose,
    dimensions: DimensionsLike,
    safety_margin: float,
    *,
    robot_id: str,
) -> Polygon:
    """Construct a footprint while retaining robot context for failures."""
    values = (pose.x, pose.y, pose.theta, safety_margin)
    if not all(math.isfinite(value) for value in values):
        raise GeometryConstructionError(robot_id, "pose and safety margin must be finite")
    if safety_margin < 0:
        raise GeometryConstructionError(robot_id, "safety margin must not be negative")

    size = _normalise_dimensions(dimensions, robot_id=robot_id)
    half_length = size.length_metres / 2.0
    half_width = size.width_metres / 2.0

    # The unrotated robot points along positive x and is centred at the origin.
    footprint: BaseGeometry = box(-half_length, -half_width, half_length, half_width)
    footprint = affinity.rotate(footprint, pose.theta, origin=(0.0, 0.0), use_radians=True)
    footprint = affinity.translate(footprint, xoff=pose.x, yoff=pose.y)
    if safety_margin > 0:
        footprint = footprint.buffer(
            _conservative_buffer_distance(safety_margin),
            quad_segs=_BUFFER_QUADRANT_SEGMENTS,
        )
    return _validated_polygon(footprint, robot_id=robot_id)


def oriented_footprint(
    pose: Pose,
    dimensions: DimensionsLike,
    safety_margin: float,
) -> Polygon:
    """Return the oriented, safety-buffered footprint centred at ``pose``.

    A two-item dimensions tuple is ordered as ``(width_metres, length_metres)``.
    Errors use ``<unknown>`` because this low-level function receives no robot
    identifier; snapshot-based envelope functions always report the actual ID.
    """
    return _oriented_footprint(
        pose,
        dimensions,
        safety_margin,
        robot_id="<unknown>",
    )


def _configured_dimensions(config: MonitorConfig) -> RobotDimensions:
    """Extract physical dimensions from monitor configuration."""
    return RobotDimensions(
        width_metres=config.robot_width_metres,
        length_metres=config.robot_length_metres,
    )


def pause_envelope(snapshot: RobotSnapshot, config: MonitorConfig) -> Polygon:
    """Return the buffered footprint occupied while the robot is paused."""
    return _oriented_footprint(
        snapshot.state.current_pose(),
        _configured_dimensions(config),
        config.safety_margin_metres,
        robot_id=snapshot.state.device_id,
    )


def resume_envelope(snapshot: RobotSnapshot, config: MonitorConfig) -> Polygon:
    """Return a conservative convex sweep between current and next poses.

    For unchanged heading, the convex hull of both buffered footprints exactly
    covers straight-line translation. When heading changes, endpoint
    footprints alone do not necessarily contain every intermediate rectangle.
    In that case, the envelope uses the convex hull of axis-aligned squares
    whose half-size is the robot's circumscribed radius plus safety margin.
    This deliberately conservative sweep contains every intermediate heading
    while the rectangle centre translates along the step.
    """
    robot_id = snapshot.state.device_id
    dimensions = _configured_dimensions(config)
    current = _oriented_footprint(
        snapshot.state.current_pose(),
        dimensions,
        config.safety_margin_metres,
        robot_id=robot_id,
    )
    following = _oriented_footprint(
        snapshot.next_pose,
        dimensions,
        config.safety_margin_metres,
        robot_id=robot_id,
    )
    endpoint_hull = current.union(following).convex_hull
    heading_delta = math.atan2(
        math.sin(snapshot.next_pose.theta - snapshot.state.theta),
        math.cos(snapshot.next_pose.theta - snapshot.state.theta),
    )
    if heading_delta == 0.0:
        return _validated_polygon(endpoint_hull, robot_id=robot_id)

    circumscribed_radius = math.hypot(
        dimensions.length_metres, dimensions.width_metres
    ) / 2.0 + _conservative_buffer_distance(config.safety_margin_metres)
    current_pose = snapshot.state.current_pose()
    rotational_start = box(
        current_pose.x - circumscribed_radius,
        current_pose.y - circumscribed_radius,
        current_pose.x + circumscribed_radius,
        current_pose.y + circumscribed_radius,
    )
    rotational_end = box(
        snapshot.next_pose.x - circumscribed_radius,
        snapshot.next_pose.y - circumscribed_radius,
        snapshot.next_pose.x + circumscribed_radius,
        snapshot.next_pose.y + circumscribed_radius,
    )
    return _validated_polygon(
        rotational_start.union(rotational_end).convex_hull,
        robot_id=robot_id,
    )


def action_envelope(
    snapshot: RobotSnapshot,
    action: Action,
    config: MonitorConfig,
) -> Polygon:
    """Return the conservative occupied area for a Pause or Resume action."""
    if action is Action.PAUSE:
        return pause_envelope(snapshot, config)
    if action is Action.RESUME:
        return resume_envelope(snapshot, config)
    raise GeometryConstructionError(
        snapshot.state.device_id,
        f"unsupported action {action!r}",
    )


def geometries_conflict(a: BaseGeometry, b: BaseGeometry) -> bool:
    """Return whether geometries overlap or touch; boundary contact is unsafe."""
    return bool(a.intersects(b))
