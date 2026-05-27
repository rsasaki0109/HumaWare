"""Pure waypoint state machine for the scripted policy provider.

The reading side of the provider parses YAML on disk and runs inside a
:class:`rclpy.node.Node`. This module keeps the plan model and the
state-machine helpers dependency-free so the rules can be exercised in
unit tests without rclpy.
"""

from dataclasses import dataclass
from typing import Mapping, Optional


@dataclass(frozen=True)
class Waypoint:
    """A single scripted command segment."""

    linear_x_mps: float
    angular_z_radps: float
    duration_s: float


@dataclass(frozen=True)
class WaypointPlan:
    """A static plan executed by the scripted provider."""

    waypoints: tuple[Waypoint, ...]
    loop: bool = False


@dataclass(frozen=True)
class PlanProgress:
    """Position within a :class:`WaypointPlan` at a moment in time."""

    waypoint_index: int
    elapsed_in_waypoint_s: float
    completed: bool


def initial_progress() -> PlanProgress:
    """Return the starting position before any tick has elapsed."""
    return PlanProgress(waypoint_index=0, elapsed_in_waypoint_s=0.0, completed=False)


def parse_plan(raw: Mapping[str, object]) -> WaypointPlan:
    """Parse a YAML-derived mapping into a :class:`WaypointPlan`.

    The YAML schema is intentionally small so scripted demos remain easy
    to author by hand:

    .. code-block:: yaml

        loop: false
        waypoints:
          - linear_x_mps: 0.1
            angular_z_radps: 0.0
            duration_s: 2.0
    """
    waypoints_raw = raw.get("waypoints", [])
    if not isinstance(waypoints_raw, list):
        raise ValueError(
            f"'waypoints' must be a list, got {type(waypoints_raw).__name__}"
        )
    waypoints = tuple(_parse_waypoint(entry, index) for index, entry in enumerate(waypoints_raw))
    loop = bool(raw.get("loop", False))
    return WaypointPlan(waypoints=waypoints, loop=loop)


def _parse_waypoint(raw: object, index: int) -> Waypoint:
    if not isinstance(raw, Mapping):
        raise ValueError(
            f"waypoint #{index} must be a mapping, got {type(raw).__name__}"
        )
    try:
        linear_x = float(raw["linear_x_mps"])
        angular_z = float(raw["angular_z_radps"])
        duration = float(raw["duration_s"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"waypoint #{index} is malformed: {exc}") from exc
    if duration <= 0.0:
        raise ValueError(
            f"waypoint #{index} must have a positive duration, got {duration}"
        )
    return Waypoint(
        linear_x_mps=linear_x,
        angular_z_radps=angular_z,
        duration_s=duration,
    )


def advance_progress(
    progress: PlanProgress,
    plan: WaypointPlan,
    dt_s: float,
) -> PlanProgress:
    """Advance the progress by ``dt_s`` seconds and return the new state.

    Behavior:

    - An empty plan or an already completed non-looping plan is returned
      unchanged.
    - When ``dt_s`` covers more than one waypoint the helper walks
      forward and consumes them one by one. This keeps coarse tick rates
      from skipping short waypoints silently.
    - When the final waypoint completes and ``plan.loop`` is true, the
      progress wraps back to waypoint 0 with the remaining ``dt_s``
      preserved.
    - When the final waypoint completes and ``plan.loop`` is false the
      progress is marked completed and stays there.
    """
    if dt_s < 0.0:
        raise ValueError(f"dt_s must be non-negative, got {dt_s}")
    if not plan.waypoints:
        return PlanProgress(waypoint_index=0, elapsed_in_waypoint_s=0.0, completed=True)
    if progress.completed:
        return progress

    index = progress.waypoint_index
    elapsed = progress.elapsed_in_waypoint_s + dt_s

    while index < len(plan.waypoints) and elapsed >= plan.waypoints[index].duration_s:
        elapsed -= plan.waypoints[index].duration_s
        index += 1
        if index >= len(plan.waypoints):
            if plan.loop:
                index = 0
            else:
                return PlanProgress(
                    waypoint_index=len(plan.waypoints),
                    elapsed_in_waypoint_s=0.0,
                    completed=True,
                )

    return PlanProgress(
        waypoint_index=index,
        elapsed_in_waypoint_s=elapsed,
        completed=False,
    )


def current_command(progress: PlanProgress, plan: WaypointPlan) -> Optional[Waypoint]:
    """Return the waypoint to emit at ``progress``, or ``None`` if idle."""
    if progress.completed or not plan.waypoints:
        return None
    if progress.waypoint_index >= len(plan.waypoints):
        return None
    return plan.waypoints[progress.waypoint_index]
