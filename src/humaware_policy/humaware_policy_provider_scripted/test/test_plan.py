"""Unit tests for the scripted policy provider waypoint state machine."""

import pytest

from humaware_policy_provider_scripted.plan import (
    PlanProgress,
    Waypoint,
    WaypointPlan,
    advance_progress,
    current_command,
    initial_progress,
    parse_plan,
)


def _plan(*waypoints: Waypoint, loop: bool = False) -> WaypointPlan:
    return WaypointPlan(waypoints=tuple(waypoints), loop=loop)


def _wp(lin: float = 0.1, ang: float = 0.0, duration: float = 1.0) -> Waypoint:
    return Waypoint(linear_x_mps=lin, angular_z_radps=ang, duration_s=duration)


# parse_plan ------------------------------------------------------------------

def test_parse_plan_with_defaults_returns_empty_non_looping_plan():
    plan = parse_plan({})

    assert plan.waypoints == ()
    assert plan.loop is False


def test_parse_plan_preserves_waypoint_order_and_values():
    raw = {
        "loop": True,
        "waypoints": [
            {"linear_x_mps": 0.1, "angular_z_radps": 0.0, "duration_s": 2.0},
            {"linear_x_mps": 0.0, "angular_z_radps": 0.2, "duration_s": 1.5},
        ],
    }

    plan = parse_plan(raw)

    assert plan.loop is True
    assert len(plan.waypoints) == 2
    assert plan.waypoints[0].linear_x_mps == pytest.approx(0.1)
    assert plan.waypoints[1].angular_z_radps == pytest.approx(0.2)
    assert plan.waypoints[1].duration_s == pytest.approx(1.5)


def test_parse_plan_rejects_non_list_waypoints():
    with pytest.raises(ValueError, match="waypoints"):
        parse_plan({"waypoints": "not a list"})


def test_parse_plan_rejects_non_mapping_waypoint_entry():
    with pytest.raises(ValueError, match="#0"):
        parse_plan({"waypoints": ["not a mapping"]})


def test_parse_plan_rejects_missing_field():
    with pytest.raises(ValueError, match="#0"):
        parse_plan({"waypoints": [{"linear_x_mps": 0.1, "duration_s": 1.0}]})


def test_parse_plan_rejects_non_positive_duration():
    with pytest.raises(ValueError, match="positive duration"):
        parse_plan({
            "waypoints": [
                {"linear_x_mps": 0.1, "angular_z_radps": 0.0, "duration_s": 0.0},
            ],
        })


# advance_progress ------------------------------------------------------------

def test_advance_stays_within_waypoint_when_dt_smaller_than_duration():
    plan = _plan(_wp(duration=2.0), _wp(duration=2.0))

    progress = advance_progress(initial_progress(), plan, dt_s=0.5)

    assert progress == PlanProgress(
        waypoint_index=0, elapsed_in_waypoint_s=0.5, completed=False
    )


def test_advance_moves_to_next_waypoint_when_duration_elapsed():
    plan = _plan(_wp(duration=1.0), _wp(duration=2.0))

    progress = advance_progress(initial_progress(), plan, dt_s=1.25)

    assert progress.waypoint_index == 1
    assert progress.elapsed_in_waypoint_s == pytest.approx(0.25)
    assert progress.completed is False


def test_advance_walks_through_multiple_waypoints_in_one_tick():
    plan = _plan(_wp(duration=0.5), _wp(duration=0.5), _wp(duration=5.0))

    progress = advance_progress(initial_progress(), plan, dt_s=1.1)

    assert progress.waypoint_index == 2
    assert progress.elapsed_in_waypoint_s == pytest.approx(0.1)
    assert progress.completed is False


def test_advance_marks_completed_when_non_looping_plan_finishes():
    plan = _plan(_wp(duration=1.0), _wp(duration=1.0))

    progress = advance_progress(initial_progress(), plan, dt_s=3.0)

    assert progress.completed is True


def test_advance_wraps_to_zero_when_loop_enabled():
    plan = _plan(_wp(duration=1.0), _wp(duration=1.0), loop=True)

    progress = advance_progress(initial_progress(), plan, dt_s=2.25)

    assert progress.waypoint_index == 0
    assert progress.elapsed_in_waypoint_s == pytest.approx(0.25)
    assert progress.completed is False


def test_advance_keeps_completed_progress_idempotent():
    plan = _plan(_wp(duration=1.0))
    completed = PlanProgress(waypoint_index=1, elapsed_in_waypoint_s=0.0, completed=True)

    progress = advance_progress(completed, plan, dt_s=10.0)

    assert progress == completed


def test_advance_returns_completed_for_empty_plan():
    plan = _plan()

    progress = advance_progress(initial_progress(), plan, dt_s=1.0)

    assert progress.completed is True


def test_advance_rejects_negative_dt():
    plan = _plan(_wp(duration=1.0))

    with pytest.raises(ValueError, match="non-negative"):
        advance_progress(initial_progress(), plan, dt_s=-0.1)


# current_command -------------------------------------------------------------

def test_current_command_returns_active_waypoint():
    plan = _plan(_wp(lin=0.2, duration=2.0), _wp(lin=0.5, duration=2.0))
    progress = PlanProgress(waypoint_index=1, elapsed_in_waypoint_s=0.1, completed=False)

    waypoint = current_command(progress, plan)

    assert waypoint is not None
    assert waypoint.linear_x_mps == pytest.approx(0.5)


def test_current_command_returns_none_on_empty_plan():
    plan = _plan()

    assert current_command(initial_progress(), plan) is None


def test_current_command_returns_none_when_completed():
    plan = _plan(_wp(duration=1.0))
    progress = PlanProgress(waypoint_index=1, elapsed_in_waypoint_s=0.0, completed=True)

    assert current_command(progress, plan) is None


def test_current_command_returns_first_waypoint_at_start():
    plan = _plan(_wp(lin=0.123, duration=2.0))

    waypoint = current_command(initial_progress(), plan)

    assert waypoint is not None
    assert waypoint.linear_x_mps == pytest.approx(0.123)
