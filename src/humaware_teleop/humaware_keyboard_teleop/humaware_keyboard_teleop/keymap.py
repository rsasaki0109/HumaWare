"""Keyboard command mapping for HumaWare teleop."""

from dataclasses import dataclass


@dataclass(frozen=True)
class TeleopVelocity:
    """Teleop velocity target."""

    linear_x_mps: float = 0.0
    angular_z_radps: float = 0.0


STOP_KEYS = {"x", " "}
HELP_KEYS = {"h", "?"}


def apply_key(
    key: str,
    velocity: TeleopVelocity,
    linear_step_mps: float,
    angular_step_radps: float,
    max_linear_velocity_mps: float,
    max_angular_velocity_radps: float,
) -> TeleopVelocity:
    """Return a new velocity target after applying a keyboard command."""

    linear = velocity.linear_x_mps
    angular = velocity.angular_z_radps

    if key == "w":
        linear += linear_step_mps
    elif key == "s":
        linear -= linear_step_mps
    elif key == "a":
        angular += angular_step_radps
    elif key == "d":
        angular -= angular_step_radps
    elif key in STOP_KEYS:
        return TeleopVelocity()

    return TeleopVelocity(
        linear_x_mps=_clamp(linear, -max_linear_velocity_mps, max_linear_velocity_mps),
        angular_z_radps=_clamp(angular, -max_angular_velocity_radps, max_angular_velocity_radps),
    )


def is_motion_key(key: str) -> bool:
    """Return true when a key updates or stops teleop velocity."""

    return key in {"w", "s", "a", "d"} or key in STOP_KEYS


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))
