from humaware_keyboard_teleop.keymap import TeleopVelocity, apply_key


def test_forward_key_increases_linear_velocity():
    velocity = apply_key(
        "w",
        TeleopVelocity(),
        linear_step_mps=0.05,
        angular_step_radps=0.1,
        max_linear_velocity_mps=0.5,
        max_angular_velocity_radps=0.5,
    )

    assert velocity.linear_x_mps == 0.05
    assert velocity.angular_z_radps == 0.0


def test_velocity_is_clamped():
    velocity = TeleopVelocity(linear_x_mps=0.49, angular_z_radps=0.45)
    velocity = apply_key(
        "w",
        velocity,
        linear_step_mps=0.05,
        angular_step_radps=0.1,
        max_linear_velocity_mps=0.5,
        max_angular_velocity_radps=0.5,
    )
    velocity = apply_key(
        "a",
        velocity,
        linear_step_mps=0.05,
        angular_step_radps=0.1,
        max_linear_velocity_mps=0.5,
        max_angular_velocity_radps=0.5,
    )

    assert velocity.linear_x_mps == 0.5
    assert velocity.angular_z_radps == 0.5


def test_stop_key_resets_velocity():
    velocity = apply_key(
        " ",
        TeleopVelocity(linear_x_mps=0.2, angular_z_radps=0.3),
        linear_step_mps=0.05,
        angular_step_radps=0.1,
        max_linear_velocity_mps=0.5,
        max_angular_velocity_radps=0.5,
    )

    assert velocity == TeleopVelocity()
