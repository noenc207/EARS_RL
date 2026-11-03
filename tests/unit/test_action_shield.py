"""Unit tests for ActionShield limit clamping and constraint validation."""

from ears_rl.control.shield import BasicActionShield
from ears_rl.core.types import JointCommand, SafetyState


def test_action_shield_passes_safe_command():
    """Verify in-bound command passes through unmodified."""
    shield = BasicActionShield(
        position_limits_rad=(-3.14, 3.14),
        velocity_limit_rad_s=5.0,
    )
    cmd = JointCommand(joint_id="j0", position_rad=1.0, velocity_rad_s=2.0)
    decision = shield.shield(cmd)

    assert decision.was_modified is False
    assert decision.shielded_command.position_rad == 1.0
    assert decision.safety_state == SafetyState.SAFE


def test_action_shield_clamps_out_of_bounds_position():
    """Verify out-of-bounds command is clamped to mechanical limits."""
    shield = BasicActionShield(
        position_limits_rad=(-2.0, 2.0),
        velocity_limit_rad_s=5.0,
    )
    cmd = JointCommand(joint_id="j0", position_rad=3.5)
    decision = shield.shield(cmd)

    assert decision.was_modified is True
    assert decision.shielded_command.position_rad == 2.0
    assert "Clamped to max position" in decision.intervention_reason
    assert decision.safety_state == SafetyState.WARNING


def test_action_shield_clamps_velocity():
    """Verify excessive velocity is saturated at limit."""
    shield = BasicActionShield(
        position_limits_rad=(-3.0, 3.0),
        velocity_limit_rad_s=4.0,
    )
    cmd = JointCommand(joint_id="j0", position_rad=0.0, velocity_rad_s=8.0)
    decision = shield.shield(cmd)

    assert decision.was_modified is True
    assert decision.shielded_command.velocity_rad_s == 4.0


def test_action_shield_strictly_rejects_nan_command_without_motion():
    """Verify non-finite command (NaN) is strictly rejected with shielded_command=None and FAULT state."""
    shield = BasicActionShield(
        position_limits_rad=(-3.0, 3.0),
        velocity_limit_rad_s=4.0,
    )
    cmd = JointCommand(joint_id="j0", position_rad=float("nan"))
    decision = shield.shield(cmd)

    assert decision.was_modified is True
    assert decision.is_rejected is True
    assert decision.shielded_command is None
    assert decision.safety_state == SafetyState.FAULT
    assert "non-finite" in decision.intervention_reason.lower()


def test_action_shield_strictly_rejects_inf_velocity():
    """Verify non-finite velocity command (Inf) is strictly rejected."""
    shield = BasicActionShield(
        position_limits_rad=(-3.0, 3.0),
        velocity_limit_rad_s=4.0,
    )
    cmd = JointCommand(joint_id="j0", position_rad=0.0, velocity_rad_s=float("inf"))
    decision = shield.shield(cmd)

    assert decision.was_modified is True
    assert decision.is_rejected is True
    assert decision.shielded_command is None
    assert decision.safety_state == SafetyState.FAULT


def test_action_shield_rejects_out_of_bounds_under_reject_policy():
    """Verify out-of-bounds command is strictly rejected when out_of_bounds_policy='reject'."""
    shield = BasicActionShield(
        position_limits_rad=(-2.0, 2.0),
        velocity_limit_rad_s=5.0,
        out_of_bounds_policy="reject",
    )
    cmd = JointCommand(joint_id="j0", position_rad=3.5)
    decision = shield.shield(cmd)

    assert decision.was_modified is True
    assert decision.is_rejected is True
    assert decision.shielded_command is None
    assert decision.safety_state == SafetyState.FAULT
    assert "rejected" in decision.intervention_reason.lower()


def test_rejected_command_suppresses_actuator_motion():
    """Verify passing rejected command (None) to actuator backend raises error and commands no motion."""
    import pytest

    from ears_rl.core.exceptions import CommandLimitError
    from ears_rl.core.types import ActuatorCapabilities
    from ears_rl.hardware.mock.actuator import MockActuator

    shield = BasicActionShield(
        position_limits_rad=(-2.0, 2.0),
        velocity_limit_rad_s=5.0,
    )
    actuator = MockActuator(
        joint_id="j0",
        capabilities=ActuatorCapabilities(limits_verified=True),
    )
    actuator.connect()
    actuator.enable_motion()

    # Shield a NaN command
    decision = shield.shield(JointCommand(joint_id="j0", position_rad=float("nan")))
    assert decision.shielded_command is None

    # Actuator must reject None and enqueue zero commands
    with pytest.raises(CommandLimitError, match="None"):
        actuator.send_position_command(decision.shielded_command)

    assert len(actuator._command_delay_queue) == 0


