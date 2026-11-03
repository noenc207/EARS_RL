"""Safety-critical tests verifying hardware invariants and fail-safe interlocks."""

import pytest

from ears_rl.application.session import ExperimentSession
from ears_rl.config.schema import (
    ActuatorConfig,
    CommunicationConfig,
    ExperimentConfig,
    SafetyConfig,
)
from ears_rl.core.clock import MockClock
from ears_rl.core.exceptions import CommandLimitError, HardwareDisabledError
from ears_rl.core.types import (
    ActuatorCapabilities,
    HardwareMode,
    JointCommand,
    SafetyState,
)
from ears_rl.hardware.mock.actuator import MockActuator
from ears_rl.hardware.safety.monitor import SafetyMonitor


def test_hardware_disabled_by_default_blocks_motion():
    """MANDATORY INVARIANT: Actuator in DISABLED mode must reject motion commands."""
    caps = ActuatorCapabilities(
        position_min_rad=-3.14,
        position_max_rad=3.14,
        limits_verified=True,
    )
    actuator = MockActuator(
        joint_id="j_safety",
        capabilities=caps,
    )
    actuator._hardware_mode = HardwareMode.DISABLED  # Set to disabled
    actuator.connect()

    cmd = JointCommand(joint_id="j_safety", position_rad=0.5)
    with pytest.raises(HardwareDisabledError, match="DISABLED"):
        actuator.send_position_command(cmd)


def test_motion_not_enabled_blocks_command():
    """Verify command is rejected if enable_motion() has not been called."""
    caps = ActuatorCapabilities(limits_verified=True)
    actuator = MockActuator(
        joint_id="j_safety",
        capabilities=caps,
    )
    actuator.connect()
    # Notice: actuator.enable_motion() was NOT called!

    cmd = JointCommand(joint_id="j_safety", position_rad=0.0)
    with pytest.raises(HardwareDisabledError, match="Motion is not enabled"):
        actuator.send_position_command(cmd)


def test_watchdog_timeout_triggers_fault():
    """Verify watchdog expiration changes safety state to FAULT."""
    clock = MockClock(start_ns=0)
    config = SafetyConfig(watchdog_timeout_ms=500.0)
    monitor = SafetyMonitor(config=config, clock=clock)

    assert monitor.get_safety_state() == SafetyState.SAFE

    # Advance clock past 500 ms without heartbeat
    clock.tick_ms(501.0)
    assert monitor.get_safety_state() == SafetyState.FAULT

    # Verify commands are now blocked
    allowed, reason = monitor.check_command_allowed(JointCommand(joint_id="j0", position_rad=0.0))
    assert allowed is False
    assert "fault" in reason.lower()


def test_estop_cannot_be_cleared_by_software():
    """MANDATORY INVARIANT: Hardware E-stop signal cannot be cleared by software reset."""
    clock = MockClock(start_ns=0)
    config = SafetyConfig()
    monitor = SafetyMonitor(config=config, clock=clock)

    # Trip E-stop
    monitor.trigger_estop()
    assert monitor.get_safety_state() == SafetyState.ESTOP

    # Software attempt to reset fault must fail while E-stop line is asserted
    success = monitor.reset_fault()
    assert success is False
    assert monitor.get_safety_state() == SafetyState.ESTOP


def test_reconnect_clears_queue_no_motion_replay():
    """Verify that disconnecting an actuator flushes pending commands, preventing motion replay on reconnect."""
    caps = ActuatorCapabilities(limits_verified=True)
    actuator = MockActuator(joint_id="j_queue", capabilities=caps, latency_ms=100.0)
    actuator.connect()
    actuator.enable_motion()

    # Enqueue a command
    cmd = JointCommand(joint_id="j_queue", position_rad=1.0)
    actuator.send_position_command(cmd)
    assert len(actuator._command_delay_queue) == 1

    # Disconnect must flush pending motion queue
    actuator.disconnect()
    assert len(actuator._command_delay_queue) == 0

    # Reconnect: motion is disabled by default upon reconnect
    actuator.connect()
    assert actuator.is_motion_enabled is False
    assert len(actuator._command_delay_queue) == 0


def test_invalid_configuration_cannot_arm_hardware():
    """INVARIANT 3: Invalid configuration (e.g. unverified limits, missing port) cannot arm hardware."""
    # Attempt 1: hardware_enabled=True but limits unverified
    with pytest.raises(ValueError, match="unverified actuator limits"):
        ExperimentConfig(
            experiment_id="exp_invalid",
            safety=SafetyConfig(hardware_enabled=True, require_limits_verified=True),
            actuators=[ActuatorConfig(joint_id="j0", limits_verified=False)],
            communication=CommunicationConfig(serial_port="COM3"),
        )

    # Attempt 2: hardware_enabled=True but missing serial port
    with pytest.raises(ValueError, match="serial_port"):
        ExperimentConfig(
            experiment_id="exp_invalid2",
            safety=SafetyConfig(hardware_enabled=True, require_limits_verified=False),
            actuators=[ActuatorConfig(joint_id="j0", limits_verified=True)],
            communication=CommunicationConfig(serial_port=None),
        )

    # Attempt 3: motion_enabled_on_startup must always be False
    with pytest.raises(ValueError, match="motion_enabled_on_startup must be False"):
        ExperimentConfig(
            experiment_id="exp_invalid3",
            safety=SafetyConfig(hardware_enabled=False, motion_enabled_on_startup=True),
        )


def test_nan_inf_commands_rejected_at_base_actuator():
    """INVARIANT 4: NaN and Inf motion commands must be strictly rejected at the backend level."""
    caps = ActuatorCapabilities(limits_verified=True, position_min_rad=-3.14, position_max_rad=3.14)
    actuator = MockActuator(joint_id="j_nan", capabilities=caps)
    actuator.connect()
    actuator.enable_motion()

    # NaN position
    with pytest.raises(CommandLimitError, match="non-finite"):
        actuator.send_position_command(JointCommand(joint_id="j_nan", position_rad=float("nan")))

    # Inf position
    with pytest.raises(CommandLimitError, match="non-finite"):
        actuator.send_position_command(JointCommand(joint_id="j_nan", position_rad=float("inf")))

    # NaN velocity
    with pytest.raises(CommandLimitError, match="non-finite"):
        actuator.send_velocity_command(JointCommand(joint_id="j_nan", velocity_rad_s=float("nan")))


def test_latched_fault_requires_deliberate_recovery():
    """INVARIANT 9: Heartbeat arrival must NOT clear a latched FAULT without explicit reset."""
    clock = MockClock(start_ns=0)
    config = SafetyConfig(watchdog_timeout_ms=500.0)
    monitor = SafetyMonitor(config=config, clock=clock)

    assert monitor.get_safety_state() == SafetyState.SAFE

    # Trigger watchdog fault
    clock.tick_ms(550.0)
    assert monitor.get_safety_state() == SafetyState.FAULT

    # A heartbeat packet arrives: feeds watchdog timer
    monitor.feed_watchdog()
    # Invariant: Feeding watchdog refreshes timestamp but MUST NOT clear latched FAULT!
    assert monitor.get_safety_state() == SafetyState.FAULT
    allowed, _ = monitor.check_command_allowed(JointCommand(joint_id="j0", position_rad=0.0))
    assert allowed is False

    # Explicit reset_fault() clears the fault
    reset_ok = monitor.reset_fault()
    assert reset_ok is True
    assert monitor.get_safety_state() == SafetyState.SAFE


def test_exception_during_session_disables_motion():
    """INVARIANT 12: An exception during an experiment must not leave actuator motion enabled."""
    caps = ActuatorCapabilities(limits_verified=True)
    actuator = MockActuator(joint_id="j_crash", capabilities=caps)
    actuator.connect()
    actuator.enable_motion()
    assert actuator.is_motion_enabled is True

    # Run inside an ExperimentSession that raises an unhandled error
    try:
        with ExperimentSession(experiment_id="test_crash", actuators=[actuator]):
            assert actuator.is_motion_enabled is True
            raise RuntimeError("Simulated crash in user policy or control loop!")
    except RuntimeError:
        pass

    # Invariant: session must guarantee motion is disabled after exception
    assert actuator.is_motion_enabled is False


def test_fault_semantics_multi_tier_boundaries():
    """Verify SafetyStatus distinguishes software command blocking from hardware power cut."""
    clock = MockClock(start_ns=0)
    config = SafetyConfig(watchdog_timeout_ms=500.0)
    monitor = SafetyMonitor(config=config, clock=clock)

    # 1. Nominal Safe state
    status_safe = monitor.get_safety_status()
    assert status_safe.safety_state == SafetyState.SAFE
    assert status_safe.is_command_blocked is False
    assert status_safe.is_pulse_suppressed is False
    assert status_safe.is_driver_disabled is False
    assert status_safe.is_power_cut is False
    assert status_safe.is_mechanically_stopped is False

    # 2. Watchdog timeout triggers FAULT
    clock.tick_ms(550.0)
    status_fault = monitor.get_safety_status()
    assert status_fault.safety_state == SafetyState.FAULT
    assert status_fault.is_command_blocked is True
    assert status_fault.is_pulse_suppressed is True
    assert status_fault.is_driver_disabled is True
    # MANDATORY INVARIANT: Software FAULT does NOT cut hardware motor power!
    assert status_fault.is_power_cut is False
    # Software cannot assume mechanical standstill
    assert status_fault.is_mechanically_stopped is False
    assert "watchdog timeout" in status_fault.fault_reason.lower()

    # 3. E-stop assertion
    monitor.trigger_estop()
    status_estop = monitor.get_safety_status()
    assert status_estop.safety_state == SafetyState.ESTOP
    assert status_estop.is_command_blocked is True
    assert status_estop.is_pulse_suppressed is True
    assert status_estop.is_driver_disabled is True
    # Physical E-stop cuts hardware motor supply power
    assert status_estop.is_power_cut is True
    assert "e-stop" in status_estop.fault_reason.lower()


def test_reset_fault_fails_if_watchdog_unfed():
    """Verify reset_fault() fails if watchdog timer is still expired."""
    clock = MockClock(start_ns=0)
    config = SafetyConfig(watchdog_timeout_ms=500.0)
    monitor = SafetyMonitor(config=config, clock=clock)

    # Advance clock to expire watchdog
    clock.tick_ms(600.0)
    assert monitor.get_safety_state() == SafetyState.FAULT

    # Attempt to reset WITHOUT feeding watchdog first: must fail!
    success = monitor.reset_fault()
    assert success is False
    assert monitor.get_safety_state() == SafetyState.FAULT

    # Now feed watchdog and reset: must succeed!
    monitor.feed_watchdog()
    success = monitor.reset_fault()
    assert success is True
    assert monitor.get_safety_state() == SafetyState.SAFE


def test_action_shield_strictly_rejects_non_finite_without_neutral_substitution():
    """Verify ActionShield strictly drops non-finite (NaN/Inf) commands without fallback substitution."""
    from ears_rl.control.shield import BasicActionShield

    shield = BasicActionShield(
        position_limits_rad=(-3.14, 3.14),
        safe_neutral_position_rad=0.0,
        out_of_bounds_policy="clamp",  # Even with 'clamp', NaN/Inf must NOT be clamped to neutral!
    )

    for bad_val in (float("nan"), float("inf"), float("-inf")):
        cmd = JointCommand(joint_id="j_nan", position_rad=bad_val)
        decision = shield.shield(cmd)

        assert decision.is_rejected is True
        assert decision.shielded_command is None
        assert decision.safety_state == SafetyState.FAULT
        assert "non-finite" in decision.intervention_reason.lower()


def test_fault_and_estop_interlocks_strictly_block_new_commands():
    """Verify SafetyMonitor in FAULT or ESTOP state refuses all subsequent motion commands."""
    clock = MockClock(start_ns=0)
    config = SafetyConfig(watchdog_timeout_ms=500.0)
    monitor = SafetyMonitor(config=config, clock=clock)

    test_cmd = JointCommand(joint_id="j0", position_rad=0.5)

    # 1. FAULT state blocks command
    clock.tick_ms(550.0)
    assert monitor.get_safety_state() == SafetyState.FAULT
    allowed, reason = monitor.check_command_allowed(test_cmd)
    assert allowed is False
    assert "fault" in reason.lower()

    # 2. ESTOP state blocks command
    monitor.trigger_estop()
    assert monitor.get_safety_state() == SafetyState.ESTOP
    allowed, reason = monitor.check_command_allowed(test_cmd)
    assert allowed is False
    assert "e-stop" in reason.lower()


def test_commissioning_gate_config_default_and_custom_propagation():
    """Verify CommissioningGateConfig thresholds propagate correctly into commissioning gates."""
    from ears_rl.application.commissioning import (
        CommissioningGateConfig,
        get_default_commissioning_gates,
    )

    # Custom config
    custom_cfg = CommissioningGateConfig(
        isolation_resistance_min_kohm=25.0,
        ground_continuity_max_ohm=0.15,
        packet_loss_max_percent=0.05,
        encoder_cpr=8192,
    )

    gates = get_default_commissioning_gates(custom_cfg)
    assert len(gates) == 5

    # Check Gate B reflects custom isolation & continuity
    gate_b = next(g for g in gates if g.gate_id == "gate_b")
    assert "25.0 kOhm" in gate_b.acceptance_criteria
    assert "0.15 Ohm" in gate_b.acceptance_criteria
    assert gate_b.requires_physical_hardware is True

    # Check Gate C reflects custom packet loss
    gate_c = next(g for g in gates if g.gate_id == "gate_c")
    assert "0.05%" in gate_c.acceptance_criteria
    assert gate_c.requires_physical_hardware is False

    # Check Gate D reflects custom encoder CPR
    gate_d = next(g for g in gates if g.gate_id == "gate_d")
    assert "+8192 counts" in gate_d.acceptance_criteria
    assert gate_d.requires_physical_hardware is True




