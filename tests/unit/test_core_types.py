"""Unit tests for core types, physical units, and exceptions."""

import pytest

from ears_rl.core.clock import MockClock
from ears_rl.core.exceptions import (
    CommandLimitError,
    EarsRLError,
    HardwareDisabledError,
    IsaacLabNotAvailableError,
)
from ears_rl.core.types import (
    ActuatorCapabilities,
    ActuatorType,
    HardwareMode,
    JointPosition,
    Timestamp,
)


def test_timestamp_units_and_conversions():
    """Verify nanosecond timestamp conversion to seconds and milliseconds."""
    ts = Timestamp(ns=1_500_000_000, wall_utc_ns=1_500_000_000)
    assert ts.seconds == 1.5
    assert ts.milliseconds == 1500.0


def test_joint_position_degrees_conversion():
    """Verify radian to degree conversion."""
    pos = JointPosition(radians=3.141592653589793, joint_id="j0")
    assert pytest.approx(pos.degrees, rel=1e-5) == 180.0


def test_actuator_capabilities_defaults():
    """Verify unverified limits default to None."""
    caps = ActuatorCapabilities(actuator_type=ActuatorType.STEPPER)
    assert caps.limits_verified is False
    assert caps.position_min_rad is None
    assert caps.position_max_rad is None
    assert caps.velocity_max_rad_s is None


def test_hardware_mode_default_is_disabled():
    """Verify default HardwareMode enum is DISABLED."""
    assert HardwareMode.DISABLED.value == "disabled"


def test_mock_clock_tick():
    """Verify MockClock advances deterministically."""
    clock = MockClock(start_ns=1000)
    assert clock.now_ns() == 1000
    clock.tick(500)
    assert clock.now_ns() == 1500
    clock.tick_ms(2.0)
    assert clock.now_ns() == 2_001_500


def test_exception_hierarchy():
    """Verify all custom exceptions inherit from EarsRLError."""
    assert issubclass(HardwareDisabledError, EarsRLError)
    assert issubclass(CommandLimitError, EarsRLError)
    assert issubclass(IsaacLabNotAvailableError, EarsRLError)
