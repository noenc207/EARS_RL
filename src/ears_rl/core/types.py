"""Core type definitions for EARS-RL.

This module defines fundamental types, enums, and dataclasses used across
the entire EARS-RL research architecture. All physical quantities carry
explicit SI units in their variable names and documentation.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Any

import numpy as np


class ActuatorType(str, enum.Enum):
    """Type of actuator in the hybrid biped drivetrain."""
    STEPPER = "stepper"
    SERVO = "servo"
    HYBRID = "hybrid"
    UNKNOWN = "unknown"


class ActuatorState(str, enum.Enum):
    """Estimated actuator operating state.

    CRITICAL SCIENTIFIC DISTINCTION:
    These represent MODEL-ESTIMATED or DETECTOR-FLAGGED states.
    They must NOT be conflated with independently verified ground truth.
    """
    NORMAL = "normal"
    SATURATED = "saturated"
    STEP_LOSS = "step_loss"
    STALL = "stall"
    RECOVERY = "recovery"
    UNKNOWN = "unknown"


class EventSource(str, enum.Enum):
    """Origin of an event classification."""
    MODEL_ESTIMATE = "model_estimate"
    DETECTOR = "detector"
    GROUND_TRUTH = "ground_truth"
    MANUAL_LABEL = "manual_label"
    SYNTHETIC_INJECTION = "synthetic_injection"
    UNKNOWN = "unknown"


class DataSourceType(str, enum.Enum):
    """Scientific data origin."""
    SYNTHETIC = "synthetic"
    SIMULATED = "simulated"
    REAL_HARDWARE = "real_hardware"
    UNKNOWN = "unknown"


class HardwareMode(str, enum.Enum):
    """Hardware operation mode.

    MANDATORY INVARIANT: Default is DISABLED.
    Hardware mode requires explicit, verified two-step configuration.
    """
    DISABLED = "disabled"
    MOCK = "mock"
    SIMULATION = "simulation"
    HARDWARE = "hardware"


class SafetyState(str, enum.Enum):
    """Safety state machine states."""
    SAFE = "safe"
    WARNING = "warning"
    FAULT = "fault"
    ESTOP = "estop"
    UNINITIALIZED = "uninitialized"


class PhysicalSafetyBoundary(str, enum.Enum):
    """Rigorous physical boundary state distinguishing software, firmware, electrical, and mechanical domains.

    CRITICAL SCIENTIFIC & SAFETY DISTINCTION:
    - COMMAND_BLOCKED: Software layer rejects new commands (e.g. ActionShield, SafetyMonitor).
    - STEP_PULSES_SUPPRESSED: Firmware/MCU timer suppresses step pulse generation.
    - DRIVER_DEASSERTED: Driver ENABLE line is set HIGH/disabled; motor phases are un-energized.
    - POWER_DISCONNECTED: Physical contactor or E-stop relay has opened the VM motor supply rail.
    - MECHANICAL_REST: Actuator shaft and connected kinematic payload have reached zero velocity.

    WARNING: Software FAULT (COMMAND_BLOCKED) must NEVER be conflated with POWER_DISCONNECTED or MECHANICAL_REST.
    """
    COMMAND_BLOCKED = "command_blocked"
    STEP_PULSES_SUPPRESSED = "step_pulses_suppressed"
    DRIVER_DEASSERTED = "driver_deasserted"
    POWER_DISCONNECTED = "power_disconnected"
    MECHANICAL_REST = "mechanical_rest"


@dataclass
class SafetyStatus:
    """Comprehensive multi-tier safety status reporting across software, electrical, and mechanical domains."""
    safety_state: SafetyState
    is_command_blocked: bool = False
    is_pulse_suppressed: bool = False
    is_driver_disabled: bool = False
    is_power_cut: bool = False
    is_mechanically_stopped: bool = False
    fault_reason: str = ""
    timestamp_ns: int = 0


class ConnectionState(str, enum.Enum):
    """Hardware bus connectivity states."""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


# --- Physical Quantity Dataclasses with Explicit Units ---

@dataclass(frozen=True)
class Timestamp:
    """Monotonic timestamp with nanosecond resolution.

    Attributes:
        ns: Monotonic elapsed time in nanoseconds [ns].
        wall_utc_ns: Wall-clock UTC time in nanoseconds [ns], optional for correlation.
    """
    ns: int
    wall_utc_ns: int | None = None

    @property
    def seconds(self) -> float:
        """Convert to seconds [s]."""
        return self.ns / 1_000_000_000.0

    @property
    def milliseconds(self) -> float:
        """Convert to milliseconds [ms]."""
        return self.ns / 1_000_000.0


@dataclass(frozen=True)
class JointPosition:
    """Angular position of a joint or actuator shaft.

    Attributes:
        radians: Angular position [rad].
        joint_id: Associated joint identifier.
        source: Origin ('motor_encoder', 'joint_encoder', 'command', 'simulated').
    """
    radians: float
    joint_id: str
    source: str = "unknown"

    @property
    def degrees(self) -> float:
        """Convert to degrees [deg]."""
        return float(np.degrees(self.radians))


@dataclass(frozen=True)
class JointVelocity:
    """Angular velocity of a joint or actuator shaft.

    Attributes:
        radians_per_second: Angular velocity [rad/s].
        joint_id: Associated joint identifier.
    """
    radians_per_second: float
    joint_id: str


@dataclass(frozen=True)
class JointCommand:
    """Motion command sent to a joint actuator.

    Attributes:
        joint_id: Target joint identifier.
        position_rad: Target position [rad].
        velocity_rad_s: Target velocity [rad/s].
        acceleration_rad_s2: Target acceleration [rad/s²].
        timestamp: Time of command creation.
        sequence_id: Monotonically increasing sequence ID.
    """
    joint_id: str
    position_rad: float | None = None
    velocity_rad_s: float | None = None
    acceleration_rad_s2: float | None = None
    timestamp: Timestamp | None = None
    sequence_id: int = 0


# Alias for backward and forward compatibility
ActuatorCommand = JointCommand


@dataclass
class EncoderReading:
    """Single reading from an angular encoder.

    Attributes:
        position_rad: Angular position [rad].
        velocity_rad_s: Angular velocity [rad/s] if estimated by driver.
        timestamp: Monotonic timestamp of acquisition.
        sequence_id: Packet sequence ID.
        valid: Whether reading passed checksum / parity checks.
        encoder_id: Unique identifier of encoder sensor.
        encoder_type: 'motor_encoder' or 'joint_encoder'.
    """
    position_rad: float
    timestamp: Timestamp
    sequence_id: int
    velocity_rad_s: float | None = None
    valid: bool = True
    encoder_id: str = "enc_0"
    encoder_type: str = "motor_encoder"


@dataclass
class IMUReading:
    """Measurement from an Inertial Measurement Unit.

    Attributes:
        accel_m_s2: Linear acceleration [m/s²] along (x, y, z).
        gyro_rad_s: Angular velocity [rad/s] around (x, y, z).
        timestamp: Acquisition timestamp.
        valid: Hardware data validity flag.
        imu_id: Identifier of IMU sensor.
    """
    accel_m_s2: tuple[float, float, float]
    gyro_rad_s: tuple[float, float, float]
    timestamp: Timestamp
    valid: bool = True
    imu_id: str = "imu_0"


@dataclass
class SensorFrame:
    """Synchronized multi-sensor reading frame."""
    timestamp: Timestamp
    sequence_id: int
    encoders: dict[str, EncoderReading] = field(default_factory=dict)
    imu: IMUReading | None = None
    current_amp: float | None = None
    voltage_v: float | None = None
    valid: bool = True


@dataclass
class ActuatorCapabilities:
    """Declared capabilities and safety envelope of an actuator."""
    actuator_type: ActuatorType = ActuatorType.UNKNOWN
    has_position_control: bool = True
    has_velocity_control: bool = True
    has_torque_control: bool = False
    has_encoder_feedback: bool = True
    has_current_sensing: bool = False

    # Limits (None = unverified / unconfigured)
    position_min_rad: float | None = None
    position_max_rad: float | None = None
    velocity_max_rad_s: float | None = None
    acceleration_max_rad_s2: float | None = None
    limits_verified: bool = False


@dataclass
class ActuatorEvent:
    """Online event record representing an anomalous or discrete transition."""
    event_type: ActuatorState
    timestamp: Timestamp
    source: EventSource
    joint_id: str
    confidence: float | None = None
    details: dict[str, Any] = field(default_factory=dict)
    ground_truth_verified: bool = False
    ground_truth_status: ActuatorState | None = None


# Alias for compatibility
EventRecord = ActuatorEvent


@dataclass
class RiskAssessment:
    """Risk estimation evaluation."""
    risk_score: float  # [0.0 (safe) to 1.0 (imminent danger)]
    joint_id: str
    state_assessment: ActuatorState
    violates_constraints: bool
    reasons: list[str] = field(default_factory=list)
    timestamp: Timestamp | None = None


@dataclass
class ActionDecision:
    """Action Shield output containing the filtered command and metadata."""
    original_command: JointCommand
    shielded_command: JointCommand | None = None
    was_modified: bool = False
    is_rejected: bool = False
    intervention_reason: str = ""
    timestamp: Timestamp | None = None
    safety_state: SafetyState = SafetyState.SAFE


@dataclass
class ExperimentMetadata:
    """Scientific experiment metadata for reproducibility."""
    experiment_id: str
    session_id: str
    data_source: DataSourceType
    software_version: str
    hardware_config_hash: str
    random_seed: int = 42
    sample_rate_hz: float = 100.0
    description: str = ""
    created_at_utc: str = ""


@dataclass
class DatasetManifest:
    """Manifest describing a completed dataset collection session."""
    experiment_id: str
    session_id: str
    data_source: DataSourceType
    total_records: int
    duration_s: float
    data_file_path: str
    config_hash: str
    software_version: str
    sample_rate_hz_measured: float
    missing_value_count: int
    tags: list[str] = field(default_factory=list)
    split: str | None = None
