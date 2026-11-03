"""Data contract and schema definitions for EARS-RL bench tests.

All records strictly enforce:
1. Explicit SI units in field descriptions.
2. Nullable fields for non-existent sensors (never silently filled with 0.0).
3. Clear distinction between:
   - Sample period vs pure delay vs command-to-feedback latency.
   - Model estimates vs detector flags vs ground truth.
   - Synthetic vs simulated vs real hardware data.
4. Strict provenance tracking: 'synthetic', 'simulated', or 'real_hardware'.
"""

from __future__ import annotations

from pydantic import BaseModel, Field

# Schema field contract classifications for validation and scientific reporting
REQUIRED_FIELDS: set[str] = {
    "experiment_id",
    "session_id",
    "sequence_id",
    "timestamp_ns",
    "joint_id",
    "data_source",
}

EXPECTED_TELEMETRY_FIELDS: set[str] = {
    "measured_motor_position_rad",
    "command_position_rad",
}

MODE_DEPENDENT_OPTIONAL_FIELDS: dict[str, str] = {
    "wall_time_utc": "Wall clock time (UTC). Optional; omitted in deterministic/real-time mock loops.",
    "command_velocity_rad_s": "Commanded target velocity. Optional; only present in velocity or impedance control modes.",
    "command_acceleration_rad_s2": "Commanded feed-forward acceleration. Optional; only present in acceleration/torque control modes.",
    "measured_joint_position_rad": "Joint output-side position from second encoder. Optional; only present in dual-encoder test rigs.",
    "measured_velocity_rad_s": "Measured joint velocity. Optional; computed or read from encoder rate estimator.",
    "current_amp": "Phase current consumption. Optional; requires current shunt / TMC current sensing.",
    "voltage_v": "DC bus voltage. Optional; requires power bus telemetry.",
    "estimated_load_nm": "Estimated external joint load. Optional; requires load cell or observer.",
    "command_sent_timestamp_ns": "Host dispatch timestamp. Optional; requires paired command tracking.",
    "feedback_received_timestamp_ns": "Host feedback reception timestamp. Optional; requires paired feedback tracking.",
    "sample_period_ms": "Consecutive feedback sampling interval [ms]. Derived from timestamp delta.",
    "command_to_feedback_delay_ms": "Measured round-trip command-to-feedback latency [ms]. Only valid when paired timestamps exist in identical clock domain.",
    "configured_simulated_delay_ms": "Configured pure transport delay [ms] in simulation / mock actuator. Null on real hardware.",
    "latency_ms": "Legacy/general latency field [ms]; maps to command_to_feedback_delay_ms or configured_simulated_delay_ms.",
    "event_label": "Online detector/model classification. Optional; populated when detector active.",
    "event_source": "Source of event label. Optional; 'model_estimate', 'detector', 'ground_truth', 'synthetic_injection'.",
    "ground_truth_status": "Independently verified ground-truth event state. Null unless verified ground truth exists.",
    "hardware_config_hash": "SHA-256 hash of experiment hardware configuration.",
    "software_version": "Software version string.",
}


class BenchRecord(BaseModel):
    """Single timestep recording from an actuator test bench.

    Units:
    - Positions: radians [rad]
    - Velocities: radians/second [rad/s]
    - Accelerations: radians/second² [rad/s²]
    - Current: Amperes [A]
    - Voltage: Volts [V]
    - Torques: Newton-meters [Nm]
    - Timestamps: nanoseconds [ns]
    - Latency / Periods: milliseconds [ms]
    """
    # Identifiers (Mandatory)
    experiment_id: str = Field(..., description="Experiment ID")
    session_id: str = Field(..., description="Session ID within experiment")
    sequence_id: int = Field(..., description="Monotonically increasing sequence number")

    # Timing (Mandatory Monotonic Timestamp)
    timestamp_ns: int = Field(..., description="Monotonic acquisition timestamp [ns]")
    wall_time_utc: str | None = Field(
        default=None,
        description="ISO-8601 wall clock timestamp (Optional; omitted in deterministic mock loops)"
    )

    # Joint ID
    joint_id: str = Field(default="joint_0", description="Actuator / joint identifier")

    # Commands (Position required in position mode; velocity/acceleration optional)
    command_position_rad: float | None = Field(default=None, description="Target position [rad]")
    command_velocity_rad_s: float | None = Field(
        default=None, description="Target velocity [rad/s] (Optional; mode-dependent)"
    )
    command_acceleration_rad_s2: float | None = Field(
        default=None, description="Target acceleration [rad/s²] (Optional; mode-dependent)"
    )

    # Feedback / Measurements
    measured_motor_position_rad: float | None = Field(
        default=None, description="Motor-side shaft encoder position [rad]"
    )
    measured_joint_position_rad: float | None = Field(
        default=None, description="Joint output-side encoder position [rad] (Optional; dual-encoder only)"
    )
    measured_velocity_rad_s: float | None = Field(
        default=None, description="Measured joint velocity [rad/s] (Optional)"
    )

    # Electrical & Load Telemetry
    current_amp: float | None = Field(default=None, description="Motor phase current [A] (Optional)")
    voltage_v: float | None = Field(default=None, description="DC bus voltage [V] (Optional)")
    estimated_load_nm: float | None = Field(default=None, description="Estimated external load [Nm] (Optional)")

    # Latency & Communication (Strictly distinguished semantics)
    command_sent_timestamp_ns: int | None = Field(
        default=None, description="Monotonic timestamp when command was sent by host [ns]"
    )
    feedback_received_timestamp_ns: int | None = Field(
        default=None, description="Monotonic timestamp when feedback was received by host [ns]"
    )
    sample_period_ms: float | None = Field(
        default=None, description="Elapsed time between consecutive telemetry samples [ms]"
    )
    command_to_feedback_delay_ms: float | None = Field(
        default=None,
        description="Measured round-trip command-to-feedback latency in same clock domain [ms]; null if unmeasured"
    )
    configured_simulated_delay_ms: float | None = Field(
        default=None,
        description="Simulated pure transport delay configured in mock/sim actuator [ms]; null on real hardware"
    )
    latency_ms: float | None = Field(
        default=None,
        description="Convenience latency field [ms]; mirrors command_to_feedback_delay_ms or simulated delay"
    )

    # Actuator State & Events
    actuator_mode: str = Field(default="mock", description="Operating mode ('mock', 'hardware', 'fault')")
    event_label: str | None = Field(
        default=None, description="Model/detector event label ('normal', 'step_loss', 'stall', 'saturated')"
    )
    event_source: str | None = Field(
        default=None, description="'model_estimate', 'detector', 'ground_truth', 'synthetic_injection'"
    )
    ground_truth_status: str | None = Field(
        default=None, description="Independently verified ground truth state (null unless verified)"
    )

    # Provenance & Data Quality
    data_quality_flags: list[str] = Field(default_factory=list, description="Quality tags ('valid', 'synthetic')")
    data_source: str = Field(
        default="synthetic", description="MANDATORY: 'synthetic', 'simulated', or 'real_hardware'"
    )
    software_version: str | None = Field(default="0.1.0", description="EARS-RL software version")
    hardware_config_hash: str | None = Field(default=None, description="SHA-256 config hash")


class DatasetMetadata(BaseModel):
    """Metadata describing a recorded dataset file."""
    experiment_id: str
    session_id: str
    description: str = ""
    data_source: str = Field(..., description="'synthetic', 'simulated', or 'real_hardware'")
    sample_rate_hz_expected: float = 100.0
    sample_rate_hz_measured: float | None = None
    total_records: int = 0
    total_duration_s: float | None = None
    sensors_used: list[str] = Field(default_factory=list)
    firmware_version: str | None = None
    hardware_config_hash: str | None = None
    software_version: str = "0.1.0"
    random_seed: int | None = None
    quality_notes: str = ""
    units: dict[str, str] = Field(
        default_factory=lambda: {
            "position": "rad",
            "velocity": "rad/s",
            "acceleration": "rad/s²",
            "current": "A",
            "voltage": "V",
            "sample_period": "ms",
            "latency": "ms",
            "timestamp": "ns",
            "torque": "Nm",
        }
    )
