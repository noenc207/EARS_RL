"""Configuration schemas for EARS-RL based on Pydantic v2.

All physical quantities carry explicit unit documentation.
Unverified parameters (mechanical limits, gear ratios) default to None
and require explicit verification before enabling hardware mode.
"""

from __future__ import annotations

from enum import Enum
from pathlib import Path

from pydantic import BaseModel, Field, field_validator, model_validator


class UnitsEnum(str, Enum):
    """Physical SI unit labels."""
    RADIANS = "rad"
    DEGREES = "deg"
    RADIANS_PER_SECOND = "rad/s"
    RADIANS_PER_SECOND_SQUARED = "rad/s²"
    AMPERES = "A"
    VOLTS = "V"
    MILLISECONDS = "ms"
    NANOSECONDS = "ns"
    HERTZ = "Hz"
    NEWTON_METERS = "Nm"


class ActuatorConfig(BaseModel):
    """Configuration for a single joint actuator."""
    joint_id: str = Field(..., description="Unique joint identifier")
    actuator_type: str = Field(default="stepper", description="'stepper', 'servo', or 'hybrid'")

    # Limits in SI units (None = unverified from physical characterization)
    position_min_rad: float | None = Field(
        default=None, description="Minimum position limit in radians [rad]. None = unverified."
    )
    position_max_rad: float | None = Field(
        default=None, description="Maximum position limit in radians [rad]. None = unverified."
    )
    velocity_max_rad_s: float | None = Field(
        default=None, description="Maximum velocity in rad/s. None = unverified."
    )
    acceleration_max_rad_s2: float | None = Field(
        default=None, description="Maximum acceleration in rad/s². None = unverified."
    )

    # Stepper drivetrain parameters
    steps_per_revolution: int | None = Field(
        default=None, description="Full steps per revolution (e.g. 200 for 1.8° stepper)"
    )
    microstep_resolution: int | None = Field(
        default=None, description="Microstep divisor (e.g. 16 for 1/16 microstepping)"
    )
    gear_ratio: float | None = Field(
        default=None, description="Mechanical gear ratio (motor : joint output shaft)"
    )

    # Verification flag
    limits_verified: bool = Field(
        default=False, description="Whether limits have been physically verified on hardware"
    )
    enabled: bool = Field(
        default=True, description="Whether actuator is active in this configuration"
    )

    @field_validator("actuator_type")
    @classmethod
    def validate_type(cls, v: str) -> str:
        valid = {"stepper", "servo", "hybrid", "unknown"}
        if v.lower() not in valid:
            raise ValueError(f"actuator_type must be in {valid}, got {v}")
        return v.lower()

    @model_validator(mode="after")
    def validate_bounds(self) -> ActuatorConfig:
        """Enforce physical kinematic consistency across limits."""
        if self.position_min_rad is not None and self.position_max_rad is not None:
            if self.position_min_rad > self.position_max_rad:
                raise ValueError(
                    f"position_min_rad ({self.position_min_rad}) cannot be greater than "
                    f"position_max_rad ({self.position_max_rad}) for joint '{self.joint_id}'"
                )
        if self.velocity_max_rad_s is not None and self.velocity_max_rad_s < 0:
            raise ValueError(
                f"velocity_max_rad_s ({self.velocity_max_rad_s}) cannot be negative for joint '{self.joint_id}'"
            )
        if self.acceleration_max_rad_s2 is not None and self.acceleration_max_rad_s2 < 0:
            raise ValueError(
                f"acceleration_max_rad_s2 ({self.acceleration_max_rad_s2}) cannot be negative for joint '{self.joint_id}'"
            )
        return self


class EncoderConfig(BaseModel):
    """Configuration for an angular encoder."""
    encoder_id: str = Field(..., description="Unique encoder identifier")
    encoder_type: str = Field(
        default="motor_encoder", description="'motor_encoder' or 'joint_encoder'"
    )
    resolution_counts_per_rev: int | None = Field(
        default=None, description="Encoder counts / pulses per full revolution"
    )
    joint_id: str = Field(..., description="Associated joint ID")


class IMUConfig(BaseModel):
    """Configuration for an IMU sensor."""
    imu_id: str = Field(default="imu_0", description="IMU identifier")
    sample_rate_hz: float | None = Field(default=200.0, description="Sample rate [Hz]")
    has_fusion: bool = Field(
        default=False, description="Whether orientation fusion algorithm is verified"
    )


class SafetyConfig(BaseModel):
    """Safety system and interlock configuration.

    MANDATORY INVARIANTS:
    - hardware_enabled MUST default to false.
    - motion_enabled_on_startup MUST default to false.
    """
    hardware_enabled: bool = Field(
        default=False,
        description="Master hardware enable switch. MUST be False by default for safety."
    )
    motion_enabled_on_startup: bool = Field(
        default=False,
        description="Whether motion commands are allowed on startup. MUST be False."
    )
    watchdog_timeout_ms: float = Field(
        default=500.0, description="Heartbeat watchdog timeout in milliseconds [ms]"
    )
    require_limits_verified: bool = Field(
        default=True, description="Enforce limits_verified=true before hardware mode activation"
    )
    max_position_error_rad: float = Field(
        default=0.35, description="Tracking error threshold [rad] that triggers safety warning"
    )


class CommunicationConfig(BaseModel):
    """Hardware serial/UART communication configuration."""
    serial_port: str | None = Field(default=None, description="Serial device port path")
    baudrate: int = Field(default=921600, description="UART baudrate (ESP32-S3 default)")
    timeout_ms: float = Field(default=100.0, description="Read timeout in ms")


class DataConfig(BaseModel):
    """Data logging and dataset storage configuration."""
    data_dir: Path = Field(default=Path("./datasets"), description="Base datasets directory")
    experiments_dir: Path = Field(default=Path("./experiments"), description="Experiments dir")
    checkpoints_dir: Path = Field(default=Path("./checkpoints"), description="Checkpoints dir")
    reports_dir: Path = Field(default=Path("./reports"), description="Reports dir")
    log_format: str = Field(default="csv", description="'csv' or 'jsonl'")
    buffer_size: int = Field(default=100, description="Record buffer size before flush")
    flush_interval_s: float = Field(default=1.0, description="Buffer flush interval [s]")


class IdentificationConfig(BaseModel):
    """System identification configuration."""
    model_type: str = Field(
        default="nominal",
        description="'nominal', 'fixed_delay', 'residual', or 'hybrid'"
    )
    input_dataset: Path | None = Field(default=None, description="Path to input dataset file")
    output_dir: Path | None = Field(default=None, description="Results output directory")
    train_split: float = Field(default=0.70, description="Train fraction")
    val_split: float = Field(default=0.15, description="Validation fraction")
    test_split: float = Field(default=0.15, description="Test fraction")
    random_seed: int = Field(default=42, description="Random seed for reproducibility")
    delay_ms_init: float = Field(default=10.0, description="Initial latency estimate [ms]")


class SimulationConfig(BaseModel):
    """Simulation integration configuration."""
    simulation_backend: str = Field(
        default="mock", description="'mock', 'isaac_lab', or 'pybullet'"
    )
    render: bool = Field(default=False, description="Enable graphical rendering")
    physics_dt_s: float = Field(default=0.002, description="Physics step size (500 Hz)")
    control_dt_s: float = Field(default=0.02, description="Policy control step size (50 Hz)")


class RLConfig(BaseModel):
    """Reinforcement learning hyperparameter configuration."""
    algorithm: str = Field(default="ppo", description="'ppo' or 'sac'")
    seed: int = Field(default=42, description="Global random seed")
    total_timesteps: int = Field(default=1_000_000, description="Total training steps")
    learning_rate: float = Field(default=3e-4, description="Optimizer learning rate")
    batch_size: int = Field(default=64, description="Training batch size")
    gamma: float = Field(default=0.99, description="Discount factor")
    checkpoint_interval: int = Field(default=10_000, description="Steps between checkpoints")
    eval_interval: int = Field(default=5_000, description="Steps between evaluations")
    device: str = Field(default="auto", description="'cpu', 'cuda', or 'auto'")


class ExperimentConfig(BaseModel):
    """Top-level experiment configuration schema."""
    experiment_id: str = Field(..., description="Unique experiment identifier")
    description: str = Field(default="", description="Scientific experiment objective")
    safety: SafetyConfig = Field(default_factory=SafetyConfig)
    data: DataConfig = Field(default_factory=DataConfig)
    actuators: list[ActuatorConfig] = Field(default_factory=list)
    encoders: list[EncoderConfig] = Field(default_factory=list)
    imu: IMUConfig | None = Field(default=None)
    communication: CommunicationConfig = Field(default_factory=CommunicationConfig)
    identification: IdentificationConfig | None = Field(default=None)
    simulation: SimulationConfig = Field(default_factory=SimulationConfig)
    rl: RLConfig | None = Field(default=None)
    software_version: str = Field(default="0.1.0", description="Software version")
    tags: list[str] = Field(default_factory=list)
    notes: str = Field(default="", description="Experimental notes")

    @model_validator(mode="after")
    def validate_safety_and_hardware_combination(self) -> ExperimentConfig:
        """Prevent dangerous or conflicting configuration combinations from executing."""
        if self.safety.motion_enabled_on_startup:
            raise ValueError(
                "Mandatory Safety Invariant Violated: motion_enabled_on_startup must be False. "
                "Actuators must remain disabled upon initialization until explicitly armed."
            )

        if self.safety.hardware_enabled:
            if self.safety.require_limits_verified:
                unverified = [act.joint_id for act in self.actuators if not act.limits_verified]
                if unverified:
                    raise ValueError(
                        f"Hardware mode cannot be enabled with unverified actuator limits: {unverified}. "
                        "Physically verify mechanical end-stops and set limits_verified=true."
                    )
            if not self.communication.serial_port:
                raise ValueError(
                    "Hardware mode cannot be enabled without a specified serial_port in communication config."
                )
        return self


class SystemConfig(BaseModel):
    """Global system-level configuration."""
    log_level: str = Field(default="INFO", description="Console logging level")
    data_dir: Path = Field(default=Path("./datasets"), description="Default data path")
    hardware_mode: str = Field(default="disabled", description="Default hardware mode")
