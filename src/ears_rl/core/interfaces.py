"""Core abstract interfaces for the EARS-RL research architecture.

These interfaces define the contracts that concrete implementations must satisfy.
They decouple hardware drivers, simulation environments, safety layers,
system identification models, and RL control algorithms.
"""

from __future__ import annotations

import abc
from typing import Any

from ears_rl.core.clock import TimeProvider
from ears_rl.core.types import (
    ActionDecision,
    ActuatorCapabilities,
    ActuatorEvent,
    ActuatorState,
    ConnectionState,
    EncoderReading,
    HardwareMode,
    IMUReading,
    JointCommand,
    JointPosition,
    JointVelocity,
    RiskAssessment,
    SafetyState,
    SensorFrame,
)

# Canonical Aliases for Clean Architecture Roles
ClockInterface = TimeProvider
Clock = TimeProvider


class ActuatorInterface(abc.ABC):
    """Abstract interface for all actuators (stepper, servo, mock).

    MANDATORY INVARIANTS:
    1. Must default to disabled mode on instantiation.
    2. Must NOT emit motion commands during initialization or import.
    3. Motion commands must raise HardwareDisabledError if hardware mode is disabled.
    """

    @abc.abstractmethod
    def connect(self) -> None:
        """Establish connection to actuator interface. Does NOT enable motion."""
        ...

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Safely disconnect and place actuator into safe quiescent state."""
        ...

    @abc.abstractmethod
    def get_state(self) -> ActuatorState:
        """Return current estimated actuator operating state."""
        ...

    @abc.abstractmethod
    def get_position(self) -> JointPosition:
        """Return current shaft or joint position reading."""
        ...

    @abc.abstractmethod
    def get_velocity(self) -> JointVelocity | None:
        """Return current shaft or joint velocity reading, if available."""
        ...

    @abc.abstractmethod
    def send_position_command(self, command: JointCommand) -> bool:
        """Send target position command.

        Returns True if command was accepted; raises CommandLimitError or
        HardwareDisabledError if invalid or disabled.
        """
        ...

    @abc.abstractmethod
    def send_velocity_command(self, command: JointCommand) -> bool:
        """Send target velocity command.

        Returns True if command was accepted; raises CommandLimitError or
        HardwareDisabledError if invalid or disabled.
        """
        ...

    @abc.abstractmethod
    def get_capabilities(self) -> ActuatorCapabilities:
        """Return declared capabilities and configured limits."""
        ...

    @abc.abstractmethod
    def get_connection_state(self) -> ConnectionState:
        """Return hardware connection status."""
        ...

    @abc.abstractmethod
    def get_hardware_mode(self) -> HardwareMode:
        """Return current operating mode (MOCK, SIMULATION, HARDWARE, DISABLED)."""
        ...

    @abc.abstractmethod
    def enable_motion(self) -> bool:
        """Explicitly enable excitation and motion commands after safety checks."""
        ...

    @abc.abstractmethod
    def disable_motion(self) -> None:
        """Immediately disable motion commands and motor excitation."""
        ...


ActuatorBackend = ActuatorInterface


class EncoderInterface(abc.ABC):
    """Abstract interface for angular position encoders."""

    @abc.abstractmethod
    def read(self) -> EncoderReading:
        """Read latest encoder frame with timestamp and sequence ID."""
        ...

    @abc.abstractmethod
    def read_position(self) -> float:
        """Read angular position in radians [rad]."""
        ...

    @abc.abstractmethod
    def read_status(self) -> bool:
        """Return health/validity status of the encoder."""
        ...

    @abc.abstractmethod
    def get_position_rad(self) -> float:
        """Convenience method returning position in radians [rad]."""
        ...

    @abc.abstractmethod
    def get_encoder_type(self) -> str:
        """Return 'motor_encoder' or 'joint_encoder'."""
        ...

    @abc.abstractmethod
    def reset(self) -> None:
        """Zero / re-home encoder position reference."""
        ...


class IMUInterface(abc.ABC):
    """Abstract interface for Inertial Measurement Units."""

    @abc.abstractmethod
    def read(self) -> IMUReading:
        """Read current raw accelerometer and gyroscope measurements."""
        ...

    @abc.abstractmethod
    def read_measurement(self) -> IMUReading:
        """Alias for read()."""
        ...

    @abc.abstractmethod
    def get_angular_velocity(self) -> tuple[float, float, float]:
        """Return angular velocity (wx, wy, wz) in rad/s."""
        ...

    @abc.abstractmethod
    def get_acceleration(self) -> tuple[float, float, float]:
        """Return linear acceleration (ax, ay, az) in m/s²."""
        ...


class DataLoggerInterface(abc.ABC):
    """Abstract interface for structured experiment data logging."""

    @abc.abstractmethod
    def start_session(self, session_id: str, metadata: dict[str, Any]) -> None:
        """Initialize and open a data logging session."""
        ...

    @abc.abstractmethod
    def append(self, record: dict[str, Any]) -> None:
        """Append a data record to the logging buffer."""
        ...

    @abc.abstractmethod
    def log_record(self, record: dict[str, Any]) -> None:
        """Alias for append()."""
        ...

    @abc.abstractmethod
    def flush(self) -> None:
        """Flush memory buffer to persistent storage (CSV / JSONL / Parquet)."""
        ...

    @abc.abstractmethod
    def close(self) -> None:
        """Flush buffer and safely close underlying storage resources."""
        ...

    @abc.abstractmethod
    def end_session(self) -> None:
        """Alias for close()."""
        ...

    @abc.abstractmethod
    def get_record_count(self) -> int:
        """Return total records logged in current session."""
        ...


class SafetyInterface(abc.ABC):
    """Abstract interface for safety monitoring and command validation."""

    @abc.abstractmethod
    def get_safety_state(self) -> SafetyState:
        """Return current system safety state."""
        ...

    @abc.abstractmethod
    def check_command_allowed(self, command: JointCommand) -> tuple[bool, str]:
        """Validate whether a command meets safety bounds and interlocks."""
        ...

    @abc.abstractmethod
    def validate_command(self, command: JointCommand) -> tuple[bool, str]:
        """Alias for check_command_allowed()."""
        ...

    @abc.abstractmethod
    def is_estop_active(self) -> bool:
        """Read hardware E-stop line status. Software interlock != E-stop."""
        ...

    @abc.abstractmethod
    def reset_fault(self) -> bool:
        """Attempt to clear a latching fault condition."""
        ...

    @abc.abstractmethod
    def get_watchdog_status(self) -> tuple[bool, float]:
        """Return (is_alive, elapsed_ms_since_last_heartbeat)."""
        ...


class SimulationEnvironmentInterface(abc.ABC):
    """Contract for RL simulation environments (gym/gymnasium compliant)."""

    @abc.abstractmethod
    def reset(self, seed: int | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
        """Reset simulation environment and return initial observation."""
        ...

    @abc.abstractmethod
    def step(self, action: Any) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        """Execute one environment step (obs, reward, terminated, truncated, info)."""
        ...

    @abc.abstractmethod
    def get_observation_space(self) -> dict[str, Any]:
        """Return observation space specification."""
        ...

    @abc.abstractmethod
    def get_action_space(self) -> dict[str, Any]:
        """Return action space specification."""
        ...

    @abc.abstractmethod
    def close(self) -> None:
        """Clean up simulator processes and GPU contexts."""
        ...


class ConstraintValidator(abc.ABC):
    """Interface for kinematic and dynamic constraint checking."""

    @abc.abstractmethod
    def validate(self, command: JointCommand) -> tuple[bool, list[str]]:
        """Return (is_valid, list_of_violation_reasons)."""
        ...


class ActionShield(abc.ABC):
    """Interface for real-time action modification and safety clamping."""

    @abc.abstractmethod
    def shield(self, command: JointCommand) -> ActionDecision:
        """Filter command to guarantee boundary and safety compliance."""
        ...

    @abc.abstractmethod
    def is_action_safe(self, command: JointCommand) -> bool:
        """Check if command would pass through without alteration."""
        ...


class RiskEstimator(abc.ABC):
    """Interface for evaluating risk given state and proposed action."""

    @abc.abstractmethod
    def estimate_risk(
        self, state: dict[str, Any], action: JointCommand | None = None
    ) -> RiskAssessment:
        """Estimate operational risk score in [0.0, 1.0]."""
        ...


class EventDetector(abc.ABC):
    """Interface for online event classification."""

    @abc.abstractmethod
    def detect_events(
        self, current_frame: SensorFrame, history: list[SensorFrame] | None = None
    ) -> list[ActuatorEvent]:
        """Detect anomalies, step loss, stalls, and saturation."""
        ...


class ActuatorModelInterface(abc.ABC):
    """Interface for actuator response models used in sim-to-real & sysID."""

    @abc.abstractmethod
    def predict(
        self, command: JointCommand, current_state: dict[str, Any]
    ) -> dict[str, Any]:
        """Predict actuator response (position, velocity, delay)."""
        ...

    @abc.abstractmethod
    def get_model_type(self) -> str:
        """Return model identifier (nominal, fixed_delay, hybrid)."""
        ...

    @abc.abstractmethod
    def is_fitted(self) -> bool:
        """Return whether model parameters have been identified."""
        ...

    @abc.abstractmethod
    def fit(self, dataset: Any) -> dict[str, Any]:
        """Fit model parameters to empirical bench data.

        Returns:
            Dictionary containing model fitting results, identified parameters,
            residual metrics, and training status.
        """
        ...


# Aliases for Clean Architecture Roles
ActuatorModel = ActuatorModelInterface
SafetyController = SafetyInterface


class CommandTransport(abc.ABC):
    """Abstract communication transport contract for binary protocol framing.

    Purpose: Decouples binary framed serial transport from high-level application workflows.
    Input/Output: Encoded bytes and protocol message types.
    Error States: ConnectionError, TimeoutError, HardwareError.
    Side Effects: Physical or virtual serial read/write; does NOT command motion directly.
    Hardware Motion: Strictly prohibited unless explicitly armed through protocol handshake.
    Mocking: MockBenchTransceiver in tests.
    """

    @property
    @abc.abstractmethod
    def is_connected(self) -> bool:
        """Check whether the transport link is active."""
        ...

    @abc.abstractmethod
    def connect(self) -> None:
        """Open physical or virtual transport connection."""
        ...

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Safely close transport and release all bus resources."""
        ...

    @abc.abstractmethod
    def send_frame(self, msg_type: Any, sequence_id: int, payload: bytes = b"") -> None:
        """Transmit binary framed packet over transport."""
        ...

    @abc.abstractmethod
    def receive_frame(self, timeout_s: float | None = None) -> tuple[Any, int, bytes]:
        """Receive and reassemble next complete framed packet."""
        ...


class TelemetrySource(abc.ABC):
    """Abstract telemetry streaming contract.

    Purpose: Provides uniform access to real-time or recorded sensor frames.
    Input/Output: Returns synchronized SensorFrame or None on end-of-stream.
    Units: Positions [rad], Velocities [rad/s], Currents [A], Voltages [V], Timestamps [ns].
    Error States: DataError, ConnectionError.
    Side Effects: Read-only sensor interrogation; zero motion.
    Mocking: MockActuator or pre-recorded CSV generator in tests.
    """

    @abc.abstractmethod
    def read_frame(self) -> SensorFrame | None:
        """Read the latest synchronized telemetry frame."""
        ...

    @abc.abstractmethod
    def get_sample_rate_hz(self) -> float:
        """Return expected or nominal telemetry sample rate in Hertz [Hz]."""
        ...


class DatasetStore(abc.ABC):
    """Abstract storage contract for experiment datasets and manifests.

    Purpose: Decouples dataset persistence (CSV, JSONL, Parquet) from business logic.
    Input/Output: Serialized BenchRecord dictionaries and DatasetMetadata.
    Units: All records carry SI units according to BenchRecord schema contract.
    Error States: DataError, FileNotFoundError, PermissionError.
    Side Effects: File I/O within workspace dataset directory; zero hardware interaction.
    Mocking: tmp_path in-memory or temporary folder fixture in tests.
    """

    @abc.abstractmethod
    def save_records(
        self,
        session_id: str,
        records: list[dict[str, Any]],
        metadata: dict[str, Any],
    ) -> str:
        """Persist records and metadata to storage, returning primary data file path."""
        ...

    @abc.abstractmethod
    def load_records(
        self, path: str
    ) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        """Load records and associated metadata from storage."""
        ...


class ExperimentRunner(abc.ABC):
    """Abstract contract for reproducible experiment execution workflows.

    Purpose: Orchestrates application use cases with complete lifecycle and provenance tracking.
    Input/Output: Receives ExperimentConfig, outputs structured result dictionary.
    Error States: ConfigurationError, HardwareError, SafetyError, DataError.
    Side Effects: Executes session, streams telemetry, flushes logs.
    Hardware Motion: Requires hardware_enabled=true and explicit arming; otherwise mock mode.
    Mocking: MockActuatorBench in test suite.
    """

    @abc.abstractmethod
    def run(self, config: Any) -> dict[str, Any]:
        """Execute experiment session according to configuration."""
        ...


class MetricsEvaluator(abc.ABC):
    """Abstract contract for model evaluation and tracking error quantification.

    Purpose: Computes standardized scientific metrics (MAE, RMSE, delays) without side effects.
    Input/Output: Receives records/datasets, outputs metric dictionaries.
    Units: Position [rad], Velocity [rad/s], Delay [ms], Sample Period [ms].
    Error States: DataError.
    Side Effects: None (pure computational evaluation).
    Mocking: Synthetic numpy arrays in unit tests.
    """

    @abc.abstractmethod
    def evaluate(self, dataset: Any) -> dict[str, Any]:
        """Evaluate tracking and latency metrics across dataset."""
        ...

