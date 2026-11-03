"""EARS-RL Exception Hierarchy.

All framework exceptions inherit from EarsRLError to allow unified error
handling, fault isolation, and safe hardware disconnection.
"""

from __future__ import annotations


class EarsRLError(Exception):
    """Base exception for all EARS-RL errors."""


# --- Configuration & Schema Errors ---

class ConfigurationError(EarsRLError):
    """Raised when configuration is invalid, incomplete, or unverified."""


class SchemaValidationError(ConfigurationError):
    """Raised when data or config fails Pydantic schema validation."""


# --- Hardware & Communication Errors ---

class HardwareError(EarsRLError):
    """Base exception for hardware communication and driver errors."""


class ConnectionError(HardwareError):
    """Raised when connection to physical or mock hardware fails."""


class HardwareDisabledError(HardwareError):
    """Raised when an operation requires hardware mode, but hardware is disabled.

    MANDATORY INVARIANT: This must be raised whenever a motion command is
    attempted while hardware_enabled is false.
    """


class WatchdogTimeoutError(HardwareError):
    """Raised when communication heartbeat expires."""


# --- Safety & Interlock Errors ---

class SafetyError(EarsRLError):
    """Base exception for safety constraint violations."""


class CommandLimitError(SafetyError):
    """Raised when a motion command violates physical or configured bounds."""


class SafetyInterlockError(SafetyError):
    """Raised when a software interlock blocks operation.

    NOTE: Software interlocks are complementary and NEVER replace a physical E-stop.
    """


class EStopActiveError(SafetyError):
    """Raised when hardware E-stop signal is asserted."""


# --- Data & Validation Errors ---

class DataError(EarsRLError):
    """Base exception for dataset, serialization, and stream errors."""


class DataSchemaError(DataError):
    """Raised when a dataset record fails schema constraints."""


class TimestampError(DataError):
    """Raised when timestamps violate monotonicity or clock consistency."""


# --- Actuator Model Errors ---

class ActuatorModelError(EarsRLError):
    """Base exception for system identification and modeling errors."""


class ModelNotFittedError(ActuatorModelError):
    """Raised when prediction is requested on an unfitted model."""


# --- Simulation Errors ---

class SimulationError(EarsRLError):
    """Base exception for simulation environments."""


class IsaacLabNotAvailableError(SimulationError):
    """Raised when Isaac Lab adapter is called without Isaac Lab installed.

    Provides actionable instructions to the user on how to install or activate.
    """


# --- Reinforcement Learning Errors ---

class RLError(EarsRLError):
    """Base exception for RL environment, policy, and training pipelines."""


class CheckpointError(RLError):
    """Raised when loading or saving a policy checkpoint fails."""
