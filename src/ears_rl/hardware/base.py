"""Base actuator implementation managing hardware lifecycle and safety invariants."""

from __future__ import annotations

import logging

import numpy as np

from ears_rl.core.exceptions import (
    CommandLimitError,
    HardwareDisabledError,
)
from ears_rl.core.interfaces import ActuatorInterface
from ears_rl.core.types import (
    ActuatorCapabilities,
    ActuatorState,
    ConnectionState,
    HardwareMode,
    JointCommand,
)

logger = logging.getLogger(__name__)


class BaseActuator(ActuatorInterface):
    """Base class enforcing mandatory hardware safety invariants across all actuators.

    Invariants:
    1. Default mode is HardwareMode.DISABLED.
    2. Motion is disabled upon initialization.
    3. Commands raise HardwareDisabledError if attempted when hardware mode is disabled.
    """

    def __init__(
        self,
        joint_id: str,
        capabilities: ActuatorCapabilities,
        hardware_mode: HardwareMode = HardwareMode.DISABLED,
    ) -> None:
        self.joint_id = joint_id
        self._capabilities = capabilities
        self._hardware_mode = hardware_mode
        self._connection_state = ConnectionState.DISCONNECTED
        self._is_motion_enabled = False
        self._state = ActuatorState.NORMAL

        logger.debug(
            "Initialized actuator '%s' in mode %s (motion_enabled=%s)",
            joint_id, hardware_mode, self._is_motion_enabled
        )

    @property
    def hardware_mode(self) -> HardwareMode:
        return self._hardware_mode

    @property
    def is_motion_enabled(self) -> bool:
        return self._is_motion_enabled

    @property
    def connection_state(self) -> ConnectionState:
        return self._connection_state

    def get_capabilities(self) -> ActuatorCapabilities:
        return self._capabilities

    def get_connection_state(self) -> ConnectionState:
        return self._connection_state

    def get_hardware_mode(self) -> HardwareMode:
        return self._hardware_mode

    def get_state(self) -> ActuatorState:
        return self._state

    def enable_motion(self) -> bool:
        """Explicitly enable motion. Requires active connection and verified limits if in hardware mode."""
        if self._connection_state != ConnectionState.CONNECTED:
            logger.error("Cannot enable motion on '%s': Not connected", self.joint_id)
            return False

        if self._hardware_mode == HardwareMode.HARDWARE and not self._capabilities.limits_verified:
            logger.error(
                "Cannot enable physical hardware motion on '%s': Limits unverified!", self.joint_id
            )
            return False

        self._is_motion_enabled = True
        logger.info("Motion enabled for actuator '%s'", self.joint_id)
        return True

    def disable_motion(self) -> None:
        """Immediately disable motion."""
        self._is_motion_enabled = False
        logger.info("Motion disabled for actuator '%s'", self.joint_id)

    def _pre_command_check(self, command: JointCommand | None) -> None:
        """Safety checks executed before every motion command."""
        if command is None:
            raise CommandLimitError("Motion command is None (suppressed/rejected by safety shield).")

        if self._hardware_mode == HardwareMode.DISABLED:
            raise HardwareDisabledError(
                f"Motion command rejected on '{self.joint_id}': Hardware mode is DISABLED"
            )

        if not self._is_motion_enabled:
            raise HardwareDisabledError(
                f"Motion command rejected on '{self.joint_id}': Motion is not enabled. Call enable_motion() first."
            )

        if self._connection_state != ConnectionState.CONNECTED:
            raise HardwareDisabledError(
                f"Motion command rejected on '{self.joint_id}': Connection state is {self._connection_state}"
            )

        # Invariant 4: Reject non-finite values (NaN / Inf) immediately
        if command.position_rad is not None:
            if not np.isfinite(command.position_rad):
                raise CommandLimitError(
                    f"Position command is non-finite (NaN or Inf): {command.position_rad}"
                )
            min_pos = self._capabilities.position_min_rad
            max_pos = self._capabilities.position_max_rad
            if min_pos is not None and command.position_rad < min_pos:
                raise CommandLimitError(
                    f"Position command {command.position_rad:.4f} rad < min limit {min_pos:.4f} rad"
                )
            if max_pos is not None and command.position_rad > max_pos:
                raise CommandLimitError(
                    f"Position command {command.position_rad:.4f} rad > max limit {max_pos:.4f} rad"
                )

        # Validate velocity limits if velocity is specified
        if command.velocity_rad_s is not None:
            if not np.isfinite(command.velocity_rad_s):
                raise CommandLimitError(
                    f"Velocity command is non-finite (NaN or Inf): {command.velocity_rad_s}"
                )
            max_vel = self._capabilities.velocity_max_rad_s
            if max_vel is not None and abs(command.velocity_rad_s) > max_vel:
                raise CommandLimitError(
                    f"Velocity command |{command.velocity_rad_s:.4f}| rad/s > max limit {max_vel:.4f} rad/s"
                )

        if command.acceleration_rad_s2 is not None:
            if not np.isfinite(command.acceleration_rad_s2):
                raise CommandLimitError(
                    f"Acceleration command is non-finite (NaN or Inf): {command.acceleration_rad_s2}"
                )
