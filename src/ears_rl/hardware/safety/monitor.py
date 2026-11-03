"""Safety monitoring, watchdog tracking, and software interlocks for EARS-RL."""

from __future__ import annotations

import logging

from ears_rl.config.schema import SafetyConfig
from ears_rl.core.clock import TimeProvider
from ears_rl.core.interfaces import SafetyInterface
from ears_rl.core.types import JointCommand, SafetyState, SafetyStatus

logger = logging.getLogger(__name__)


class SafetyMonitor(SafetyInterface):
    """Software safety guardrail monitoring interlocks, bounds, and watchdogs.

    WARNING:
    Software safety interlocks are a secondary defense layer and NEVER
    a substitute for a physical, hardwired E-stop circuit.
    """

    def __init__(
        self,
        config: SafetyConfig,
        clock: TimeProvider,
        position_limits: tuple[float, float] | None = None,
        velocity_limit: float | None = None,
    ) -> None:
        self.config = config
        self._clock = clock
        self._position_limits = position_limits
        self._velocity_limit = velocity_limit

        self._state = SafetyState.SAFE
        self._estop_tripped = False
        self._fault_reason: str = ""
        self._last_heartbeat_ns = self._clock.now_ns()

    def feed_watchdog(self) -> None:
        """Feed heartbeat timer.

        NOTE: Feeding watchdog refreshes the timer but does NOT clear an existing
        latched FAULT. Latched faults require an explicit reset_fault() call.
        """
        self._last_heartbeat_ns = self._clock.now_ns()

    def get_safety_state(self) -> SafetyState:
        # Check watchdog expiration
        elapsed_ms = (self._clock.now_ns() - self._last_heartbeat_ns) / 1e6
        if elapsed_ms > self.config.watchdog_timeout_ms:
            self._state = SafetyState.FAULT
            if not self._fault_reason:
                self._fault_reason = (
                    f"Fault: Watchdog timeout ({elapsed_ms:.1f}ms exceeds limit {self.config.watchdog_timeout_ms:.1f}ms)"
                )

        if self._estop_tripped:
            return SafetyState.ESTOP

        return self._state

    def get_fault_reason(self) -> str:
        """Return human-readable diagnostic reason for current fault/estop state."""
        self.get_safety_state()  # refresh watchdog
        return self._fault_reason

    def get_safety_status(self) -> SafetyStatus:
        """Return comprehensive safety status across software, electrical, and mechanical domains.

        CRITICAL SAFETY PRINCIPLE:
        Software FAULT (is_command_blocked=True) does NOT mean motor power is cut (is_power_cut=False)
        or that the physical shaft is mechanically stationary (is_mechanically_stopped=False).
        """
        state = self.get_safety_state()
        is_blocked = state in (SafetyState.FAULT, SafetyState.ESTOP)
        return SafetyStatus(
            safety_state=state,
            is_command_blocked=is_blocked,
            is_pulse_suppressed=is_blocked,
            is_driver_disabled=is_blocked,
            is_power_cut=self._estop_tripped,  # Only physical E-stop cuts motor supply power!
            is_mechanically_stopped=False,      # Software cannot guarantee mechanical standstill without sensor verification
            fault_reason=self._fault_reason,
            timestamp_ns=self._clock.now_ns(),
        )

    def check_command_allowed(self, command: JointCommand) -> tuple[bool, str]:
        current_state = self.get_safety_state()

        if current_state in (SafetyState.ESTOP, SafetyState.FAULT):
            reason = self._fault_reason or f"Blocked: System is in {current_state.value} state"
            return False, reason

        # Check position bounds
        if command.position_rad is not None and self._position_limits is not None:
            min_pos, max_pos = self._position_limits
            if not (min_pos <= command.position_rad <= max_pos):
                return False, f"Position {command.position_rad:.4f} rad exceeds [{min_pos:.4f}, {max_pos:.4f}]"

        # Check velocity bounds
        if command.velocity_rad_s is not None and self._velocity_limit is not None:
            if abs(command.velocity_rad_s) > self._velocity_limit:
                return False, f"Velocity {command.velocity_rad_s:.4f} rad/s exceeds limit {self._velocity_limit:.4f}"

        return True, ""

    def validate_command(self, command: JointCommand) -> tuple[bool, str]:
        return self.check_command_allowed(command)

    def is_estop_active(self) -> bool:
        return self._estop_tripped

    def trigger_estop(self) -> None:
        """Simulate hardware E-stop trip (for safety test verification)."""
        self._estop_tripped = True
        self._state = SafetyState.ESTOP
        self._fault_reason = "Physical E-stop signal asserted (motor power cut)"
        logger.critical("E-STOP SIGNAL ASSERTED!")

    def reset_fault(self) -> bool:
        """Reset fault state. E-stop cannot be cleared via software."""
        if self._estop_tripped:
            logger.warning("Cannot reset fault: Hardware E-stop signal is still asserted!")
            return False

        # Resetting fault requires watchdog to be currently alive
        now_ns = self._clock.now_ns()
        elapsed_ms = (now_ns - self._last_heartbeat_ns) / 1e6
        if elapsed_ms > self.config.watchdog_timeout_ms:
            logger.warning("Cannot reset fault: Watchdog timer is still expired (elapsed: %.1fms)!", elapsed_ms)
            return False

        self._state = SafetyState.SAFE
        self._fault_reason = ""
        logger.info("Safety fault cleared.")
        return True

    def get_watchdog_status(self) -> tuple[bool, float]:
        elapsed_ms = (self._clock.now_ns() - self._last_heartbeat_ns) / 1e6
        is_alive = elapsed_ms <= self.config.watchdog_timeout_ms
        return is_alive, elapsed_ms
