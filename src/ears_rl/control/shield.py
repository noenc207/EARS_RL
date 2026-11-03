"""Action Shield for online safety clamping and command constraint enforcement."""

from __future__ import annotations

import logging

import numpy as np

from ears_rl.core.interfaces import ActionShield, ConstraintValidator
from ears_rl.core.types import (
    ActionDecision,
    JointCommand,
    SafetyState,
)

logger = logging.getLogger(__name__)


class KinematicConstraintValidator(ConstraintValidator):
    """Checks commands against configured position, velocity, and acceleration limits."""

    def __init__(
        self,
        position_limits_rad: tuple[float, float] = (-3.14159, 3.14159),
        velocity_limit_rad_s: float = 6.28,
        acceleration_limit_rad_s2: float = 30.0,
    ) -> None:
        self.min_pos, self.max_pos = position_limits_rad
        self.max_vel = velocity_limit_rad_s
        self.max_accel = acceleration_limit_rad_s2

    def validate(self, command: JointCommand) -> tuple[bool, list[str]]:
        violations: list[str] = []

        if command.position_rad is not None:
            if not np.isfinite(command.position_rad):
                violations.append(f"Position command is non-finite (NaN or Inf): {command.position_rad}")
            elif not (self.min_pos <= command.position_rad <= self.max_pos):
                violations.append(
                    f"Position command {command.position_rad:.4f} rad exceeds [{self.min_pos:.4f}, {self.max_pos:.4f}]"
                )

        if command.velocity_rad_s is not None:
            if not np.isfinite(command.velocity_rad_s):
                violations.append(f"Velocity command is non-finite (NaN or Inf): {command.velocity_rad_s}")
            elif abs(command.velocity_rad_s) > self.max_vel:
                violations.append(
                    f"Velocity command |{command.velocity_rad_s:.4f}| rad/s exceeds limit {self.max_vel:.4f}"
                )

        return len(violations) == 0, violations


class BasicActionShield(ActionShield):
    """Filters, clamps, or blocks commands exceeding physical limits.

    Records intervention reason, original command, and shielded command for scientific ablation studies.
    """

    def __init__(
        self,
        validator: ConstraintValidator | None = None,
        position_limits_rad: tuple[float, float] = (-3.14159, 3.14159),
        velocity_limit_rad_s: float = 6.28,
        safe_neutral_position_rad: float | None = None,
        out_of_bounds_policy: str = "clamp",
    ) -> None:
        if out_of_bounds_policy not in ("clamp", "reject"):
            raise ValueError(
                f"Invalid out_of_bounds_policy '{out_of_bounds_policy}'. Must be 'clamp' or 'reject'."
            )
        self.validator = validator or KinematicConstraintValidator(
            position_limits_rad=position_limits_rad,
            velocity_limit_rad_s=velocity_limit_rad_s,
        )
        self.min_pos, self.max_pos = position_limits_rad
        self.max_vel = velocity_limit_rad_s
        self.out_of_bounds_policy = out_of_bounds_policy
        self.safe_neutral_pos = (
            safe_neutral_position_rad
            if safe_neutral_position_rad is not None
            else float(np.clip(0.0, self.min_pos, self.max_pos))
        )

    def is_action_safe(self, command: JointCommand) -> bool:
        safe, _ = self.validator.validate(command)
        return safe

    def shield(self, command: JointCommand) -> ActionDecision:
        safe, violations = self.validator.validate(command)

        if safe:
            return ActionDecision(
                original_command=command,
                shielded_command=command,
                was_modified=False,
                is_rejected=False,
                safety_state=SafetyState.SAFE,
            )

        # Check for non-finite values (NaN / Inf)
        has_non_finite = False
        if command.position_rad is not None and not np.isfinite(command.position_rad):
            has_non_finite = True
        if command.velocity_rad_s is not None and not np.isfinite(command.velocity_rad_s):
            has_non_finite = True
        if command.acceleration_rad_s2 is not None and not np.isfinite(command.acceleration_rad_s2):
            has_non_finite = True

        if has_non_finite:
            logger.error(
                "ActionShield intercepted non-finite command (NaN/Inf)! Strictly rejecting without motion."
            )
            return ActionDecision(
                original_command=command,
                shielded_command=None,
                was_modified=True,
                is_rejected=True,
                intervention_reason="Non-finite command (NaN/Inf) strictly rejected; motion suppressed.",
                safety_state=SafetyState.FAULT,
            )

        # Policy for finite out-of-bounds commands
        if self.out_of_bounds_policy == "reject":
            reason_str = "; ".join(violations)
            logger.warning("ActionShield rejecting out-of-bounds command under 'reject' policy: %s", reason_str)
            return ActionDecision(
                original_command=command,
                shielded_command=None,
                was_modified=True,
                is_rejected=True,
                intervention_reason=f"Out-of-bounds command rejected: {reason_str}",
                safety_state=SafetyState.FAULT,
            )

        # Apply kinematic limit clamping (out_of_bounds_policy == "clamp")
        clamped_pos = command.position_rad
        clamped_vel = command.velocity_rad_s
        reasons: list[str] = []

        if clamped_pos is not None:
            if clamped_pos < self.min_pos:
                clamped_pos = self.min_pos
                reasons.append("Clamped to min position limit")
            elif clamped_pos > self.max_pos:
                clamped_pos = self.max_pos
                reasons.append("Clamped to max position limit")

        if clamped_vel is not None:
            if abs(clamped_vel) > self.max_vel:
                clamped_vel = self.max_vel if clamped_vel > 0 else -self.max_vel
                reasons.append("Clamped to max velocity limit")

        shielded_cmd = JointCommand(
            joint_id=command.joint_id,
            position_rad=clamped_pos,
            velocity_rad_s=clamped_vel,
            acceleration_rad_s2=command.acceleration_rad_s2,
            timestamp=command.timestamp,
            sequence_id=command.sequence_id,
        )

        reason_str = "; ".join(reasons or violations)
        logger.warning(
            "ActionShield modified command for '%s': %s (original pos: %s -> clamped: %s)",
            command.joint_id,
            reason_str,
            command.position_rad,
            clamped_pos,
        )

        return ActionDecision(
            original_command=command,
            shielded_command=shielded_cmd,
            was_modified=True,
            is_rejected=False,
            intervention_reason=reason_str,
            safety_state=SafetyState.WARNING,
        )
