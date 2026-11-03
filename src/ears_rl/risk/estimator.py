"""Operational risk estimation module for EARS-RL."""

from __future__ import annotations

from typing import Any

from ears_rl.core.interfaces import RiskEstimator
from ears_rl.core.types import (
    ActuatorState,
    JointCommand,
    RiskAssessment,
)


class SimpleRiskEstimator(RiskEstimator):
    """Estimates operational risk based on state flags, tracking error, and boundary proximity."""

    def __init__(
        self,
        joint_id: str = "joint_0",
        position_limits_rad: tuple[float, float] = (-3.14159, 3.14159),
        warning_margin_rad: float = 0.20,
    ) -> None:
        self.joint_id = joint_id
        self.min_pos, self.max_pos = position_limits_rad
        self.warning_margin = warning_margin_rad

    def estimate_risk(
        self, state: dict[str, Any], action: JointCommand | None = None
    ) -> RiskAssessment:
        curr_pos = state.get("position_rad", 0.0)
        act_state_str = state.get("actuator_state", "normal")
        reasons: list[str] = []
        risk_score = 0.0

        # Boundary proximity risk
        dist_to_min = curr_pos - self.min_pos
        dist_to_max = self.max_pos - curr_pos
        min_dist = min(dist_to_min, dist_to_max)

        if min_dist < 0:
            risk_score = max(risk_score, 1.0)
            reasons.append("Position violates hard mechanical limits")
        elif min_dist < self.warning_margin:
            norm_risk = 1.0 - (min_dist / self.warning_margin)
            risk_score = max(risk_score, float(norm_risk * 0.8))
            reasons.append(f"Position close to limit (margin={min_dist:.3f} rad)")

        # Discrete state risk
        if act_state_str == "step_loss":
            risk_score = max(risk_score, 0.7)
            reasons.append("Step loss desynchronization active")
        elif act_state_str == "stall":
            risk_score = max(risk_score, 0.9)
            reasons.append("Actuator stall active")

        return RiskAssessment(
            risk_score=min(1.0, risk_score),
            joint_id=self.joint_id,
            state_assessment=ActuatorState(act_state_str) if act_state_str in ActuatorState._value2member_map_ else ActuatorState.UNKNOWN,
            violates_constraints=(min_dist < 0),
            reasons=reasons,
        )
