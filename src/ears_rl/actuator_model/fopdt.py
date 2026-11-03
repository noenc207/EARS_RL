"""Grey-Box First-Order Plus Dead Time (FOPDT) Actuator Models.

Transfer Function:
    G(s) = K * exp(-theta * s) / (tau * s + 1)

Parameters:
    K: static gain [dimensionless rad/rad]
    tau: mechanical time constant [seconds]
    theta: pure transport dead time [seconds]

Differentiates:
    - sample_period_ms: Telemetry acquisition interval Delta t
    - identified_dead_time_ms: Pure transport dead time theta
    - identified_time_constant_ms: Bandwidth lag parameter tau
    - identified_apparent_delay_ms: Lumped effective phase lag theta + tau
"""

from __future__ import annotations

import logging
from collections import deque
from typing import Any

import numpy as np
import pandas as pd
from scipy.optimize import minimize

from ears_rl.actuator_model.base import BaseActuatorModel
from ears_rl.core.types import JointCommand

logger = logging.getLogger(__name__)


def _delay_signal(
    u: np.ndarray,
    dt_s: float,
    delay_s: float,
    initial_val: float = 0.0,
) -> np.ndarray:
    """Delay input signal u by delay_s seconds using linear fractional shift.

    When negative indices occur before t=0, pads with initial_val (pre-excitation baseline)
    instead of u[0]. This ensures step excitation dead time is not corrupted.
    """
    if delay_s <= 1e-6:
        return u.copy()

    n = len(u)
    shift_samples = delay_s / max(1e-6, dt_s)
    d = int(np.floor(shift_samples))
    m = shift_samples - d  # fractional remainder in [0, 1)

    u_delayed = np.zeros(n, dtype=np.float64)

    for i in range(n):
        idx0 = i - d
        idx1 = i - d - 1

        val0 = u[idx0] if 0 <= idx0 < n else (initial_val if idx0 < 0 else u[-1])
        val1 = u[idx1] if 0 <= idx1 < n else (initial_val if idx1 < 0 else u[-1])

        u_delayed[i] = (1.0 - m) * val0 + m * val1

    return u_delayed


class FOPDTActuatorModel(BaseActuatorModel):
    """First-Order Plus Dead Time (FOPDT) Grey-Box Actuator Model.

    Combines pure transport dead time (theta) with first-order low-pass dynamics (tau).
    Allows true parametric separation between communication delay and mechanical bandwidth.
    """

    def __init__(
        self,
        gain: float = 1.0,
        tau_s: float = 0.05,
        dead_time_s: float = 0.01,
    ) -> None:
        super().__init__(model_type="fopdt")
        self.gain = float(gain)
        self.tau_s = max(0.0001, float(tau_s))
        self.dead_time_s = max(0.0, float(dead_time_s))

        self._parameters["gain"] = self.gain
        self._parameters["tau_s"] = self.tau_s
        self._parameters["dead_time_s"] = self.dead_time_s

        self._history: deque[tuple[int, float]] = deque()
        self._last_position_rad: float = 0.0

    def reset(self) -> None:
        self._history.clear()
        self._last_position_rad = 0.0

    def simulate(
        self,
        commands: np.ndarray,
        dt_s: float,
        initial_position_rad: float = 0.0,
        gain: float | None = None,
        tau_s: float | None = None,
        dead_time_s: float | None = None,
    ) -> np.ndarray:
        """Simulate discrete FOPDT response given command sequence."""
        k_val = self.gain if gain is None else gain
        tau_val = self.tau_s if tau_s is None else max(1e-4, tau_s)
        theta_val = self.dead_time_s if dead_time_s is None else max(0.0, dead_time_s)

        u_delayed = _delay_signal(commands, dt_s, theta_val, initial_val=initial_position_rad)

        alpha = float(np.exp(-dt_s / tau_val))
        beta = float(k_val * (1.0 - alpha))

        n = len(commands)
        y = np.zeros(n, dtype=np.float64)
        curr = initial_position_rad

        for i in range(n):
            y[i] = curr
            curr = alpha * curr + beta * u_delayed[i]

        return y

    def predict(
        self, command: JointCommand, current_state: dict[str, Any]
    ) -> dict[str, Any]:
        """Predict output position delayed by dead_time_s and filtered by tau_s."""
        target_pos = (
            command.position_rad
            if command.position_rad is not None
            else current_state.get("position_rad", 0.0)
        )
        curr_time_ns = current_state.get("timestamp_ns", 0)
        dt_s = max(1e-4, current_state.get("dt_s", 0.01))

        self._history.append((curr_time_ns, target_pos))

        # Look back by dead_time_s
        lookup_time_ns = curr_time_ns - int(self.dead_time_s * 1e9)
        delayed_pos = target_pos
        for ts, pos in self._history:
            if ts <= lookup_time_ns:
                delayed_pos = pos
            else:
                break

        # Trim history
        cutoff_ns = curr_time_ns - int((self.dead_time_s + 0.2) * 1e9)
        while len(self._history) > 1 and self._history[0][0] < cutoff_ns:
            self._history.popleft()

        curr_pos = current_state.get("position_rad", self._last_position_rad)
        alpha = float(np.exp(-dt_s / self.tau_s))
        beta = float(self.gain * (1.0 - alpha))
        pred_pos = alpha * curr_pos + beta * delayed_pos
        pred_vel = (pred_pos - curr_pos) / dt_s

        self._last_position_rad = pred_pos
        return {
            "predicted_position_rad": pred_pos,
            "predicted_velocity_rad_s": pred_vel,
            "predicted_dead_time_ms": self.dead_time_s * 1000.0,
            "predicted_apparent_delay_ms": (self.dead_time_s + self.tau_s) * 1000.0,
            "predicted_state": "normal",
            "model_type": self._model_type,
        }

    def fit(self, dataset: Any) -> dict[str, Any]:
        """Identify K, tau, and theta via discrete delay scan followed by bounded L-BFGS-B optimization."""
        try:
            df = dataset if isinstance(dataset, pd.DataFrame) else pd.DataFrame(dataset)
            if df.empty:
                return {
                    "status": "failed",
                    "error": "Dataset is empty",
                    "is_identifiable": False,
                }

            # 1. Derive sample period dt_s
            dt_s = 0.01
            if "timestamp_ns" in df.columns and len(df["timestamp_ns"].dropna()) > 1:
                ts_vals = df["timestamp_ns"].dropna().astype(float).values
                diffs_s = np.diff(ts_vals) / 1e9
                valid_diffs = diffs_s[diffs_s > 0]
                if len(valid_diffs) > 0:
                    dt_s = float(np.median(valid_diffs))

            # 2. Extract input / output columns
            if "command_position_rad" not in df.columns or "measured_motor_position_rad" not in df.columns:
                return {
                    "status": "failed",
                    "error": "Missing required command or measured position columns",
                    "is_identifiable": False,
                }

            cmd = df["command_position_rad"].dropna().astype(float).values
            meas = df["measured_motor_position_rad"].dropna().astype(float).values
            min_len = min(len(cmd), len(meas))
            if min_len < 15:
                return {
                    "status": "failed",
                    "error": f"Insufficient sample count ({min_len} < 15)",
                    "is_identifiable": False,
                }

            cmd = cmd[:min_len]
            meas = meas[:min_len]

            finite_mask = np.isfinite(cmd) & np.isfinite(meas)
            cmd = cmd[finite_mask]
            meas = meas[finite_mask]
            min_len = len(cmd)

            # 3. Excitation / identifiability check
            cmd_variance = float(np.var(cmd))
            if cmd_variance < 1e-6:
                return {
                    "status": "unidentifiable",
                    "error": f"Insufficient excitation: command variance ({cmd_variance:.2e} rad^2) is negligible.",
                    "is_identifiable": False,
                    "command_variance_rad2": cmd_variance,
                    "identified_gain": None,
                    "identified_time_constant_ms": None,
                    "identified_dead_time_ms": None,
                    "identified_apparent_delay_ms": None,
                }

            nominal_rmse = float(np.sqrt(np.mean((cmd - meas) ** 2)))
            y0 = float(meas[0])

            # 4. Stage 1: Discrete dead time grid search
            # Scan shifts up to 200 ms with sub-sample resolution (step <= 5 ms)
            step_scan = min(0.005, dt_s)
            max_shift = min(40, max(1, int(0.200 / step_scan)))
            best_init_theta = 0.0
            best_init_tau = 0.05
            best_init_k = 1.0
            best_init_loss = float("inf")

            for shift_steps in range(max_shift):
                theta_cand = shift_steps * step_scan
                u_cand = _delay_signal(cmd, dt_s, theta_cand, initial_val=y0)

                def loss_at_theta(params: list[float]) -> float:
                    k_val, tau_val = params
                    alpha = float(np.exp(-dt_s / max(1e-4, tau_val)))
                    beta = float(k_val * (1.0 - alpha))
                    y_sim = np.zeros(min_len)
                    curr = y0
                    for j in range(min_len):
                        y_sim[j] = curr
                        curr = alpha * curr + beta * u_cand[j]
                    return float(np.sum((y_sim - meas) ** 2))

                sub_res = minimize(
                    loss_at_theta,
                    [1.0, 0.05],
                    method="L-BFGS-B",
                    bounds=[(0.5, 2.0), (0.001, 1.0)],
                )
                if sub_res.fun < best_init_loss:
                    best_init_loss = sub_res.fun
                    best_init_theta = theta_cand
                    best_init_k = float(sub_res.x[0])
                    best_init_tau = float(sub_res.x[1])

            # 5. Stage 2: Continuous bounded optimization over (K, tau, theta)
            def joint_objective(params: list[float]) -> float:
                k_val, tau_val, theta_val = params
                y_sim = self.simulate(cmd, dt_s, initial_position_rad=y0, gain=k_val, tau_s=tau_val, dead_time_s=theta_val)
                return float(np.sum((y_sim - meas) ** 2))

            bounds = [(0.5, 2.0), (0.001, 1.0), (0.0, 0.3)]
            opt_res = minimize(
                joint_objective,
                [best_init_k, best_init_tau, best_init_theta],
                method="L-BFGS-B",
                bounds=bounds,
            )

            if opt_res.success or np.isfinite(opt_res.fun):
                opt_k, opt_tau, opt_theta = (
                    float(opt_res.x[0]),
                    float(opt_res.x[1]),
                    float(opt_res.x[2]),
                )
            else:
                opt_k, opt_tau, opt_theta = best_init_k, best_init_tau, best_init_theta

            y_best = self.simulate(cmd, dt_s, initial_position_rad=y0, gain=opt_k, tau_s=opt_tau, dead_time_s=opt_theta)
            residual_rmse = float(np.sqrt(np.mean((y_best - meas) ** 2)))

            self.gain = opt_k
            self.tau_s = opt_tau
            self.dead_time_s = opt_theta
            self._parameters["gain"] = opt_k
            self._parameters["tau_s"] = opt_tau
            self._parameters["dead_time_s"] = opt_theta
            self._is_fitted = True

            apparent_delay_ms = (opt_theta + opt_tau) * 1000.0

            # 6. Structural mismatch check: detect if rate limiting / saturation is actively present
            structural_mismatch = False
            mismatch_reason = ""
            if min_len > 1:
                meas_vel = np.abs(np.diff(meas) / dt_s)
                cmd_vel = np.abs(np.diff(cmd) / dt_s)
                max_meas_vel = float(np.max(meas_vel))
                max_cmd_vel = float(np.max(cmd_vel))

                if max_cmd_vel > 1.5 * max_meas_vel and max_meas_vel > 0.05:
                    near_peak = meas_vel[meas_vel > 0.85 * max_meas_vel]
                    if len(near_peak) >= 5 and float(np.std(near_peak)) < 0.12 * max_meas_vel:
                        structural_mismatch = True
                        mismatch_reason = (
                            f"Actuator exhibits velocity saturation plateau at ~{max_meas_vel:.2f} rad/s. "
                            f"Linear FOPDT cannot capture rate limiting without parameter distortion."
                        )
                        logger.warning("Structural mismatch in FOPDTActuatorModel: %s", mismatch_reason)

            return {
                "status": "fitted",
                "is_identifiable": True,
                "identified_gain": round(opt_k, 4),
                "identified_time_constant_s": round(opt_tau, 5),
                "identified_time_constant_ms": round(opt_tau * 1000.0, 2),
                "identified_dead_time_s": round(opt_theta, 5),
                "identified_dead_time_ms": round(opt_theta * 1000.0, 2),
                "identified_apparent_delay_ms": round(apparent_delay_ms, 2),
                "sample_period_ms": round(dt_s * 1000.0, 2),
                "nominal_rmse_rad": round(nominal_rmse, 5),
                "residual_rmse_rad": round(residual_rmse, 5),
                "rmse_reduction_rad": round(nominal_rmse - residual_rmse, 5),
                "structural_mismatch": structural_mismatch,
                "structural_mismatch_warning": mismatch_reason if structural_mismatch else None,
                "estimator_method": "fopdt_two_stage_bounded_lbfgsb",
            }

        except Exception as e:
            logger.exception("FOPDT identification failed: %s", e)
            return {
                "status": "error",
                "error": str(e),
                "is_identifiable": False,
            }


class FOPDTRateLimitedActuatorModel(BaseActuatorModel):
    """FOPDT model incorporating explicit velocity rate-limiting / saturation.

    Physics:
        desired_velocity = (K * u(t - theta) - y(t)) / tau
        actual_velocity = clip(desired_velocity, -v_max, v_max)
        dy/dt = actual_velocity

    Prevents rate limiting from being mistakenly conflated with pure dead time.
    """

    def __init__(
        self,
        gain: float = 1.0,
        tau_s: float = 0.05,
        dead_time_s: float = 0.01,
        velocity_max_rad_s: float = 6.28,
    ) -> None:
        super().__init__(model_type="fopdt_rate_limited")
        self.gain = float(gain)
        self.tau_s = max(0.0001, float(tau_s))
        self.dead_time_s = max(0.0, float(dead_time_s))
        self.velocity_max_rad_s = max(0.1, float(velocity_max_rad_s))

        self._parameters["gain"] = self.gain
        self._parameters["tau_s"] = self.tau_s
        self._parameters["dead_time_s"] = self.dead_time_s
        self._parameters["velocity_max_rad_s"] = self.velocity_max_rad_s

        self._last_position_rad: float = 0.0

    def simulate(
        self,
        commands: np.ndarray,
        dt_s: float,
        initial_position_rad: float = 0.0,
        gain: float | None = None,
        tau_s: float | None = None,
        dead_time_s: float | None = None,
        velocity_max_rad_s: float | None = None,
    ) -> np.ndarray:
        """Simulate rate-limited FOPDT response."""
        k_val = self.gain if gain is None else gain
        tau_val = self.tau_s if tau_s is None else max(1e-4, tau_s)
        theta_val = self.dead_time_s if dead_time_s is None else max(0.0, dead_time_s)
        v_max = self.velocity_max_rad_s if velocity_max_rad_s is None else max(0.1, velocity_max_rad_s)

        u_delayed = _delay_signal(commands, dt_s, theta_val, initial_val=initial_position_rad)
        n = len(commands)
        y = np.zeros(n, dtype=np.float64)
        curr = initial_position_rad
        alpha = float(np.exp(-dt_s / tau_val))
        max_step = v_max * dt_s

        for i in range(n):
            y[i] = curr
            target = k_val * u_delayed[i]
            delta_ideal = (1.0 - alpha) * (target - curr)
            delta_clamped = float(np.clip(delta_ideal, -max_step, max_step))
            curr += delta_clamped

        return y

    def predict(
        self, command: JointCommand, current_state: dict[str, Any]
    ) -> dict[str, Any]:
        """Predict next step under rate limit."""
        target_pos = (
            command.position_rad
            if command.position_rad is not None
            else current_state.get("position_rad", 0.0)
        )
        curr_pos = current_state.get("position_rad", self._last_position_rad)
        dt_s = max(1e-4, current_state.get("dt_s", 0.01))

        desired_vel = (self.gain * target_pos - curr_pos) / self.tau_s
        clamped_vel = float(np.clip(desired_vel, -self.velocity_max_rad_s, self.velocity_max_rad_s))
        pred_pos = curr_pos + clamped_vel * dt_s

        self._last_position_rad = pred_pos
        is_saturated = abs(desired_vel) > self.velocity_max_rad_s

        return {
            "predicted_position_rad": pred_pos,
            "predicted_velocity_rad_s": clamped_vel,
            "predicted_state": "saturated" if is_saturated else "normal",
            "model_type": self._model_type,
        }

    def fit(self, dataset: Any) -> dict[str, Any]:
        """Fit rate-limited FOPDT model via multi-parameter bounded optimization.

        Jointly estimates static gain K, time constant tau, dead time theta,
        and maximum velocity limit velocity_max_rad_s using rate-limited physics simulation.
        """
        try:
            df = dataset if isinstance(dataset, pd.DataFrame) else pd.DataFrame(dataset)
            if df.empty:
                return {
                    "status": "failed",
                    "error": "Dataset is empty",
                    "is_identifiable": False,
                }

            # 1. Derive sample period dt_s
            dt_s = 0.01
            if "timestamp_ns" in df.columns and len(df["timestamp_ns"].dropna()) > 1:
                ts_vals = df["timestamp_ns"].dropna().astype(float).values
                diffs_s = np.diff(ts_vals) / 1e9
                valid_diffs = diffs_s[diffs_s > 0]
                if len(valid_diffs) > 0:
                    dt_s = float(np.median(valid_diffs))

            # 2. Extract input / output columns
            if "command_position_rad" not in df.columns or "measured_motor_position_rad" not in df.columns:
                return {
                    "status": "failed",
                    "error": "Missing required command or measured position columns",
                    "is_identifiable": False,
                }

            cmd = df["command_position_rad"].dropna().astype(float).values
            meas = df["measured_motor_position_rad"].dropna().astype(float).values
            min_len = min(len(cmd), len(meas))
            if min_len < 15:
                return {
                    "status": "failed",
                    "error": f"Insufficient sample count ({min_len} < 15)",
                    "is_identifiable": False,
                }

            cmd = cmd[:min_len]
            meas = meas[:min_len]
            finite_mask = np.isfinite(cmd) & np.isfinite(meas)
            cmd = cmd[finite_mask]
            meas = meas[finite_mask]
            min_len = len(cmd)

            # Excitation check
            cmd_variance = float(np.var(cmd))
            if cmd_variance < 1e-6:
                return {
                    "status": "unidentifiable",
                    "error": f"Insufficient excitation: command variance ({cmd_variance:.2e} rad^2) is negligible.",
                    "is_identifiable": False,
                    "command_variance_rad2": cmd_variance,
                    "identified_gain": None,
                    "identified_time_constant_ms": None,
                    "identified_dead_time_ms": None,
                    "identified_apparent_delay_ms": None,
                    "identified_velocity_max_rad_s": None,
                }

            nominal_rmse = float(np.sqrt(np.mean((cmd - meas) ** 2)))
            y0 = float(meas[0])

            # 3. Fit base linear FOPDT first to obtain baseline metrics
            base_fopdt = FOPDTActuatorModel()
            fopdt_res = base_fopdt.fit(df)
            if not fopdt_res.get("is_identifiable", False):
                return fopdt_res

            linear_residual_rmse = fopdt_res["residual_rmse_rad"]

            # Estimate physical velocity ceiling from telemetry
            meas_vel = np.abs(np.diff(meas) / dt_s)
            if len(meas_vel) > 0:
                v_ceil = float(np.max(meas_vel))
                init_vmax = max(0.5, v_ceil)
            else:
                init_vmax = self.velocity_max_rad_s

            # 4. Multi-start grid search over (theta, tau) to initialize rate-limited optimization
            max_shift = min(25, max(1, min_len // 4))
            best_init_theta = float(fopdt_res["identified_dead_time_s"])
            best_init_tau = float(fopdt_res["identified_time_constant_s"])
            best_init_k = float(fopdt_res["identified_gain"])
            best_init_loss = float("inf")

            # Candidate seeds
            test_seeds = []
            # Linear FOPDT result seed
            test_seeds.append((best_init_k, best_init_tau, best_init_theta))
            # Grid search seeds
            for shift_steps in range(max_shift):
                theta_cand = shift_steps * dt_s
                for tau_cand in (0.02, 0.05, 0.10):
                    test_seeds.append((1.0, tau_cand, theta_cand))

            for k_cand, tau_cand, th_cand in test_seeds:
                y_test = self.simulate(
                    cmd,
                    dt_s,
                    initial_position_rad=y0,
                    gain=k_cand,
                    tau_s=tau_cand,
                    dead_time_s=th_cand,
                    velocity_max_rad_s=init_vmax,
                )
                loss_cand = float(np.sum((y_test - meas) ** 2))
                if loss_cand < best_init_loss:
                    best_init_loss = loss_cand
                    best_init_k = k_cand
                    best_init_tau = tau_cand
                    best_init_theta = th_cand

            # 5. Joint bounded optimization over [K, tau, theta, vmax] using rate-limited simulation
            def rate_limited_objective(params: list[float]) -> float:
                k_val, tau_val, theta_val, vmax_val = params
                y_sim = self.simulate(
                    cmd,
                    dt_s,
                    initial_position_rad=y0,
                    gain=k_val,
                    tau_s=tau_val,
                    dead_time_s=theta_val,
                    velocity_max_rad_s=vmax_val,
                )
                return float(np.sum((y_sim - meas) ** 2))

            bounds = [(0.5, 2.0), (0.001, 1.0), (0.0, 0.3), (0.1, 25.0)]
            init_params = [best_init_k, best_init_tau, best_init_theta, init_vmax]
            opt_res = minimize(
                rate_limited_objective,
                init_params,
                method="L-BFGS-B",
                bounds=bounds,
            )

            if opt_res.success or np.isfinite(opt_res.fun):
                opt_k, opt_tau, opt_theta, opt_vmax = (
                    float(opt_res.x[0]),
                    float(opt_res.x[1]),
                    float(opt_res.x[2]),
                    float(opt_res.x[3]),
                )
            else:
                opt_k, opt_tau, opt_theta, opt_vmax = best_init_k, best_init_tau, best_init_theta, init_vmax

            y_best = self.simulate(
                cmd,
                dt_s,
                initial_position_rad=y0,
                gain=opt_k,
                tau_s=opt_tau,
                dead_time_s=opt_theta,
                velocity_max_rad_s=opt_vmax,
            )
            residual_rmse = float(np.sqrt(np.mean((y_best - meas) ** 2)))

            self.gain = opt_k
            self.tau_s = opt_tau
            self.dead_time_s = opt_theta
            self.velocity_max_rad_s = opt_vmax
            self._parameters["gain"] = opt_k
            self._parameters["tau_s"] = opt_tau
            self._parameters["dead_time_s"] = opt_theta
            self._parameters["velocity_max_rad_s"] = opt_vmax
            self._is_fitted = True

            apparent_delay_ms = (opt_theta + opt_tau) * 1000.0

            return {
                "status": "fitted",
                "is_identifiable": True,
                "model_type": "fopdt_rate_limited",
                "identified_gain": round(opt_k, 4),
                "identified_time_constant_s": round(opt_tau, 5),
                "identified_time_constant_ms": round(opt_tau * 1000.0, 2),
                "identified_dead_time_s": round(opt_theta, 5),
                "identified_dead_time_ms": round(opt_theta * 1000.0, 2),
                "identified_apparent_delay_ms": round(apparent_delay_ms, 2),
                "identified_velocity_max_rad_s": round(opt_vmax, 4),
                "sample_period_ms": round(dt_s * 1000.0, 2),
                "nominal_rmse_rad": round(nominal_rmse, 5),
                "residual_rmse_rad": round(residual_rmse, 5),
                "rmse_reduction_rad": round(nominal_rmse - residual_rmse, 5),
                "linear_fopdt_residual_rmse_rad": round(linear_residual_rmse, 5),
                "estimator_method": "fopdt_rate_limited_bounded_lbfgsb",
            }
        except Exception as e:
            logger.exception("Rate-limited FOPDT identification failed: %s", e)
            return {
                "status": "error",
                "error": str(e),
                "is_identifiable": False,
            }
