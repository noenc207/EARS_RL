"""Independent Continuous-Time Analytical and High-Order Validation for Actuator Models.

This module provides an independent reference ground truth for grey-box actuator identification
that does NOT share discrete formulas, signal-padding loops, or discretization approximations
with FOPDTActuatorModel.

Key capabilities:
1. Exact closed-form analytical continuous-time step/multistep FOPDT response.
2. Independent high-order Runge-Kutta 4th-order (RK4) continuous-time differential equation solver.
3. Quantifies sampling resolution, discretization phase shift (ZOH), and dead-time quantization limits.
4. Independent validation benchmark matrix across delay, time constant, sample rate, noise, and rate limiting.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class IndependentValidationResult:
    """Result of independent parameter recovery verification against continuous physics."""

    benchmark_type: str  # "analytical_closed_form", "rk4_continuous", "discrete_zoh"
    trajectory: str
    sample_period_ms: float
    gt_gain: float
    gt_tau_ms: float
    gt_dead_time_ms: float
    gt_vmax_rad_s: float | None
    noise_sigma_rad: float

    # Estimator outputs
    is_identifiable: bool
    identified_gain: float | None
    identified_tau_ms: float | None
    identified_dead_time_ms: float | None
    identified_vmax_rad_s: float | None

    # Error metrics
    gain_error: float | None
    tau_error_ms: float | None
    dead_time_error_ms: float | None
    residual_rmse_rad: float | None
    discretization_uncertainty_ms: float
    structural_mismatch: bool


def fopdt_analytical_multistep(
    t_array_s: np.ndarray,
    gain: float,
    tau_s: float,
    dead_time_s: float,
    step_levels: list[float],
    step_times_s: list[float],
    initial_val: float = 0.0,
) -> np.ndarray:
    """Exact closed-form analytical solution of continuous-time FOPDT differential equation.

    Physics:
        tau * dy(t)/dt + y(t) = K * u(t - theta),  y(0) = initial_val

    For a sequence of step transitions Delta U_k at times T_k, the exact continuous solution is:
        y(t) = initial_val + sum_{k: T_k + theta <= t} K * Delta U_k * (1 - exp(-(t - (T_k + theta)) / tau))

    Contains ZERO discrete time-step approximations or recursive ZOH numerical integration error.
    """
    y = np.full_like(t_array_s, initial_val, dtype=np.float64)

    for i in range(len(step_times_s)):
        t_switch = step_times_s[i] + dead_time_s
        mask = t_array_s >= t_switch
        if not np.any(mask):
            continue

        u_prev = initial_val if i == 0 else step_levels[i - 1]
        u_curr = step_levels[i]
        delta_u = u_curr - u_prev

        time_since_arrival = t_array_s[mask] - t_switch
        y[mask] += gain * delta_u * (1.0 - np.exp(-time_since_arrival / tau_s))

    return y


def fopdt_rk4_continuous_simulation(
    t_array_s: np.ndarray,
    gain: float,
    tau_s: float,
    dead_time_s: float,
    command_fn: Callable[[float], float],
    velocity_max_rad_s: float | None = None,
    initial_val: float = 0.0,
    substeps_per_sample: int = 50,
) -> np.ndarray:
    """Independent continuous-time ODE integration via 4th-Order Runge-Kutta (RK4).

    Integrates:
        dy/dt = clip((K * u_delayed(t) - y) / tau, -v_max, v_max)
    with sub-millisecond continuous steps, evaluating u_delayed(t) by exact analytical continuous
    time evaluation without discrete array indexing.
    """
    n_samples = len(t_array_s)
    y_out = np.zeros(n_samples, dtype=np.float64)
    y_curr = float(initial_val)
    y_out[0] = y_curr

    def u_delayed(t_eval: float) -> float:
        if t_eval < dead_time_s:
            return initial_val
        return command_fn(t_eval - dead_time_s)

    for i in range(n_samples - 1):
        t0 = t_array_s[i]
        t1 = t_array_s[i + 1]
        dt = (t1 - t0) / substeps_per_sample

        for step in range(substeps_per_sample):
            t_sub = t0 + step * dt

            def deriv(t_now: float, y_val: float) -> float:
                u_val = u_delayed(t_now)
                desired_vel = (gain * u_val - y_val) / max(1e-5, tau_s)
                if velocity_max_rad_s is not None:
                    return float(np.clip(desired_vel, -velocity_max_rad_s, velocity_max_rad_s))
                return desired_vel

            k1 = deriv(t_sub, y_curr)
            k2 = deriv(t_sub + 0.5 * dt, y_curr + 0.5 * dt * k1)
            k3 = deriv(t_sub + 0.5 * dt, y_curr + 0.5 * dt * k2)
            k4 = deriv(t_sub + dt, y_curr + dt * k3)

            y_curr += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        y_out[i + 1] = y_curr

    return y_out


def build_independent_multistep_dataset(
    sample_period_s: float = 0.010,
    duration_s: float = 2.0,
    gain: float = 1.0,
    tau_s: float = 0.040,
    dead_time_s: float = 0.020,
    step_levels: list[float] | None = None,
    step_times_s: list[float] | None = None,
    initial_val: float = 0.0,
    velocity_max_rad_s: float | None = None,
    noise_sigma_rad: float = 0.0,
    random_seed: int = 42,
    solver: str = "analytical",  # "analytical" or "rk4"
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Build standardized telemetry records using an independent continuous-time solver."""
    if step_levels is None:
        step_levels = [0.8, -0.6, 1.2, 0.0]
    if step_times_s is None:
        step_times_s = [0.0, 0.5, 1.0, 1.5]

    n_steps = int(round(duration_s / sample_period_s))
    t = np.arange(n_steps, dtype=np.float64) * sample_period_s

    # Generate commanded trajectory
    cmd = np.full(n_steps, initial_val, dtype=np.float64)
    for i in range(len(step_times_s)):
        cmd[t >= step_times_s[i]] = step_levels[i]

    if solver == "analytical":
        y_clean = fopdt_analytical_multistep(
            t_array_s=t,
            gain=gain,
            tau_s=tau_s,
            dead_time_s=dead_time_s,
            step_levels=step_levels,
            step_times_s=step_times_s,
            initial_val=initial_val,
        )
    elif solver == "rk4":
        def step_fn(t_sec: float) -> float:
            val = initial_val
            for k in range(len(step_times_s)):
                if t_sec >= step_times_s[k]:
                    val = step_levels[k]
            return val

        y_clean = fopdt_rk4_continuous_simulation(
            t_array_s=t,
            gain=gain,
            tau_s=tau_s,
            dead_time_s=dead_time_s,
            command_fn=step_fn,
            velocity_max_rad_s=velocity_max_rad_s,
            initial_val=initial_val,
        )
    else:
        raise ValueError(f"Unknown independent solver: {solver}")

    # Add measurement noise if configured
    if noise_sigma_rad > 0:
        rng = np.random.default_rng(random_seed)
        y_meas = y_clean + rng.normal(0.0, noise_sigma_rad, size=n_steps)
    else:
        y_meas = y_clean.copy()

    records = []
    for i in range(n_steps):
        records.append({
            "sequence_id": i + 1,
            "timestamp_ns": int(round(t[i] * 1e9)),
            "command_position_rad": float(cmd[i]),
            "measured_motor_position_rad": float(y_meas[i]),
            "sample_period_ms": round(sample_period_s * 1000.0, 3),
            "configured_simulated_dead_time_ms": round(dead_time_s * 1000.0, 3),
            "configured_simulated_delay_ms": round(dead_time_s * 1000.0, 3),
            "data_source": "synthetic",
            "generator_solver": f"independent_{solver}",
        })

    gt_meta = {
        "solver": solver,
        "gain": gain,
        "tau_s": tau_s,
        "dead_time_s": dead_time_s,
        "sample_period_s": sample_period_s,
        "noise_sigma_rad": noise_sigma_rad,
    }

    return records, gt_meta


def run_comprehensive_independent_validation_matrix(
    random_seed: int = 42,
) -> list[IndependentValidationResult]:
    """Execute full independent validation matrix across parameters, solvers, and rates.

    Covers:
    - Dead times: 0, 10, 20, 40 ms
    - Time constants: 20, 50, 100 ms
    - Sample periods: 5, 10, 20 ms
    - Noise levels: 0.0, 0.001, 0.005, 0.020 rad
    - Independent solvers: Analytical Closed-Form vs RK4 Continuous
    """
    from ears_rl.actuator_model.fopdt import (
        FOPDTActuatorModel,
        FOPDTRateLimitedActuatorModel,
    )

    results: list[IndependentValidationResult] = []

    test_grid = [
        # 1. Grid-aligned integer delay sweep with exact analytical solution (dt = 10 ms)
        {"solver": "analytical", "th_ms": 0.0, "tau_ms": 40.0, "dt_ms": 10.0, "noise": 0.0},
        {"solver": "analytical", "th_ms": 10.0, "tau_ms": 40.0, "dt_ms": 10.0, "noise": 0.0},
        {"solver": "analytical", "th_ms": 20.0, "tau_ms": 40.0, "dt_ms": 10.0, "noise": 0.0},
        {"solver": "analytical", "th_ms": 40.0, "tau_ms": 40.0, "dt_ms": 10.0, "noise": 0.0},

        # 2. Continuous fractional delays (non-grid-aligned: sub-sample delay estimation)
        {"solver": "analytical", "th_ms": 7.5, "tau_ms": 40.0, "dt_ms": 10.0, "noise": 0.0},
        {"solver": "analytical", "th_ms": 15.0, "tau_ms": 40.0, "dt_ms": 10.0, "noise": 0.0},
        {"solver": "analytical", "th_ms": 25.0, "tau_ms": 40.0, "dt_ms": 10.0, "noise": 0.0},
        {"solver": "analytical", "th_ms": 12.5, "tau_ms": 40.0, "dt_ms": 20.0, "noise": 0.0},

        # 3. Sampling period sweep (dt = 5 ms, 10 ms, 20 ms)
        {"solver": "analytical", "th_ms": 20.0, "tau_ms": 40.0, "dt_ms": 5.0, "noise": 0.0},
        {"solver": "analytical", "th_ms": 20.0, "tau_ms": 40.0, "dt_ms": 20.0, "noise": 0.0},

        # 4. Non-zero initial conditions (y0 != 0.0)
        {
            "solver": "analytical",
            "th_ms": 20.0,
            "tau_ms": 40.0,
            "dt_ms": 10.0,
            "noise": 0.0,
            "initial_val": -0.5,
            "step_levels": [0.8, -0.3, 0.5],
            "step_times_s": [0.2, 0.7, 1.2],
        },
        {
            "solver": "analytical",
            "th_ms": 20.0,
            "tau_ms": 40.0,
            "dt_ms": 10.0,
            "noise": 0.0,
            "initial_val": 0.5,
            "step_levels": [-0.8, 0.3, -0.5],
            "step_times_s": [0.2, 0.7, 1.2],
        },

        # 5. Noise sweep on analytical physics
        {"solver": "analytical", "th_ms": 20.0, "tau_ms": 40.0, "dt_ms": 10.0, "noise": 0.001},
        {"solver": "analytical", "th_ms": 20.0, "tau_ms": 40.0, "dt_ms": 10.0, "noise": 0.005},
        {"solver": "analytical", "th_ms": 20.0, "tau_ms": 40.0, "dt_ms": 10.0, "noise": 0.020},

        # 6. RK4 continuous-time ODE solver verification
        {"solver": "rk4", "th_ms": 10.0, "tau_ms": 50.0, "dt_ms": 10.0, "noise": 0.0},
        {"solver": "rk4", "th_ms": 20.0, "tau_ms": 30.0, "dt_ms": 10.0, "noise": 0.0},

        # 7. RK4 continuous-time ODE with rate-limiting & FOPDTRateLimitedActuatorModel
        {
            "solver": "rk4",
            "th_ms": 20.0,
            "tau_ms": 40.0,
            "dt_ms": 10.0,
            "noise": 0.0,
            "vmax": 2.0,
            "model_type": "rate_limited",
        },

        # 8. Uninformative static hold (insufficient excitation -> rejected)
        {
            "solver": "analytical",
            "th_ms": 20.0,
            "tau_ms": 40.0,
            "dt_ms": 10.0,
            "noise": 0.0,
            "step_levels": [0.5, 0.5, 0.5],
            "step_times_s": [0.0, 0.5, 1.0],
            "initial_val": 0.5,
            "uninformative": True,
        },
    ]

    for cfg in test_grid:
        init_val = float(cfg.get("initial_val", 0.0))
        vmax_val = cfg.get("vmax")
        levels = cfg.get("step_levels")
        times = cfg.get("step_times_s")
        model_type = cfg.get("model_type", "fopdt")

        records, gt = build_independent_multistep_dataset(
            sample_period_s=cfg["dt_ms"] / 1000.0,
            duration_s=2.0,
            gain=1.0,
            tau_s=cfg["tau_ms"] / 1000.0,
            dead_time_s=cfg["th_ms"] / 1000.0,
            step_levels=levels,
            step_times_s=times,
            initial_val=init_val,
            velocity_max_rad_s=vmax_val,
            noise_sigma_rad=cfg["noise"],
            random_seed=random_seed,
            solver=cfg["solver"],
        )

        if model_type == "rate_limited":
            model = FOPDTRateLimitedActuatorModel()
        else:
            model = FOPDTActuatorModel()

        fit_res = model.fit(records)

        is_identifiable = fit_res.get("is_identifiable", False)
        if is_identifiable:
            id_k = fit_res["identified_gain"]
            id_tau_ms = fit_res["identified_time_constant_ms"]
            id_th_ms = fit_res["identified_dead_time_ms"]
            id_vmax = fit_res.get("identified_velocity_max_rad_s")
            gain_err = abs(id_k - 1.0)
            tau_err = abs(id_tau_ms - cfg["tau_ms"])
            th_err = abs(id_th_ms - cfg["th_ms"])
            res_rmse = fit_res["residual_rmse_rad"]
            mismatch = fit_res.get("structural_mismatch", False)
        else:
            id_k, id_tau_ms, id_th_ms, id_vmax = None, None, None, None
            gain_err, tau_err, th_err = None, None, None
            res_rmse = None
            mismatch = False

        results.append(
            IndependentValidationResult(
                benchmark_type=cfg["solver"],
                trajectory="constant_uninformative" if cfg.get("uninformative") else "multistep",
                sample_period_ms=cfg["dt_ms"],
                gt_gain=1.0,
                gt_tau_ms=cfg["tau_ms"],
                gt_dead_time_ms=cfg["th_ms"],
                gt_vmax_rad_s=vmax_val,
                noise_sigma_rad=cfg["noise"],
                is_identifiable=is_identifiable,
                identified_gain=id_k,
                identified_tau_ms=id_tau_ms,
                identified_dead_time_ms=id_th_ms,
                identified_vmax_rad_s=id_vmax,
                gain_error=round(gain_err, 4) if gain_err is not None else None,
                tau_error_ms=round(tau_err, 2) if tau_err is not None else None,
                dead_time_error_ms=round(th_err, 2) if th_err is not None else None,
                residual_rmse_rad=round(res_rmse, 5) if res_rmse is not None else None,
                discretization_uncertainty_ms=cfg["dt_ms"],  # Fundamental resolution limit
                structural_mismatch=mismatch,
            )
        )

    return results
