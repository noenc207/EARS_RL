"""Synthetic Actuator Identification Benchmark Suite with Parameter Ground Truth.

Generates ground-truth datasets for 7 isolated engineering cases:
    Case A: Pure Delay (tau -> 0, theta = 30 ms)
    Case B: First-Order Dynamics without Dead Time (tau = 50 ms, theta = 0 ms)
    Case C: FOPDT Standard (K = 1.0, tau = 40 ms, theta = 20 ms)
    Case D: FOPDT with Velocity Rate Limiting (K = 1.0, tau = 40 ms, theta = 20 ms, vmax = 2.0 rad/s)
    Case E: FOPDT with Measurement Noise (sigma = 0.005 rad)
    Case F: Severe Saturation / High-Frequency Dynamics
    Case G: Uninformative / Insufficient Excitation (Constant hold)

Evaluates on multiple trajectories:
    - Chirp (frequency sweep: 0.2 Hz -> 2.5 Hz)
    - Multistep (staircase displacement)
    - Multisine (sum of distinct harmonics)
    - Constant hold (uninformative)
"""

from __future__ import annotations

import logging
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ears_rl.actuator_model.first_order import FirstOrderActuatorModel
from ears_rl.actuator_model.fixed_delay import FixedDelayActuatorModel
from ears_rl.actuator_model.fopdt import (
    FOPDTActuatorModel,
    FOPDTRateLimitedActuatorModel,
    _delay_signal,
)
from ears_rl.actuator_model.nominal import NominalActuatorModel

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class BenchmarkGroundTruth:
    """True generative parameters for synthetic benchmark evaluation."""
    case_id: str
    description: str
    gain: float
    tau_s: float
    dead_time_s: float
    velocity_max_rad_s: float | None
    noise_std_rad: float
    is_informative_excitation: bool
    generator_version: str = "1.2.0"


class SyntheticBenchmarkGenerator:
    """Generates synthetic actuator telemetry with bitwise-reproducible ground truth."""

    def __init__(self, random_seed: int = 42) -> None:
        self.random_seed = random_seed
        self.rng = np.random.default_rng(random_seed)

    def generate_trajectory(
        self,
        trajectory_type: str,
        duration_s: float,
        dt_s: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Generate time array and commanded position sequence."""
        n_steps = int(round(duration_s / dt_s))
        t = np.arange(n_steps, dtype=np.float64) * dt_s

        if trajectory_type == "chirp":
            f0, f1 = 0.2, 2.5
            phase = 2.0 * np.pi * (f0 + 0.5 * (f1 - f0) * (t / duration_s)) * t
            cmd = 1.0 * np.sin(phase)
        elif trajectory_type == "step":
            # Multistep sequence
            cmd = np.zeros(n_steps, dtype=np.float64)
            period = int(1.0 / dt_s)
            levels = [0.0, 0.8, -0.6, 1.2, -0.4, 0.0]
            for i in range(n_steps):
                level_idx = min(len(levels) - 1, i // max(1, period))
                cmd[i] = levels[level_idx]
        elif trajectory_type == "prbs":
            # Pseudo-Random Binary Sequence (informative broadband excitation)
            rng = np.random.default_rng(self.random_seed + 101)
            cmd = np.zeros(n_steps, dtype=np.float64)
            val = 0.8
            switch_prob = 0.15
            for i in range(n_steps):
                if rng.random() < switch_prob:
                    val = -val
                cmd[i] = val
        elif trajectory_type == "multisine":
            cmd = (
                0.6 * np.sin(2.0 * np.pi * 0.5 * t)
                + 0.3 * np.sin(2.0 * np.pi * 1.5 * t)
                + 0.15 * np.sin(2.0 * np.pi * 3.0 * t)
            )
        elif trajectory_type == "constant_uninformative":
            cmd = np.full(n_steps, 0.5, dtype=np.float64)
        else:
            cmd = np.sin(2.0 * np.pi * 1.0 * t)

        return t, cmd

    def generate_dataset(
        self,
        case_id: str,
        trajectory_type: str = "chirp",
        duration_s: float = 2.0,
        sample_rate_hz: float = 100.0,
    ) -> tuple[list[dict[str, Any]], BenchmarkGroundTruth]:
        """Generate a complete telemetry dataset and ground truth metadata for a benchmark case."""
        dt_s = 1.0 / sample_rate_hz
        t, cmd = self.generate_trajectory(trajectory_type, duration_s, dt_s)
        n = len(t)

        # Define case configurations
        if case_id == "case_a_pure_delay":
            gt = BenchmarkGroundTruth(
                case_id=case_id,
                description="Pure Transport Delay without dynamics",
                gain=1.0,
                tau_s=0.0001,
                dead_time_s=0.030,
                velocity_max_rad_s=None,
                noise_std_rad=0.0,
                is_informative_excitation=True,
            )
        elif case_id == "case_b_first_order":
            gt = BenchmarkGroundTruth(
                case_id=case_id,
                description="First-Order Lag without Dead Time",
                gain=1.0,
                tau_s=0.050,
                dead_time_s=0.0,
                velocity_max_rad_s=None,
                noise_std_rad=0.0,
                is_informative_excitation=True,
            )
        elif case_id == "case_c_fopdt_standard":
            gt = BenchmarkGroundTruth(
                case_id=case_id,
                description="Standard FOPDT with distinct delay and lag",
                gain=1.0,
                tau_s=0.040,
                dead_time_s=0.020,
                velocity_max_rad_s=None,
                noise_std_rad=0.0,
                is_informative_excitation=True,
            )
        elif case_id == "case_d_fopdt_rate_limited":
            gt = BenchmarkGroundTruth(
                case_id=case_id,
                description="FOPDT with Velocity Rate Limiting",
                gain=1.0,
                tau_s=0.040,
                dead_time_s=0.020,
                velocity_max_rad_s=2.0,
                noise_std_rad=0.0,
                is_informative_excitation=True,
            )
        elif case_id == "case_e_fopdt_noisy":
            gt = BenchmarkGroundTruth(
                case_id=case_id,
                description="FOPDT with Measurement Noise",
                gain=1.0,
                tau_s=0.040,
                dead_time_s=0.020,
                velocity_max_rad_s=None,
                noise_std_rad=0.005,
                is_informative_excitation=True,
            )
        elif case_id == "case_f_saturation":
            gt = BenchmarkGroundTruth(
                case_id=case_id,
                description="Severe Velocity Saturation",
                gain=1.0,
                tau_s=0.030,
                dead_time_s=0.015,
                velocity_max_rad_s=1.0,
                noise_std_rad=0.0,
                is_informative_excitation=True,
            )
        elif case_id == "case_g_uninformative":
            gt = BenchmarkGroundTruth(
                case_id=case_id,
                description="Uninformative static excitation",
                gain=1.0,
                tau_s=0.040,
                dead_time_s=0.020,
                velocity_max_rad_s=None,
                noise_std_rad=0.0,
                is_informative_excitation=False,
            )
        else:
            raise ValueError(f"Unknown benchmark case: {case_id}")

        # Simulate physics with unconditionally stable discrete ZOH
        u_delayed = _delay_signal(cmd, dt_s, gt.dead_time_s)
        y_true = np.zeros(n, dtype=np.float64)
        curr_y = 0.0
        alpha = float(np.exp(-dt_s / max(1e-5, gt.tau_s)))

        for i in range(n):
            y_true[i] = curr_y
            target = gt.gain * u_delayed[i]
            delta_ideal = (1.0 - alpha) * (target - curr_y)

            if gt.velocity_max_rad_s is not None:
                max_step = gt.velocity_max_rad_s * dt_s
                delta_clamped = float(np.clip(delta_ideal, -max_step, max_step))
            else:
                delta_clamped = delta_ideal

            curr_y += delta_clamped

        # Add noise if configured
        if gt.noise_std_rad > 0:
            noise = self.rng.normal(0.0, gt.noise_std_rad, size=n)
            y_meas = y_true + noise
        else:
            y_meas = y_true.copy()

        # Build records
        records = []
        for i in range(n):
            records.append({
                "sequence_id": i + 1,
                "timestamp_ns": int(t[i] * 1e9),
                "command_position_rad": float(cmd[i]),
                "measured_motor_position_rad": float(y_meas[i]),
                "sample_period_ms": round(dt_s * 1000.0, 3),
                "configured_simulated_dead_time_ms": round(gt.dead_time_s * 1000.0, 3),
                "configured_simulated_delay_ms": round(gt.dead_time_s * 1000.0, 3),
                "data_source": "synthetic",
                "trajectory_type": trajectory_type,
            })

        return records, gt

    def generate_custom_dataset(
        self,
        gain: float = 1.0,
        tau_s: float = 0.040,
        dead_time_s: float = 0.020,
        velocity_max_rad_s: float | None = None,
        noise_std_rad: float = 0.0,
        trajectory_type: str = "step",
        duration_s: float = 2.0,
        sample_rate_hz: float = 100.0,
    ) -> tuple[list[dict[str, Any]], BenchmarkGroundTruth]:
        """Generate telemetry dataset with custom physical parameters and trajectory shape."""
        dt_s = 1.0 / sample_rate_hz
        t, cmd = self.generate_trajectory(trajectory_type, duration_s, dt_s)
        n = len(t)

        gt = BenchmarkGroundTruth(
            case_id=f"custom_th{int(round(dead_time_s*1000))}_tau{int(round(tau_s*1000))}",
            description=f"Custom FOPDT: theta={dead_time_s*1000:.1f}ms, tau={tau_s*1000:.1f}ms",
            gain=gain,
            tau_s=tau_s,
            dead_time_s=dead_time_s,
            velocity_max_rad_s=velocity_max_rad_s,
            noise_std_rad=noise_std_rad,
            is_informative_excitation=(trajectory_type != "constant_uninformative"),
        )

        u_delayed = _delay_signal(cmd, dt_s, gt.dead_time_s, initial_val=0.0)
        y_true = np.zeros(n, dtype=np.float64)
        curr_y = 0.0
        alpha = float(np.exp(-dt_s / max(1e-5, gt.tau_s)))

        for i in range(n):
            y_true[i] = curr_y
            target = gt.gain * u_delayed[i]
            delta_ideal = (1.0 - alpha) * (target - curr_y)
            if gt.velocity_max_rad_s is not None:
                max_step = gt.velocity_max_rad_s * dt_s
                delta_clamped = float(np.clip(delta_ideal, -max_step, max_step))
            else:
                delta_clamped = delta_ideal
            curr_y += delta_clamped

        if gt.noise_std_rad > 0:
            noise = self.rng.normal(0.0, gt.noise_std_rad, size=n)
            y_meas = y_true + noise
        else:
            y_meas = y_true.copy()

        records = []
        for i in range(n):
            records.append({
                "sequence_id": i + 1,
                "timestamp_ns": int(t[i] * 1e9),
                "command_position_rad": float(cmd[i]),
                "measured_motor_position_rad": float(y_meas[i]),
                "sample_period_ms": round(dt_s * 1000.0, 3),
                "configured_simulated_dead_time_ms": round(gt.dead_time_s * 1000.0, 3),
                "configured_simulated_delay_ms": round(gt.dead_time_s * 1000.0, 3),
                "data_source": "synthetic",
                "trajectory_type": trajectory_type,
            })

        return records, gt


def evaluate_model_on_held_out(
    model: Any,
    eval_records: list[dict[str, Any]],
) -> dict[str, float]:
    """Evaluate fitted model on independent, held-out evaluation trajectory."""
    cmd = np.array([r["command_position_rad"] for r in eval_records], dtype=np.float64)
    meas = np.array([r["measured_motor_position_rad"] for r in eval_records], dtype=np.float64)
    dt_s = 0.01
    if len(eval_records) > 1:
        ts = np.array([r["timestamp_ns"] for r in eval_records], dtype=np.float64)
        dt_s = float(np.median(np.diff(ts)) / 1e9)

    if hasattr(model, "simulate"):
        y_pred = model.simulate(cmd, dt_s, initial_position_rad=float(meas[0]))
    else:
        # Fallback for FixedDelay / Nominal
        y_pred = np.zeros_like(cmd)
        curr_state = {"position_rad": float(meas[0]), "dt_s": dt_s}
        for i in range(len(cmd)):
            from ears_rl.core.types import JointCommand
            p_res = model.predict(JointCommand(joint_id="j0", position_rad=float(cmd[i])), curr_state)
            y_pred[i] = p_res["predicted_position_rad"]
            curr_state["position_rad"] = y_pred[i]

    diff = y_pred - meas
    mae = float(np.mean(np.abs(diff)))
    rmse = float(np.sqrt(np.mean(diff ** 2)))
    max_err = float(np.max(np.abs(diff)))

    return {
        "held_out_mae_rad": round(mae, 5),
        "held_out_rmse_rad": round(rmse, 5),
        "held_out_max_error_rad": round(max_err, 5),
    }


def run_synthetic_identification_benchmark(
    output_dir: Path | str | None = None,
    random_seed: int = 42,
) -> dict[str, Any]:
    """Execute complete synthetic benchmark across Cases A-G, comparing 4 model tiers."""
    generator = SyntheticBenchmarkGenerator(random_seed=random_seed)
    cases = [
        "case_a_pure_delay",
        "case_b_first_order",
        "case_c_fopdt_standard",
        "case_d_fopdt_rate_limited",
        "case_e_fopdt_noisy",
        "case_f_saturation",
        "case_g_uninformative",
    ]

    benchmark_results: dict[str, Any] = {
        "benchmark_version": "1.2.0",
        "random_seed": random_seed,
        "data_source": "synthetic",
        "provenance": "SyntheticActuatorBenchmarkGenerator",
        "cases": {},
    }

    for case_id in cases:
        traj_train = "constant_uninformative" if case_id == "case_g_uninformative" else "chirp"
        traj_eval = "step"

        train_records, gt = generator.generate_dataset(
            case_id=case_id, trajectory_type=traj_train, duration_s=2.0
        )
        eval_records, _ = generator.generate_dataset(
            case_id=case_id, trajectory_type=traj_eval, duration_s=2.0
        )

        case_entry: dict[str, Any] = {
            "ground_truth": asdict(gt),
            "train_trajectory": traj_train,
            "eval_trajectory": traj_eval,
            "sample_count_train": len(train_records),
            "sample_count_eval": len(eval_records),
            "models": {},
        }

        # 1. Fit Nominal Model
        nom_model = NominalActuatorModel()
        nom_fit = nom_model.fit(train_records)
        nom_eval = evaluate_model_on_held_out(nom_model, eval_records)
        case_entry["models"]["nominal"] = {
            "fit_metrics": nom_fit,
            "eval_metrics": nom_eval,
        }

        # 2. Fit Fixed-Delay Model
        fixed_delay = FixedDelayActuatorModel()
        fixed_fit = fixed_delay.fit(train_records)
        fixed_eval = evaluate_model_on_held_out(fixed_delay, eval_records)
        case_entry["models"]["fixed_delay"] = {
            "fit_metrics": fixed_fit,
            "eval_metrics": fixed_eval,
            "identified_apparent_delay_ms": fixed_fit.get("identified_apparent_delay_ms"),
        }

        # 3. Fit First-Order Model
        fo_model = FirstOrderActuatorModel()
        fo_fit = fo_model.fit(train_records)
        fo_eval = evaluate_model_on_held_out(fo_model, eval_records) if fo_fit.get("is_identifiable") else None
        case_entry["models"]["first_order"] = {
            "fit_metrics": fo_fit,
            "eval_metrics": fo_eval,
        }

        # 4. Fit FOPDT Model
        fopdt_model = FOPDTActuatorModel()
        fopdt_fit = fopdt_model.fit(train_records)
        fopdt_eval = evaluate_model_on_held_out(fopdt_model, eval_records) if fopdt_fit.get("is_identifiable") else None

        # Compute parameter recovery errors if ground truth exists and model is identifiable
        param_recovery = None
        if fopdt_fit.get("is_identifiable") and gt.is_informative_excitation:
            id_k = fopdt_fit["identified_gain"]
            id_tau_s = fopdt_fit["identified_time_constant_s"]
            id_theta_s = fopdt_fit["identified_dead_time_s"]

            param_recovery = {
                "gain_error": round(abs(id_k - gt.gain), 4),
                "tau_error_s": round(abs(id_tau_s - gt.tau_s), 5),
                "tau_error_ms": round(abs(id_tau_s - gt.tau_s) * 1000.0, 2),
                "dead_time_error_s": round(abs(id_theta_s - gt.dead_time_s), 5),
                "dead_time_error_ms": round(abs(id_theta_s - gt.dead_time_s) * 1000.0, 2),
            }

        case_entry["models"]["fopdt"] = {
            "fit_metrics": fopdt_fit,
            "eval_metrics": fopdt_eval,
            "parameter_recovery": param_recovery,
        }

        # 5. Fit FOPDT Rate-Limited Model
        fopdt_rl = FOPDTRateLimitedActuatorModel(velocity_max_rad_s=gt.velocity_max_rad_s or 6.28)
        fopdt_rl_fit = fopdt_rl.fit(train_records)
        fopdt_rl_eval = evaluate_model_on_held_out(fopdt_rl, eval_records) if fopdt_rl_fit.get("is_identifiable") else None
        case_entry["models"]["fopdt_rate_limited"] = {
            "fit_metrics": fopdt_rl_fit,
            "eval_metrics": fopdt_rl_eval,
        }

        benchmark_results["cases"][case_id] = case_entry

    if output_dir is not None:
        out_p = Path(output_dir)
        out_p.mkdir(parents=True, exist_ok=True)
        import json
        with open(out_p / "synthetic_benchmark_report.json", "w", encoding="utf-8") as f:
            json.dump(benchmark_results, f, indent=2)
        logger.info("Synthetic benchmark report saved to %s", out_p / "synthetic_benchmark_report.json")

    return benchmark_results


def run_dead_time_recovery_matrix(
    theta_list_ms: list[float] | None = None,
    tau_list_ms: list[float] | None = None,
    noise_list_rad: list[float] | None = None,
    trajectories: list[str] | None = None,
    random_seed: int = 42,
) -> list[dict[str, Any]]:
    """Evaluate FOPDT parameter recovery across a comprehensive ground-truth test matrix.

    Varies:
        - dead time theta in [0, 10, 20, 40] ms
        - time constant tau in [20, 50] ms
        - noise sigma in [0.0, 0.005] rad
        - trajectory shape in ['step', 'prbs', 'chirp']
    """
    if theta_list_ms is None:
        theta_list_ms = [0.0, 10.0, 20.0, 40.0]
    if tau_list_ms is None:
        tau_list_ms = [20.0, 50.0]
    if noise_list_rad is None:
        noise_list_rad = [0.0, 0.005]
    if trajectories is None:
        trajectories = ["step", "prbs", "chirp"]

    generator = SyntheticBenchmarkGenerator(random_seed=random_seed)
    matrix_results = []

    for th_ms in theta_list_ms:
        for tau_ms in tau_list_ms:
            for noise_rad in noise_list_rad:
                for traj in trajectories:
                    records, _ = generator.generate_custom_dataset(
                        gain=1.0,
                        tau_s=tau_ms / 1000.0,
                        dead_time_s=th_ms / 1000.0,
                        noise_std_rad=noise_rad,
                        trajectory_type=traj,
                        duration_s=2.0,
                    )

                    model = FOPDTActuatorModel()
                    fit_res = model.fit(records)

                    if fit_res.get("is_identifiable"):
                        id_theta_ms = fit_res["identified_dead_time_ms"]
                        id_tau_ms = fit_res["identified_time_constant_ms"]
                        err_theta_ms = abs(id_theta_ms - th_ms)
                        err_tau_ms = abs(id_tau_ms - tau_ms)
                    else:
                        id_theta_ms = None
                        id_tau_ms = None
                        err_theta_ms = None
                        err_tau_ms = None

                    matrix_results.append({
                        "gt_theta_ms": th_ms,
                        "gt_tau_ms": tau_ms,
                        "noise_std_rad": noise_rad,
                        "trajectory": traj,
                        "is_identifiable": fit_res.get("is_identifiable", False),
                        "identified_theta_ms": id_theta_ms,
                        "identified_tau_ms": id_tau_ms,
                        "theta_error_ms": round(err_theta_ms, 2) if err_theta_ms is not None else None,
                        "tau_error_ms": round(err_tau_ms, 2) if err_tau_ms is not None else None,
                        "residual_rmse_rad": fit_res.get("residual_rmse_rad"),
                    })

    return matrix_results
