"""Baseline system identification runner for EARS-RL."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

from ears_rl.actuator_model.evaluation import evaluate_dataset_tracking
from ears_rl.actuator_model.first_order import FirstOrderActuatorModel
from ears_rl.actuator_model.fixed_delay import FixedDelayActuatorModel
from ears_rl.actuator_model.fopdt import (
    FOPDTActuatorModel,
    FOPDTRateLimitedActuatorModel,
)
from ears_rl.actuator_model.nominal import NominalActuatorModel
from ears_rl.config.schema import IdentificationConfig
from ears_rl.core.exceptions import DataError
from ears_rl.data.validation import validate_dataset

logger = logging.getLogger(__name__)


def run_baseline_identification(
    dataset_path: Path | str,
    config: IdentificationConfig | None = None,
    output_dir: Path | str | None = None,
) -> dict[str, Any]:
    """Run baseline system identification (Nominal and Fixed Delay models) on a bench dataset.

    Args:
        dataset_path: Path to dataset .csv or .jsonl.
        config: Optional identification hyperparameters.
        output_dir: Directory where identification report will be saved.

    Returns:
        Dictionary containing identified parameters, metrics, and dataset stats.
    """
    path = Path(dataset_path)
    if not path.exists():
        raise DataError(f"Dataset not found: {path}")

    # 1. Validate dataset first
    val_report = validate_dataset(path)
    if not val_report.is_valid:
        raise DataError(f"Cannot identify from invalid dataset:\n{val_report.summary()}")

    def _safe_cast(val: Any) -> Any:
        if val is None or val == "":
            return None
        try:
            return int(val)
        except ValueError:
            try:
                return float(val)
            except ValueError:
                return val

    # 2. Load records
    import csv
    records: list[dict[str, Any]] = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append({k: _safe_cast(v) for k, v in row.items()})

    if not records:
        raise DataError("Dataset contains zero records")

    # Warn if synthetic data is used
    first_record = records[0]
    data_source = first_record.get("data_source", "unknown")
    if data_source == "synthetic":
        logger.warning(
            "SCIENTIFIC NOTICE: Identification is running on SYNTHETIC data from MockActuator."
        )

    # 3. Compute baseline tracking and timing metrics
    baseline_metrics = evaluate_dataset_tracking(records)

    # 4. Fit FixedDelayActuatorModel via discrete cross-correlation (apparent delay)
    fixed_delay_model = FixedDelayActuatorModel(delay_ms=10.0)
    fit_res = fixed_delay_model.fit(records)

    # 5. Fit Nominal model (zero-hypothesis baseline predictor)
    nominal_model = NominalActuatorModel()
    nom_res = nominal_model.fit(records)

    # 6. Fit FirstOrderActuatorModel (Grey-Box without dead time)
    first_order_model = FirstOrderActuatorModel()
    fo_res = first_order_model.fit(records)

    # 7. Fit FOPDTActuatorModel (Grey-Box with Dead Time and Time Constant)
    fopdt_model = FOPDTActuatorModel()
    fopdt_res = fopdt_model.fit(records)

    # 8. Fit FOPDTRateLimitedActuatorModel (Grey-Box with Velocity Limiting)
    fopdt_rl_model = FOPDTRateLimitedActuatorModel()
    fopdt_rl_res = fopdt_rl_model.fit(records)

    pos_mae = baseline_metrics["position_tracking"]["mae_rad"]
    pos_rmse = baseline_metrics["position_tracking"]["rmse_rad"]

    # Extract timing metrics
    timing_metrics = baseline_metrics.get("timing_metrics", {})
    sample_period_info = timing_metrics.get("sample_period") or {}
    sample_period_ms = sample_period_info.get("mean_ms", 10.0)
    configured_delay_ms = timing_metrics.get("configured_simulated_delay_ms")

    # Fixed-delay reports apparent lumped delay
    apparent_delay_ms = fit_res.get("identified_apparent_delay_ms", fit_res.get("identified_delay_ms", 0.0))

    # FOPDT separates pure dead time theta from dynamic lag tau
    fopdt_dead_time_ms = fopdt_res.get("identified_dead_time_ms")
    fopdt_tau_ms = fopdt_res.get("identified_time_constant_ms")
    fopdt_gain = fopdt_res.get("identified_gain")

    dead_time_error_ms = None
    if configured_delay_ms is not None and fopdt_dead_time_ms is not None:
        dead_time_error_ms = round(abs(fopdt_dead_time_ms - configured_delay_ms), 3)

    apparent_gap_ms = None
    if configured_delay_ms is not None and apparent_delay_ms is not None:
        apparent_gap_ms = round(abs(apparent_delay_ms - configured_delay_ms), 3)

    results: dict[str, Any] = {
        "dataset_path": str(path),
        "data_source": data_source,
        "sample_count": len(records),
        "provenance_classification": {
            "data_source": data_source,
            "is_real_hardware": data_source == "real_hardware",
            "is_synthetic_bench": data_source in ("synthetic", "simulated"),
            "scientific_disclaimer": (
                "REAL HARDWARE MEASUREMENT: Experimental data recorded from physical sensor bench."
                if data_source == "real_hardware"
                else "SYNTHETIC BENCHMARK DATA: Results represent numerical simulation and must not be conflated with physical robot hardware experiments."
            ),
        },
        "ground_truth_parameters": {
            "configured_simulated_dead_time_ms": configured_delay_ms,
            "configured_simulated_tau_ms": first_record.get("configured_simulated_tau_ms"),
            "sample_period_ms": sample_period_ms,
        },
        "estimated_parameters": {
            "gain": fopdt_gain,
            "time_constant_ms": fopdt_tau_ms,
            "dead_time_ms": fopdt_dead_time_ms,
            "velocity_max_rad_s": fopdt_rl_res.get("identified_velocity_max_rad_s"),
        },
        "structural_mismatch_audit": {
            "is_detected": fopdt_res.get("structural_mismatch", False),
            "warning_details": fopdt_res.get("structural_mismatch_warning"),
        },
        "discretization_uncertainty_bound_ms": sample_period_ms,
        "nominal_model": {
            "type": "nominal",
            "role": "zero_hypothesis_baseline_predictor",
            "fit_metrics": nom_res,
            "prediction_errors": {
                "position_mae_rad": pos_mae,
                "position_rmse_rad": pos_rmse,
            },
        },
        "fixed_delay_model": {
            "type": "fixed_delay",
            "role": "pure_delay_apparent_model",
            "identified_delay_ms": apparent_delay_ms,
            "identified_apparent_delay_ms": apparent_delay_ms,
            "best_shift_steps": fit_res.get("best_shift_steps"),
            "residual_rmse_rad": fit_res.get("residual_rmse_rad"),
            "nominal_rmse_rad": fit_res.get("nominal_rmse_rad"),
            "rmse_reduction_rad": fit_res.get("rmse_reduction_rad"),
            "fit_metrics": fit_res,
        },
        "first_order_model": {
            "type": "first_order",
            "role": "grey_box_lag_without_dead_time",
            "fit_metrics": fo_res,
        },
        "fopdt_model": {
            "type": "fopdt",
            "role": "grey_box_first_order_plus_dead_time",
            "identified_dead_time_ms": fopdt_dead_time_ms,
            "identified_time_constant_ms": fopdt_tau_ms,
            "identified_gain": fopdt_gain,
            "dead_time_error_ms": dead_time_error_ms,
            "residual_rmse_rad": fopdt_res.get("residual_rmse_rad"),
            "fit_metrics": fopdt_res,
        },
        "fopdt_rate_limited_model": {
            "type": "fopdt_rate_limited",
            "role": "grey_box_fopdt_with_velocity_saturation",
            "fit_metrics": fopdt_rl_res,
        },
        "timing_and_latency_audit": {
            "sample_period_ms": sample_period_ms,
            "configured_simulated_delay_ms": configured_delay_ms,
            "configured_simulated_dead_time_ms": configured_delay_ms,
            "identified_pure_delay_ms": apparent_delay_ms,
            "identified_apparent_delay_ms": apparent_delay_ms,
            "identified_dead_time_ms": fopdt_dead_time_ms,
            "identified_time_constant_ms": fopdt_tau_ms,
            "dead_time_estimation_error_ms": dead_time_error_ms,
            "apparent_delay_vs_configured_gap_ms": apparent_gap_ms,
            "delay_estimation_error_ms": apparent_gap_ms,
            "metrics_derivation_explanation": {
                "sample_period_ms": "Derived from consecutive timestamp_ns differences (100 Hz = 10.0 ms).",
                "configured_simulated_dead_time_ms": "Configured pure transport dead time in simulation/mock actuator.",
                "identified_dead_time_ms": "Pure transport dead time theta isolated by FOPDT grey-box model.",
                "identified_apparent_delay_ms": "Lumped delay (theta + tau) identified by zero-order shift minimization.",
                "command_to_feedback_delay_ms": "Round-trip communication packet RTT (unmeasured in synchronous mock loop).",
            },
        },
        "empirical_tracking": baseline_metrics,
        "model_comparison": [
            {
                "model": "Nominal (Zero-Hypothesis)",
                "parameters": 0,
                "apparent_delay_ms": 0.0,
                "dead_time_ms": None,
                "time_constant_ms": None,
                "rmse_rad": pos_rmse,
            },
            {
                "model": "Fixed Delay (Apparent Shift)",
                "parameters": 1,
                "apparent_delay_ms": apparent_delay_ms,
                "dead_time_ms": None,
                "time_constant_ms": None,
                "rmse_rad": fit_res.get("residual_rmse_rad"),
            },
            {
                "model": "First Order (No Delay)",
                "parameters": 2,
                "apparent_delay_ms": fo_res.get("identified_time_constant_ms"),
                "dead_time_ms": 0.0,
                "time_constant_ms": fo_res.get("identified_time_constant_ms"),
                "rmse_rad": fo_res.get("residual_rmse_rad"),
            },
            {
                "model": "FOPDT (Grey-Box)",
                "parameters": 3,
                "apparent_delay_ms": (
                    (fopdt_dead_time_ms + fopdt_tau_ms)
                    if (fopdt_dead_time_ms is not None and fopdt_tau_ms is not None)
                    else None
                ),
                "dead_time_ms": fopdt_dead_time_ms,
                "time_constant_ms": fopdt_tau_ms,
                "rmse_rad": fopdt_res.get("residual_rmse_rad"),
            },
            {
                "model": "FOPDT + Rate Limited",
                "parameters": 4,
                "apparent_delay_ms": (
                    (fopdt_dead_time_ms + fopdt_tau_ms)
                    if (fopdt_dead_time_ms is not None and fopdt_tau_ms is not None)
                    else None
                ),
                "dead_time_ms": fopdt_dead_time_ms,
                "time_constant_ms": fopdt_tau_ms,
                "rmse_rad": fopdt_rl_res.get("residual_rmse_rad"),
            },
        ],
    }

    # 6. Save results to output_dir if specified
    if output_dir is not None:
        out_p = Path(output_dir)
        out_p.mkdir(parents=True, exist_ok=True)
        report_file = out_p / "identification_report.json"
        with open(report_file, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2)
        logger.info("Saved identification report to %s", report_file)

    return results
