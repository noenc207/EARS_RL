"""Actuator model evaluation and scientific metrics computation for EARS-RL."""

from __future__ import annotations

import logging
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


def compute_regression_metrics(
    y_true: np.ndarray, y_pred: np.ndarray
) -> dict[str, float]:
    """Compute MAE, RMSE, Max Error, and residual mean/std."""
    if len(y_true) == 0 or len(y_pred) == 0:
        return {"mae": float("nan"), "rmse": float("nan"), "max_error": float("nan")}

    mask = np.isfinite(y_true) & np.isfinite(y_pred)
    if not np.any(mask):
        return {"mae": float("nan"), "rmse": float("nan"), "max_error": float("nan")}

    yt = y_true[mask]
    yp = y_pred[mask]
    residuals = yt - yp
    errors = np.abs(residuals)

    mae = float(np.mean(errors))
    rmse = float(np.sqrt(np.mean(residuals ** 2)))
    max_err = float(np.max(errors))

    return {
        "mae": mae,
        "rmse": rmse,
        "max_error": max_err,
        "mean_residual": float(np.mean(residuals)),
        "std_residual": float(np.std(residuals)),
    }


def compute_event_classification_metrics(
    y_true_events: list[str], y_pred_events: list[str], target_event: str = "step_loss"
) -> dict[str, float]:
    """Compute Precision, Recall, F1, and False Alarm Rate for an event class."""
    if len(y_true_events) != len(y_pred_events) or len(y_true_events) == 0:
        return {"precision": 0.0, "recall": 0.0, "f1": 0.0, "false_alarm_rate": 0.0}

    tp = sum(1 for yt, yp in zip(y_true_events, y_pred_events) if yt == target_event and yp == target_event)
    fp = sum(1 for yt, yp in zip(y_true_events, y_pred_events) if yt != target_event and yp == target_event)
    fn = sum(1 for yt, yp in zip(y_true_events, y_pred_events) if yt == target_event and yp != target_event)
    tn = sum(1 for yt, yp in zip(y_true_events, y_pred_events) if yt != target_event and yp != target_event)

    precision = float(tp / (tp + fp)) if (tp + fp) > 0 else 0.0
    recall = float(tp / (tp + fn)) if (tp + fn) > 0 else 0.0
    f1 = float(2 * precision * recall / (precision + recall)) if (precision + recall) > 0 else 0.0
    far = float(fp / (fp + tn)) if (fp + tn) > 0 else 0.0

    return {
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "false_alarm_rate": far,
        "true_positives": tp,
        "false_positives": fp,
        "false_negatives": fn,
        "true_negatives": tn,
    }


def evaluate_dataset_tracking(
    records: list[dict[str, Any]]
) -> dict[str, Any]:
    """Compute tracking performance and timing metrics across recorded bench session.

    Scientific Integrity:
    - Strictly differentiates sample period vs command-to-feedback delay vs simulated delay.
    - Accurately tracks data provenance ('synthetic', 'simulated', 'real_hardware').
    - Reports unavailable metrics as null with explicit explanation rather than 0.0.
    """
    if not records:
        return {
            "sample_count": 0,
            "data_source": "unknown",
            "position_tracking": {"mae_rad": None, "rmse_rad": None, "max_error_rad": None},
            "velocity_tracking": None,
            "timing_metrics": {
                "sample_period_ms": None,
                "command_to_feedback_delay_ms": None,
                "configured_simulated_delay_ms": None,
            },
            "latency_statistics": {},
            "status": "empty_dataset",
        }

    cmd_pos = []
    meas_pos = []
    cmd_vel = []
    meas_vel = []
    timestamps_ns = []
    cmd_to_fb_delays = []
    simulated_delays = []
    legacy_latencies = []

    data_source = records[0].get("data_source", "unknown") if records else "unknown"

    for r in records:
        ts = r.get("timestamp_ns")
        if ts is not None:
            try:
                timestamps_ns.append(int(ts))
            except (ValueError, TypeError):
                pass

        cp = r.get("command_position_rad")
        mp = r.get("measured_motor_position_rad")
        if cp is not None and mp is not None:
            try:
                cmd_pos.append(float(cp))
                meas_pos.append(float(mp))
            except (ValueError, TypeError):
                pass

        cv = r.get("command_velocity_rad_s")
        mv = r.get("measured_velocity_rad_s")
        if cv is not None and mv is not None:
            try:
                cmd_vel.append(float(cv))
                meas_vel.append(float(mv))
            except (ValueError, TypeError):
                pass

        cfd = r.get("command_to_feedback_delay_ms")
        if cfd is not None:
            try:
                cfd_f = float(cfd)
                if cfd_f >= 0:  # Physical latency must be non-negative
                    cmd_to_fb_delays.append(cfd_f)
            except (ValueError, TypeError):
                pass

        csd = r.get("configured_simulated_delay_ms")
        if csd is not None:
            try:
                simulated_delays.append(float(csd))
            except (ValueError, TypeError):
                pass

        lat = r.get("latency_ms")
        if lat is not None:
            try:
                legacy_latencies.append(float(lat))
            except (ValueError, TypeError):
                pass

    # 1. Position tracking metrics
    pos_metrics = compute_regression_metrics(np.array(meas_pos), np.array(cmd_pos))

    # 2. Velocity tracking metrics (optional mode)
    vel_metrics = None
    if cmd_vel and meas_vel:
        vel_res = compute_regression_metrics(np.array(meas_vel), np.array(cmd_vel))
        vel_metrics = {
            "mae_rad_s": vel_res["mae"],
            "rmse_rad_s": vel_res["rmse"],
            "max_error_rad_s": vel_res["max_error"],
        }

    # 3. Sample period statistics from timestamp deltas
    sample_period_stats = None
    if len(timestamps_ns) >= 2:
        dt_arr = np.diff(np.array(timestamps_ns, dtype=np.float64)) / 1e6
        valid_dt = dt_arr[dt_arr > 0]
        if len(valid_dt) > 0:
            mean_dt = float(np.mean(valid_dt))
            sample_period_stats = {
                "mean_ms": round(mean_dt, 3),
                "std_ms": round(float(np.std(valid_dt)), 3),
                "min_ms": round(float(np.min(valid_dt)), 3),
                "max_ms": round(float(np.max(valid_dt)), 3),
                "derived_sample_rate_hz": round(1000.0 / mean_dt, 2) if mean_dt > 0 else None,
                "data_source_field": "timestamp_ns delta",
            }

    # 4. Command-to-feedback delay statistics
    cmd_feedback_delay_stats = None
    if cmd_to_fb_delays:
        arr = np.array(cmd_to_fb_delays)
        cmd_feedback_delay_stats = {
            "mean_ms": round(float(np.mean(arr)), 3),
            "std_ms": round(float(np.std(arr)), 3),
            "min_ms": round(float(np.min(arr)), 3),
            "max_ms": round(float(np.max(arr)), 3),
            "data_source_field": "command_to_feedback_delay_ms",
        }

    # 5. Configured simulated pure delay
    sim_delay_val = float(simulated_delays[0]) if simulated_delays else None

    # 6. Legacy latency statistics (for backward compatibility)
    latency_stats = {}
    if legacy_latencies:
        lat_arr = np.array(legacy_latencies)
        latency_stats = {
            "mean_ms": round(float(np.mean(lat_arr)), 2),
            "std_ms": round(float(np.std(lat_arr)), 2),
            "p95_ms": round(float(np.percentile(lat_arr, 95)), 2),
            "max_ms": round(float(np.max(lat_arr)), 2),
        }

    return {
        "sample_count": len(records),
        "data_source": data_source,
        "position_tracking": {
            "mae_rad": pos_metrics["mae"],
            "rmse_rad": pos_metrics["rmse"],
            "max_error_rad": pos_metrics["max_error"],
        },
        "velocity_tracking": vel_metrics,
        "timing_metrics": {
            "sample_period": sample_period_stats,
            "command_to_feedback_delay": cmd_feedback_delay_stats,
            "configured_simulated_delay_ms": sim_delay_val,
        },
        "latency_statistics": latency_stats,
    }
