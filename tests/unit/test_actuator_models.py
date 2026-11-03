"""Unit tests for Nominal, Fixed Delay, and Hybrid actuator response models."""

import numpy as np
import pytest

from ears_rl.actuator_model.evaluation import (
    compute_event_classification_metrics,
    compute_regression_metrics,
)
from ears_rl.actuator_model.fixed_delay import FixedDelayActuatorModel
from ears_rl.actuator_model.hybrid import HybridEventAwareActuatorModel
from ears_rl.actuator_model.nominal import NominalActuatorModel
from ears_rl.core.types import ActuatorState, JointCommand


def test_nominal_model_prediction():
    """Verify nominal model predicts commanded target directly."""
    model = NominalActuatorModel(velocity_max_rad_s=5.0)
    cmd = JointCommand(joint_id="j0", position_rad=1.5)
    pred = model.predict(cmd, {"position_rad": 0.0, "dt_s": 0.01})
    assert pred["predicted_position_rad"] == 1.5
    assert pred["predicted_delay_ms"] == 0.0


def test_fixed_delay_model_history():
    """Verify fixed-delay model retrieves command delayed by tau_d."""
    model = FixedDelayActuatorModel(delay_ms=20.0)

    # Dispatch command 1 at t=0ms
    model.predict(JointCommand(joint_id="j0", position_rad=1.0), {"timestamp_ns": 0, "position_rad": 0.0})

    # Dispatch command 2 at t=10ms
    model.predict(JointCommand(joint_id="j0", position_rad=2.0), {"timestamp_ns": 10_000_000, "position_rad": 0.0})

    # Query at t=25ms -> should retrieve command from t=0ms (1.0 rad)
    res = model.predict(JointCommand(joint_id="j0", position_rad=3.0), {"timestamp_ns": 25_000_000, "position_rad": 0.0})
    assert res["predicted_position_rad"] == 1.0


def test_hybrid_model_state_transition():
    """Verify hybrid model accounts for discrete step loss slip."""
    model = HybridEventAwareActuatorModel()
    model.update_discrete_state(ActuatorState.STEP_LOSS, slip_rad=0.30)
    cmd = JointCommand(joint_id="j0", position_rad=1.0)
    res = model.predict(cmd, {"position_rad": 0.0})
    assert pytest.approx(res["predicted_position_rad"], rel=1e-5) == 1.30


def test_regression_metrics_computation():
    """Verify MAE and RMSE computation."""
    y_true = np.array([1.0, 2.0, 3.0])
    y_pred = np.array([1.1, 1.9, 3.2])
    metrics = compute_regression_metrics(y_true, y_pred)
    assert pytest.approx(metrics["mae"], rel=1e-3) == 0.1333
    assert metrics["rmse"] > 0.0


def test_event_classification_metrics():
    """Verify precision, recall, and false alarm rate computation."""
    yt = ["normal", "normal", "step_loss", "normal"]
    yp = ["normal", "step_loss", "step_loss", "normal"]
    m = compute_event_classification_metrics(yt, yp, target_event="step_loss")
    assert m["true_positives"] == 1
    assert m["false_positives"] == 1
    assert m["precision"] == 0.5
    assert m["recall"] == 1.0
