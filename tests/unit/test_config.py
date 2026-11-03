"""Unit tests for configuration validation and hashing."""

from pathlib import Path

import pytest
from pydantic import ValidationError

from ears_rl.config.loader import compute_config_hash, load_experiment_config
from ears_rl.config.schema import ActuatorConfig, ExperimentConfig, SafetyConfig


def test_safety_config_defaults():
    """Verify hardware_enabled is False by default."""
    safety = SafetyConfig()
    assert safety.hardware_enabled is False
    assert safety.motion_enabled_on_startup is False


def test_actuator_config_invalid_type():
    """Verify actuator_type rejects unlisted values."""
    with pytest.raises(ValidationError):
        ActuatorConfig(joint_id="j0", actuator_type="nuclear_reactor")


def test_load_sample_configs():
    """Verify existing sample YAML configs pass validation."""
    mock_cfg_path = Path("configs/hardware/bench_mock.yaml")
    assert mock_cfg_path.exists()
    cfg = load_experiment_config(mock_cfg_path)
    assert cfg.experiment_id == "bench_mock_001"
    assert cfg.safety.hardware_enabled is False
    assert len(cfg.actuators) > 0


def test_config_hash_determinism():
    """Verify compute_config_hash returns identical SHA-256 for identical configs."""
    cfg1 = ExperimentConfig(experiment_id="test_exp")
    cfg2 = ExperimentConfig(experiment_id="test_exp")
    assert compute_config_hash(cfg1) == compute_config_hash(cfg2)
    assert len(compute_config_hash(cfg1)) == 64  # SHA-256 hex string length


def test_actuator_config_invalid_kinematic_limits():
    """Verify kinematic bounds validation rejects inverted or negative limits."""
    # Min > Max
    with pytest.raises(ValidationError, match="cannot be greater than"):
        ActuatorConfig(joint_id="j0", position_min_rad=1.0, position_max_rad=-1.0)

    # Negative velocity
    with pytest.raises(ValidationError, match="cannot be negative"):
        ActuatorConfig(joint_id="j0", velocity_max_rad_s=-5.0)

    # Negative acceleration
    with pytest.raises(ValidationError, match="cannot be negative"):
        ActuatorConfig(joint_id="j0", acceleration_max_rad_s2=-10.0)

