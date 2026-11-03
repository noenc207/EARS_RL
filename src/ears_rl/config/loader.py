"""Configuration loading, validation, and SHA-256 hashing for EARS-RL."""

from __future__ import annotations

import hashlib
import logging
from pathlib import Path
from typing import Any, TypeVar

import yaml
from pydantic import BaseModel, ValidationError

from ears_rl.config.schema import ExperimentConfig, SystemConfig
from ears_rl.core.exceptions import ConfigurationError, SchemaValidationError

logger = logging.getLogger(__name__)

T = TypeVar("T", bound=BaseModel)


def load_yaml(path: Path | str) -> dict[str, Any]:
    """Load a YAML file into a dictionary."""
    file_path = Path(path)
    if not file_path.exists():
        raise ConfigurationError(f"Configuration file not found: {file_path}")
    if file_path.suffix.lower() not in (".yaml", ".yml"):
        raise ConfigurationError(f"Expected a YAML file, got: {file_path.suffix}")

    try:
        with open(file_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise ConfigurationError(f"Failed to parse YAML in {file_path}: {e}") from e

    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ConfigurationError(f"Root of YAML file {file_path} must be a mapping, got {type(data).__name__}")

    return data


def validate_config(data: dict[str, Any], schema_cls: type[T]) -> T:
    """Validate a raw dictionary against a Pydantic schema class."""
    try:
        return schema_cls(**data)
    except ValidationError as e:
        raise SchemaValidationError(f"Configuration validation failed for {schema_cls.__name__}:\n{e}") from e


def load_experiment_config(path: Path | str) -> ExperimentConfig:
    """Load, parse, and validate an ExperimentConfig."""
    data = load_yaml(path)
    return validate_config(data, ExperimentConfig)


def load_system_config(path: Path | str) -> SystemConfig:
    """Load, parse, and validate a SystemConfig."""
    data = load_yaml(path)
    return validate_config(data, SystemConfig)


def compute_config_hash(config: BaseModel) -> str:
    """Compute deterministic SHA-256 hash of configuration for scientific provenance."""
    canonical_json = config.model_dump_json(indent=None)
    return hashlib.sha256(canonical_json.encode("utf-8")).hexdigest()
