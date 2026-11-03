"""Benchmark execution application workflow.

Orchestrates simulated or hardware actuator bench sessions with
fail-safe lifecycle wrapping, logging, and manifest generation.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

from ears_rl.application.session import ExperimentSession
from ears_rl.config.schema import ExperimentConfig
from ears_rl.core.clock import TimeProvider
from ears_rl.core.types import DatasetManifest, DataSourceType
from ears_rl.hardware.mock.bench import MockActuatorBench

logger = logging.getLogger(__name__)


class RunBenchmarkWorkflow:
    """Application use case for executing an actuator bench session."""

    def __init__(
        self,
        config: ExperimentConfig,
        output_dir: Path | str,
        clock: TimeProvider | None = None,
        random_seed: int = 42,
    ) -> None:
        self.config = config
        self.output_dir = Path(output_dir)
        self.clock = clock
        self.random_seed = random_seed

    def execute(
        self,
        duration_s: float = 2.0,
        trajectory: str = "chirp",
        sample_rate_hz: float = 100.0,
        inject_fault_at_s: float | None = None,
    ) -> dict[str, Any]:
        """Execute bench test session safely within an ExperimentSession context."""
        bench = MockActuatorBench(
            config=self.config,
            output_dir=self.output_dir,
            clock=self.clock,
            random_seed=self.random_seed,
        )

        with ExperimentSession(
            experiment_id=self.config.experiment_id,
            actuators=[bench.actuator],
            loggers=[bench.logger],
        ):
            data_file = bench.run_session(
                duration_s=duration_s,
                sample_rate_hz=sample_rate_hz,
                trajectory_type=trajectory,
                inject_fault_at_s=inject_fault_at_s,
            )

        # Generate scientific DatasetManifest
        total_records = bench.logger.get_record_count()
        manifest = DatasetManifest(
            experiment_id=self.config.experiment_id,
            session_id="session_001",
            data_source=DataSourceType.SYNTHETIC,
            total_records=total_records,
            duration_s=duration_s,
            data_file_path=str(data_file),
            config_hash=self.config.model_dump().get("software_version", "0.1.0"),
            software_version=self.config.software_version,
            sample_rate_hz_measured=sample_rate_hz,
            missing_value_count=0,
            tags=["benchmark", trajectory, "synthetic"],
        )

        manifest_path = self.output_dir / "manifest.json"
        manifest_dict = manifest.__dict__
        manifest_dict["data_source"] = manifest.data_source.value
        with open(manifest_path, "w", encoding="utf-8") as f:
            json.dump(manifest_dict, f, indent=2)

        return {
            "data_file": data_file,
            "manifest_file": manifest_path,
            "metadata_file": self.output_dir / "metadata.json",
            "total_records": total_records,
            "duration_s": duration_s,
            "trajectory": trajectory,
        }
