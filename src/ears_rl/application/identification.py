"""System identification application workflow.

Orchestrates baseline response modeling (Nominal and Fixed Delay models)
with dataset validation, timing taxonomy audit, and reporting.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from ears_rl.config.schema import IdentificationConfig
from ears_rl.identification.baseline import run_baseline_identification

logger = logging.getLogger(__name__)


class RunIdentificationWorkflow:
    """Application use case for running baseline actuator system identification."""

    def __init__(
        self,
        config: IdentificationConfig | None = None,
        output_dir: Path | str | None = None,
    ) -> None:
        self.config = config
        self.output_dir = Path(output_dir) if output_dir is not None else None

    def execute(self, dataset_path: Path | str) -> dict[str, Any]:
        """Execute baseline system identification on dataset."""
        logger.info("Executing system identification workflow on %s", dataset_path)
        return run_baseline_identification(
            dataset_path=dataset_path,
            config=self.config,
            output_dir=self.output_dir,
        )
