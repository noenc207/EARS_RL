"""Actuator tracking and latency evaluation application workflow."""

from __future__ import annotations

import csv
import logging
from pathlib import Path
from typing import Any

from ears_rl.actuator_model.evaluation import evaluate_dataset_tracking
from ears_rl.core.exceptions import DataError

logger = logging.getLogger(__name__)


class RunEvaluationWorkflow:
    """Application use case for evaluating dataset tracking performance."""

    @staticmethod
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

    def execute(self, dataset_path: Path | str) -> dict[str, Any]:
        """Load dataset and compute tracking accuracy and latency metrics."""
        path = Path(dataset_path)
        if not path.exists():
            raise DataError(f"Dataset file not found: {path}")

        records: list[dict[str, Any]] = []
        with open(path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                records.append({k: self._safe_cast(v) for k, v in row.items()})

        if not records:
            raise DataError(f"Dataset at {path} contains zero records")

        logger.info("Evaluating tracking performance across %d records from %s", len(records), path)
        metrics = evaluate_dataset_tracking(records)
        metrics["dataset_path"] = str(path)
        return metrics
