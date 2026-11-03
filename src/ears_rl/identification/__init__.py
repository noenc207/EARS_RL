"""EARS-RL System Identification Package."""

from ears_rl.identification.baseline import run_baseline_identification
from ears_rl.identification.synthetic_bench import (
    BenchmarkGroundTruth,
    SyntheticBenchmarkGenerator,
    run_synthetic_identification_benchmark,
)

__all__ = [
    "BenchmarkGroundTruth",
    "SyntheticBenchmarkGenerator",
    "run_baseline_identification",
    "run_synthetic_identification_benchmark",
]
