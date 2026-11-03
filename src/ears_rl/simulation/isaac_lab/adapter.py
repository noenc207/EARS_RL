"""NVIDIA Isaac Lab adapter module for EARS-RL.

MANDATORY ISOLATION PRINCIPLE:
This module is strictly isolated from core dependencies.
It raises IsaacLabNotAvailableError if Isaac Sim/Lab is not installed,
ensuring unit tests, CLI commands, and sysID work on standard CPU machines.
"""

from __future__ import annotations

import logging
from typing import Any

from ears_rl.core.exceptions import IsaacLabNotAvailableError
from ears_rl.simulation.base import BaseSimulationEnvironment

logger = logging.getLogger(__name__)


def check_isaac_lab_available() -> tuple[bool, str]:
    """Check whether NVIDIA Isaac Lab is available in the current Python environment."""
    try:
        import omni.isaac.lab  # type: ignore # noqa: F401
        return True, "NVIDIA Isaac Lab is available."
    except ImportError as e:
        return (
            False,
            f"NVIDIA Isaac Lab is not available: {e}\n"
            "To install, please follow NVIDIA Isaac Lab documentation:\n"
            "https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html"
        )


class IsaacLabAdapter(BaseSimulationEnvironment):
    """Adapter bridging EARS-RL simulation interface to Isaac Lab environments."""

    def __init__(self, task_name: str = "Isaac-Biped-v0", headless: bool = True) -> None:
        super().__init__(backend_name="isaac_lab")
        available, msg = check_isaac_lab_available()
        if not available:
            raise IsaacLabNotAvailableError(msg)

        self.task_name = task_name
        self.headless = headless
        # When available, initialize Isaac Lab environment here
        logger.info("Initialized Isaac Lab adapter for task '%s'", task_name)

    def reset(self, seed: int | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
        raise NotImplementedError("Isaac Lab environment reset requires active Isaac Sim session.")

    def step(self, action: Any) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        raise NotImplementedError("Isaac Lab environment step requires active Isaac Sim session.")

    def get_observation_space(self) -> dict[str, Any]:
        return {"shape": [36], "dtype": "float32"}

    def get_action_space(self) -> dict[str, Any]:
        return {"shape": [6], "dtype": "float32", "low": -1.0, "high": 1.0}
