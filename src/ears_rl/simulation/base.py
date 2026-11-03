"""Base simulation environment class for EARS-RL."""

from __future__ import annotations

from ears_rl.core.interfaces import SimulationEnvironmentInterface


class BaseSimulationEnvironment(SimulationEnvironmentInterface):
    """Base class for simulation environments (Isaac Lab, PyBullet, Mock)."""

    def __init__(self, backend_name: str = "base") -> None:
        self.backend_name = backend_name
        self._is_closed = False

    def close(self) -> None:
        self._is_closed = True
