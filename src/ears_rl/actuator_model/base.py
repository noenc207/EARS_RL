"""Base actuator model class for system identification in EARS-RL."""

from __future__ import annotations

from ears_rl.core.exceptions import ModelNotFittedError
from ears_rl.core.interfaces import ActuatorModelInterface


class BaseActuatorModel(ActuatorModelInterface):
    """Base class providing common infrastructure for actuator response models."""

    def __init__(self, model_type: str = "base") -> None:
        self._model_type = model_type
        self._is_fitted: bool = False
        self._parameters: dict[str, float] = {}

    def get_model_type(self) -> str:
        return self._model_type

    def is_fitted(self) -> bool:
        return self._is_fitted

    def _ensure_fitted(self) -> None:
        if not self._is_fitted:
            raise ModelNotFittedError(
                f"{self.__class__.__name__} has not been fitted yet. Call fit() first."
            )

    def get_parameters(self) -> dict[str, float]:
        return dict(self._parameters)
