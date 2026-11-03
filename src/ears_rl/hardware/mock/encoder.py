"""Mock angular encoder implementation for hardware-in-the-loop simulation."""

from __future__ import annotations

import numpy as np

from ears_rl.core.clock import TimeProvider
from ears_rl.core.interfaces import EncoderInterface
from ears_rl.core.types import EncoderReading


class MockEncoder(EncoderInterface):
    """Simulated encoder with configurable resolution, quantization, and noise."""

    def __init__(
        self,
        encoder_id: str,
        clock: TimeProvider,
        resolution_counts_per_rev: int = 4096,
        encoder_type: str = "motor_encoder",
        noise_std_rad: float = 0.0005,
        random_seed: int | None = 42,
    ) -> None:
        self.encoder_id = encoder_id
        self._clock = clock
        self.resolution = resolution_counts_per_rev
        self.encoder_type = encoder_type
        self.noise_std_rad = noise_std_rad
        self._position_rad: float = 0.0
        self._sequence_id: int = 0
        self._is_healthy: bool = True
        self._rng = np.random.default_rng(random_seed)

    def set_true_position(self, true_pos_rad: float) -> None:
        """Update the underlying physical shaft position from the actuator simulator."""
        # Quantize to encoder resolution
        step_rad = (2.0 * np.pi) / self.resolution
        quantized = round(true_pos_rad / step_rad) * step_rad

        # Add Gaussian noise
        noise = self._rng.normal(0.0, self.noise_std_rad) if self.noise_std_rad > 0 else 0.0
        self._position_rad = quantized + noise

    def read(self) -> EncoderReading:
        self._sequence_id += 1
        return EncoderReading(
            position_rad=self._position_rad,
            timestamp=self._clock.now(),
            sequence_id=self._sequence_id,
            valid=self._is_healthy,
            encoder_id=self.encoder_id,
            encoder_type=self.encoder_type,
        )

    def read_position(self) -> float:
        return self._position_rad

    def read_status(self) -> bool:
        return self._is_healthy

    def get_position_rad(self) -> float:
        return self._position_rad

    def get_encoder_type(self) -> str:
        return self.encoder_type

    def reset(self) -> None:
        self._position_rad = 0.0
        self._sequence_id = 0
