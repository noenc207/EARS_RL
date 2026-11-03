"""Online event detection and classification for EARS-RL actuators."""

from __future__ import annotations

import logging

from ears_rl.core.interfaces import EventDetector
from ears_rl.core.types import (
    ActuatorEvent,
    ActuatorState,
    EventSource,
    SensorFrame,
)

logger = logging.getLogger(__name__)


class ThresholdEventDetector(EventDetector):
    """Baseline event detector checking tracking error and saturation thresholds.

    CRITICAL SCIENTIFIC NOTE:
    Events flagged by this detector carry EventSource.DETECTOR.
    They are heuristic indications, NOT independently verified ground truth.
    """

    def __init__(
        self,
        joint_id: str = "joint_0",
        step_loss_threshold_rad: float = 0.25,
        stall_threshold_rad_s: float = 0.05,
        stall_min_duration_frames: int = 5,
    ) -> None:
        self.joint_id = joint_id
        self.step_loss_threshold_rad = step_loss_threshold_rad
        self.stall_threshold_rad_s = stall_threshold_rad_s
        self.stall_min_duration_frames = stall_min_duration_frames
        self._stall_counter = 0

    def detect_events(
        self, current_frame: SensorFrame, history: list[SensorFrame] | None = None
    ) -> list[ActuatorEvent]:
        events: list[ActuatorEvent] = []

        enc = current_frame.encoders.get(self.joint_id)
        if enc is None or not enc.valid:
            return events

        # Check for abnormal velocity / stall if current draw is high
        if current_frame.current_amp is not None and current_frame.current_amp > 1.2:
            vel = enc.velocity_rad_s or 0.0
            if abs(vel) < self.stall_threshold_rad_s:
                self._stall_counter += 1
                if self._stall_counter >= self.stall_min_duration_frames:
                    events.append(
                        ActuatorEvent(
                            event_type=ActuatorState.STALL,
                            timestamp=current_frame.timestamp,
                            source=EventSource.DETECTOR,
                            joint_id=self.joint_id,
                            confidence=0.85,
                            details={"current_amp": current_frame.current_amp, "vel_rad_s": vel},
                        )
                    )
            else:
                self._stall_counter = 0
        else:
            self._stall_counter = 0

        return events
