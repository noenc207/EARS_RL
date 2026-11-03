"""Experiment lifecycle context manager for EARS-RL.

Ensures fail-safe cleanup:
1. Actuator excitation and motion are ALWAYS disabled upon exit or exception.
2. Loggers are flushed and closed.
3. Transports and hardware connections are safely closed.
4. No resources remain armed if an unhandled error occurs.
"""

from __future__ import annotations

import logging
from types import TracebackType

from ears_rl.core.interfaces import ActuatorInterface, CommandTransport, DataLoggerInterface

logger = logging.getLogger(__name__)


class ExperimentSession:
    """Context manager guaranteeing fail-safe lifecycle and resource management."""

    def __init__(
        self,
        experiment_id: str,
        actuators: list[ActuatorInterface] | None = None,
        transports: list[CommandTransport] | None = None,
        loggers: list[DataLoggerInterface] | None = None,
    ) -> None:
        self.experiment_id = experiment_id
        self.actuators = actuators or []
        self.transports = transports or []
        self.loggers = loggers or []
        self._is_active = False

    def __enter__(self) -> ExperimentSession:
        self._is_active = True
        logger.info("Starting safe experiment session '%s'", self.experiment_id)
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        logger.info("Exiting safe experiment session '%s'", self.experiment_id)

        # 1. Mandatory Invariant: Disable motion on all actuators first
        for act in self.actuators:
            try:
                act.disable_motion()
            except Exception as e:
                logger.error("Error disabling motion on actuator '%s': %s", getattr(act, "joint_id", "unknown"), e)

        # 2. Flush and close data loggers
        for log in self.loggers:
            try:
                log.flush()
                log.close()
            except Exception as e:
                logger.error("Error flushing/closing data logger: %s", e)

        # 3. Disconnect actuators
        for act in self.actuators:
            try:
                act.disconnect()
            except Exception as e:
                logger.error("Error disconnecting actuator: %s", e)

        # 4. Disconnect command transports
        for trans in self.transports:
            try:
                trans.disconnect()
            except Exception as e:
                logger.error("Error disconnecting transport: %s", e)

        self._is_active = False
        if exc_val is not None:
            logger.warning(
                "Experiment session '%s' terminated with exception: %s",
                self.experiment_id,
                exc_val,
            )
