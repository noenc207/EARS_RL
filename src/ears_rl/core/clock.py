"""Time synchronization and clock providers for EARS-RL.

Provides deterministic time abstraction to enable repeatable testing of
latency, timestamps, and watchdog expirations without depending on wall time.
"""

from __future__ import annotations

import time

from ears_rl.core.types import Timestamp


class TimeProvider:
    """Abstract base time provider interface."""

    def now(self) -> Timestamp:
        """Return current monotonic timestamp."""
        raise NotImplementedError

    def now_ns(self) -> int:
        """Return current monotonic time in nanoseconds [ns]."""
        raise NotImplementedError


class SystemClock(TimeProvider):
    """Production system clock using OS monotonic high-resolution counter."""

    def now(self) -> Timestamp:
        mono_ns = time.monotonic_ns()
        wall_ns = time.time_ns()
        return Timestamp(ns=mono_ns, wall_utc_ns=wall_ns)

    def now_ns(self) -> int:
        return time.monotonic_ns()


class MockClock(TimeProvider):
    """Deterministic mock clock for unit tests and reproducible simulations.

    Allows discrete stepping of time without invoking time.sleep().
    """

    def __init__(self, start_ns: int = 1_000_000_000) -> None:
        self._current_ns = start_ns

    def now(self) -> Timestamp:
        return Timestamp(ns=self._current_ns, wall_utc_ns=self._current_ns)

    def now_ns(self) -> int:
        return self._current_ns

    def tick(self, delta_ns: int = 10_000_000) -> None:
        """Advance mock time by delta_ns nanoseconds (default 10ms)."""
        if delta_ns < 0:
            raise ValueError("Time cannot flow backwards")
        self._current_ns += delta_ns

    def tick_ms(self, delta_ms: float) -> None:
        """Advance mock time by delta_ms milliseconds."""
        self.tick(int(delta_ms * 1_000_000))

    def set_time_ns(self, ns: int) -> None:
        """Directly set mock clock timestamp."""
        self._current_ns = ns
