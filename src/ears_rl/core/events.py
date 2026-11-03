"""Event bus and pub-sub mechanism for decoupled communication in EARS-RL."""

from __future__ import annotations

import logging
from collections import defaultdict
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from ears_rl.core.types import Timestamp

logger = logging.getLogger(__name__)


@dataclass
class BusEvent:
    """Base event payload."""
    name: str
    timestamp: Timestamp | None = None
    data: dict[str, Any] = field(default_factory=dict)
    source: str = ""


class EventBus:
    """Synchronous in-process event bus for telemetry and state notifications."""

    def __init__(self) -> None:
        self._handlers: dict[str, list[Callable[[BusEvent], None]]] = defaultdict(list)

    def subscribe(self, event_name: str, handler: Callable[[BusEvent], None]) -> None:
        """Register an event callback."""
        self._handlers[event_name].append(handler)

    def unsubscribe(self, event_name: str, handler: Callable[[BusEvent], None]) -> None:
        """Unregister an event callback."""
        if event_name in self._handlers:
            try:
                self._handlers[event_name].remove(handler)
            except ValueError:
                pass

    def publish(self, event: BusEvent) -> None:
        """Publish event to all subscribers."""
        for handler in self._handlers.get(event.name, []):
            try:
                handler(event)
            except Exception as e:
                logger.error("Error in handler %s for event %s: %s", handler, event.name, e)

    def clear(self) -> None:
        """Clear all subscriptions."""
        self._handlers.clear()
