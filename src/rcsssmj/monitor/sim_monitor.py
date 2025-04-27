from abc import ABC, abstractmethod
from queue import Queue
from typing import Any

from rcsssmj.monitor.commands import MonitorCommand


class SimMonitor(ABC):
    """
    Base class for simulation monitors.
    """

    @abstractmethod
    def is_active(self) -> bool:
        """
        Check if the monitor is still active.
        """

    @abstractmethod
    def get_command_queue(self) -> Queue[MonitorCommand]:
        """
        Return the command queue associated with this client.
        """

    @abstractmethod
    def shutdown(self, *, no_wait: bool = True) -> None:
        """
        Stop the monitor.
        """

    @abstractmethod
    def update(self, mj_model: Any, mj_data: Any) -> None:
        """
        Update the monitor state.
        """
