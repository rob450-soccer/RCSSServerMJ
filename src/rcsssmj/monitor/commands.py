import logging
from abc import ABC, abstractmethod

from rcsssmj.simulation_interfaces import PSimCommandInterface

logger = logging.getLogger(__name__)


class MonitorCommand(ABC):
    """Base class for monitor commands."""

    @abstractmethod
    def perform(self, ci: PSimCommandInterface) -> None:
        """Perform this command.

        Parameter
        ---------
        ci: PSimCommandInterface
            The simulation command interface.
        """


class KillSimCommand(MonitorCommand):
    """The kill-sim command."""

    def perform(self, ci: PSimCommandInterface) -> None:
        ci.kill_sim()
