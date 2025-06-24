import logging
from abc import ABC, abstractmethod
from collections.abc import Sequence
from enum import Enum
from queue import Queue
from threading import Thread

from rcsssmj.communication.connection import PConnection
from rcsssmj.monitor.commands import MonitorCommand
from rcsssmj.monitor.parser import CommandParser
from rcsssmj.monitor.state import SimStateInformation

logger = logging.getLogger(__name__)


class SimMonitorState(Enum):
    """Simulation monitor state enum."""

    ACTIVE = 'active'
    """The monitor is actively receiving simulation state messages and can issue commands."""

    SHUTDOWN = 'shutdown'
    """The monitor has been shut down and is waiting to be removed from the simulation server."""


class SimMonitor(ABC):
    """Base class for simulation monitors, monitoring and interacting with the simulation."""

    def __init__(self, update_interval: int = 1):
        super().__init__()

        self._state: SimMonitorState = SimMonitorState.ACTIVE
        """The current simulation monitor state."""

        self.update_interval: int = update_interval
        """The update interval of the monitor."""

        self._command_queue: Queue[MonitorCommand] = Queue()
        """The monitor command queue."""

    def get_state(self) -> SimMonitorState:
        """Return the current monitor state."""

        return self._state

    def get_command_queue(self) -> Queue[MonitorCommand]:
        """Return the command queue associated with this monitor."""

        return self._command_queue

    def shutdown(self, *, wait: bool = False) -> None:
        """Stop the monitor.

        Parameter
        ---------
        wait: bool, default=False
            True, if the calling thread should be blocked until the monitor has finished its shutdown process, false if not.
        """

        self._state = SimMonitorState.SHUTDOWN

    @abstractmethod
    def update(self, state_info: Sequence[SimStateInformation], frame_id: int) -> None:
        """Update the monitor state.

        Parameter
        ---------
        state_info: Sequence[SimStateInformation]
            The list of simulation state information.

        frame_id: int
            The current simulation frame id.
        """


class RemoteSimMonitor(SimMonitor):
    """Remote simulation monitor, utilizing a message based connection to communicate with an external monitor process."""

    def __init__(self, conn: PConnection, parser: CommandParser) -> None:
        """Construct a new remote simulation monitor client.

        Parameter
        ---------
        conn: PConnection
            The monitor connection.

        parser: CommandParser
            The monitor command parser instance.
        """

        super().__init__()

        self._conn: PConnection = conn
        self._parser: CommandParser = parser

        self._receive_thread: Thread = Thread(target=self._receive_loop)

        # start receive loop
        self._receive_thread.start()

    def shutdown(self, *, wait: bool = False) -> None:
        self._conn.shutdown()

        if wait:
            self._receive_thread.join()

    def update(self, state_info: Sequence[SimStateInformation], frame_id: int) -> None:
        # TODO: Send simulation state message to monitor
        pass

    def _receive_loop(self) -> None:
        """The internal receive loop for continuously receiving monitor client commands."""

        while True:
            # receive next command message
            try:
                msg = self._conn.receive_message()
            except ConnectionError:
                logger.debug('Monitor connection %s closed!', self._conn)
                break

            # parse commands
            commands = self._parser.parse(msg)

            # forward commands
            for command in commands:
                self._command_queue.put(command)

        self._state = SimMonitorState.SHUTDOWN
        self._conn.close()

        logger.debug('Monitor thread for %s finished!', self._conn)
