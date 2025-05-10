import logging
from abc import ABC, abstractmethod
from enum import Enum
from queue import Queue
from threading import Thread
from typing import Any

from rcsssmj.communication.tcp_lpm_connection import TCPLPMConnection
from rcsssmj.monitor.commands import MonitorCommand
from rcsssmj.monitor.parser import CommandParser, SExprCommandParser

logger = logging.getLogger(__name__)


class SimMonitorState(Enum):
    """Simulation monitor state enum."""

    CONNECTED = 'connected'
    """The monitor is connected and active."""

    DISCONNECTED = 'disconnected'
    """The monitor is disconnected and waiting to be removed from the simulation."""


class SimMonitor(ABC):
    """Base class for simulation monitors, monitoring and interacting with the simulation."""

    def __init__(self, update_interval: int = 1):
        super().__init__()

        self._state: SimMonitorState = SimMonitorState.CONNECTED
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

        self._state = SimMonitorState.DISCONNECTED

    @abstractmethod
    def update(self, mj_model: Any, mj_data: Any, frame_id: int) -> None:
        """Update the monitor state.

        Parameter
        ---------
        mj_model: MjModel
            The current simulation model.

        mj_data: MjData
            The current simulation data.

        frame_id: int
            The current simulation frame id.
        """


class TCPSimMonitor(SimMonitor):
    """TCP connection based monitor client."""

    def __init__(self, conn: TCPLPMConnection) -> None:
        """Construct a new monitor client representation.

        Parameter
        ---------
        conn: TCPLPMConnection
            The monitor connection.
        """

        super().__init__()

        self._conn: TCPLPMConnection = conn
        self._parser: CommandParser = SExprCommandParser()

        self._receive_thread: Thread = Thread(target=self._receive_loop)

        # start receive loop
        self._receive_thread.start()

    def shutdown(self, *, wait: bool = False) -> None:
        self._conn.shutdown()

        if wait:
            self._receive_thread.join()

    def update(self, mj_model: Any, mj_data: Any, frame_id: int) -> None:
        # TODO: Generate and send simulation state message to monitor
        pass

    def _receive_loop(self) -> None:
        """The internal receive loop for continuously receiving monitor client commands."""

        while True:
            # receive next command message
            try:
                msg = self._conn.receive_message()
            except ConnectionError:
                logger.debug('Monitor connection %s closed!', self._conn.addr)
                break

            # parse commands
            commands = self._parser.parse(msg)

            # forward commands
            for command in commands:
                self._command_queue.put(command)

        self._state = SimMonitorState.DISCONNECTED
        self._conn.close()

        logger.debug('Monitor thread for %s finished!', self._conn.addr)
