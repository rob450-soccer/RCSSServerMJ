import logging
from queue import Queue
from threading import Thread
from typing import Any

from rcsssmj.communication.tcp_lpm_connection import TCPLPMConnection
from rcsssmj.monitor.commands import MonitorCommand
from rcsssmj.monitor.parser import CommandParser, SExprCommandParser
from rcsssmj.monitor.sim_monitor import SimMonitor

logger = logging.getLogger(__name__)


class MonitorClient(SimMonitor):
    """
    Monitor client monitoring and interacting with the simulation.
    """

    def __init__(self, conn: TCPLPMConnection) -> None:
        """
        Construct a new monitor client representation.
        """

        self._conn: TCPLPMConnection = conn
        self._parser: CommandParser = SExprCommandParser()

        self._receive_thread: Thread = Thread(target=self._receive_loop)

        self._command_queue: Queue[MonitorCommand] = Queue()

        # start receive loop
        self._receive_thread.start()

    def is_active(self) -> bool:
        """
        Check if the monitor is still active.
        """

        return self._conn.is_active()

    def get_command_queue(self) -> Queue[MonitorCommand]:
        """
        Return the command queue associated with this client.
        """

        return self._command_queue

    def shutdown(self, *, no_wait: bool = True) -> None:
        """
        Stop the monitor client.
        """

        self._conn.shutdown()

        if not no_wait:
            self._receive_thread.join()

    def update(self, mj_model: Any, mj_data: Any) -> None:
        """
        Update the monitor state.
        """

        # TODO: Generate and send simulation state message to monitor

    def _receive_loop(self) -> None:
        """
        The internal receive loop for continuously receiving monitor client commands.
        """

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

        self._conn.close()

        logger.debug('Monitor thread for %s finished!', self._conn.addr)
