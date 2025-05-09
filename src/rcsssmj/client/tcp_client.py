import logging
import socket
from threading import Thread
from typing import cast

from rcsssmj.agent import AgentID
from rcsssmj.client.encoder import PerceptionEncoder, SExprPerceptionEncoder
from rcsssmj.client.parser import ActionParser, SExprActionParser
from rcsssmj.client.sim_client import SimClient, SimClientState
from rcsssmj.communication.tcp_lpm_connection import TCPLPMConnection

logger = logging.getLogger(__name__)


class TCPSimClient(SimClient):
    """TCP connection based simulation client."""

    def __init__(self, conn: TCPLPMConnection) -> None:
        """Construct a new TCP based client.

        Parameter
        ---------
        conn: TCPLPMConnection
            The client connection.
        """

        super().__init__()

        self._conn: TCPLPMConnection = conn
        self._parser: ActionParser = SExprActionParser()
        self._encoder: PerceptionEncoder = SExprPerceptionEncoder()

        self._receive_thread: Thread = Thread(target=self._receive_loop)

        # start receive loop
        self._receive_thread.start()

    def get_addr(self) -> socket.AddressInfo:
        """Return the client address information."""

        return self._conn.addr

    def shutdown(self, *, no_wait: bool = True) -> None:
        self._conn.shutdown()

        if not no_wait:
            self._receive_thread.join()

    def send_perceptions(self) -> None:
        if not self._conn.is_active():
            # skip sending perception as the client connection is already closed
            return

        # encode perceptions message
        msg = self._encoder.encode(self._perceptions)
        if msg is None:
            # encoding failed or resulted in an empty message
            logger.warning('Perception message encoding for %s %d failed!', self._team_name, self._player_no)
            msg = b'(error)'
        elif not msg:
            # no perceptions encoded
            msg = b'(syn)'

        # print(f'Sending perception to client "{self._team_name} {self._player_no}": {msg}')

        self._conn.send_message(msg)

    def _receive_loop(self) -> None:
        """The internal receive loop for continuously receiving client actions."""

        while True:
            # receive next action message
            try:
                msg = self._conn.receive_message()
            except ConnectionError:
                logger.debug('Client connection %s closed!', self._conn.addr)
                break

            if self._state == SimClientState.CONNECTED:
                # client is in CONNECTED state -> process initialization message
                init_action = self._parser.parse_init(msg)

                if init_action is None:
                    logger.warning('Initialization for client %s failed! Disconnecting!', self._conn.addr)
                    self._conn.shutdown()
                    break

                # set client information
                self._model_name = init_action.model_name
                self._team_name = init_action.team_name
                self._player_no = init_action.player_no

                # signal ready state
                self._state = SimClientState.READY

            elif self._state == SimClientState.ACTIVE:
                # client is in ACTIVE state -> process action message
                actions = self._parser.parse_action(msg, cast(AgentID, self._agent_id).prefix)

                # forward action
                self._action_queue.put(actions)

            else:
                # client is in READY or DISCONNECTED state -> we don't expect any messages from the client in these states
                pass

        self._state = SimClientState.DISCONNECTED
        self._conn.close()

        # add a dummy action to prevent possible timeout in sync-mode (one should be enough, but two don't hurt either)
        self._action_queue.put([])
        self._action_queue.put([])

        logger.debug('Client thread for %s finished!', self._conn.addr)

    def __str__(self) -> str:
        return f'{self._team_name} #{self._player_no} @ {self._conn.addr}'

    def __repr__(self) -> str:
        return f'TCPSimClient({self._conn.__repr__()})'
