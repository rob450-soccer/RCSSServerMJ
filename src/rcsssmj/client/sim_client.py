import logging
import socket
from collections.abc import Sequence
from enum import Enum
from queue import Queue
from threading import Thread
from typing import Any, cast

from rcsssmj.agent import AgentID
from rcsssmj.client.action import SimAction
from rcsssmj.client.encoder import PerceptionEncoder, SExprPerceptionEncoder
from rcsssmj.client.parser import ActionParser, SExprActionParser
from rcsssmj.client.perception import Perception
from rcsssmj.communication.tcp_lpm_connection import TCPLPMConnection

logger = logging.getLogger(__name__)


class SimClientState(Enum):
    """
    Simulation client state enum.
    """

    CONNECTED = 'connected'
    """
    The client is connected, but has not yet received sufficient initialization information.
    """

    READY = 'ready'
    """
    The client is ready for integration into the simulation.
    """

    ACTIVE = 'active'
    """
    The client is actively receiving actions from a connected agent.
    """

    DISCONNECTED = 'disconnected'
    """
    The client is disconnected as waiting to be removed from the simulation.
    The client thread has shutdown.
    """


class SimClient:
    """
    Simulation client controlling a robot in simulation.
    """

    def __init__(self, conn: TCPLPMConnection) -> None:
        """
        Construct a new simulation client representation.
        """

        self._conn: TCPLPMConnection = conn
        self._parser: ActionParser = SExprActionParser()
        self._encoder: PerceptionEncoder = SExprPerceptionEncoder()

        self._state: SimClientState = SimClientState.CONNECTED
        self._model_name: str = ''
        self._team_name: str = ''
        self._player_no: int = -1

        self._receive_thread: Thread = Thread(target=self._receive_loop)

        self._agent_id: AgentID | None = None
        self._model_spec: Any | None = None
        self._model_markers: list[tuple[str, str]] = []

        self._perceptions: list[Perception] = []
        self._action_queue: Queue[list[SimAction]] = Queue()

        # start receive loop
        self._receive_thread.start()

    def get_addr(self) -> socket.AddressInfo:
        """
        Return the client address information.
        """

        return self._conn.addr

    def get_state(self) -> SimClientState:
        """
        Return the current client state.
        """

        return self._state

    def get_model_name(self) -> str:
        """
        Return the name of the robot model this client has selected.
        """

        return self._model_name

    def get_team_name(self) -> str:
        """
        Return the name of the team this client belongs to.
        """

        return self._team_name

    def get_player_no(self) -> int:
        """
        Return the player number of this client.
        """

        return self._player_no

    def get_id(self) -> AgentID | None:
        """
        Return the agent id associated with this client.
        """

        return self._agent_id

    def get_model_spec(self) -> Any | None:
        """
        Return the robot model specification associated with this client.
        """

        return self._model_spec

    def get_model_markers(self) -> Sequence[tuple[str, str]]:
        """
        Return the list of visual markers of the robot model associated with this client.
        """

        return self._model_markers

    def get_action_queue(self) -> Queue[list[SimAction]]:
        """
        Return the action queue associated with this client.
        """

        return self._action_queue

    def activate(self, agent_id: AgentID, spec: Any) -> None:
        """
        Activate the client.

        This method is called by the main simulation loop.
        """

        if self._state == SimClientState.READY:
            self._agent_id = agent_id
            self._model_spec = spec
            self._model_markers = [(site.name, ''.join(site.name.split('-')[3:-1])) for site in spec.sites if site.name.endswith('-vismarker')]

            # add a dummy action to prevent possible timeout in sync-mode
            self._action_queue.put([])

            self._state = SimClientState.ACTIVE

    def shutdown(self, *, no_wait: bool = True) -> None:
        """
        Stop the client.
        """

        self._conn.shutdown()

        if not no_wait:
            self._receive_thread.join()

    def add_perception(self, perception: Perception) -> None:
        """
        Add the given perception to the agent perceptions for this simulation cycle.

        This method is called by the main simulation loop.
        """

        # buffer perception
        self._perceptions.append(perception)

    def send_perceptions(self) -> None:
        """
        Send buffered agent perceptions.

        This method is called by the main simulation loop.
        """

        # fetch and reset perception list
        perceptions = self._perceptions
        self._perceptions = []

        if not self._conn.is_active():
            # skip sending perception as the client connection is already closed
            return

        # encode perceptions message
        msg = self._encoder.encode(perceptions)
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
        """
        The internal receive loop for continuously receiving client actions.
        """

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
