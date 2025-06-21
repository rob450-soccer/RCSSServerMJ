import logging
from abc import ABC, abstractmethod
from collections.abc import Sequence
from enum import Enum
from queue import Queue
from threading import Thread
from typing import Any, cast

from rcsssmj.agent import AgentID
from rcsssmj.client.action import SimAction
from rcsssmj.client.encoder import PerceptionEncoder
from rcsssmj.client.parser import ActionParser
from rcsssmj.client.perception import Perception
from rcsssmj.communication.connection import PConnection

logger = logging.getLogger(__name__)


class SimClientState(Enum):
    """Simulation client state enum."""

    CONNECTED = 'connected'
    """The client is connected, but has not yet received sufficient initialization information."""

    READY = 'ready'
    """The client is ready for integration into the simulation."""

    ACTIVE = 'active'
    """The client is actively receiving actions from a connected agent."""

    DISCONNECTED = 'disconnected'
    """The client is disconnected and waiting to be removed from the simulation."""


class SimClient(ABC):
    """Simulation client controlling a robot in simulation."""

    def __init__(self, model_name: str = '', team_name: str = '', player_no: int = -1) -> None:
        """Construct a new simulation client representation.

        Parameter
        ---------
        model_name: str, default=''
            The name of the robot model to load for this client.

        team_name: str, default=''
            The name of the team this client belongs to.

        player_no: int, default=-1
            The client player number.
        """

        self._state: SimClientState = SimClientState.READY if len(model_name) > 0 and len(team_name) > 0 and player_no >= 0 else SimClientState.CONNECTED
        """The client state."""

        self._model_name: str = model_name
        """The name of the robot model used for this client."""

        self._team_name: str = team_name
        """The name of the team the client belongs to."""

        self._player_no: int = player_no
        """The player number."""

        self._agent_id: AgentID | None = None
        """Unique identifier for the agent in the simulation associated with this client."""

        self._model_spec: Any | None = None
        """The robot model specification."""

        self._model_markers: Sequence[tuple[str, str]] = []
        """The visible markers of the robot model."""

        self._perceptions: Sequence[Perception] = []
        """The current perceptions of the client."""

        self._action_queue: Queue[Sequence[SimAction]] = Queue()
        """The queue to which incoming agent actions are forwarded."""

    def get_state(self) -> SimClientState:
        """Return the current client state."""

        return self._state

    def get_model_name(self) -> str:
        """Return the name of the robot model this client has selected."""

        return self._model_name

    def get_team_name(self) -> str:
        """Return the name of the team this client belongs to."""

        return self._team_name

    def get_player_no(self) -> int:
        """Return the player number of this client."""

        return self._player_no

    def get_id(self) -> AgentID | None:
        """Return the agent id associated with this client."""

        return self._agent_id

    def get_model_spec(self) -> Any | None:
        """Return the robot model specification associated with this client."""

        return self._model_spec

    def get_model_markers(self) -> Sequence[tuple[str, str]]:
        """Return the list of visual markers of the robot model associated with this client."""

        return self._model_markers

    def get_action_queue(self) -> Queue[Sequence[SimAction]]:
        """Return the action queue associated with this client."""

        return self._action_queue

    def activate(self, agent_id: AgentID, spec: Any) -> None:
        """Activate the client.

        This method is called by the main simulation loop.
        """

        if self._state == SimClientState.READY:
            self._agent_id = agent_id
            self._model_spec = spec
            self._model_markers = [(site.name, ''.join(site.name.split('-')[3:-1])) for site in spec.sites if site.name.endswith('-vismarker')]

            self._state = SimClientState.ACTIVE

    def shutdown(self, *, wait: bool = False) -> None:
        """Stop the client.

        Parameter
        ---------
        wait: bool, default=False
            True, if the calling thread should be blocked until the client has finished its shutdown process, false if not.
        """

        self._state = SimClientState.DISCONNECTED

    def set_perceptions(self, perceptions: Sequence[Perception]) -> None:
        """Set the perceptions of the agent for this simulation cycle.

        This method is called by the main simulation loop.

        Parameter
        ---------
        perceptions: Sequence[Perception]
            The list of perceptions of the agent for this simulation cycle.
        """

        # buffer perception
        self._perceptions = perceptions

    @abstractmethod
    def send_perceptions(self) -> None:
        """Send buffered agent perceptions.

        This method is called by the main simulation loop.
        """

    def __str__(self) -> str:
        return f'{self._team_name} #{self._player_no}'

    def __repr__(self) -> str:
        return f'SimClient({self._model_name}, {self._team_name}, {self._player_no})'


class RemoteSimClient(SimClient):
    """Remote simulation client, utilizing a message based connection to communicate with an external agent process."""

    def __init__(self, conn: PConnection, parser: ActionParser, encoder: PerceptionEncoder) -> None:
        """Construct a new remote simulation client.

        Parameter
        ---------
        conn: PConnection
            The client connection.

        parser: ActionParser
            The action message parser instance.

        encoder: PerceptionEncoder
            The perception message encoder instance.
        """

        super().__init__()

        self._conn: PConnection = conn
        """The client connection for exchanging perception and action messages."""

        self._parser: ActionParser = parser
        """Parser for parsing incoming action messages."""

        self._encoder: PerceptionEncoder = encoder
        """Encoder for encoding outgoing perception messages."""

        self._receive_thread: Thread = Thread(target=self._receive_loop)
        """The receive thread, running the receive loop."""

        # start receive loop
        self._receive_thread.start()

    def shutdown(self, *, wait: bool = False) -> None:
        self._conn.shutdown()

        if wait:
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
                logger.debug('Client connection %s closed!', self._conn)
                break

            if self._state == SimClientState.CONNECTED:
                # client is in CONNECTED state -> process initialization message
                init_action = self._parser.parse_init(msg)

                if init_action is None:
                    logger.warning('Initialization for client %s failed! Disconnecting!', self._conn)
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

        logger.debug('Client thread for %s finished!', self._conn)

    def __str__(self) -> str:
        return f'{self._team_name} #{self._player_no} @ {self._conn}'

    def __repr__(self) -> str:
        return f'TCPSimClient({self._conn.__repr__()})'


class ManagedSimClient(SimClient):
    """(Externally) managed client controlling a robot in simulation and running synchronous with the simulation (one step behind).

    The managed client can be used in learning scenarios, where observations and actions are processed / generated by a learning algorithm in a gymnasium like environment.
    The list of current agent perceptions as well as the action queue is exposed to the user and it's the users responsibility to process / set them accordingly before progressing the simulation.
    """

    def __init__(self, model_name: str, team_name: str, player_no: int) -> None:
        """Construct a new managed client representation.

        Parameter
        ---------
        model_name: str
            The name of the robot model to load for this client.

        team_name: str
            The name of the team this client belongs to.

        player_no: int
            The player number.
        """

        super().__init__(model_name, team_name, player_no)

    def send_perceptions(self) -> None:
        # noting to send here...
        pass

    def get_perceptions(self) -> Sequence[Perception]:
        """Return the list of client perceptions."""

        return self._perceptions

    def put_action(self, actions: Sequence[SimAction]) -> None:
        """Put the given actions to the action queue.

        Parameter
        ---------
        actions: Sequence[SimAction]
            The list of actions.
        """

        self._action_queue.put(actions)

    def __repr__(self) -> str:
        return f'LearningClient({self._model_name}, {self._team_name}, {self._player_no})'
