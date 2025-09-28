import logging
from abc import ABC, abstractmethod
from collections.abc import Sequence
from enum import Enum
from queue import Queue
from threading import Thread
from typing import Any, cast

from rcsssmj.agent.action import SimAction
from rcsssmj.agent.encoder import PerceptionEncoder
from rcsssmj.agent.parser import ActionParser
from rcsssmj.agent.perception import Perception
from rcsssmj.agents import AgentID
from rcsssmj.communication.connection import PConnection

logger = logging.getLogger(__name__)


class SimAgentState(Enum):
    """Simulation agnet state enum."""

    INIT = 'init'
    """The agent has been added to the simulation server, but can not yet provide sufficient initialization information."""

    READY = 'ready'
    """The agent is ready for integration into the simulation."""

    ACTIVE = 'active'
    """The agent is actively receiving actions from a connected agent within the simulation."""

    SHUTDOWN = 'shutdown'
    """The agent has been shut down and is waiting to be removed from the simulation server."""


class SimAgent(ABC):
    """Simulation agent controlling a robot in simulation."""

    def __init__(self, model_name: str = '', team_name: str = '', player_no: int = -1) -> None:
        """Construct a new simulation agent representation.

        Parameter
        ---------
        model_name: str, default=''
            The name of the robot model to load for this agent.

        team_name: str, default=''
            The name of the team this agent belongs to.

        player_no: int, default=-1
            The agent player number.
        """

        self._state: SimAgentState = SimAgentState.READY if len(model_name) > 0 and len(team_name) > 0 and player_no >= 0 else SimAgentState.INIT
        """The agent state."""

        self._model_name: str = model_name
        """The name of the robot model used for this agent."""

        self._team_name: str = team_name
        """The name of the team the agent belongs to."""

        self._player_no: int = player_no
        """The player number."""

        self._agent_id: AgentID | None = None
        """Unique identifier in the simulation associated with this agent."""

        self._model_spec: Any | None = None
        """The robot model specification."""

        self._model_markers: Sequence[tuple[str, str]] = []
        """The visible markers of the robot model."""

        self._perceptions: Sequence[Perception] = []
        """The current perceptions of the agent."""

        self._action_queue: Queue[Sequence[SimAction]] = Queue()
        """The queue to which incoming agent actions are forwarded."""

    def get_state(self) -> SimAgentState:
        """Return the current agent state."""

        return self._state

    def get_model_name(self) -> str:
        """Return the name of the robot model this agent has selected."""

        return self._model_name

    def get_team_name(self) -> str:
        """Return the name of the team this agent belongs to."""

        return self._team_name

    def get_player_no(self) -> int:
        """Return the player number of this agent."""

        return self._player_no

    def get_id(self) -> AgentID | None:
        """Return the unique id associated with this agent (if existing)."""

        return self._agent_id

    def get_model_spec(self) -> Any | None:
        """Return the robot model specification associated with this agent."""

        return self._model_spec

    def get_model_markers(self) -> Sequence[tuple[str, str]]:
        """Return the list of visual markers of the robot model associated with this agent."""

        return self._model_markers

    def get_action_queue(self) -> Queue[Sequence[SimAction]]:
        """Return the action queue associated with this agent."""

        return self._action_queue

    def activate(self, agent_id: AgentID, spec: Any) -> None:
        """Activate the agent.

        This method is called by the main simulation loop.
        """

        if self._state == SimAgentState.READY:
            self._agent_id = agent_id
            self._model_spec = spec
            self._model_markers = [(site.name, ''.join(site.name.split('-')[3:-1])) for site in spec.sites if site.name.endswith('-vismarker')]

            self._state = SimAgentState.ACTIVE

    def shutdown(self, *, wait: bool = False) -> None:
        """Stop the agent.

        Parameter
        ---------
        wait: bool, default=False
            True, if the calling thread should be blocked until the agent has finished its shutdown process, false if not.
        """

        self._state = SimAgentState.SHUTDOWN

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
        return f'SimAgent({self._model_name}, {self._team_name}, {self._player_no})'


class RemoteSimAgent(SimAgent):
    """Remote simulation agent, utilizing a message based connection to communicate with an external agent process."""

    def __init__(self, conn: PConnection, parser: ActionParser, encoder: PerceptionEncoder) -> None:
        """Construct a new remote simulation agent.

        Parameter
        ---------
        conn: PConnection
            The agent connection.

        parser: ActionParser
            The action message parser instance.

        encoder: PerceptionEncoder
            The perception message encoder instance.
        """

        super().__init__()

        self._conn: PConnection = conn
        """The agent connection for exchanging perception and action messages."""

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
            # skip sending perception as the agent connection is already closed
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

        # print(f'Sending perception to agent "{self._team_name} {self._player_no}": {msg}')

        self._conn.send_message(msg)

    def _receive_loop(self) -> None:
        """The internal receive loop for continuously receiving agent actions."""

        while True:
            # receive next action message
            try:
                msg = self._conn.receive_message()
            except ConnectionError:
                logger.debug('Agent connection %s closed!', self._conn)
                break

            if self._state == SimAgentState.INIT:
                # agent is in INIT state -> process initialization message
                init_action = self._parser.parse_init(msg)

                if init_action is None:
                    logger.warning('Initialization for agent %s failed! Disconnecting!', self._conn)
                    self._conn.shutdown()
                    break

                # set agent information
                self._model_name = init_action.model_name
                self._team_name = init_action.team_name
                self._player_no = init_action.player_no

                # signal ready state
                self._state = SimAgentState.READY

            elif self._state == SimAgentState.ACTIVE:
                # agent is in ACTIVE state -> process action message
                actions = self._parser.parse_action(msg, cast(AgentID, self._agent_id).prefix)

                # forward action
                self._action_queue.put(actions)

            else:
                # agent is in READY or SHUTDOWN state -> we don't expect any messages from the agent in these states
                pass

        self._state = SimAgentState.SHUTDOWN
        self._conn.close()

        # add a dummy action to prevent possible timeout in sync-mode (one should be enough, but two don't hurt either)
        self._action_queue.put([])
        self._action_queue.put([])

        logger.debug('Agent thread for %s finished!', self._conn)

    def __str__(self) -> str:
        return f'{self._team_name} #{self._player_no} @ {self._conn}'

    def __repr__(self) -> str:
        return f'TCPSimAgent({self._conn.__repr__()})'


class ManagedSimAgent(SimAgent):
    """(Externally) managed agent controlling a robot in simulation and running synchronous with the simulation (one step behind).

    The managed agent can be used in learning scenarios, where observations and actions are processed / generated by a learning algorithm in a gymnasium like environment.
    The list of current agent perceptions as well as the action queue is exposed to the user and it's the users responsibility to process / set them accordingly before progressing the simulation.
    """

    def __init__(self, model_name: str, team_name: str, player_no: int) -> None:
        """Construct a new managed agent representation.

        Parameter
        ---------
        model_name: str
            The name of the robot model to load for this agent.

        team_name: str
            The name of the team this agent belongs to.

        player_no: int
            The player number.
        """

        super().__init__(model_name, team_name, player_no)

    def send_perceptions(self) -> None:
        # noting to send here...
        pass

    def get_perceptions(self) -> Sequence[Perception]:
        """Return the list of agent perceptions."""

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
        return f'ManagedSimAgent({self._model_name}, {self._team_name}, {self._player_no})'
