import logging
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


class RemoteAgentState(Enum):
    """Remote simulation agent state enum."""

    INIT = 'init'
    """The agent has been added to the simulation server, but can not yet provide sufficient initialization information."""

    READY = 'ready'
    """The agent is ready for integration into the simulation."""

    ACTIVE = 'active'
    """The agent is actively receiving actions from a connected agent within the simulation."""

    SHUTDOWN = 'shutdown'
    """The agent has been shut down and is waiting to be removed from the simulation server."""


class RemoteAgent:
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

        self._state: RemoteAgentState = RemoteAgentState.INIT
        """The agent state."""

        self._model_name: str = ''
        """The name of the robot model used for this agent."""

        self._team_name: str = ''
        """The name of the team the agent belongs to."""

        self._player_no: int = -1
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

    def get_state(self) -> RemoteAgentState:
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

        if self._state == RemoteAgentState.READY:
            self._agent_id = agent_id
            self._model_spec = spec
            self._model_markers = [(site.name, ''.join(site.name.split('-')[3:-1])) for site in spec.sites if site.name.endswith('-vismarker')]

            self._state = RemoteAgentState.ACTIVE

    def shutdown(self, *, wait: bool = False) -> None:
        """Stop the agent.

        Parameter
        ---------
        wait: bool, default=False
            True, if the calling thread should be blocked until the agent has finished its shutdown process, false if not.
        """

        self._conn.shutdown()

        if wait:
            self._receive_thread.join()

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

            if self._state == RemoteAgentState.INIT:
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
                self._state = RemoteAgentState.READY

            elif self._state == RemoteAgentState.ACTIVE:
                # agent is in ACTIVE state -> process action message
                actions = self._parser.parse_action(msg, cast(AgentID, self._agent_id).prefix)

                # forward action
                self._action_queue.put(actions)

            else:
                # agent is in READY or SHUTDOWN state -> we don't expect any messages from the agent in these states
                pass

        self._state = RemoteAgentState.SHUTDOWN
        self._conn.close()

        # add a dummy action to prevent possible timeout in sync-mode (one should be enough, but two don't hurt either)
        self._action_queue.put([])
        self._action_queue.put([])

        logger.debug('Agent thread for %s finished!', self._conn)

    def __str__(self) -> str:
        return f'{self._team_name} #{self._player_no} @ {self._conn}'

    def __repr__(self) -> str:
        return f'RemoteAgent({self._conn.__repr__()})'
