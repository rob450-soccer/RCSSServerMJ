import logging
from collections.abc import Sequence
from enum import Enum
from queue import Queue
from threading import Thread
from typing import cast

from rcsssmj.server.action_parser import ActionParser
from rcsssmj.server.communication.connection import PConnection
from rcsssmj.server.perception_encoder import PerceptionEncoder
from rcsssmj.sim.actions import SimAction
from rcsssmj.sim.sim_agent import SimAgent

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

        self._sim_agent: SimAgent | None = None
        """The simulation agent instance associated with this remote agent."""

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

    @property
    def state(self) -> RemoteAgentState:
        """The current agent state."""

        return self._state

    @property
    def model_name(self) -> str:
        """The name of the robot model this agent has selected."""

        return self._model_name

    @property
    def team_name(self) -> str:
        """The name of the team this agent belongs to."""

        return self._team_name

    @property
    def player_no(self) -> int:
        """The player number of this agent."""

        return self._player_no

    @property
    def sim_agent(self) -> SimAgent | None:
        """The simulation agent instance associated with this remote agent (if existing)."""

        return self._sim_agent

    @property
    def action_queue(self) -> Queue[Sequence[SimAction]]:
        """The action queue associated with this remote agent."""

        return self._action_queue

    def activate(self, agent: SimAgent) -> None:
        """Activate the agent.

        This method is called by the main simulation loop.
        """

        if self._state == RemoteAgentState.READY:
            self._sim_agent = agent

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

    def send_perceptions(self) -> None:
        if not self._conn.is_active():
            # skip sending perception as the agent connection is already closed
            return

        if self._sim_agent is None:
            # remote agent not registered to a simulation, yet
            return

        # encode perceptions message
        msg = self._encoder.encode(self._sim_agent.perceptions)
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
                actions = self._parser.parse_action(msg, cast(SimAgent, self._sim_agent).agent_id.prefix)

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
