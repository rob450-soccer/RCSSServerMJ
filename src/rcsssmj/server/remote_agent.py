import logging
from abc import ABC, abstractmethod
from collections.abc import Sequence
from enum import Enum
from queue import Empty, Queue
from threading import Thread, current_thread
from typing import Generic, TypeVar, cast

from rcsssmj.server.action_parser import ActionParser
from rcsssmj.server.communication.connection import PConnection
from rcsssmj.server.perception_encoder import PerceptionEncoder
from rcsssmj.sim.actions import SimAction
from rcsssmj.sim.sim_agent import SimAgent, TypedSimAgent
from rcsssmj.sim.sim_interfaces import PSimActionInterface
from rcsssmj.sim.simulation import BaseSimulation

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


S = TypeVar('S', bound=BaseSimulation)


class RemoteAgent(Generic[S], ABC):
    """Remote simulation agent, utilizing a message based connection to communicate with an external agent process."""

    def __init__(self, conn: PConnection, encoder: PerceptionEncoder) -> None:
        """Construct a new remote simulation agent.

        Parameter
        ---------
        conn: PConnection
            The agent connection.

        encoder: PerceptionEncoder
            The perception message encoder instance.
        """

        self._state: RemoteAgentState = RemoteAgentState.INIT
        """The agent state."""

        self._model_name: str = ''
        """The name of the robot model used for this agent."""

        self._conn: PConnection = conn
        """The agent connection for exchanging perception and action messages."""

        self._encoder: PerceptionEncoder = encoder
        """Encoder for encoding outgoing perception messages."""

        self._agent_thread: Thread | None = None
        """The thread, running the agent receive loop (if existing)."""

    @property
    def state(self) -> RemoteAgentState:
        """The current agent state."""

        return self._state

    @property
    def model_name(self) -> str:
        """The name of the robot model this agent has selected."""

        return self._model_name

    @property
    @abstractmethod
    def sim_agent(self) -> SimAgent | None:
        """The simulation agent instance associated with this remote agent (if existing)."""

    def activate(self, sim: S) -> None:
        """Activate the remote agent."""

        if self._state == RemoteAgentState.READY:
            # try activating the agent
            self._activate(sim)

            if self.sim_agent is not None:
                # simulation agent successfully created
                logger.info('Agent %s activated.', self)
                self._state = RemoteAgentState.ACTIVE
            else:
                # TODO: maybe switch to an ERROR state?
                # failed to create simulation agent --> shutdown and remove agent
                logger.info('Failed to activate agent %s. Shutting agent down again.', self)
                self.shutdown()

    @abstractmethod
    def _activate(self, sim: S) -> None:
        """Activate the remote agent."""

    @abstractmethod
    def deactivate(self, sim: S) -> None:
        """Deactivate remote agent instance."""

    def shutdown(self) -> None:
        """Stop the agent."""

        self._conn.shutdown()

    def join(self) -> None:
        """Wait until the agent listener thread terminates (if existing)."""

        if self._agent_thread is not None:
            self._agent_thread.join()

    @abstractmethod
    def collect_actions(self, *, block: bool = False, timeout: float = 5) -> None:
        """Collect actions for the current cycle."""

    def send_perceptions(self) -> None:
        if not self._conn.is_active():
            # skip sending perception as the agent connection is already closed
            return

        if self.sim_agent is None:
            # remote agent not registered to a simulation, yet
            return

        # encode perceptions message
        msg = self._encoder.encode(self.sim_agent.perceptions)
        if msg is None:
            # encoding failed or resulted in an empty message
            logger.warning('Perception message encoding for %s failed!', self)
            msg = b'(error)'
        elif not msg:
            # no perceptions encoded
            msg = b'(syn)'

        # print(f'Sending perception to agent "{self._team_name} {self._player_no}": {msg}')

        self._conn.send_message(msg)

    def run(self) -> None:
        """Continuously receive and process agent actions.

        Note: This method is blocking until the agent connection has been closed and thus supposed to be executed in a separate thread.
        """

        if self._agent_thread is not None:
            logger.warning('run()-method of remote agent %s called multiple times! Joining existing agent thread.', self)
            self._agent_thread.join()
            return

        # fetch agent thread instance
        self._agent_thread = current_thread()

        while True:
            # receive next action message
            try:
                msg = self._conn.receive_message()
            except ConnectionError:
                logger.debug('Agent connection %s closed!', self._conn)
                break

            if self._state == RemoteAgentState.INIT:
                # agent is in INIT state -> process initialization message
                if self._parse_init(msg):
                    # initialization successful --> signal ready state
                    self._state = RemoteAgentState.READY
                else:
                    # initialization failed --> shutdown agent again
                    logger.warning('Initialization for agent %s failed! Disconnecting!', self._conn)
                    self._conn.shutdown()
                    # TODO: Switch to an ERROR state...
                    break

            elif self._state == RemoteAgentState.ACTIVE:
                # agent is in ACTIVE state -> process action message
                self._parse_action(msg)

            else:
                # agent is in READY or SHUTDOWN state -> we don't expect any messages from the agent in these states
                pass

        self._state = RemoteAgentState.SHUTDOWN
        self._conn.close()

        # add a dummy action to prevent possible timeout in sync-mode (one should be enough, but two don't hurt either)
        if self.sim_agent is not None:
            self._parse_action(b'(syn)')
            self._parse_action(b'(syn)')

        logger.debug('Agent thread for %s finished!', self._conn)

    @abstractmethod
    def _parse_init(self, msg: bytes | bytearray) -> bool:
        """Parse initialization message."""

    @abstractmethod
    def _parse_action(self, msg: bytes | bytearray) -> None:
        """Parse action message."""

    def __str__(self) -> str:
        return f'RemoteAgent @ {self._conn}'

    def __repr__(self) -> str:
        return f'RemoteAgent({self._conn.__repr__()})'


SAI = TypeVar('SAI', bound=PSimActionInterface)


class TypedRemoteAgent(RemoteAgent[S], Generic[S, SAI], ABC):
    """Remote simulation agent with generic action type."""

    def __init__(self, conn: PConnection, parser: ActionParser[SAI], encoder: PerceptionEncoder) -> None:
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

        super().__init__(conn, encoder)

        self._action_queue: Queue[Sequence[SimAction[PSimActionInterface] | SimAction[SAI]]] = Queue()
        """The queue to which incoming agent actions are forwarded."""

        self._parser: ActionParser[SAI] = parser
        """Parser for parsing incoming action messages."""

    @property
    @abstractmethod
    def sim_agent(self) -> TypedSimAgent[SAI] | None:
        """The simulation agent instance associated with this remote agent or ``None`` in case the remote agent is not actively participating in the simulation, yet / any longer."""

    @property
    def action_queue(self) -> Queue[Sequence[SimAction[PSimActionInterface] | SimAction[SAI]]]:
        """The action queue associated with this remote agent."""

        return self._action_queue

    def collect_actions(self, *, block: bool = False, timeout: float = 5) -> None:
        """Collect actions for the current cycle."""

        if self.sim_agent is None:
            return

        try:
            if block:
                # wait for exactly one agent action
                self.sim_agent.set_actions(self._action_queue.get(timeout=timeout))
            else:
                # fetch all currently available actions
                actions: list[SimAction[PSimActionInterface] | SimAction[SAI]] = []
                while self._action_queue.qsize() > 0:
                    actions += self._action_queue.get_nowait()
                self.sim_agent.set_actions(actions)
        except Empty:
            if block:
                # agent took too long to answer -> kill it
                # logger.info('Team %s: Agent %d did not respond for more than %.3f seconds. Forcing agent shutdown.', self.team_name, self.player_no, timeout)
                self.shutdown()

    def _parse_action(self, msg: bytes | bytearray) -> None:
        actions = self._parser.parse_action(msg, cast(TypedSimAgent, self.sim_agent).agent_id.prefix)

        # forward action
        self._action_queue.put(actions)
