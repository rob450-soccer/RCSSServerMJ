import logging
from abc import ABC, abstractmethod
from collections.abc import Sequence
from enum import Enum
from queue import Queue
from typing import Any

from rcsssmj.agent import AgentID
from rcsssmj.client.action import SimAction
from rcsssmj.client.perception import Perception

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


class SimClient(ABC):
    """
    Simulation client controlling a robot in simulation.
    """

    def __init__(self, model_name: str = '', team_name: str = '', player_no: int = -1) -> None:
        """
        Construct a new simulation client representation.
        """

        self._state: SimClientState = SimClientState.READY if len(model_name) > 0 and len(team_name) > 0 and player_no >= 0 else SimClientState.CONNECTED
        self._model_name: str = model_name
        self._team_name: str = team_name
        self._player_no: int = player_no

        self._agent_id: AgentID | None = None
        self._model_spec: Any | None = None
        self._model_markers: list[tuple[str, str]] = []

        self._perceptions: list[Perception] = []
        self._action_queue: Queue[list[SimAction]] = Queue()

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

        self._state = SimClientState.DISCONNECTED

    def reset_perceptions(self) -> None:
        """
        Reset the perception list.

        This method is called by the main simulation loop.
        """

        self._perceptions = []

    def add_perception(self, perception: Perception) -> None:
        """
        Add the given perception to the agent perceptions for this simulation cycle.

        This method is called by the main simulation loop.
        """

        # buffer perception
        self._perceptions.append(perception)

    @abstractmethod
    def send_perceptions(self) -> None:
        """
        Send buffered agent perceptions.

        This method is called by the main simulation loop.
        """

    def __str__(self) -> str:
        return f'{self._team_name} #{self._player_no}'

    def __repr__(self) -> str:
        return f'SimClient({self._model_name}, {self._team_name}, {self._player_no})'
