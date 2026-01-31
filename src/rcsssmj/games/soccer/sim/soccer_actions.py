import logging
from abc import ABC, abstractmethod
from typing import Final

from rcsssmj.games.soccer.sim.soccer_sim_interfaces import PSoccerSimActionInterface
from rcsssmj.sim.actions import SimAction
from rcsssmj.sim.sim_interfaces import PSimActionInterface

logger = logging.getLogger(__name__)


class SoccerSimAction(SimAction, ABC):
    """Base class for soccer simulation actions."""

    def __init__(self, actuator_name: str) -> None:
        """Construct a new soccer simulation action.

        Parameter
        ---------
        actuator_name : str
            The name of the actuator to which this action applies.
        """

        super().__init__(actuator_name)

    def perform(self, ai: PSimActionInterface) -> None:
        if isinstance(ai, PSoccerSimActionInterface):
            self._perform(ai)
        else:
            logger.warning('Expected soccer simulation action interface instance in soccer simulation action!')

    @abstractmethod
    def _perform(self, sai: PSoccerSimActionInterface) -> None:
        """Perform this action.

        Parameter
        ---------
        sai: PSoccerSimActionInterface
            The soccer simulation action interface.
        """


class BeamAction(SoccerSimAction):
    """Class for representing a beam action."""

    def __init__(self, actuator_name: str, target_pose: tuple[float, float, float]):
        """Construct a new beam action.

        Parameter
        ---------
        actuator_name: str
            The name of the beam effector.

        target_pose: tuple[float, float, float]
            The desired target 2D beam pose [x, y, theta].
        """

        super().__init__(actuator_name)

        self.target_pose: Final[tuple[float, float, float]] = target_pose
        """The desired target 2D beam pose [x, y, theta]"""

    def _perform(self, sai: PSoccerSimActionInterface) -> None:
        sai.beam_agent(self.actuator_name, self.target_pose)


class SayAction(SoccerSimAction):
    """Class for representing a say action."""

    def __init__(self, actuator_name: str, message: str):
        """Construct a new say action.

        Parameter
        ---------
        actuator_name: str
            The name of the beam effector.

        message: str
            The message to say.
        """

        super().__init__(actuator_name)

        self.message: Final[str] = message
        """The message to say."""

    def _perform(self, sai: PSoccerSimActionInterface) -> None:
        # TODO: implement audio logic
        pass
