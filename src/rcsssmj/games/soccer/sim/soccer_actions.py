import logging
from typing import Final

from rcsssmj.games.soccer.sim.soccer_sim_interfaces import PSoccerSimActionInterface
from rcsssmj.sim.actions import SimAction

logger = logging.getLogger(__name__)


class BeamAction(SimAction[PSoccerSimActionInterface]):
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

    def perform(self, sai: PSoccerSimActionInterface) -> None:
        sai.beam_agent(self.actuator_name, self.target_pose)
