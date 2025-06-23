from typing import Protocol, runtime_checkable

from rcsssmj.game.soccer import TeamSide
from rcsssmj.simulation_interfaces import PSimActionInterface, PSimCommandInterface


@runtime_checkable
class PSoccerSimActionInterface(PSimActionInterface, Protocol):
    """Protocol for a soccer action interface."""

    def beam_agent(self, actuator_name: str, beam_pose: tuple[float, float, float]) -> None:
        """Perform a beam action for the agent posing the given effector.

        Parameter
        ---------
        actuator_name: str
            The name of the beam actuator.

        beam_pose: tuple[float, float, float]
            The desired 2D beam pose (x, y, theta).
            Theta is given in radians.
        """


@runtime_checkable
class PSoccerSimCommandInterface(PSimCommandInterface, Protocol):
    """Protocol for the soccer command interface."""

    def request_kick_off(self, team_side: TeamSide | int) -> None:
        """Instruct kickoff for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the kick off.
        """

    def request_drop_ball(self, pos: tuple[float, float] | None = None) -> None:
        """Drop the ball at the specified position and instruct the normal progressing of the game.

        Parameter
        ---------
        pos: tuple[float, float] | None, default=None
            The position at which to drop the ball or none, to drop it at its current location.
        """
