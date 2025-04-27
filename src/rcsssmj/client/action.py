from abc import ABC, abstractmethod
from typing import Any, Final

from rcsssmj.game.referee import SoccerReferee


class InitRequest:
    """
    Simple class for holding an initialization request message of a simulation client.
    """

    def __init__(
            self,
            model_name: str,
            team_name: str,
            player_no: int
        ) -> None:
        self.model_name: Final[str] = model_name
        self.team_name: Final[str] = team_name
        self.player_no: Final[int] = player_no


class SimAction(ABC):
    """
    Base class for simulation actions.
    """

    def __init__(self, actuator_name: str) -> None:
        """
        Construct a new simulation action.
        """

        super().__init__()

        self.actuator_name: Final[str] = actuator_name

    @abstractmethod
    def perform(self, referee: SoccerReferee, mj_data: Any) -> None:
        """
        Perform this action.
        """


class MotorAction(SimAction):
    """
    Class for representing a motor action.
    """

    def __init__(self, actuator_name: str, target_val: float):
        """
        Construct a new motor action.
        """

        super().__init__(actuator_name)

        self.target_val: Final[float] = target_val

    def perform(self, referee: SoccerReferee, mj_data: Any) -> None:
        del referee  # signal unused parameter

        actuator = mj_data.actuator(self.actuator_name)
        if actuator is not None:
            actuator.ctrl = self.target_val


class BeamAction(SimAction):
    """
    Class for representing a beam action.
    """

    def __init__(self, actuator_name: str, target_pose: tuple[float, float, float]):
        """
        Construct a new beam action.
        """

        super().__init__(actuator_name)

        self.target_pose: Final[tuple[float, float, float]] = target_pose

    def perform(self, referee: SoccerReferee, mj_data: Any) -> None:
        referee.beam_agent(self.actuator_name, mj_data, self.target_pose)


class SayAction(SimAction):
    """
    Class for representing a say action.
    """

    def __init__(self, actuator_name: str, message: str):
        """
        Construct a new say action.
        """

        super().__init__(actuator_name)

        self.message: Final[str] = message

    def perform(self, referee: SoccerReferee, mj_data: Any) -> None:
        # TODO: implement audio logic
        pass
