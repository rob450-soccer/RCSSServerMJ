import logging
from abc import ABC, abstractmethod
from typing import Final, Generic, TypeVar

from rcsssmj.sim.sim_interfaces import PSimActionInterface

logger = logging.getLogger(__name__)


class InitRequest:
    """Simple class for holding an initialization request message of a simulation agent client."""

    def __init__(
        self,
        model_name: str,
        team_name: str,
        player_no: int,
    ) -> None:
        """Construct a new initialization request message.

        Parameter
        ---------
        model_name : str
            The name of the robot model.

        team_name : str
            The name of the team.

        player_no : int
            The number of the player.
        """

        self.model_name: Final[str] = model_name
        """The requested robot model name."""

        self.team_name: Final[str] = team_name
        """The requested team name."""

        self.player_no: Final[int] = player_no
        """The requested player number."""


SAI = TypeVar('SAI', bound=PSimActionInterface)


class SimAction(ABC, Generic[SAI]):
    """Base class for simulation actions."""

    def __init__(self, actuator_name: str) -> None:
        """Construct a new simulation action.

        Parameter
        ---------
        actuator_name : str
            The name of the actuator to which this action applies.
        """

        super().__init__()

        self.actuator_name: Final[str] = actuator_name
        """The name of the actuator."""

    @abstractmethod
    def perform(self, ai: SAI) -> None:
        """Perform this action.

        Parameter
        ---------
        ai: SAI
            The simulation action interface.
        """


class MotorAction(SimAction[PSimActionInterface]):
    """Class for representing a motor action."""

    def __init__(self, actuator_name: str, q: float, dq: float, kp: float, kd: float, tau: float):
        """Construct a new motor action.

        Parameter
        ---------
        actuator_name : str
            The name of the actuator to which this action applies.

        q : float
            The target position of the actuator.

        dq : float
            The target velocity of the actuator.

        kp : float
            The proportional gain of the actuator.

        kd : float
            The derivative gain of the actuator.

        tau : float
            The torque of the actuator.
        """

        super().__init__(actuator_name)

        self.q: Final[float] = q
        """The target position of the actuator."""

        self.dq: Final[float] = dq
        """The target velocity of the actuator."""

        self.kp: Final[float] = kp
        """The proportional gain of the actuator."""

        self.kd: Final[float] = kd
        """The derivative gain of the actuator."""

        self.tau: Final[float] = tau
        """The extra torque of the actuator."""

    def perform(self, ai: PSimActionInterface) -> None:
        ai.ctrl_motor(self.actuator_name, self.q, self.dq, self.kp, self.kd, self.tau)


class SpeakerAction(SimAction[PSimActionInterface]):
    """Class for representing a speaker action."""

    def __init__(
        self,
        actuator_name: str,
        volume: float,
        message: bytes | bytearray,
    ):
        """Construct a new speaker action.

        Parameter
        ---------
        actuator_name: str
            The name of the speaker effector.

        volume: float
            The speaker volume gain parameter.

        message: bytes | bytearray
            The message to broadcast.
        """

        super().__init__(actuator_name)

        self.volume: Final[float] = volume
        """The speaker gain parameter."""

        self.message: Final[bytes | bytearray] = message
        """The message to broadcast."""

    def perform(self, ai: PSimActionInterface) -> None:
        ai.say_message(self.actuator_name, self.volume, self.message)
