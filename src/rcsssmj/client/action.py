import logging
from abc import ABC, abstractmethod
from typing import Final

from rcsssmj.game.soccer_sim_interfaces import PSoccerSimActionInterface
from rcsssmj.simulation_interfaces import PSimActionInterface

logger = logging.getLogger(__name__)


class InitRequest:
    """Simple class for holding an initialization request message of a simulation client."""

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


class SimAction(ABC):
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
    def perform(self, ai: PSimActionInterface) -> None:
        """Perform this action.

        Parameter
        ---------
        ai: PSimActionInterface
            The simulation action interface.
        """


class MotorAction(SimAction):
    """Class for representing a motor action."""

    def __init__(self, actuator_name: str, q: float, dq: float, kp: float, kd: float, tau: float):
        """Construct a new motor action, which produces a torque on the actuator via a PD controller:
        applied_torque = kp * (q - q_current) + kd * (dq - dq_current) + tau

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
        actuator_tau_model = ai.mj_model.actuator(self.actuator_name + '_tau')
        actuator_tau_data = ai.mj_data.actuator(self.actuator_name + '_tau')
        actuator_pos_model = ai.mj_model.actuator(self.actuator_name + '_pos')
        actuator_pos_data = ai.mj_data.actuator(self.actuator_name + '_pos')
        actuator_vel_model = ai.mj_model.actuator(self.actuator_name + '_vel')
        actuator_vel_data = ai.mj_data.actuator(self.actuator_name + '_vel')
        if actuator_tau_model is not None:
            actuator_tau_data.ctrl = self.tau
            actuator_pos_data.ctrl = self.q
            actuator_vel_data.ctrl = self.dq
            actuator_pos_model.gainprm[0] = self.kp
            actuator_pos_model.biasprm[1] = -self.kp
            actuator_vel_model.gainprm[0] = self.kd
            actuator_vel_model.biasprm[2] = -self.kd


class SoccerSimAction(SimAction):
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
