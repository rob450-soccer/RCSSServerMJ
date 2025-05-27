from abc import ABC, abstractmethod
from typing import Any, Final

from rcsssmj.game.referee import SoccerReferee


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

    @abstractmethod
    def perform(self, referee: SoccerReferee, mj_model: Any, mj_data: Any) -> None:
        """Perform this action.

        Parameter
        ---------
        referee: SoccerReferee
            The soccer referee instance.

        mj_model: MjModel
            The mujoco model.

        mj_data: MjData
            The mujoco data array.
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
        self.dq: Final[float] = dq
        self.kp: Final[float] = kp
        self.kd: Final[float] = kd
        self.tau: Final[float] = tau

    def perform(self, referee: SoccerReferee, mj_model: Any, mj_data: Any) -> None:
        del referee  # signal unused parameter

        actuator_tau_model = mj_model.actuator(self.actuator_name + '_tau')
        actuator_tau_data = mj_data.actuator(self.actuator_name + '_tau')
        actuator_pos_model = mj_model.actuator(self.actuator_name + '_pos')
        actuator_pos_data = mj_data.actuator(self.actuator_name + '_pos')
        actuator_vel_model = mj_model.actuator(self.actuator_name + '_vel')
        actuator_vel_data = mj_data.actuator(self.actuator_name + '_vel')
        if actuator_tau_model is not None:
            actuator_tau_data.ctrl = self.tau
            actuator_pos_data.ctrl = self.q
            actuator_vel_data.ctrl = self.dq
            actuator_pos_model.gainprm[0] = self.kp
            actuator_pos_model.biasprm[1] = -self.kp
            actuator_vel_model.gainprm[0] = self.kd
            actuator_vel_model.biasprm[2] = -self.kd


class BeamAction(SimAction):
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

    def perform(self, referee: SoccerReferee, mj_model: Any, mj_data: Any) -> None:
        referee.beam_agent(self.actuator_name, mj_model, mj_data, self.target_pose)


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

    def perform(self, referee: SoccerReferee, mj_model: Any, mj_data: Any) -> None:
        # TODO: implement audio logic
        pass
