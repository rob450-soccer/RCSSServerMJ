from abc import ABC, abstractmethod
from typing import Any, Final

import mujoco

from rcsssmj.game.referee import SoccerReferee


class InitRequest:
    """
    Simple class for holding an initialization request message of a simulation client.
    """

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
        self.team_name: Final[str] = team_name
        self.player_no: Final[int] = player_no


class SimAction(ABC):
    """
    Base class for simulation actions.
    """

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
        """
        Perform this action.
        """


class MotorAction(SimAction):
    """
    Class for representing a motor action.
    """

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

        actuator_model = mj_model.actuator(self.actuator_name)
        actuator_data = mj_data.actuator(self.actuator_name)
        if actuator_model is not None:
            joint_id = actuator_model.trnid[0]
            joint_name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            joint_qpos_adr = mj_model.joint(joint_name).qposadr[0]
            joint_qvel_adr = mj_model.joint(joint_name).dofadr[0]
            current_q = mj_data.qpos[joint_qpos_adr]
            current_dq = mj_data.qvel[joint_qvel_adr]
            actuator_data.ctrl = self.kp * (self.q - current_q) + self.kd * (self.dq - current_dq) + self.tau


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
