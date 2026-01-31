from collections.abc import Sequence
from typing import Any, Final

import numpy as np
from mujoco import mjtJoint
from numpy.typing import NDArray

from rcsssmj.agent.perception import Perception
from rcsssmj.agents import AgentID
from rcsssmj.sim_object import SimObject


class SimAgent(SimObject):
    """Abstraction of an simulation agent."""

    def __init__(self, agent_id: AgentID, team_name: str, spec: Any) -> None:
        """Construct a new simulation agent.

        Parameter
        ---------
        agent_id: AgentID
            The id of the agent.

        team_name: str
            The name of the team the agent belongs to.

        spec: Any
            The agent model specification.
        """

        super().__init__(str(agent_id))

        self.agent_id: Final[AgentID] = agent_id
        """The unique id of the agent."""

        self.team_name: Final[str] = team_name
        """The name of the team the agent belongs to."""

        self.spec: Final[Any] = spec
        """The robot model specification."""

        self._markers: Sequence[tuple[str, str]] = []
        """The visible markers of the object model."""

        self._perceptions: Sequence[Perception] = []
        """The current perceptions of the agent."""

        self._joints: list[Any] = []
        """The list of joints data of the agent."""

    @property
    def markers(self) -> Sequence[tuple[str, str]]:
        """The visible markers of the object."""

        return self._markers

    @property
    def perceptions(self) -> Sequence[Perception]:
        """The current perceptions of this agent."""

        return self._perceptions

    def bind(self, mj_model: Any, mj_data: Any) -> None:
        super().bind(mj_model, mj_data)

        # extract visual markers
        prefix_len = len(self.agent_id.prefix)
        self._markers = [(site.name, site.name[prefix_len:-10]) for site in self.spec.sites if site.name.endswith('-vismarker')]

        # extract joint references
        self._joints = [mj_data.joint(jnt_spec.name) for jnt_spec in self.spec.joints if jnt_spec.type in {mjtJoint.mjJNT_HINGE, mjtJoint.mjJNT_SLIDE}]

    @property
    def root_body_name(self) -> str:
        return self.name + '-torso'

    def set_perceptions(self, perceptions: Sequence[Perception]) -> None:
        """Set the perceptions of the agent for this simulation cycle.

        Parameter
        ---------
        perceptions: Sequence[Perception]
            The list of perceptions of the agent for this simulation cycle.
        """

        self._perceptions = perceptions

    def init_joints(
        self,
        pos: NDArray[np.float64] | None = None,
        vel: NDArray[np.float64] | None = None,
        acc: NDArray[np.float64] | None = None,
    ) -> None:
        """Initialize joint states.

        Parameter
        ---------
        pos: NDArray[np.float64] | None, default=None
            The initial joint positions, or ``None`` for initializing all joints to zero position.

        vel: NDArray[np.float64] | None, default=None
            The initial joint velocities, or ``None`` for initializing all joints to zero velocities.

        acc: NDArray[np.float64] | None, default=None
            The initial joint accelerations, or ``None`` for initializing all joints to zero acceleration.
        """

        n_joints = len(self._joints)

        # ensure position value for each joint
        if pos is None:
            pos = np.zeros(n_joints)
        elif len(pos) < n_joints:
            pos = np.pad(pos, (0, n_joints < len(pos)))

        # ensure velocity value for each joint
        if vel is None:
            vel = np.zeros(n_joints)
        elif len(vel) < n_joints:
            vel = np.pad(vel, (0, n_joints < len(vel)))

        # ensure acceleration value for each joint
        if acc is None:
            acc = np.zeros(n_joints)
        elif len(acc) < n_joints:
            acc = np.pad(acc, (0, n_joints < len(acc)))

        for i, jnt in enumerate(self._joints):
            jnt.qpos[:] = pos[i]
            jnt.qvel[:] = vel[i]
            jnt.qacc[:] = acc[i]

    def __str__(self) -> str:
        return f'{self.team_name} #{self.agent_id.player_no}'
