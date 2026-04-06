from typing import Any, Final

from rcsssmj.games.soccer.sim.soccer_sim_interfaces import PSoccerSimActionInterface
from rcsssmj.sim.agent_id import AgentID
from rcsssmj.sim.sim_agent import TypedSimAgent
from rcsssmj.utils.mjutils import quat_from_axis_angle


class SoccerAgent(TypedSimAgent[PSoccerSimActionInterface]):
    """A soccer agent object in simulation."""

    def __init__(
        self,
        agent_id: AgentID,
        team_name: str,
        robot_spec: Any,
        ai: PSoccerSimActionInterface,
    ) -> None:
        """Construct a new soccer agent.

        Parameter
        ---------
        agent_id: AgentID
            The unique identifier of the agent.

        team_name: str
            The name of the team the agent belongs to.

        robot_spec: Any
            The robot model specification used for representing the agent in the simulation.

        ai: PSoccerSimActionInterface
            The soccer specific action interface reference.
        """

        super().__init__(agent_id, team_name, robot_spec)

        self.ai: Final[PSoccerSimActionInterface] = ai
        """The action interface."""

    def _get_action_interface(self) -> PSoccerSimActionInterface:
        return self.ai

    def drop(self) -> None:
        """Drop the agent at its current location and reset all joints."""

        base_height: float = self.spec.body(self.root_body_name).pos[2]

        self.place_at((self.xpos[0], self.xpos[1], base_height))
        self.init_joints()

    def drop_at(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
    ) -> None:
        """Drop the agent at the specified location and reset all joints.

        Parameter
        ---------
        x: float, default=0.0
            The x-position at which to drop the agent.

        y: float, default=0.0
            The y-position at which to drop the agent.

        theta: float, default=0.0
            The horizontal orientation with which to drop the agent.
        """

        base_height: float = self.spec.body(self.root_body_name).pos[2]

        quat = quat_from_axis_angle((0, 0, 1), theta)

        self.place_at((x, y, base_height), quat)
        self.init_joints()
