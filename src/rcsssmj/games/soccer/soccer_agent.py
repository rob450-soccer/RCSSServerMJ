from typing import Any

from rcsssmj.agents import AgentID
from rcsssmj.sim_agent import SimAgent


class SoccerAgent(SimAgent):
    """A soccer agent object in simulation."""

    def __init__(self, agent_id: AgentID, team_name: str, robot_spec: Any) -> None:
        """Construct a new soccer agent."""

        super().__init__(agent_id, team_name, robot_spec)

        self.place_pos: tuple[float, float, float] | None = None
        """The target position to place the agent (if a agent placement is requested by some referee command)."""

        self.place_quat: tuple[float, float, float, float] | None = None
        """The target rotation quaternion to place the agent (if a agent placement is requested)."""

    def relocate(self) -> None:
        """Place the object at the buffered relocation position (if existing)."""

        if self.place_pos is not None:
            quat = (1.0, 0.0, 0.0, 0.0) if self.place_quat is None else self.place_quat

            self.place_at(self.place_pos, quat)
            self.init_joints()

            self.place_pos = None
            self.place_quat = None
