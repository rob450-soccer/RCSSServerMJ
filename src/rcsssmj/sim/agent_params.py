from typing import Protocol


class PAgentParameter(Protocol):
    """Protocol for agent parameter collections."""

    @property
    def model_name(self) -> str:
        """The name of the robot model associated with the agent."""

    @property
    def team_name(self) -> str:
        """The name of the team the agent belongs to."""

    @property
    def player_no(self) -> int:
        """The player number of the agent."""
