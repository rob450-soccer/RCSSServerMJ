from __future__ import annotations

from typing import Final, Protocol


class PAgent(Protocol):
    """Protocol for agents participating in a game."""

    def get_model_name(self) -> str:
        """Return the name of the robot model associated with the agent."""

    def get_team_name(self) -> str:
        """Return the name of the team the agent belongs to."""

    def get_player_no(self) -> int:
        """Return the player number of the agent."""


class AgentID:
    """Unique identifier for an agent in simulation."""

    def __init__(self, team_id: int, player_no: int):
        """Construct a new agent ID.

        Parameter
        ---------
        team_id: int
            The id of the team the agent belongs to.

        player_no: int
            The agent player number.
        """

        self.team_id: Final[int] = team_id
        """The id of the team the agent belongs to."""

        self.player_no: Final[int] = player_no
        """The agent player number."""

        self.prefix: Final[str] = encode_agent_prefix(team_id, player_no)
        """The model prefix associated to the team id and player number of this agent."""

    def __eq__(self, other: object) -> bool:
        return isinstance(other, AgentID) and self.team_id == other.team_id and self.player_no == other.player_no

    def __hash__(self) -> int:
        return hash((self.team_id, self.player_no))

    def __str__(self) -> str:
        return f'r-{self.team_id}-{self.player_no}'

    def __repr__(self) -> str:
        return f'AgentID(team_id={self.team_id}, player_no={self.player_no})'

    @staticmethod
    def from_prefixed_name(prefixed_name: str) -> AgentID | None:
        """Parse an agent id from the given prefixed name.

        Parameter
        ---------
        prefixed_name: str
            the prefixed name from which to extract the AgentID.
        """

        return decode_agent_prefix(prefixed_name)


def encode_agent_prefix(team_id: int, player_no: int) -> str:
    """Encoded the agent prefix for the given team id and player number.

    Prefix encoding: 'r-<team_id>-<player_no>-'

    Parameter
    ---------
    team_id: int
        The team id of the agent.

    player_no: int
        The player number of the agent.
    """

    return f'r-{team_id}-{player_no}-'


def decode_agent_prefix(prefixed_name: str) -> AgentID | None:
    """Decode the agent id from the given prefix or named entity.

    Prefixed entity: 'r-<team_id>-<player_no>-...'

    Parameter
    ---------
    prefixed_name: str
        A name starting with an agent prefix.
    """

    if not prefixed_name.startswith('r-'):
        return None

    chunks = prefixed_name.split('-')

    if len(chunks) < 3:
        return None

    return AgentID(int(chunks[1]), int(chunks[2]))
