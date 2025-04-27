from __future__ import annotations

from typing import Final, Protocol


class PAgent(Protocol):
    """
    Protocol for agents participating in a game.
    """

    def get_model_name(self) -> str:
        """
        Return the name of the robot model associated with the agent.
        """

    def get_team_name(self) -> str:
        """
        Return the name of the team the agent belongs to.
        """

    def get_player_no(self) -> int:
        """
        Return the player number of the agent.
        """


class AgentID:
    """
    Unique identifier for an agent in simulation.
    """

    def __init__(self, team_id: int, player_no: int):
        """
        Construct a new agent ID.
        """

        self.team_id: Final[int] = team_id
        self.player_no: Final[int] = player_no
        self.prefix: Final[str] = encode_agent_prefix(team_id, player_no)

    @staticmethod
    def from_prefixed_name(prefixed_name: str) -> AgentID:
        """
        Parse an agent id from the given prefixed name.
        """

        return decode_agent_prefix(prefixed_name)


def encode_agent_prefix(team_id: int, player_no: int) -> str:
    """
    Encoded the agent prefix for the given team id and player number.

    Prefix encoding: 'r-<team_id>-<player_no>-'
    """

    return f'r-{team_id}-{player_no}-'


def decode_agent_prefix(prefixed_name: str) -> AgentID:
    """
    Decode the agent id from the given prefix or named entity.

    Prefixed entity: 'r-<team_id>-<player_no>-...'
    """

    chunks = prefixed_name.split('-')
    return AgentID(int(chunks[1]), int(chunks[2]))
