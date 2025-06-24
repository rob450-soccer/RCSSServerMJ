from __future__ import annotations

from enum import Enum


class TeamSide(Enum):
    """Enum specifying team sides."""

    UNKNOWN = -1
    """The unknown / invalid team side used for initialization and / or to signal an invalid play side."""

    LEFT = 0
    """The left play side."""

    RIGHT = 1
    """The right play side."""

    @staticmethod
    def is_valid(team_id: int) -> bool:
        """Check if the given team id is a valid team side (LEFT or RIGHT).

        Parameter
        ---------
        team_id: int
            The team id to check.
        """

        return team_id in (TeamSide.LEFT.value, TeamSide.RIGHT.value)

    @staticmethod
    def from_id(team_id: int) -> TeamSide:
        """Return the team side for the given team id.

        Parameter
        ---------
        team_id: int
            The team id.
        """

        if team_id == TeamSide.LEFT.value:
            return TeamSide.LEFT

        if team_id == TeamSide.RIGHT.value:
            return TeamSide.RIGHT

        return TeamSide.UNKNOWN

    @staticmethod
    def get_opposing_side(team_id: TeamSide | int) -> TeamSide:
        """Return the opposite team.

        Parameter
        ---------
        team_id: int
            The team id.
        """

        if not isinstance(team_id, TeamSide):
            team_id = TeamSide.from_id(team_id)

        if team_id == TeamSide.LEFT:
            return TeamSide.RIGHT

        if team_id == TeamSide.RIGHT:
            return TeamSide.LEFT

        return TeamSide.UNKNOWN
