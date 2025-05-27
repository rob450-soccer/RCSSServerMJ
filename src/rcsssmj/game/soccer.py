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


class PlayMode(Enum):
    """Enum specifying possible play modes."""

    BEFORE_KICK_OFF = 'BeforeKickOff'
    """The game hasn't started yet."""

    KICK_OFF_LEFT = 'KickOff_Left'
    """The left team has kick off."""

    KICK_OFF_RIGHT = 'KickOff_Right'
    """The right team has kick off."""

    PLAY_ON = 'PlayOn'
    """The game is running normal."""

    THROW_IN_LEFT = 'KickIn_Left'
    """The ball left the field and the left team has throw in."""

    THROW_IN_RIGHT = 'KickIn_Right'
    """The ball left the field and the right team has throw in."""

    CORNER_KICK_LEFT = 'corner_kick_left'
    """The left team has corner kick."""

    CORNER_KICK_RIGHT = 'corner_kick_right'
    """The right team has corner kick."""

    GOAL_KICK_LEFT = 'goal_kick_left'
    """The left team has goal kick."""

    GOAL_KICK_RIGHT = 'goal_kick_right'
    """The right team has goal kick."""

    OFFSIDE_LEFT = 'offside_left'
    """The right team violated the offside rule."""

    OFFSIDE_RIGHT = 'offside_right'
    """The left team violated the offside rule."""

    GAME_OVER = 'GameOver'
    """The game has ended."""

    GOAL_LEFT = 'Goal_Left'
    """The left team scored a goal."""

    GOAL_RIGHT = 'Goal_Right'
    """The right team scored a goal."""

    FREE_KICK_LEFT = 'free_kick_left'
    """The left team has a free kick."""

    FREE_KICK_RIGHT = 'free_kick_right'
    """The right team has a free kick."""

    DIRECT_FREE_KICK_LEFT = 'direct_free_kick_left'
    """The left team has a direct free kick."""

    DIRECT_FREE_KICK_RIGHT = 'direct_free_kick_right'
    """The right team has a direct free kick."""

    PENALTY_KICK_LEFT = 'penalty_kick_left'
    """The left team has a penalty kick (from the penalty spot)."""

    PENALTY_KICK_RIGHT = 'penalty_kick_right'
    """The right team has a penalty kick (from the penalty spot)."""

    PENALTY_SHOOT_LEFT = 'penalty_shoot_left'
    """The left team has a penalty shoot (starting from somewhere on the field, allowed to touch the ball more than once)."""

    PENALTY_SHOOT_RIGHT = 'penalty_shoot_right'
    """The right team has a penalty shoot (starting from somewhere on the field, allowed to touch the ball more than once)."""
