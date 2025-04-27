from dataclasses import dataclass


@dataclass(frozen=True)
class SoccerRules:
    """
    A game rule book.
    """

    max_team_size: int = 11
    """
    The maximum number of players per team.
    """

    max_player_no: int = 99
    """
    The maximum player number a player can choose.
    """

    half_duration: int = 300
    """
    The duration of a game half.
    """

    extra_half_duration: int = 180
    """
    The duration of an extra half.
    """
