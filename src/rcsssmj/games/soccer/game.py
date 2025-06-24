from collections.abc import Mapping, Sequence
from typing import Protocol, runtime_checkable

from rcsssmj.games.game import PGame
from rcsssmj.games.soccer.field import SoccerField
from rcsssmj.games.soccer.game_object import SoccerBall, SoccerPlayer
from rcsssmj.games.soccer.game_state import GameState
from rcsssmj.games.soccer.rules import SoccerRules
from rcsssmj.games.teams import TeamSide


@runtime_checkable
class PSoccerGame(PGame, Protocol):
    """Protocol for a soccer game."""

    @property
    def field(self) -> SoccerField:
        """The soccer field specification of the game."""

    @property
    def rules(self) -> SoccerRules:
        """The soccer game rule book."""

    @property
    def game_state(self) -> GameState:
        """The current soccer game state."""

    @property
    def ball(self) -> SoccerBall:
        """The soccer ball object representation."""

    @property
    def left_players(self) -> Mapping[int, SoccerPlayer]:
        """The active soccer player representations of the left team."""

    @property
    def right_players(self) -> Mapping[int, SoccerPlayer]:
        """The active soccer player representations of the right team."""

    def get_players(self, side: TeamSide | int) -> Mapping[int, SoccerPlayer]:
        """Return the active soccer player representations for the team corresponding to the given side.

        Parameter
        ---------
        side: TeamSide | int
            The team side or side id for which to return the players.
        """

    def get_all_players(self) -> Sequence[SoccerPlayer]:
        """Return all active soccer player representations (from left and right team)."""
