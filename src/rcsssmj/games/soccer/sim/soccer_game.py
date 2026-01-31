from collections.abc import Mapping
from typing import Protocol, runtime_checkable

from rcsssmj.games.game import PGame
from rcsssmj.games.soccer.sim.soccer_agent import SoccerAgent
from rcsssmj.games.soccer.sim.soccer_ball import SoccerBall
from rcsssmj.games.soccer.sim.soccer_game_state import GameState
from rcsssmj.games.soccer.soccer_fields import SoccerField
from rcsssmj.games.soccer.soccer_rules import SoccerRules
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
    def left_players(self) -> Mapping[int, SoccerAgent]:
        """The active soccer player representations of the left team."""

    @property
    def right_players(self) -> Mapping[int, SoccerAgent]:
        """The active soccer player representations of the right team."""

    def get_players(self, side: TeamSide | int) -> Mapping[int, SoccerAgent]:
        """Return the active soccer player representations for the team corresponding to the given side.

        Parameter
        ---------
        side: TeamSide | int
            The team side or side id for which to return the players.
        """
