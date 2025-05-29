from rcsssmj.game.soccer import PlayMode, TeamSide


class GameState:
    """A game state."""

    def __init__(self) -> None:
        """Construct a new game state."""

        self._sim_time: float = 0.0
        self._play_time_ms: int = 0
        self._play_mode: PlayMode = PlayMode.BEFORE_KICK_OFF

        self._left_team_name: str | None = None
        self._right_team_name: str | None = None

        self._left_team_score: int = 0
        self._right_team_score: int = 0

        self._play_mode_history_ms: dict[PlayMode, float] = {pm: 0 for pm in PlayMode}

    def get_sim_time(self) -> float:
        """Return the current simulation time."""

        return self._sim_time

    def get_play_time(self) -> float:
        """Return the current play time."""

        return self._play_time_ms / 1000.0

    def get_play_mode(self) -> PlayMode:
        """Return the current play mode."""

        return self._play_mode

    def get_team_name(self, side: TeamSide) -> str | None:
        """Return the team name for the given team side."""

        if side == TeamSide.UNKNOWN:
            return None

        return self._left_team_name if side == TeamSide.LEFT else self._right_team_name

    def get_team_score(self, side: TeamSide) -> int:
        """Return the team score for the given team side."""

        if side == TeamSide.UNKNOWN:
            return 0

        return self._left_team_score if side == TeamSide.LEFT else self._right_team_score

    def get_play_mode_time(self, play_mode: PlayMode | None = None) -> float:
        """Return the play time given play mode has last been activated.

        Parameter
        ---------
        play_mode: PlayMode | None, default=None
            The play mode for which to return the last activation time, or None to use the currently active play mode.
        """

        return self._play_mode_history_ms[self._play_mode if play_mode is None else play_mode] / 1000.0

    def get_play_mode_age(self, play_mode: PlayMode | None = None) -> float:
        """Return the play time that has passed since the play mode has been set.

        Parameter
        ---------
        play_mode: PlayMode | None, default=None
            The play mode for which to return the age, or None to use the currently active play mode.
        """

        return (self._play_time_ms - self._play_mode_history_ms[self._play_mode if play_mode is None else play_mode]) / 1000.0

    def update_team_names(self, team_name: str) -> None:
        """Update the available team names of the game."""

        if self._left_team_name is None:
            self._left_team_name = team_name
        elif self._left_team_name != team_name and self._right_team_name is None:
            self._right_team_name = team_name
        else:
            # no third team allowed!
            pass

    def get_team_side(self, team_name: str) -> TeamSide:
        """Return the team side for the given team name."""

        if self._left_team_name == team_name:
            return TeamSide.LEFT

        if self._right_team_name == team_name:
            return TeamSide.RIGHT

        return TeamSide.UNKNOWN

    def set_play_mode(self, play_mode: PlayMode) -> None:
        """Set the play mode of the game state.

        Parameter
        ---------
        play_mode: PlayMode
            The new play mode to set.
        """

        self._play_mode = play_mode
        self._play_mode_history_ms[play_mode] = self._play_time_ms

    def set_play_mode_for_team(self, team_side: TeamSide, play_mode_left: PlayMode, play_mode_right: PlayMode) -> None:
        """Set the play mode of the game state based on the given team side.

        Parameter
        ---------
        team_side: TeamSide
            The team side.

        play_mode_left: PlayMode
            The play mode for the left team side.

        play_mode_right: PlayMode
            The play mode for the right team side.
        """

        if team_side == TeamSide.LEFT:
            self.set_play_mode(play_mode_left)

        elif team_side == TeamSide.RIGHT:
            self.set_play_mode(play_mode_right)

    def set_score(self, team_side: TeamSide, score: int) -> None:
        """Set the score for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team for which to set the score.

        score: int
            The team score.
        """

        if team_side == TeamSide.LEFT:
            self._left_team_score = score

        elif team_side == TeamSide.RIGHT:
            self._right_team_score = score

    def update(self, sim_time: float, *, progress_play_time: bool = True) -> None:
        """Update the game state with the given simulation time.

        Parameter
        ---------
        sim_time: float
            The current simulation time.

        progress_play_time: bool = True
            Flag if the play time should be progressed based on simulation time.
        """

        dt = sim_time - self._sim_time
        self._sim_time = sim_time

        if progress_play_time:
            self._play_time_ms += int(0.5 + dt * 1000)

    def goal(self, team_side: TeamSide) -> None:
        """Count a goal for the given team and set the play mode accordingly.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the goal.
        """

        if team_side == TeamSide.LEFT:
            self._left_team_score += 1
            self.set_play_mode(PlayMode.GOAL_LEFT)

        elif team_side == TeamSide.RIGHT:
            self._right_team_score += 1
            self.set_play_mode(PlayMode.GOAL_RIGHT)
