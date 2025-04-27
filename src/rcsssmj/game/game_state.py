from rcsssmj.game.soccer import PlayMode, TeamSide


class GameState:
    """
    A game state.
    """

    def __init__(self) -> None:
        """
        Construct a new game state.
        """

        self._sim_time: float = 0.0
        self._play_time_ms: int = 0
        self._play_mode: PlayMode = PlayMode.BEFORE_KICK_OFF

        self._left_team_name: str | None = None
        self._right_team_name: str | None = None

        self._left_team_score: int = 0
        self._right_team_score: int = 0

    def get_sim_time(self) -> float:
        """
        Return the current simulation time.
        """

        return self._sim_time

    def get_play_time(self) -> float:
        """
        Return the current play time.
        """

        return self._play_time_ms / 1000.0

    def get_play_mode(self) -> PlayMode:
        """
        Return the current play mode.
        """

        return self._play_mode

    def get_team_name(self, side: TeamSide) -> str | None:
        """
        Return the team name for the given team side.
        """

        if side == TeamSide.UNKNOWN:
            return None

        return self._left_team_name if side == TeamSide.LEFT else self._right_team_name

    def get_team_score(self, side: TeamSide) -> int:
        """
        Return the team score for the given team side.
        """

        if side == TeamSide.UNKNOWN:
            return 0

        return self._left_team_score if side == TeamSide.LEFT else self._right_team_score

    def update_team_names(self, team_name: str) -> None:
        """
        Update the available team names of the game.
        """

        if self._left_team_name is None:
            self._left_team_name = team_name
        elif self._left_team_name != team_name and self._right_team_name is None:
            self._right_team_name = team_name
        else:
            # no third team allowed!
            pass

    def get_team_side(self, team_name: str) -> TeamSide:
        """
        Return the team side for the given team name.
        """

        if self._left_team_name == team_name:
            return TeamSide.LEFT

        if self._right_team_name == team_name:
            return TeamSide.RIGHT

        return TeamSide.UNKNOWN

    def progress(self, sim_time: float) -> None:
        """
        Progress the game state by the given time.
        """

        dt = sim_time - self._sim_time
        self._sim_time = sim_time

        if self._play_mode not in (PlayMode.BEFORE_KICK_OFF, PlayMode.GAME_OVER):
            self._play_time_ms += int(0.5 + dt * 1000)

    def kick_off(self, team_side: TeamSide) -> None:
        """
        Instruct kickoff for the given team.
        """

        if team_side == TeamSide.LEFT:
            self._play_mode = PlayMode.KICK_OFF_LEFT

        elif team_side == TeamSide.RIGHT:
            self._play_mode = PlayMode.KICK_OFF_RIGHT

    def play_on(self) -> None:
        """
        Instruct the normal progressing of the game.
        """

        self._play_mode = PlayMode.GAME_OVER

    def throw_in(self, team_side: TeamSide) -> None:
        """
        Instruct a throw in for the given team.
        """

        if team_side == TeamSide.LEFT:
            self._play_mode = PlayMode.THROW_IN_LEFT

        elif team_side == TeamSide.RIGHT:
            self._play_mode = PlayMode.THROW_IN_RIGHT

    def corner_kick(self, team_side: TeamSide) -> None:
        """
        Instruct corner kick for the given team.
        """

        if team_side == TeamSide.LEFT:
            self._play_mode = PlayMode.CORNER_KICK_LEFT

        elif team_side == TeamSide.RIGHT:
            self._play_mode = PlayMode.CORNER_KICK_RIGHT

    def goal_kick(self, team_side: TeamSide) -> None:
        """
        Instruct goal kick for the given team.
        """

        if team_side == TeamSide.LEFT:
            self._play_mode = PlayMode.GOAL_KICK_LEFT

        elif team_side == TeamSide.RIGHT:
            self._play_mode = PlayMode.GOAL_KICK_RIGHT

    def offsite(self, team_side: TeamSide) -> None:
        """
        Offsite state for the given team.
        """

        if team_side == TeamSide.LEFT:
            self._play_mode = PlayMode.OFFSIDE_LEFT

        elif team_side == TeamSide.RIGHT:
            self._play_mode = PlayMode.OFFSIDE_RIGHT

    def game_over(self) -> None:
        """
        Instruct the end of the game.
        """

        self._play_mode = PlayMode.GAME_OVER

    def goal(self, team_side: TeamSide) -> None:
        """
        Count a goal for the given team and set the play mode accordingly.
        """

        if team_side == TeamSide.LEFT:
            self._left_team_score += 1
            self._play_mode = PlayMode.GOAL_LEFT

        elif team_side == TeamSide.RIGHT:
            self._right_team_score += 1
            self._play_mode = PlayMode.GOAL_RIGHT

    def free_kick(self, team_side: TeamSide) -> None:
        """
        Instruct an indirect free kick for the given team.
        """

        if team_side == TeamSide.LEFT:
            self._play_mode = PlayMode.FREE_KICK_LEFT

        elif team_side == TeamSide.RIGHT:
            self._play_mode = PlayMode.FREE_KICK_RIGHT

    def direct_free_kick(self, team_side: TeamSide) -> None:
        """
        Instruct a direct free kick for the given team.
        """

        if team_side == TeamSide.LEFT:
            self._play_mode = PlayMode.DIRECT_FREE_KICK_LEFT

        elif team_side == TeamSide.RIGHT:
            self._play_mode = PlayMode.DIRECT_FREE_KICK_RIGHT

    def penalty_kick(self, team_side: TeamSide) -> None:
        """
        Instruct a penalty kick for the given team.
        """

        if team_side == TeamSide.LEFT:
            self._play_mode = PlayMode.PENALTY_KICK_LEFT

        elif team_side == TeamSide.RIGHT:
            self._play_mode = PlayMode.PENALTY_KICK_RIGHT

    def penalty_shoot(self, team_side: TeamSide) -> None:
        """
        Instruct a penalty shoot for the given team.
        """

        if team_side == TeamSide.LEFT:
            self._play_mode = PlayMode.PENALTY_SHOOT_LEFT

        elif team_side == TeamSide.RIGHT:
            self._play_mode = PlayMode.PENALTY_SHOOT_RIGHT
