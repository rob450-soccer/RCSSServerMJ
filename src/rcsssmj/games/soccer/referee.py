import logging
from typing import TYPE_CHECKING, cast

from rcsssmj.games.soccer.game import PSoccerGame
from rcsssmj.games.soccer.play_mode import PlayMode
from rcsssmj.games.teams import TeamSide

if TYPE_CHECKING:
    from rcsssmj.agents import AgentID

logger = logging.getLogger(__name__)


class SoccerReferee:
    """A referee, applying soccer game rules."""

    def __init__(self, game: PSoccerGame | None = None) -> None:
        """Create a new soccer referee.

        Parameter
        ---------
        game: PSoccerGame | None, default=None
            The soccer game instance to referee. If ``None``, the game instance must be injected from externally before calling any method of the referee.
        """

        # MYPY-HACK: The game instance may initially be `None` but will be automatically injected in this case by the simulation (cast is used to silence mypy while preventing repetitive `None` checks)
        self.game: PSoccerGame = cast(PSoccerGame, game)
        """The soccer game instance to referee."""

        self._agent_na_touch_ball: AgentID | None = None
        """The agent not allowed to touch the ball a second time (if existing)."""

        self._team_na_score: TeamSide | None = None
        """The team side, which is not allowed to score a goal until another agent touches the ball again (if existing)."""

        self._did_act: bool = False
        """Flag if the referee has already taken a decision in this referee cycle."""

    def reset(self) -> None:
        """Reinitialize the referee."""

        # init referee state
        self._agent_na_touch_ball = None
        self._team_na_score = None
        self._did_act = False

    def is_beaming_allowed(self) -> bool:
        """Check if an agent is allowed to beam in the current game state."""

        return self.game.game_state.get_play_mode() in (PlayMode.BEFORE_KICK_OFF, PlayMode.GOAL_LEFT, PlayMode.GOAL_RIGHT)

    def kick_off(self, team_side: TeamSide | int) -> None:
        """Instruct kickoff for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the kick off.
        """

        if not isinstance(team_side, TeamSide):
            team_side = TeamSide.from_id(team_side)

        self._did_act = True
        self.game.game_state.set_play_mode_for_team(team_side, PlayMode.KICK_OFF_LEFT, PlayMode.KICK_OFF_RIGHT)
        self.game.ball.place_pos = (0, 0)
        self.game.ball.reset_contacts()
        self._agent_na_touch_ball = None
        self._team_na_score = team_side

    def play_on(self) -> None:
        """Instruct the normal progressing of the game."""

        self._did_act = True
        self.game.game_state.set_play_mode(PlayMode.PLAY_ON)

    def throw_in(self, team_side: TeamSide) -> None:
        """Instruct a throw in for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the throw in.
        """

        self._did_act = True
        self.game.game_state.set_play_mode_for_team(team_side, PlayMode.THROW_IN_LEFT, PlayMode.THROW_IN_RIGHT)
        y = self.game.field.field_area.min_y if self.game.ball.position[1] < 0 else self.game.field.field_area.max_y
        self.game.ball.place_pos = (self.game.ball.position[0], y)
        self.game.ball.reset_contacts()
        self._agent_na_touch_ball = None
        self._team_na_score = None

    def corner_kick(self, team_side: TeamSide) -> None:
        """Instruct corner kick for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the corner kick.
        """

        self._did_act = True
        self.game.game_state.set_play_mode_for_team(team_side, PlayMode.CORNER_KICK_LEFT, PlayMode.CORNER_KICK_RIGHT)
        x = self.game.field.field_area.max_x if team_side == TeamSide.LEFT else self.game.field.field_area.min_x
        y = self.game.field.field_area.min_y if self.game.ball.position[1] < 0 else self.game.field.field_area.max_y
        self.game.ball.place_pos = (x, y)
        self.game.ball.reset_contacts()
        self._agent_na_touch_ball = None
        self._team_na_score = None

    def goal_kick(self, team_side: TeamSide) -> None:
        """Instruct goal kick for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the goal kick.
        """

        self._did_act = True
        self.game.game_state.set_play_mode_for_team(team_side, PlayMode.GOAL_KICK_LEFT, PlayMode.GOAL_KICK_RIGHT)
        self.game.ball.place_pos = self.game.field.left_goalie_area.center() if team_side == TeamSide.LEFT else self.game.field.right_goalie_area.center()
        self.game.ball.reset_contacts()
        self._agent_na_touch_ball = None
        self._team_na_score = None

    def offsite(self, team_side: TeamSide) -> None:
        """Offsite state for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the offside.
        """

        self._did_act = True
        self.game.game_state.set_play_mode_for_team(team_side, PlayMode.OFFSIDE_LEFT, PlayMode.OFFSIDE_RIGHT)

    def game_over(self) -> None:
        """Instruct the end of the game."""

        self._did_act = True
        self.game.game_state.set_play_mode(PlayMode.GAME_OVER)
        self.game.ball.reset_contacts()
        self._agent_na_touch_ball = None
        self._team_na_score = None

    def goal(self, team_side: TeamSide) -> None:
        """Count a goal for the given team and set the play mode accordingly.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the goal.
        """

        self._did_act = True
        self.game.game_state.goal(team_side)
        self._agent_na_touch_ball = None
        self._team_na_score = None

    def free_kick(self, team_side: TeamSide) -> None:
        """Instruct an indirect free kick for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the free kick.
        """

        self._did_act = True
        self.game.game_state.set_play_mode_for_team(team_side, PlayMode.FREE_KICK_LEFT, PlayMode.FREE_KICK_RIGHT)
        self.game.ball.place_pos = self.game.ball.position[0:2]
        self.game.ball.reset_contacts()
        self._agent_na_touch_ball = None
        self._team_na_score = team_side

    def direct_free_kick(self, team_side: TeamSide) -> None:
        """Instruct a direct free kick for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the direct free kick.
        """

        self._did_act = True
        self.game.game_state.set_play_mode_for_team(team_side, PlayMode.DIRECT_FREE_KICK_LEFT, PlayMode.DIRECT_FREE_KICK_RIGHT)
        self.game.ball.place_pos = self.game.ball.position[0:2]
        self.game.ball.reset_contacts()
        self._agent_na_touch_ball = None
        self._team_na_score = None

    def penalty_kick(self, team_side: TeamSide) -> None:
        """Instruct a penalty kick for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the penalty kick.
        """

        self._did_act = True
        self.game.game_state.set_play_mode_for_team(team_side, PlayMode.PENALTY_KICK_LEFT, PlayMode.PENALTY_KICK_RIGHT)
        penalty_spot_x = self.game.field.field_area.max_x - self.game.field.penalty_spot_distance
        self.game.ball.place_pos = (-penalty_spot_x if team_side == TeamSide.LEFT else penalty_spot_x, 0)
        self.game.ball.reset_contacts()
        self._agent_na_touch_ball = None
        self._team_na_score = None

    def penalty_shoot(self, team_side: TeamSide) -> None:
        """Instruct a penalty shoot for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the penalty shoot.
        """

        self._did_act = True
        self.game.game_state.set_play_mode_for_team(team_side, PlayMode.PENALTY_SHOOT_LEFT, PlayMode.PENALTY_SHOOT_RIGHT)
        # TODO: place ball for penalty-shoot

    def drop_ball(self, pos: tuple[float, float] | None = None) -> None:
        """Drop the ball at the specified position and instruct the normal progressing of the game.

        Parameter
        ---------
        pos: tuple[float, float] | None, default=None
            The position at which to drop the ball or none, to drop it at its current location.
        """

        self._did_act = True
        self.game.ball.place_pos = self.game.ball.position[0:2] if pos is None else pos
        # TODO: cause relocation of all agents nearby the ball (within a radius defined here)

        self.game.game_state.set_play_mode(PlayMode.PLAY_ON)

    def move_player(
        self,
        player_id: int,
        team_name: str,
        pos: tuple[float, float, float],
        quat: tuple[float, float, float, float] | None = None,
    ) -> None:
        """Move the specified player to the specified position.

        Parameter
        ---------
        player_id: int
            The unique id of the player in its team.

        team_name: str
            The name of the team the player plays in or "Left" or "Right" for the left or the right team.

        pos: tuple[float, float, float]
            The position to which to move the player.

        quat: tuple[float, float, float, float], default=None
            The 3D rotation quaternion of the torso.
        """

        # check if team exists
        team_id = TeamSide.UNKNOWN
        if team_name == 'Left' or team_name == self.game.game_state.get_team_name(TeamSide.LEFT):
            team_id = TeamSide.LEFT
        elif team_name == 'Right' or team_name == self.game.game_state.get_team_name(TeamSide.RIGHT):
            team_id = TeamSide.RIGHT
        else:
            logger.warning('Team %s does not exist!', team_name)
            return

        # check if player exists
        player = self.game.get_players(team_id).get(player_id, None)
        if player is not None:
            player.place_pos = pos
            player.place_quat = quat
        else:
            logger.warning('Player %d of team %s does not exist!', player_id, team_name)

    def referee(self) -> None:
        """Referee the current game situation."""

        # update game state times
        self.game.game_state.update(self.game.sim_time, progress_play_time=self.game.game_state.get_play_mode() not in (PlayMode.BEFORE_KICK_OFF, PlayMode.GAME_OVER))

        # check game over
        if self.game.game_state.get_play_time() >= self.game.rules.half_time:
            self.game_over()
            return

        # check for rule violations
        self._check_fouls()

        # automatically progress play mode based on timeouts, object locations and action triggers
        self._check_timeouts()
        self._check_location_triggers()
        self._check_contact_triggers()

        # relocate players (according to play mode restrictions)
        self._relocate_misplaced_players()

        # reset decision flag
        self._did_act = False

    def _check_fouls(self) -> None:
        """Check fouls / violations of game rules."""

        # check no score rule
        if self._team_na_score is not None and self.game.ball.active_contact is not None and self.game.ball.last_contact is not None:
            self._team_na_score = None

        # check double-touch rule
        if self._agent_na_touch_ball is not None and self.game.ball.active_contact is not None and self.game.ball.last_contact is not None:
            if self._agent_na_touch_ball == self.game.ball.last_contact and self._agent_na_touch_ball == self.game.ball.active_contact:
                self.free_kick(TeamSide.get_opposing_side(self._agent_na_touch_ball.team_id))
                return

            self._agent_na_touch_ball = None

    def _check_timeouts(self) -> None:
        """Check timeouts (kick-off time, throw-in time, etc.) for the current play mode."""

        if self._did_act:
            # the referee has already taken a decision in this simulation cycle
            return

        pm = self.game.game_state.get_play_mode()

        if pm == PlayMode.PLAY_ON:
            # shortcut, as remaining rules only apply to other states than play-on
            return

        def check_timeout(timeout: int, *play_modes: PlayMode) -> bool:
            """Helper function for checking a play mode specific timeout."""
            return timeout >= 0 and pm in play_modes and self.game.game_state.get_play_mode_age() > timeout

        # check kick-off, throw-in, corner-kick and free-kick times
        if (
            check_timeout(self.game.rules.kick_off_time, PlayMode.KICK_OFF_LEFT, PlayMode.KICK_OFF_RIGHT)
            or check_timeout(self.game.rules.throw_in_time, PlayMode.THROW_IN_LEFT, PlayMode.THROW_IN_RIGHT)
            or check_timeout(self.game.rules.corner_kick_time, PlayMode.CORNER_KICK_LEFT, PlayMode.CORNER_KICK_RIGHT)
            or check_timeout(self.game.rules.free_kick_time, PlayMode.FREE_KICK_LEFT, PlayMode.FREE_KICK_RIGHT)
            or check_timeout(self.game.rules.direct_free_kick_time, PlayMode.DIRECT_FREE_KICK_LEFT, PlayMode.DIRECT_FREE_KICK_RIGHT)
        ):
            self.play_on()
            return

        # check goal-kick times
        if check_timeout(self.game.rules.goal_kick_time, PlayMode.GOAL_KICK_LEFT):
            # drop ball at a corner of the left goalie area
            self.drop_ball((self.game.field.left_goalie_area.max_x, self.game.field.left_goalie_area.max_y))
            return

        if check_timeout(self.game.rules.goal_kick_time, PlayMode.GOAL_KICK_RIGHT):
            self.drop_ball((self.game.field.right_goalie_area.min_x, self.game.field.right_goalie_area.max_y))
            return

        # check goal pause time
        if pm == PlayMode.GOAL_LEFT and self.game.game_state.get_play_mode_age() > self.game.rules.goal_pause_time:
            self.kick_off(TeamSide.RIGHT)
            return

        if pm == PlayMode.GOAL_RIGHT and self.game.game_state.get_play_mode_age() > self.game.rules.goal_pause_time:
            self.kick_off(TeamSide.LEFT)
            return

    def _check_location_triggers(self) -> None:
        """Check location triggers (ball leaving the field in play-on, leaving the goalie-area in goal-kick, etc.) for the current play mode."""

        if self._did_act:
            # the referee has already taken a decision in this simulation cycle
            return

        pm = self.game.game_state.get_play_mode()

        if pm in (PlayMode.GOAL_LEFT, PlayMode.GOAL_RIGHT):
            # no location triggers in goal states
            return

        # check left goal
        if self.game.field.left_goal_box.contains(self.game.ball.position[0], self.game.ball.position[1], self.game.ball.position[2]):
            if pm == PlayMode.GOAL_KICK_LEFT:
                # drop ball at a corner of the left goalie area
                self.drop_ball((self.game.field.left_goalie_area.max_x, self.game.field.left_goalie_area.max_y))
            elif self._team_na_score == TeamSide.RIGHT:
                self.goal_kick(TeamSide.LEFT)
            else:
                self.goal(TeamSide.RIGHT)
            return

        # check right goal
        if self.game.field.right_goal_box.contains(self.game.ball.position[0], self.game.ball.position[1], self.game.ball.position[2]):
            if pm == PlayMode.GOAL_KICK_RIGHT:
                # drop ball at a corner of the left goalie area
                self.drop_ball((self.game.field.right_goalie_area.min_x, self.game.field.right_goalie_area.max_y))
            elif self._team_na_score == TeamSide.LEFT:
                self.goal_kick(TeamSide.RIGHT)
            else:
                self.goal(TeamSide.LEFT)
            return

        # check if the ball left the field
        if not self.game.field.field_area.contains(self.game.ball.position[0], self.game.ball.position[1]):
            agent_contact = self.game.ball.get_most_recent_contact()
            last_team_contact = TeamSide.UNKNOWN if agent_contact is None else TeamSide.from_id(agent_contact.team_id)

            if self.game.ball.position[0] < self.game.field.field_area.min_x:
                # corner-kick right / goal-kick left
                if last_team_contact == TeamSide.LEFT:
                    self.corner_kick(TeamSide.RIGHT)
                else:
                    self.goal_kick(TeamSide.LEFT)

            elif self.game.ball.position[0] > self.game.field.field_area.max_x:
                # corner-kick left / goal-kick right
                if last_team_contact == TeamSide.RIGHT:
                    self.corner_kick(TeamSide.LEFT)
                else:
                    self.goal_kick(TeamSide.RIGHT)

            # elif self.game.ball.position[1] < -self.game.field.field_area.min_y or self.game.ball.position[1] > self.game.field.field_area.max_y:
            else:
                # throw-in
                self.throw_in(TeamSide.get_opposing_side(last_team_contact))

            return

        # check if ball left the goalie area on goal-kick
        if (pm == PlayMode.GOAL_KICK_LEFT and not self.game.field.left_goalie_area.contains(self.game.ball.position[0], self.game.ball.position[1])) or (
            pm == PlayMode.GOAL_KICK_RIGHT and not self.game.field.right_goalie_area.contains(self.game.ball.position[0], self.game.ball.position[1])
        ):
            self.play_on()
            return

    def _check_contact_triggers(self) -> None:
        """Check contact action triggers (touching the ball in kick-off, throw-in, etc.) for the current play mode."""

        if self._did_act:
            # the referee has already taken a decision in this simulation cycle
            return

        if self.game.ball.active_contact is None:
            # no action trigger
            return

        pm = self.game.game_state.get_play_mode()

        if pm == PlayMode.PLAY_ON:
            # shortcut, as remaining rules only apply to other states than play-on
            return

        # kick-off, throw-in, corner-kick and free kicks
        if pm in (
            PlayMode.KICK_OFF_LEFT,
            PlayMode.KICK_OFF_RIGHT,
            PlayMode.THROW_IN_LEFT,
            PlayMode.THROW_IN_RIGHT,
            PlayMode.CORNER_KICK_LEFT,
            PlayMode.CORNER_KICK_RIGHT,
            PlayMode.FREE_KICK_LEFT,
            PlayMode.FREE_KICK_RIGHT,
            PlayMode.DIRECT_FREE_KICK_LEFT,
            PlayMode.DIRECT_FREE_KICK_RIGHT,
        ):
            if len(self.game.get_players(self.game.ball.active_contact.team_id)) > 1:
                self._agent_na_touch_ball = self.game.ball.active_contact

            self.play_on()
            return

    def _relocate_misplaced_players(self) -> None:
        """Check if players are within areas they are not allowed in and, in case, relocate them accordingly.

        Parameter
        ---------
        mj_model: MjModel
            The mujoco simulation model.

        mj_data: MjData
            The mujoco simulation data array.
        """

        pm = self.game.game_state.get_play_mode()

        if pm == PlayMode.KICK_OFF_LEFT:
            # TODO: relocate all players of the left team that are on the right side and all players of the right team that are on the left side or within the middle circle
            pass

        elif pm == PlayMode.KICK_OFF_RIGHT:
            # TODO: relocate all players of the right team that are on the left side and all players of the left team that are on the right side or within the middle circle
            pass

        elif pm == PlayMode.GOAL_KICK_LEFT:
            # TODO: relocate all players of the right team that are within the left goalie area
            pass

        elif pm == PlayMode.GOAL_KICK_RIGHT:
            # TODO: relocate all players of the left team that are within the right goalie area
            pass

        elif pm in (PlayMode.THROW_IN_RIGHT, PlayMode.CORNER_KICK_RIGHT, PlayMode.FREE_KICK_RIGHT, PlayMode.DIRECT_FREE_KICK_RIGHT):
            # TODO: relocate all players of the left team that are too close to the ball
            pass

        elif pm in (PlayMode.THROW_IN_LEFT, PlayMode.CORNER_KICK_LEFT, PlayMode.FREE_KICK_LEFT, PlayMode.DIRECT_FREE_KICK_LEFT):
            # TODO: relocate all players of the right team that are too close to the ball
            pass


class KickChallengeReferee(SoccerReferee):
    """A referee, applying soccer game rules for a kick-challenge."""

    def __init__(self, game: PSoccerGame | None = None) -> None:
        """Create a new kick challenge referee.

        Parameter
        ---------
        game: PSoccerGame | None, default=None
            The soccer game instance to referee. If ``None``, the game instance must be injected from externally before calling any method of the referee.
        """

        super().__init__(game)

        self._start_time: float = -1
        """The time at which to automatically start the game."""

    def reset(self) -> None:
        super().reset()

        self._start_time = -1

    def referee(self) -> None:
        # check if an agent is connected
        if self._start_time < 0 and len(self.game.get_players(TeamSide.LEFT)) > 0:
            self._start_time = self.game.game_state.get_sim_time() + 2

        # automatically kick-off left 2 seconds after the first agent connected
        if self.game.game_state.get_play_mode() == PlayMode.BEFORE_KICK_OFF and self._start_time >= 0 and self._start_time <= self.game.game_state.get_sim_time():
            self.kick_off(TeamSide.LEFT)

        # forward to base class
        super().referee()

        # calculate challenge score
        if self.game.game_state.get_play_mode() != PlayMode.GAME_OVER:
            score = int((self.game.ball.position[0] - abs(self.game.ball.position[1])) * 100)
            self.game.game_state.set_score(TeamSide.LEFT, score)

    def _check_fouls(self) -> None:
        # disable no score rule
        self._team_na_score = None

        # check double-touch rule
        if self.game.ball.active_contact is not None and self.game.ball.active_contact == self.game.ball.last_contact:
            self.game_over()

    def _check_location_triggers(self) -> None:
        # no location triggers in challenge
        return

    def _relocate_misplaced_players(self) -> None:
        # no misplacement of agents in challenges
        return
