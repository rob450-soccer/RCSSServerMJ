import contextlib
import logging
from math import degrees, pi
from typing import Any, Final

from rcsssmj.agent import AgentID, PAgent
from rcsssmj.client.perception import GameStatePerception, Perception
from rcsssmj.game.game_state import GameState
from rcsssmj.game.rules import SoccerRules
from rcsssmj.game.soccer import TeamSide
from rcsssmj.mjutils import place_robot_2d, place_robot_3d

logger = logging.getLogger(__name__)


class SoccerReferee:
    """
    A referee, applying soccer game rules.
    """

    def __init__(self, rules: SoccerRules) -> None:
        """
        Create a new game.
        """

        self.rules: Final[SoccerRules] = rules
        self._state: GameState = GameState()

        self._team_players: dict[int, list[int]] = {
            TeamSide.LEFT.value: [],
            TeamSide.RIGHT.value: [],
        }

    def get_state(self) -> GameState:
        """
        Return the current game state.
        """

        return self._state

    def request_participation(self, agent: PAgent, model_spec: Any) -> AgentID | None:
        """
        Validate and add a new agent requesting to join the game.
        """

        # check player number
        if agent.get_player_no() > self.rules.max_player_no:
            return None

        # update known team names
        self._state.update_team_names(agent.get_team_name())

        # fetch team side for client
        team_id = self._state.get_team_side(agent.get_team_name()).value

        if not TeamSide.is_valid(team_id):
            return None

        # check if a player with the same player number of the client is already present in the game
        if agent.get_player_no() in self._team_players[team_id]:
            return None

        # check if the new player would exceed the maximum number of allowed players per team
        if len(self._team_players[team_id]) >= self.rules.max_team_size:
            return None

        agent_id = AgentID(team_id, agent.get_player_no())

        # append player number to team list
        self._team_players[team_id].append(agent.get_player_no())

        # set team color
        root_body = model_spec.body('torso')
        root_body.first_geom().rgba = [0, 0, 1, 1] if team_id == TeamSide.LEFT.value else [1, 0, 0, 1]

        return agent_id

    def spawn_agent(self, agent_id: AgentID, mj_data: Any) -> None:
        """
        Place the given agent at a save (collision free) initial location.
        """

        if not TeamSide.is_valid(agent_id.team_id):
            msg = 'Invalid team!'
            raise ValueError(msg)

        # field_half_x = self.rules.field.field_dim[0] / 2
        field_half_y = self.rules.field.field_dim[1] / 2
        field_border = self.rules.field.field_border

        x_sign = -1 if agent_id.team_id == TeamSide.LEFT.value else 1
        pos = (x_sign * (2 * agent_id.player_no + 1), field_half_y + field_border, 1.2)
        quat = (1, 0, 0, 0)

        logger.debug('Spawn Team #%d Player #%02d @ (%.3f %.3f, %.3f)', agent_id.team_id, agent_id.player_no, pos[0], pos[1], pos[2])

        place_robot_3d(agent_id.prefix, mj_data, pos, quat)

    def handle_withdrawal(self, aid: AgentID) -> None:
        """
        Handle the withdrawal of a client participating in the game.
        """

        if TeamSide.is_valid(aid.team_id):
            with contextlib.suppress(KeyError):
                self._team_players[aid.team_id].remove(aid.player_no)

    def generate_perception(self) -> Perception:
        """
        Generate a perception representing the current game state to participating players.
        """

        return GameStatePerception(
            score_left=self._state.get_team_score(TeamSide.LEFT),
            score_right=self._state.get_team_score(TeamSide.RIGHT),
            play_time=self._state.get_play_time(),
            play_mode=self._state.get_play_mode().value,
        )

    def beam_agent(self, actuator_name: str, mj_data: Any, beam_pose: tuple[float, float, float]) -> None:
        """
        Get a save beam pose for the given client.
        """

        agent_id = AgentID.from_prefixed_name(actuator_name)

        if not TeamSide.is_valid(agent_id.team_id):
            msg = 'Invalid team!'
            raise ValueError(msg)

        x_factor = -1 if agent_id.team_id == TeamSide.LEFT.value else 1
        theta_shift = 0 if agent_id.team_id == TeamSide.LEFT.value else pi
        pose = (abs(beam_pose[0]) * x_factor, beam_pose[1], beam_pose[2] + theta_shift)

        logger.debug('Beam Team #%d Player #%02d to (%.3f, %.3f, %.3f)', agent_id.team_id, agent_id.player_no, pose[0], pose[1], degrees(pose[2]))

        place_robot_2d(agent_id.prefix, mj_data, pose)

    def referee(self, mj_data: Any) -> None:
        """
        Referee the current simulation state.
        """

        self._state.progress(mj_data.time)

        # check if game is over
        self._check_game_over()

    def request_kickoff(self, team_id: int) -> None:
        """
        Request kickoff for the given team.
        """

        self._state.kick_off(TeamSide.from_id(team_id))

    def _check_game_over(self) -> None:
        """
        Check if the game is over.
        """

        if self._state.get_play_time() > self.rules.half_time:
            self._state.game_over()
