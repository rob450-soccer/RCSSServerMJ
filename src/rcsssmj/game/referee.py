import contextlib
import logging
from math import degrees, pi
from typing import Any, Final

import mujoco

from rcsssmj.agent import AgentID, PAgent
from rcsssmj.client.perception import GameStatePerception, Perception
from rcsssmj.game.game_state import GameState
from rcsssmj.game.rules import SoccerRules
from rcsssmj.game.soccer import PlayMode, TeamSide
from rcsssmj.mjutils import place_robot_3d, quat_from_axis_angle
from rcsssmj.resources.spec_provider import ModelSpecProvider

logger = logging.getLogger(__name__)


class SoccerReferee:
    """A referee, applying soccer game rules."""

    def __init__(self, rules: SoccerRules) -> None:
        """Create a new soccer referee.

        Parameter
        ---------
        rules: SoccerRules
            The soccer rule book to apply.
        """

        self.rules: Final[SoccerRules] = rules

        self._state: GameState = GameState()
        self._team_players: dict[int, list[int]] = {
            TeamSide.LEFT.value: [],
            TeamSide.RIGHT.value: [],
        }

    def get_state(self) -> GameState:
        """Return the current game state."""

        return self._state

    def init_game(self, spec_provider: ModelSpecProvider) -> Any | None:
        """(Re-)initialize the game and create a new world specification.

        Parameter
        ---------
        spec_provider: ModelSpecProvider
            The model specification provider instance.

        Returns
        -------
        MjSpec
            The game specific simulation world / environment specification.
        """

        # init referee state
        self._state = GameState()
        self._team_players[TeamSide.LEFT.value].clear()
        self._team_players[TeamSide.RIGHT.value].clear()

        # load a new soccer world environment
        return self._load_soccer_world(spec_provider)

    def _load_soccer_world(self, spec_provider: ModelSpecProvider) -> Any | None:
        """Load the soccer world.

        Parameter
        ---------
        spec_provider: ModelSpecProvider
            The model specification provider instance.

        Returns
        -------
        MjSpec
            The game specific simulation world / environment specification.
        """

        # load world specification
        world_spec = spec_provider.load_environment('soccer')
        if world_spec is None:
            return None

        # manipulate world to match field description
        def add_marker(name: str, x: float, y: float, z: float) -> Any:
            """Helper function for adding markers to the world."""
            site = world_spec.worldbody.add_site()
            site.name = name + '-vismarker'
            site.pos[0] = x
            site.pos[1] = y
            site.pos[2] = z

        field_half_x = self.rules.field.field_dim[0] / 2
        field_half_y = self.rules.field.field_dim[1] / 2
        field_half_z = self.rules.field.field_dim[2] / 2

        goal_half_y = self.rules.field.goal_dim[1] / 2
        goal_z = self.rules.field.goal_dim[2]

        goalie_area_x = field_half_x - self.rules.field.goalie_area_dim[0]
        goalie_area_half_y = self.rules.field.goalie_area_dim[1]

        # resize field
        pitch_geom = world_spec.geom('pitch')
        pitch_geom.size[0] = field_half_x
        pitch_geom.size[1] = field_half_y
        pitch_geom.size[2] = field_half_z

        # Add floor around the field
        floor_color = [0.2, 0.5, 0.2, 1]  # dark green color
        left_right_floor_x_size = (field_half_x * 1.05) - field_half_x
        top_bottom_floor_y_size = (field_half_y * 1.05) - field_half_y
        left_floor_body = world_spec.worldbody.add_body(
            name='left-floor',
            pos=(-field_half_x, 0, -field_half_z),
        )
        left_floor_body.add_geom(
            name='left-floor',
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0 - left_right_floor_x_size, 0, field_half_z),
            size=(left_right_floor_x_size, field_half_y, field_half_z),
            rgba=floor_color,
        )
        right_floor_body = world_spec.worldbody.add_body(
            name='right-floor',
            pos=(field_half_x, 0, -field_half_z),
        )
        right_floor_body.add_geom(
            name='right-floor',
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0 + left_right_floor_x_size, 0, field_half_z),
            size=(left_right_floor_x_size, field_half_y, field_half_z),
            rgba=floor_color,
        )
        top_floor_body = world_spec.worldbody.add_body(
            name='top-floor',
            pos=(0, field_half_y, -field_half_z),
        )
        top_floor_body.add_geom(
            name='top-floor',
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0, 0 + top_bottom_floor_y_size, field_half_z),
            size=(field_half_x + (left_right_floor_x_size * 2), top_bottom_floor_y_size, field_half_z),
            rgba=floor_color,
        )
        bottom_floor_body = world_spec.worldbody.add_body(
            name='bottom-floor',
            pos=(0, -field_half_y, -field_half_z),
        )
        bottom_floor_body.add_geom(
            name='bottom-floor',
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0, 0 - top_bottom_floor_y_size, field_half_z),
            size=(field_half_x + (left_right_floor_x_size * 2), top_bottom_floor_y_size, field_half_z),
            rgba=floor_color,
        )

        # add goals
        goal_post_color = [0.8, 0.8, 0.8, 1]
        goal_net_color  = [1, 1, 1, 0.2]
        for side, x_sign in [('left', -1), ('right', 1)]:
            goal_name = f'goal-{side}'
            goal_body = world_spec.worldbody.add_body(
                name=goal_name,
                pos=(x_sign * field_half_x, 0, 0),
            )
            depth = x_sign * self.rules.field.goal_dim[0]

            # vertical posts
            goal_body.add_geom(
                name=f'{goal_name}-front-left-post',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[0, -goal_half_y, goal_z/2],
                size=[self.rules.field.goal_post_radius, goal_z/2, 0],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-front-right-post',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[0, goal_half_y, goal_z/2],
                size=[self.rules.field.goal_post_radius, goal_z/2, 0],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-back-left-post',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth, -goal_half_y, goal_z/2],
                size=[self.rules.field.goal_post_radius, goal_z/2, 0],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-back-right-post',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth, goal_half_y, goal_z/2],
                size=[self.rules.field.goal_post_radius, goal_z/2, 0],
                rgba=goal_post_color,
            )

            # crossbars
            goal_body.add_geom(
                name=f'{goal_name}-front-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[0, 0, goal_z],
                size=[self.rules.field.goal_post_radius, goal_half_y, 0],
                quat=[0, 0, 0.7071068, 0.7071068],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-back-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth, 0, goal_z],
                size=[self.rules.field.goal_post_radius, goal_half_y, 0],
                quat=[0, 0, 0.7071068, 0.7071068],
                rgba=goal_post_color,
            )
            # side crossbars (roof)
            goal_body.add_geom(
                name=f'{goal_name}-left-side-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth/2, -goal_half_y, goal_z],
                size=[self.rules.field.goal_post_radius, abs(depth)/2, 0],
                quat=[0, 0.7071068 * x_sign, 0, 0.7071068],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-right-side-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth/2, goal_half_y, goal_z],
                size=[self.rules.field.goal_post_radius, abs(depth)/2, 0],
                quat=[0, -0.7071068 * x_sign, 0, 0.7071068],
                rgba=goal_post_color,
            )

            # lower crossbars (ground level)
            goal_body.add_geom(
                name=f'{goal_name}-back-lower-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth, 0, 0],
                size=[self.rules.field.goal_post_radius, goal_half_y, 0],
                quat=[0, 0, 0.7071068, 0.7071068],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-lower-left-side-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth/2, -goal_half_y, 0],
                size=[self.rules.field.goal_post_radius, abs(depth)/2, 0],
                quat=[0, 0.7071068 * x_sign, 0, 0.7071068],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-lower-right-side-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth/2, goal_half_y, 0],
                size=[self.rules.field.goal_post_radius, abs(depth)/2, 0],
                quat=[0, -0.7071068 * x_sign, 0, 0.7071068],
                rgba=goal_post_color,
            )

            # nets
            goal_body.add_geom(
                name=f'{goal_name}-top-net',
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[depth/2, 0, goal_z],
                size=[abs(depth)/2, goal_half_y, self.rules.field.goal_post_radius],
                rgba=goal_net_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-back-net',
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[depth, 0, goal_z/2],
                size=[self.rules.field.goal_post_radius, goal_half_y, goal_z/2],
                rgba=goal_net_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-left-side-net',
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[depth/2, -goal_half_y, goal_z/2],
                size=[abs(depth)/2, self.rules.field.goal_post_radius, goal_z/2],
                rgba=goal_net_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-right-side-net',
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[depth/2, goal_half_y, goal_z/2],
                size=[abs(depth)/2, self.rules.field.goal_post_radius, goal_z/2],
                rgba=goal_net_color,
            )


        # fmt: off
        # field markers
        add_marker('l_luf', -field_half_x,  field_half_y, 0)  # L-junction: left upper field
        add_marker('l_llf', -field_half_x, -field_half_y, 0)  # L-junction: left lower field
        add_marker('l_ruf',  field_half_x,  field_half_y, 0)  # L-junction: right upper field
        add_marker('l_rlf',  field_half_x, -field_half_y, 0)  # L-junction: right lower field

        add_marker('t_cuf', 0,  field_half_y, 0)  # T-junction: center upper field
        add_marker('t_clf', 0, -field_half_y, 0)  # T-junction: center lower field

        # center circle markers
        add_marker('x_cuc', 0,  self.rules.field.center_circle_radius, 0)  # X-junction: center upper circle
        add_marker('x_clc', 0, -self.rules.field.center_circle_radius, 0)  # X-junction: center lower circle

        # penalty spot markers
        penalty_marker_x = field_half_x - self.rules.field.penalty_spot_distance
        add_marker('p_lpm', -penalty_marker_x, 0, 0)  # X-junction: center upper circle
        add_marker('p_rpm',  penalty_marker_x, 0, 0)  # X-junction: center lower circle

        # add goal post markers
        add_marker('g_lup', -field_half_x,  goal_half_y, goal_z)  # Goal: left upper post
        add_marker('g_llp', -field_half_x, -goal_half_y, goal_z)  # Goal: left lower post
        add_marker('g_rup',  field_half_x,  goal_half_y, goal_z)  # Goal: right upper post
        add_marker('g_rlp',  field_half_x, -goal_half_y, goal_z)  # Goal: right lower post

        # goalie area markers
        add_marker('l_luga', -goalie_area_x,  goalie_area_half_y, 0)  # L-junction: left upper goalie area
        add_marker('l_llga', -goalie_area_x, -goalie_area_half_y, 0)  # L-junction: left lower goalie area
        add_marker('l_ruga',  goalie_area_x,  goalie_area_half_y, 0)  # L-junction: right upper goalie area
        add_marker('l_rlga',  goalie_area_x, -goalie_area_half_y, 0)  # L-junction: right lower goalie area

        add_marker('t_luga', -field_half_x,  goalie_area_half_y, 0)  # T-junction: left upper goalie area
        add_marker('t_llga', -field_half_x, -goalie_area_half_y, 0)  # T-junction: left lower goalie area
        add_marker('t_ruga',  field_half_x,  goalie_area_half_y, 0)  # T-junction: right upper goalie area
        add_marker('t_rlga',  field_half_x, -goalie_area_half_y, 0)  # T-junction: right lower goalie area

        # penalty area markers
        if self.rules.field.penalty_area_dim is not None:
            pen_area_x = field_half_x - self.rules.field.penalty_area_dim[0]
            pen_area_half_y = self.rules.field.penalty_area_dim[1]

            add_marker('l_lupa', -pen_area_x,  pen_area_half_y, 0)  # L-junction: left upper penalty area
            add_marker('l_llpa', -pen_area_x, -pen_area_half_y, 0)  # L-junction: left lower penalty area
            add_marker('l_rupa',  pen_area_x,  pen_area_half_y, 0)  # L-junction: right upper penalty area
            add_marker('l_rlpa',  pen_area_x, -pen_area_half_y, 0)  # L-junction: right lower penalty area

            add_marker('t_lupa', -field_half_x,  pen_area_half_y, 0)  # T-junction: left upper penalty area
            add_marker('t_llpa', -field_half_x, -pen_area_half_y, 0)  # T-junction: left lower penalty area
            add_marker('t_rupa',  field_half_x,  pen_area_half_y, 0)  # T-junction: right upper penalty area
            add_marker('t_rlpa',  field_half_x, -pen_area_half_y, 0)  # T-junction: right lower penalty area
        # fmt: on

        return world_spec

    def request_participation(self, agent: PAgent, model_spec: Any) -> AgentID | None:
        """Validate and add a new agent requesting to join the game.

        Parameter
        ---------
        agent: PAgent
            The agent the requests participation in the game.

        model_spec: MjSpec
            The robot model specification of the new agent.

        Returns
        -------
        AgentID | None
            The agent id identifying the new agent if participation has been accepted, None if participation of the new agent has been rejected.
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

    def spawn_agent(self, agent_id: AgentID, mj_model: Any, mj_data: Any) -> None:
        """Place the given agent at a save (collision free) initial location.

        Parameter
        ---------
        agent_id: AgentID
            The id of the agent to spawn.

        mj_data: MjData
            The mujoco simulation data array.
        """

        if not TeamSide.is_valid(agent_id.team_id):
            msg = 'Invalid team!'
            raise ValueError(msg)

        # field_half_x = self.rules.field.field_dim[0] / 2
        field_half_y = self.rules.field.field_dim[1] / 2
        field_border = self.rules.field.field_border

        x_sign = -1 if agent_id.team_id == TeamSide.LEFT.value else 1
        pos = (x_sign * (2 * agent_id.player_no + 1), field_half_y + field_border, 0.6745)
        quat = quat_from_axis_angle((0, 0, 1), -pi / 2)

        logger.debug('Spawn Team #%d Player #%02d @ (%.3f %.3f, %.3f)', agent_id.team_id, agent_id.player_no, pos[0], pos[1], pos[2])

        place_robot_3d(agent_id.prefix, mj_model, mj_data, pos, quat)

    def handle_withdrawal(self, aid: AgentID) -> None:
        """Handle the withdrawal of an agent participating in the game.

        Parameter
        ---------
        aid: AgentID
            The id of the agent that withdrawed from the game.
        """

        if TeamSide.is_valid(aid.team_id):
            with contextlib.suppress(KeyError):
                self._team_players[aid.team_id].remove(aid.player_no)

    def generate_perception(self) -> Perception:
        """Generate a perception representing the current game state to participating players.

        Returns
        -------
        Perception
            The game state perception.
        """

        return GameStatePerception(
            play_time=self._state.get_play_time(),
            play_mode=self._state.get_play_mode().value,
            team_left=self._state.get_team_name(TeamSide.LEFT),
            team_right=self._state.get_team_name(TeamSide.RIGHT),
            score_left=self._state.get_team_score(TeamSide.LEFT),
            score_right=self._state.get_team_score(TeamSide.RIGHT),
        )

    def beam_agent(self, actuator_name: str, mj_model: Any, mj_data: Any, beam_pose: tuple[float, float, float]) -> None:
        """Perform a beam action for the agent posing the given effector.

        Parameter
        ---------
        actuator_name: str
            The name of the beam actuator.

        mj_data: MjData
            The mujoco simulation data array.

        beam_pose: tuple[float, float, float]
            The desired 2D beam pose (x, y, theta).
            Theta is given in radians.
        """

        # check if agents are allowed to beam
        if not self._is_beaming_allowed():
            return

        agent_id = AgentID.from_prefixed_name(actuator_name)

        if not TeamSide.is_valid(agent_id.team_id):
            msg = 'Invalid team!'
            raise ValueError(msg)

        x_factor = -1 if agent_id.team_id == TeamSide.LEFT.value else 1
        theta_shift = 0 if agent_id.team_id == TeamSide.LEFT.value else pi
        pose = (abs(beam_pose[0]) * x_factor, beam_pose[1], beam_pose[2] + theta_shift)

        logger.debug('Beam Team #%d Player #%02d to (%.3f, %.3f, %.3f)', agent_id.team_id, agent_id.player_no, pose[0], pose[1], degrees(pose[2]))

        pos = (pose[0], pose[1], 0.6745)
        mujoco_quat = quat_from_axis_angle((0, 0, 1), pose[2])

        place_robot_3d(agent_id.prefix, mj_model, mj_data, pos, mujoco_quat)

    def _is_beaming_allowed(self) -> bool:
        """Check if an agent is allowed to beam in the current game state."""

        return self._state.get_play_mode() in (PlayMode.BEFORE_KICK_OFF, PlayMode.GOAL_LEFT, PlayMode.GOAL_RIGHT)

    def referee(self, mj_data: Any) -> None:
        """Referee the current simulation state.

        Parameter
        ---------
        mj_data: MjData
            The mujoco simulation data array.
        """

        self._state.progress(mj_data.time)

        # automatically progress play mode based on timeouts
        self._check_timeouts()

    def request_kickoff(self, team_id: int) -> None:
        """Request kickoff for the given team.

        Parameter
        ---------
        team_id: int
            The team id for which to grant a kick-off.
        """

        self._state.kick_off(TeamSide.from_id(team_id))

    def _check_timeouts(self) -> None:
        """Check timeouts (game over, kick-off time, throw-in time, etc.) for the current play mode."""

        # check game over
        if self._state.get_play_time() > self.rules.half_time:
            self._state.game_over()
            return

        def check_timeout(timeout: int, *left_modes: PlayMode) -> bool:
            """Helper function for checking a play mode specific timeout."""
            return timeout >= 0 and self._state.in_play_mode(*left_modes) and self._state.get_play_mode_age() > timeout

        # check kick-off time
        if check_timeout(self.rules.kick_off_time, PlayMode.KICK_OFF_LEFT, PlayMode.KICK_OFF_RIGHT):
            self._state.play_on()
            return

        # check throw-in time
        if check_timeout(self.rules.throw_in_time, PlayMode.THROW_IN_LEFT, PlayMode.THROW_IN_RIGHT):
            self._state.play_on()
            return

        # check corner-kick time
        if check_timeout(self.rules.corner_kick_time, PlayMode.CORNER_KICK_LEFT, PlayMode.CORNER_KICK_RIGHT):
            self._state.play_on()
            return

        # check goal-kick time
        if check_timeout(self.rules.goal_kick_time, PlayMode.GOAL_KICK_LEFT, PlayMode.GOAL_KICK_RIGHT):
            self._state.play_on()
            return

        # check free-kick time
        if check_timeout(self.rules.free_kick_time, PlayMode.FREE_KICK_LEFT, PlayMode.FREE_KICK_RIGHT):
            self._state.play_on()
            return

        # check direct-free-kick time
        if check_timeout(self.rules.direct_free_kick_time, PlayMode.DIRECT_FREE_KICK_LEFT, PlayMode.DIRECT_FREE_KICK_RIGHT):
            self._state.play_on()
            return

        # check goal pause time
        if self._state.in_play_mode(PlayMode.GOAL_LEFT) and self._state.get_play_mode_age() > self.rules.goal_pause_time:
            self._state.kick_off(TeamSide.RIGHT)
            return

        if self._state.in_play_mode(PlayMode.GOAL_RIGHT) and self._state.get_play_mode_age() > self.rules.goal_pause_time:
            self._state.kick_off(TeamSide.LEFT)
            return
