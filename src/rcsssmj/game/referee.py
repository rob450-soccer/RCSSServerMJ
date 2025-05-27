import contextlib
import logging
from math import degrees, pi
from typing import Any, Final

import mujoco

from rcsssmj.agent import AgentID, PAgent
from rcsssmj.client.perception import GameStatePerception, Perception
from rcsssmj.game.game_object import SoccerBall, SoccerPlayer
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
        """The soccer game rule book."""

        self._state: GameState = GameState()
        """The current soccer game state."""

        self._ball: SoccerBall = SoccerBall()
        """The soccer ball representation."""

        self._team_players: dict[int, dict[int, SoccerPlayer]] = {
            TeamSide.LEFT.value: {},
            TeamSide.RIGHT.value: {},
        }
        """The list of active team players."""

        self._place_ball_pos: tuple[float, float] | None = None
        """The ball placement target position (if existing)."""

        self._agent_na_touch_ball: AgentID | None = None
        """The agent not allowed to touch the ball a second time (if existing)."""

        self._team_na_score: TeamSide | None = None
        """The team side, which is not allowed to score a goal until another agent touches the ball again (if existing)."""

        self._did_act: bool = False
        """Flag if the referee has already taken a decision in this referee cycle."""

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
        self._place_ball_pos = None
        self._agent_na_touch_ball = None
        self._team_na_score = None
        self._did_act = False

        # load a new soccer world environment
        world_spec = self._load_soccer_world(spec_provider)

        # initialize ball
        if world_spec is not None:
            self._ball.activate(world_spec.body('ball'))

        return world_spec

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

        # add floor around the field
        floor_color = [0.2, 0.5, 0.2, 1]  # dark green color
        left_right_floor_x_size = max(1, field_half_x * 0.05)
        top_bottom_floor_y_size = max(1, field_half_y * 0.05)
        left_floor_body = world_spec.worldbody.add_body(name='left-floor', pos=(-field_half_x, 0, 0))
        left_floor_body.add_geom(
            name='left-floor',
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0 - left_right_floor_x_size, 0, 0),
            size=(left_right_floor_x_size, field_half_y, field_half_z),
            rgba=floor_color,
            contype=0,
            conaffinity=0,
            group=2,
            density=0,
        )
        right_floor_body = world_spec.worldbody.add_body(name='right-floor', pos=(field_half_x, 0, 0))
        right_floor_body.add_geom(
            name='right-floor',
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0 + left_right_floor_x_size, 0, 0),
            size=(left_right_floor_x_size, field_half_y, field_half_z),
            rgba=floor_color,
            contype=0,
            conaffinity=0,
            group=2,
            density=0,
        )
        top_floor_body = world_spec.worldbody.add_body(name='top-floor', pos=(0, field_half_y, 0))
        top_floor_body.add_geom(
            name='top-floor',
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0, 0 + top_bottom_floor_y_size, 0),
            size=(field_half_x + (left_right_floor_x_size * 2), top_bottom_floor_y_size, field_half_z),
            rgba=floor_color,
            contype=0,
            conaffinity=0,
            group=2,
            density=0,
        )
        bottom_floor_body = world_spec.worldbody.add_body(name='bottom-floor', pos=(0, -field_half_y, 0))
        bottom_floor_body.add_geom(
            name='bottom-floor',
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0, 0 - top_bottom_floor_y_size, 0),
            size=(field_half_x + (left_right_floor_x_size * 2), top_bottom_floor_y_size, field_half_z),
            rgba=floor_color,
            contype=0,
            conaffinity=0,
            group=2,
            density=0,
        )

        # add goals
        goal_post_color = [0.8, 0.8, 0.8, 1]
        goal_net_color = [1, 1, 1, 0.2]
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
                pos=[0, -goal_half_y, goal_z / 2],
                size=[self.rules.field.goal_post_radius, goal_z / 2, 0],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-front-right-post',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[0, goal_half_y, goal_z / 2],
                size=[self.rules.field.goal_post_radius, goal_z / 2, 0],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-back-left-post',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth, -goal_half_y, goal_z / 2],
                size=[self.rules.field.goal_post_radius, goal_z / 2, 0],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-back-right-post',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth, goal_half_y, goal_z / 2],
                size=[self.rules.field.goal_post_radius, goal_z / 2, 0],
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
                pos=[depth / 2, -goal_half_y, goal_z],
                size=[self.rules.field.goal_post_radius, abs(depth) / 2, 0],
                quat=[0, 0.7071068 * x_sign, 0, 0.7071068],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-right-side-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth / 2, goal_half_y, goal_z],
                size=[self.rules.field.goal_post_radius, abs(depth) / 2, 0],
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
                pos=[depth / 2, -goal_half_y, 0],
                size=[self.rules.field.goal_post_radius, abs(depth) / 2, 0],
                quat=[0, 0.7071068 * x_sign, 0, 0.7071068],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-lower-right-side-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth / 2, goal_half_y, 0],
                size=[self.rules.field.goal_post_radius, abs(depth) / 2, 0],
                quat=[0, -0.7071068 * x_sign, 0, 0.7071068],
                rgba=goal_post_color,
            )

            # nets
            goal_body.add_geom(
                name=f'{goal_name}-top-net',
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[depth / 2, 0, goal_z],
                size=[abs(depth) / 2, goal_half_y, self.rules.field.goal_post_radius],
                rgba=goal_net_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-back-net',
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[depth, 0, goal_z / 2],
                size=[self.rules.field.goal_post_radius, goal_half_y, goal_z / 2],
                rgba=goal_net_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-left-side-net',
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[depth / 2, -goal_half_y, goal_z / 2],
                size=[abs(depth) / 2, self.rules.field.goal_post_radius, goal_z / 2],
                rgba=goal_net_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-right-side-net',
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[depth / 2, goal_half_y, goal_z / 2],
                size=[abs(depth) / 2, self.rules.field.goal_post_radius, goal_z / 2],
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

        # append new player to team dict
        self._team_players[team_id][agent.get_player_no()] = SoccerPlayer(agent_id, model_spec)

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

        mj_model: MjModel
            The mujoco simulation model.

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

    def handle_withdrawal(self, agent_id: AgentID) -> None:
        """Handle the withdrawal of an agent participating in the game.

        Parameter
        ---------
        aid: AgentID
            The id of the agent that withdrawed from the game.
        """

        if TeamSide.is_valid(agent_id.team_id):
            with contextlib.suppress(KeyError):
                del self._team_players[agent_id.team_id][agent_id.player_no]

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

        mj_model: MjModel
            The mujoco simulation model.

        mj_data: MjData
            The mujoco simulation data array.

        beam_pose: tuple[float, float, float]
            The desired 2D beam pose (x, y, theta).
            Theta is given in radians.
        """

        # check if agents are allowed to beam
        if not self._is_beaming_allowed():
            return

        # fetch agent id
        agent_id = AgentID.from_prefixed_name(actuator_name)
        if agent_id is None:
            return

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
        self._state.set_play_mode_for_team(team_side, PlayMode.KICK_OFF_LEFT, PlayMode.KICK_OFF_RIGHT)
        self._place_ball_pos = (0, 0)
        self._ball.reset_contacts()
        self._agent_na_touch_ball = None
        self._team_na_score = team_side

    def play_on(self) -> None:
        """Instruct the normal progressing of the game."""

        self._did_act = True
        self._state.set_play_mode(PlayMode.PLAY_ON)

    def throw_in(self, team_side: TeamSide) -> None:
        """Instruct a throw in for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the throw in.
        """

        self._did_act = True
        self._state.set_play_mode_for_team(team_side, PlayMode.THROW_IN_LEFT, PlayMode.THROW_IN_RIGHT)
        y = self.rules.field.field_area.min_y if self._ball.position[1] < 0 else self.rules.field.field_area.max_y
        self._place_ball_pos = (self._ball.position[0], y)
        self._ball.reset_contacts()
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
        self._state.set_play_mode_for_team(team_side, PlayMode.CORNER_KICK_LEFT, PlayMode.CORNER_KICK_RIGHT)
        x = self.rules.field.field_area.max_x if team_side == TeamSide.LEFT else self.rules.field.field_area.min_x
        y = self.rules.field.field_area.min_y if self._ball.position[1] < 0 else self.rules.field.field_area.max_y
        self._place_ball_pos = (x, y)
        self._ball.reset_contacts()
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
        self._state.set_play_mode_for_team(team_side, PlayMode.GOAL_KICK_LEFT, PlayMode.GOAL_KICK_RIGHT)
        self._place_ball_pos = self.rules.field.left_goalie_area.center() if team_side == TeamSide.LEFT else self.rules.field.right_goalie_area.center()
        self._ball.reset_contacts()
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
        self._state.set_play_mode_for_team(team_side, PlayMode.OFFSIDE_LEFT, PlayMode.OFFSIDE_RIGHT)

    def game_over(self) -> None:
        """Instruct the end of the game."""

        self._did_act = True
        self._state.set_play_mode(PlayMode.GAME_OVER)
        self._place_ball_pos = None
        self._ball.reset_contacts()
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
        self._state.goal(team_side)
        self._place_ball_pos = None
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
        self._state.set_play_mode_for_team(team_side, PlayMode.FREE_KICK_LEFT, PlayMode.FREE_KICK_RIGHT)
        self._place_ball_pos = self._ball.position[0:2]
        self._ball.reset_contacts()
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
        self._state.set_play_mode_for_team(team_side, PlayMode.DIRECT_FREE_KICK_LEFT, PlayMode.DIRECT_FREE_KICK_RIGHT)
        self._place_ball_pos = self._ball.position[0:2]
        self._ball.reset_contacts()
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
        self._state.set_play_mode_for_team(team_side, PlayMode.PENALTY_KICK_LEFT, PlayMode.PENALTY_KICK_RIGHT)
        penalty_spot_x = self.rules.field.field_area.max_x - self.rules.field.penalty_spot_distance
        self._place_ball_pos = (-penalty_spot_x if team_side == TeamSide.LEFT else penalty_spot_x, 0)
        self._ball.reset_contacts()
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
        self._state.set_play_mode_for_team(team_side, PlayMode.PENALTY_SHOOT_LEFT, PlayMode.PENALTY_SHOOT_RIGHT)
        # TODO: place ball for penalty-shoot

    def drop_ball(self, pos: tuple[float, float] | None = None) -> None:
        """Drop the ball at the specified position and instruct the normal progressing of the game.

        Parameter
        ---------
        pos: tuple[float, float] | None, default=None
            The position at which to drop the ball or none, to drop it at its current location.
        """

        self._did_act = True
        self._place_ball_pos = self._ball.position[0:2] if pos is None else pos
        # TODO: cause relocation of all agents nearby the ball (within a radius defined here)

        self._state.set_play_mode(PlayMode.PLAY_ON)

    def referee(self, mj_model: Any, mj_data: Any) -> None:
        """Referee the current simulation state.

        Parameter
        ---------
        mj_model: MjModel
            The mujoco simulation model.

        mj_data: MjData
            The mujoco simulation data array.
        """

        # progress game state
        self._state.progress(mj_data.time)

        # check game over
        if self._state.get_play_time() >= self.rules.half_time:
            self.game_over()
            return

        # update the state of the ball and player representations
        self._update_game_objects(mj_model, mj_data)

        # check for rule violations
        self._check_fouls()

        # automatically progress play mode based on timeouts, object locations and action triggers
        self._check_timeouts()
        self._check_location_triggers()
        self._check_contact_triggers()

        # relocate ball (if requested) and players (according to play mode restrictions)
        self._relocate_ball(mj_model, mj_data)
        self._relocate_misplaced_players(mj_model, mj_data)

        # reset decision flag
        self._did_act = False

    def _update_game_objects(self, mj_model: Any, mj_data: Any) -> None:
        """Update the state of the game objects (ball and players).

        Parameter
        ---------
        mj_model: MjModel
            The mujoco simulation model.

        mj_data: MjData
            The mujoco simulation data array.
        """

        self._ball.update(mj_model, mj_data)

        for p in self._team_players[TeamSide.LEFT.value].values():
            p.update(mj_model, mj_data)

        for p in self._team_players[TeamSide.RIGHT.value].values():
            p.update(mj_model, mj_data)

    def _check_fouls(self) -> None:
        """Check fouls / violations of game rules."""

        # check no score rule
        if self._team_na_score is not None and self._ball.active_contact is not None and self._ball.last_contact is not None:
            self._team_na_score = None

        # check double-touch rule
        if self._agent_na_touch_ball is not None and self._ball.active_contact is not None and self._ball.last_contact is not None:
            if self._agent_na_touch_ball == self._ball.last_contact and self._agent_na_touch_ball == self._ball.active_contact:
                self.free_kick(TeamSide.get_opposing_side(self._agent_na_touch_ball.team_id))
                return

            self._agent_na_touch_ball = None

    def _check_timeouts(self) -> None:
        """Check timeouts (kick-off time, throw-in time, etc.) for the current play mode."""

        if self._did_act:
            # the referee has already taken a decision in this simulation cycle
            return

        pm = self._state.get_play_mode()

        if pm == PlayMode.PLAY_ON:
            # shortcut, as remaining rules only apply to other states than play-on
            return

        def check_timeout(timeout: int, *play_modes: PlayMode) -> bool:
            """Helper function for checking a play mode specific timeout."""
            return timeout >= 0 and pm in play_modes and self._state.get_play_mode_age() > timeout

        # check kick-off, throw-in, corner-kick and free-kick times
        if (
            check_timeout(self.rules.kick_off_time, PlayMode.KICK_OFF_LEFT, PlayMode.KICK_OFF_RIGHT)
            or check_timeout(self.rules.throw_in_time, PlayMode.THROW_IN_LEFT, PlayMode.THROW_IN_RIGHT)
            or check_timeout(self.rules.corner_kick_time, PlayMode.CORNER_KICK_LEFT, PlayMode.CORNER_KICK_RIGHT)
            or check_timeout(self.rules.free_kick_time, PlayMode.FREE_KICK_LEFT, PlayMode.FREE_KICK_RIGHT)
            or check_timeout(self.rules.direct_free_kick_time, PlayMode.DIRECT_FREE_KICK_LEFT, PlayMode.DIRECT_FREE_KICK_RIGHT)
        ):
            self.play_on()
            return

        # check goal-kick times
        if check_timeout(self.rules.goal_kick_time, PlayMode.GOAL_KICK_LEFT):
            # drop ball at a corner of the left goalie area
            self.drop_ball((self.rules.field.left_goalie_area.max_x, self.rules.field.left_goalie_area.max_y))
            return

        if check_timeout(self.rules.goal_kick_time, PlayMode.GOAL_KICK_RIGHT):
            self.drop_ball((self.rules.field.right_goalie_area.min_x, self.rules.field.right_goalie_area.max_y))
            return

        # check goal pause time
        if pm == PlayMode.GOAL_LEFT and self._state.get_play_mode_age() > self.rules.goal_pause_time:
            self.kick_off(TeamSide.RIGHT)
            return

        if pm == PlayMode.GOAL_RIGHT and self._state.get_play_mode_age() > self.rules.goal_pause_time:
            self.kick_off(TeamSide.LEFT)
            return

    def _check_location_triggers(self) -> None:
        """Check location triggers (ball leaving the field in play-on, leaving the goalie-area in goal-kick, etc.) for the current play mode."""

        if self._did_act:
            # the referee has already taken a decision in this simulation cycle
            return

        pm = self._state.get_play_mode()

        if pm in (PlayMode.GOAL_LEFT, PlayMode.GOAL_RIGHT):
            # no location triggers in goal states
            return

        # check left goal
        if self.rules.field.left_goal_box.contains(self._ball.position[0], self._ball.position[1], self._ball.position[2]):
            if pm == PlayMode.GOAL_KICK_LEFT:
                # drop ball at a corner of the left goalie area
                self.drop_ball((self.rules.field.left_goalie_area.max_x, self.rules.field.left_goalie_area.max_y))
            elif self._team_na_score == TeamSide.RIGHT:
                self.goal_kick(TeamSide.LEFT)
            else:
                self.goal(TeamSide.RIGHT)
            return

        # check right goal
        if self.rules.field.right_goal_box.contains(self._ball.position[0], self._ball.position[1], self._ball.position[2]):
            if pm == PlayMode.GOAL_KICK_RIGHT:
                # drop ball at a corner of the left goalie area
                self.drop_ball((self.rules.field.right_goalie_area.min_x, self.rules.field.right_goalie_area.max_y))
            elif self._team_na_score == TeamSide.LEFT:
                self.goal_kick(TeamSide.RIGHT)
            else:
                self.goal(TeamSide.LEFT)
            return

        # check if the ball left the field
        if not self.rules.field.field_area.contains(self._ball.position[0], self._ball.position[1]):
            agent_contact = self._ball.get_most_recent_contact()
            last_team_contact = TeamSide.UNKNOWN if agent_contact is None else TeamSide.from_id(agent_contact.team_id)

            if self._ball.position[0] < self.rules.field.field_area.min_x:
                # corner-kick right / goal-kick left
                if last_team_contact == TeamSide.LEFT:
                    self.corner_kick(TeamSide.RIGHT)
                else:
                    self.goal_kick(TeamSide.LEFT)

            elif self._ball.position[0] > self.rules.field.field_area.max_x:
                # corner-kick left / goal-kick right
                if last_team_contact == TeamSide.RIGHT:
                    self.corner_kick(TeamSide.LEFT)
                else:
                    self.goal_kick(TeamSide.RIGHT)

            # elif self._ball.position[1] < -self.rules.field.field_area.min_y or self._ball.position[1] > self.rules.field.field_area.max_y:
            else:
                # throw-in
                self.throw_in(TeamSide.get_opposing_side(last_team_contact))

            return

        # check if ball left the goalie area on goal-kick
        if (pm == PlayMode.GOAL_KICK_LEFT and not self.rules.field.left_goalie_area.contains(self._ball.position[0], self._ball.position[1])) or (
            pm == PlayMode.GOAL_KICK_RIGHT and not self.rules.field.right_goalie_area.contains(self._ball.position[0], self._ball.position[1])
        ):
            self.play_on()
            return

    def _check_contact_triggers(self) -> None:
        """Check contact action triggers (touching the ball in kick-off, throw-in, etc.) for the current play mode."""

        if self._did_act:
            # the referee has already taken a decision in this simulation cycle
            return

        if self._ball.active_contact is None:
            # no action trigger
            return

        pm = self._state.get_play_mode()

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
            if len(self._team_players[self._ball.active_contact.team_id]) > 1:
                self._agent_na_touch_ball = self._ball.active_contact

            self.play_on()
            return

    def _relocate_ball(self, mj_model: Any, mj_data: Any) -> None:
        """Relocate ball (if requested).

        Parameter
        ---------
        mj_model: MjModel
            The mujoco simulation model.

        mj_data: MjData
            The mujoco simulation data array.
        """

        if self._place_ball_pos is not None:
            pos = (self._place_ball_pos[0], self._place_ball_pos[1], 0.12)
            rot = (1, 0, 0, 0)

            place_robot_3d('ball-', mj_model, mj_data, pos, rot)
            self._ball.last_position = pos
            self._ball.position = pos
            self._place_ball_pos = None

    def _relocate_misplaced_players(self, mj_model: Any, mj_data: Any) -> None:
        """Check if players are within areas they are not allowed in and, in case, relocate them accordingly.

        Parameter
        ---------
        mj_model: MjModel
            The mujoco simulation model.

        mj_data: MjData
            The mujoco simulation data array.
        """

        pm = self._state.get_play_mode()

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
